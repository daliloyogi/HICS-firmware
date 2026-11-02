// ============================================================================
//  Hidroponik Selada — ESP32-MAIN (src/main.cpp)
//  Aligned to live Firebase structure: Hics/hics2026/{control,dataStream}.
//  Sensors: ADS1115 (pH=A0, TDS=A1), AHT10 (air temp/humidity), DS18B20 (GPIO32),
//           ultrasonic JSN/AJ-SR04M (water level).
//  Actuators: phUp, phDown, nutrisiA, nutrisiB, waterPump.
//  Cloud: FirebaseClient async — WRITE dataStream + STREAM control commands.
//  Needs secrets.h with FIREBASE_* and WIFI_SSID/PASS.
// ============================================================================

#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <VoltX_UART.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_AHTX0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_bt.h>
#include "secrets.h"

#define WDT_TIMEOUT_S 30

#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "time.google.com"

// Max time at boot to wait for WiFi before giving up and running OFFLINE (sensors only,
// no cloud). Non-blocking — handleWiFi() keeps retrying in the background afterwards, and
// firebaseInit()/NTP still kick in later if the connection eventually comes up.
#define WIFI_BOOT_TIMEOUT_MS 15000UL

// How often the sensor snapshot is pushed to ESP32-LCD over UART. Deliberately decoupled from the
// 1s sensor read: readSensor() must stay fast because controlAuto() doses on its output and the
// ultrasonic fault counter needs 10 reads to trip the interlock — slowing the read to match this
// would stretch that detection to LCD_PUSH_MS * 10. Only the display refresh is throttled here.
#define LCD_PUSH_MS 3000UL

using namespace firebase_ns;

// ----------------------------------------------------------------------------
//  SAFETY: set to 0 for bench testing without pumps connected.
// ----------------------------------------------------------------------------
#define ACTUATORS_ENABLED 1

// ---- Relay drive polarity ----
// The 4 dosing pumps are switched by an ACTIVE-LOW relay module (replaced the L298N, which dropped
// 12V->10V and starved the pumps): the relay energizes (pump ON) when the control pin is driven LOW.
// The waterPump is different — an SSR wired ACTIVE-HIGH (ON = pin HIGH). So polarity is per-actuator:
// use relayLevel(i, on) rather than these macros directly. RELAY_ON/OFF below are the dosing default.
#define RELAY_ACTIVE_LOW 1
#if RELAY_ACTIVE_LOW
#define RELAY_ON LOW
#define RELAY_OFF HIGH
#else
#define RELAY_ON HIGH
#define RELAY_OFF LOW
#endif

// ---- Device ID ----
// Identifies THIS unit within the DB tree (/Hics/<DEVICE_ID>/...). Set it per unit in secrets.h:
//   #define DEVICE_ID "hics2026"
// The fallback below keeps the build working if secrets.h doesn't define it yet. To provision a
// new unit, change DEVICE_ID in that unit's secrets.h and reflash — nothing else needs editing.
#ifndef DEVICE_ID
#define DEVICE_ID "hics2026"
#endif

// ---- Firebase paths (match live DB: Hics/<DEVICE_ID>/...) ----
// C adjacent string-literal concatenation: "/Hics/" DEVICE_ID "/control" is fused into a single
// compile-time literal, so every PATH_* stays a plain const char* — no runtime cost, and every
// existing call site (String(PATH_X)+..., snprintf("%s"...), API calls) keeps working unchanged.
#define PATH_CONTROL "/Hics/" DEVICE_ID "/control"             // app writes commands here (we subscribe)
#define PATH_STATUS "/Hics/" DEVICE_ID "/status"               // we write actual actuator on/off here (never subscribed to — see actSet())
#define PATH_SENSORS "/Hics/" DEVICE_ID "/dataStream"          // we write telemetry here
#define PATH_SETTING "/Hics/" DEVICE_ID "/setting"             // app config: phMin/Max, ppmMin/Max, intervalUpdate
#define PATH_DEVICE "/Hics/" DEVICE_ID "/device"               // device status: connected, lastUpdate, ssid
#define PATH_HISTORY "/Hics/" DEVICE_ID "/history"             // hourly snapshots: history/YYYY/MM/DD/HH
#define PATH_NOTIFICATIONS "/Hics/" DEVICE_ID "/notifications" // alerts: notifications/<timestamp_ms>
#define PATH_CALIBRATION "/Hics/" DEVICE_ID "/calibration"     // sensor calib capture: command/sensor in, rawData/data0..30 out (POLLED, no stream — see updateCalibrationCapture())

// ---- Actuator count ----
#define ACT_COUNT 5
#define WATER_PUMP_IDX 4 // index of waterPump in actPin[]/actName[] — an SSR wired ACTIVE-HIGH,
                         // unlike the 4 dosing relays (ACTIVE-LOW). See relayLevel().

// ---- Pins ----
#define PIN_PH_UP 13
#define PIN_PH_DOWN 33 // moved off GPIO12: 12 is a strapping pin held HIGH by the active-LOW relay at boot -> wrong flash voltage -> "invalid header" boot loop. 33 is a safe output pin.
#define PIN_NUTRISI_A 14
#define PIN_NUTRISI_B 27
#define PIN_WATER_PUMP 19 // moved off 26 (26 collided with ultrasonic trig)
#define PIN_DS18B20 32
#define PIN_TRIG 25
#define PIN_ECHO 26
#define PIN_RXD2 16
#define PIN_TXD2 17

VoltX_UART ESP_LCD;

// ---- WiFi credential store (NVS via Preferences) ----
// Falls back to compile-time WIFI_SSID/WIFI_PASS (secrets.h) on first boot / empty store.
// connect_wifi persists whatever network the user picks here, so handleWiFi()'s auto-reconnect
// and the next reboot both target the LAST network actually connected, not the compile-time one.
Preferences wifiPrefs;
String cfg_wifiSsid;
String cfg_wifiPass;
bool wifiUserDisconnected = false; // true after disconnect_wifi — handleWiFi() stops auto-reconnecting

void loadWifiCredentials()
{
  wifiPrefs.begin("wifi", true); // read-only
  cfg_wifiSsid = wifiPrefs.getString("ssid", WIFI_SSID);
  cfg_wifiPass = wifiPrefs.getString("pass", WIFI_PASS);
  wifiPrefs.end();
}

void saveWifiCredentials(const String &ssid, const String &pass)
{
  cfg_wifiSsid = ssid;
  cfg_wifiPass = pass;
  wifiPrefs.begin("wifi", false);
  wifiPrefs.putString("ssid", ssid);
  wifiPrefs.putString("pass", pass);
  wifiPrefs.end();
  Serial.printf("[wifi] credentials saved to NVS -> '%s'\n", ssid.c_str());
}

// ---- Settings (defaults; overridden at runtime from /setting Firebase node) ----
float cfg_phMin = 5.5f;
float cfg_phMax = 7.5f;
int cfg_ppmMin = 400;
int cfg_ppmMax = 1200;             // pegang obeng wkwkkwkwkw
uint32_t cfg_dataPushMs = 10000UL; // intervalUpdate (seconds * 1000)
float cfg_dosingMl = 10.0f;        // dosingMl — volume per dose (mL), shared by all dosing pumps
// Per-pump peristaltic flow rate (mL/min) — index matches actName/actIndexFromName (0=phUp 1=phDown 2=nutrisiA 3=nutrisiB).
// Each pump/tube wears differently, so each gets its own calibrated value — see "Pump Calibration" section.
float cfg_pumpFlowMlMin[4] = {100.0f, 100.0f, 100.0f, 100.0f};
// computed — do NOT set from Firebase directly, use dosingMl + cfg_pumpFlowMlMin[i]
uint32_t cfg_dosingMs[4] = {6000UL, 6000UL, 6000UL, 6000UL};
uint32_t cfg_mixingMs = 30000UL; // mixingS (seconds * 1000)
uint32_t cfg_pumpOnHr = 1UL;     // pumpOnHour (hours)
uint32_t cfg_pumpOffHr = 2UL;    // pumpOffHour (hours)
float cfg_tankDepthCm = 150.0f;  // tankDepthCm — total tank depth from sensor to bottom (cm)
float cfg_waterLowPct = 5.0f;    // waterLowPct — water level % below this halts dosing & pump
bool cfg_waterGuard = true;      // waterGuard — set false to make controlAuto() ignore water level (use only while the ultrasonic sensor is unreliable; removes dry-run/over-dose protection)
String cfg_ppmUnit = "PPM";      // ppmUnit — display unit for the nutrient/EC reading, forwarded to LCD
String cfg_tempUnit = "C";       // tempUnit — display unit shared by air & water temperature, forwarded to LCD

// ---- Calibration (overridden at runtime from /setting Firebase node) ----
float cfg_phSlope = -5.72f;
float cfg_phOffset = 15.44f;
float cfg_tdsFactor = 0.65f;

// ---- Debug (dummy sensor injection, overridden at runtime from /setting/debugSensor) ----
bool cfg_debugPh = false;
bool cfg_debugTds = false;
bool cfg_debugAht = false;
bool cfg_debugDs18b20 = false;
bool cfg_debugUltrasonic = false;

// ---- Sensors ----
Adafruit_ADS1115 ads;
Adafruit_AHTX0 aht;
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds(&oneWire);
bool adsOK = false, ahtOK = false, dsOK = false;
bool waterLevelValid = false; // true after first successful ultrasonic reading
bool ultrasonicOK = false;    // false after 10 consecutive timeouts — blocks Firebase push

struct
{
  float pH = 0;
  int ppm = 0;
  float airTemp = 0, humidity = 0, waterTemp = 0, waterLevel = 0;
} data;

// ---- Actuators (0=phUp 1=phDown 2=nutrisiA 3=nutrisiB 4=pump) ----
const uint8_t actPin[ACT_COUNT] = {PIN_PH_UP, PIN_PH_DOWN, PIN_NUTRISI_A, PIN_NUTRISI_B, PIN_WATER_PUMP};
const char *actName[ACT_COUNT] = {"phUp", "phDown", "nutrisiA", "nutrisiB", "waterPump"};
bool actState[ACT_COUNT] = {false};
uint32_t pulseStart[ACT_COUNT] = {0};
uint32_t pulseDur[ACT_COUNT] = {0};
bool pulseActive[ACT_COUNT] = {false};

// ---- Control ----
enum Mode
{
  AUTO,
  MANUAL
};
Mode mode = MANUAL;
uint32_t lastPhDose[2] = {0};  // [0]=phUp  [1]=phDown
uint32_t lastNutrientDose = 0; // shared cooldown for nutrisiUp+nutrisiDown pair
uint32_t pumpCycleStart = 0;
bool pumpOn = false;

// ---- App-commanded dosing (see "App Dosing" section) ----
// One independent node per dosing pump: control/<pump>/{start,volume,status,reason}
float doseVolumeSel[4] = {0, 0, 0, 0}; // per-pump dose volume (mL), cached from control/<pump>/volume
bool doseFromApp[4] = {false};         // true while an app-commanded dose is mid-pulse (drives the "success" write)
// Dose status is eventually-consistent: setDoseStatus() records the DESIRED status in RAM and
// marks it dirty; flushDoseStatus() pushes it whenever Firebase is up, and every reconnect
// re-asserts it. A single dropped write (e.g. SSL error -118 mid-dose) therefore self-heals
// instead of freezing Firebase on a stale "running".
String doseDesiredStatus[4] = {"idle", "idle", "idle", "idle"};
String doseDesiredReason[4] = {"", "", "", ""};
bool doseStatusDirty[4] = {false};

// ---- Pump calibration (per-pump peristaltic flow rate, see "Pump Calibration" section) ----
// One independent node per pump: control/calibrate/<pump>/{durationS,start,measuredMl}
uint32_t calibDurationSel[4] = {10UL, 10UL, 10UL, 10UL}; // per-pump test duration (s), cached from .../durationS
int calibIdx = -1;                                       // actuator index (0-3) currently being calibrated, -1 = idle
uint32_t calibDurationS = 0;                             // duration actually used for the run in progress
bool calibRunning = false;                               // test pulse in progress
bool calibAwaitingResult = false;                        // pulse finished, waiting for measuredMl submission
bool calibViaLcd = false;                                // current run was triggered from the LCD (report over UART, not Firebase notif)

// ---- Sensor calibration (TDS 1-point) ----
// pH is NOT calibrated on-device: cfg_phSlope/cfg_phOffset come straight from /setting.
float calibTdsRef = 500.0f; // known reference ppm, cached from control/calibrate/tds/refPpm

// ---- Firebase: THREE clients (writes + control stream + setting stream), all GLOBAL ----
WiFiClientSecure ssl_client;
WiFiClientSecure stream_ssl_client;
WiFiClientSecure setting_ssl_client;
AsyncClientClass async_client(ssl_client);           // one-off writes
AsyncClientClass stream_client(stream_ssl_client);   // persistent SSE stream: control
AsyncClientClass setting_client(setting_ssl_client); // persistent SSE stream: setting
FirebaseApp app;
RealtimeDatabase Database;
UserAuth user_auth(FIREBASE_API_KEY, FIREBASE_EMAIL, FIREBASE_PASSWORD);
bool fbInit = false;
bool streamInit = false;
bool settingStreamStarted = false; // /setting stream requested — config load in flight
bool settingsLoaded = false;       // set once the first full /setting payload has been applied

// ---- Boot sequencing: hold sensor reads until WiFi resolves (connected or timed out) ----
bool bootReady = false;
bool bootOffline = false; // true bila boot berakhir lewat timeout (mode OFFLINE)
uint32_t bootStartMs = 0;
uint32_t lastBootMsgMs = 0; // heartbeat kirim status boot ke LCD (tiap ~1s selama belum bootReady)

// ---- Timing ----
uint32_t lastSensorRead = 0, lastHeartbeat = 0, lastDataPush = 0, lastLcdPush = 0;
bool dsStarted = false;   // DS18B20: a conversion has been requested
bool sensorsRead = false; // true after the FIRST readSensor() — gates checkNotifications().
                          // Not derivable from lastSensorRead: the boot gate seeds that with
                          // millis() before any read happens, so it is already non-zero here.

// ---- Hourly history accumulators — snapshot per jam = RATA-RATA sampel valid sejam (bukan nilai
// sesaat). Nilai 0 pada pH/ppm/suhu/kelembapan = ERROR sensor -> DIKECUALIKAN (tiap field punya
// penghitung sendiri). waterLevel: 0 itu sah (tangki kosong) -> hanya digate ultrasonicOK. ----
double accPh = 0, accPpm = 0, accAirT = 0, accHum = 0, accWaterT = 0, accLevel = 0;
uint32_t cntPh = 0, cntPpm = 0, cntAirT = 0, cntHum = 0, cntWaterT = 0, cntLevel = 0;
uint32_t accCount = 0; // total pembacaan sejam (untuk tahu ada aktivitas / tidak kosong)

// ============================================================================
//  Actuators
// ============================================================================
void processFirebaseData(AsyncResult &r);                      // forward declaration
void startCalibration(int idx);                                // forward declaration
void submitCalibrationResult(int idx, float measuredMl);       // forward declaration
bool canDosePH(int i);                                         // forward declaration
bool canDoseNutrient();                                        // forward declaration
void captureTdsCalib();                                        // forward declaration
void sendNotification(const char *title, const char *message); // forward declaration

// Digital level to write for a desired on/off state, honouring per-actuator polarity:
// dosing relays 0..3 are ACTIVE-LOW, the waterPump (WATER_PUMP_IDX) is an ACTIVE-HIGH SSR.
inline uint8_t relayLevel(int i, bool on)
{
  if (i == WATER_PUMP_IDX)
    return on ? HIGH : LOW; // SSR active-HIGH
  return on ? LOW : HIGH;   // relay active-LOW
}

void actSet(int i, bool on)
{
#if ACTUATORS_ENABLED
  digitalWrite(actPin[i], relayLevel(i, on));
#endif
  if (actState[i] == on)
    return; // no change — skip Firebase write
  actState[i] = on;
  if (fbInit && app.ready())
  {
    // Reported state goes to /status, NOT /control — /control is the command node
    // stream_client is subscribed to, so writing actual state there would self-echo
    // back as an incoming command (e.g. a manual dose's actSet(i,true) would come back
    // as "phUp:true", re-triggering applyManual() and resetting the pulse timer,
    // silently extending every calibrated dose by one Firebase round-trip).
    String path = String(PATH_STATUS) + "/" + actName[i];
    Database.set<bool>(async_client, path, on, processFirebaseData, "actFb");
    Serial.printf("[actFb] %s -> %s\n", actName[i], on ? "true" : "false");
  }
}
void actPulse(int i, uint32_t ms)
{
  pulseStart[i] = millis();
  pulseDur[i] = ms;
  pulseActive[i] = true;
  actSet(i, true);
}

// Reports an app-commanded dose's outcome to control/<pump>/{status,reason}.
// The device owns `status` end to end: "running" on accept, "success" on completion,
// "rejected" (+reason) on refusal. Every path that refuses or aborts a dose MUST land here —
// a dose that goes quiet leaves the app's UI pinned on "running" forever.
// This does NOT write directly: it records the desired state and lets flushDoseStatus() push
// it. That way a status produced while Firebase is down (dose finishing during an SSL drop) is
// still pending and gets written the moment the link returns — no stale "running" left behind.
void setDoseStatus(int idx, const char *status, const char *reason)
{
  if (idx < 0 || idx > 3)
    return;
  // "jangan ada penolakan": status "rejected" TIDAK dipublish ke Firebase — diperlakukan sebagai
  // kembali "idle" (dosis batal tanpa penolakan). Alasan asli tetap dilog ke Serial untuk debug.
  const char *pub = (strcmp(status, "rejected") == 0) ? "idle" : status;
  doseDesiredStatus[idx] = pub;
  doseDesiredReason[idx] = reason ? reason : "";
  doseStatusDirty[idx] = true;
  Serial.printf("[dose] %s -> status=%s%s%s%s\n", actName[idx], pub,
                (strcmp(status, "rejected") == 0) ? " (ditolak, tak dipublish)" : "",
                (reason && *reason) ? " reason=" : "", (reason && *reason) ? reason : "");
}

// Pushes any dirty dose status to Firebase once the link is up. Called every loop() and on
// each reconnect. Writes into /control (which stream_client watches), so status/reason echo
// back as incoming events; the app-dosing branch in processFirebaseData() ignores them.
void flushDoseStatus()
{
  if (!fbInit || !app.ready())
    return;
  for (int i = 0; i < 4; i++)
  {
    if (!doseStatusDirty[i])
      continue;
    String base = String(PATH_CONTROL) + "/" + actName[i];
    Database.set<string_t>(async_client, base + "/status", string_t(doseDesiredStatus[i].c_str()), processFirebaseData, "doseStatus");
    // reason TIDAK ditulis ke Firebase (permintaan: jangan ada reason). Tetap dilog ke Serial.
    doseStatusDirty[i] = false;
  }
}

void actUpdate()
{
  uint32_t now = millis();
  for (int i = 0; i < ACT_COUNT; i++)
    if (pulseActive[i] && now - pulseStart[i] >= pulseDur[i])
    {
      pulseActive[i] = false;
      actSet(i, false);
      if (calibRunning && i == calibIdx)
      {
        calibRunning = false;
        calibAwaitingResult = true;
        Serial.printf("[calib] %s selesai — ukur volume, lalu kirim measuredMl (mL)\n", actName[calibIdx]);
        if (calibViaLcd)
          ESP_LCD.send("calib_pump_result", "data", String("done")); // LCD: minta input volume
        else
          sendNotification("Kalibrasi Selesai", "Ukur volume yang keluar, lalu kirim hasilnya (mL).");
      }
      // App-commanded dose ran its full duration — this is the only path that reports success.
      if (i <= 3 && doseFromApp[i])
      {
        doseFromApp[i] = false;
        setDoseStatus(i, "success", "");
      }
    }
}

// True while any actuator is mid-pulse. Anything that blocks the main loop long enough to delay
// actUpdate() (WiFi scan/connect) must consult this first — a stalled actUpdate() means the pump
// stays energized past its calibrated duration.
bool anyPulseActive()
{
  for (int i = 0; i < ACT_COUNT; i++)
    if (pulseActive[i])
      return true;
  return false;
}

int actIndexFromName(const String &n)
{
  if (n == "phUp")
    return 0;
  if (n == "phDown")
    return 1;
  if (n == "nutrisiA")
    return 2;
  if (n == "nutrisiB")
    return 3;
  if (n == "waterPump")
    return 4;
  return -1;
}

// ============================================================================
//  Initial state sync — called once after stream is ready
// ============================================================================
void syncInitialState()
{
  // Actuators are physically OFF on boot — report that to /status (reported state).
  // Also defensively clear any stale /control command left from a previous session
  // (e.g. a crash mid-dose). Safe now: the full-node sync in processFirebaseData()
  // intentionally never replays actuator commands, so this can't race into an
  // on-boot dose the way it could when /status and /control were the same node.
  // Mode is intentionally NOT reset: AUTO persists across reboot.
  // SSE stream will deliver the saved mode right after this.
  for (int i = 0; i < ACT_COUNT; i++)
    Database.set<bool>(async_client, String(PATH_STATUS) + "/" + actName[i],
                       false, processFirebaseData, "initSync");

  // Dosing pumps take the object form control/<pump>/{start,volume,status,reason}, so clear
  // ONLY the trigger — a set<bool> on the pump node itself would replace the whole object and
  // destroy volume/status. Any dose that was mid-pulse when the device reset died with it, so
  // retire a stale "running" here too: nothing else would ever move it off that value.
  for (int i = 0; i < 4; i++)
  {
    Database.set<bool>(async_client, String(PATH_CONTROL) + "/" + actName[i] + "/start",
                       false, processFirebaseData, "initSync");
    Database.remove(async_client, String(PATH_CONTROL) + "/" + actName[i] + "/reason", processFirebaseData, "initSync"); // hapus reason lama
    setDoseStatus(i, "idle", "");
  }

  // waterPump is still a plain bool command (sustained on/off, no volume).
  Database.set<bool>(async_client, String(PATH_CONTROL) + "/waterPump",
                     false, processFirebaseData, "initSync");

  Serial.println("Firebase: actuator status synced to OFF, stale control commands cleared");
}

// ============================================================================
//  Command handlers (from Firebase stream)
// ============================================================================
// Convention: mode == true -> AUTO,  mode == false -> MANUAL.
void applyMode(bool autoOn)
{
  Mode newMode = autoOn ? AUTO : MANUAL;
  if (newMode == mode)
    return; // no change — skip sync

  if (newMode == MANUAL)
  {
    for (int i = 0; i < ACT_COUNT; i++)
    {
      pulseActive[i] = false;
      actSet(i, false);
      // Killing the pulse here skips actUpdate(), so an app-commanded dose caught mid-run
      // would never get its terminal status and the app would wait on "running" forever.
      if (i <= 3 && doseFromApp[i])
      {
        doseFromApp[i] = false;
        setDoseStatus(i, "rejected", "dibatalkan saat ganti mode");
      }
    }
    pumpOn = false;
  }
  else
  {
    // Any dosing pump left ON by the manual toggle must stop — AUTO owns dosing from here.
    // (Before the toggle change these were momentary pulses that ended on their own.)
    for (int i = 0; i < 4; i++)
    {
      pulseActive[i] = false;
      actSet(i, false);
    }
    // Calibration requires MANUAL mode (see startCalibration) — if the app flips to AUTO
    // mid-test, cancel it rather than let auto-dosing race the calibration pulse.
    if (calibRunning || calibAwaitingResult)
    {
      Serial.println("[calib] dibatalkan — mode berubah ke AUTO saat kalibrasi berlangsung");
      if (calibRunning && calibIdx >= 0)
      {
        pulseActive[calibIdx] = false;
        actSet(calibIdx, false);
      }
      calibRunning = false;
      calibAwaitingResult = false;
      calibIdx = -1;
      if (calibViaLcd)
        ESP_LCD.send("calib_pump_result", "data", String("err;dibatalkan (mode AUTO)"));
      calibViaLcd = false;
    }
    pumpCycleStart = millis();
  }
  mode = newMode;
  Serial.printf("[control] mode -> %s\n", mode == MANUAL ? "manual" : "auto");

  // Sync ke Firebase
  if (fbInit && app.ready())
    Database.set<bool>(async_client, String(PATH_CONTROL) + "/mode", autoOn, processFirebaseData, "modeFb");

  // Sync ke LCD agar mode-nya tidak desync
  ESP_LCD.send("mode", "value", autoOn);
}

void applyManual(const String &name, bool on)
{
  if (mode != MANUAL)
    return; // manual commands ignored in auto
  int idx = actIndexFromName(name);
  if (idx < 0)
    return;

  if (idx <= 3) // phUp/phDown/nutrisiA/nutrisiB — peristaltic dosing pumps
  {
    // LCD buttons are now a SUSTAINED TOGGLE (like waterPump), not a one-shot dose: ON keeps the
    // pump running until toggled OFF. Operator controls the volume by how long it stays on.
    if (calibIdx == idx && (calibRunning || calibAwaitingResult))
    {
      Serial.printf("[control] %s ditolak — sedang dalam proses kalibrasi\n", name.c_str());
      return;
    }
    if (on)
    {
      actSet(idx, true); // sustained ON — no pulse timer, no cfg_dosingMs limit
      Serial.printf("[control] %s -> ON (saklar manual)\n", name.c_str());
    }
    else
    {
      pulseActive[idx] = false; // defensive: clear any stray pulse flag
      actSet(idx, false);
      // Manual run ended — arm the AUTO cooldown from now so AUTO won't dose immediately after a
      // mode switch (cooldown only matters in AUTO; see controlAuto/canDosePH).
      if (idx <= 1)
        lastPhDose[idx] = millis();
      else
        lastNutrientDose = millis();
      // If an app-commanded dose was somehow mid-run, report the abort (actUpdate() can't once the
      // pulse flag is cleared) so the app UI doesn't sit on "running" forever.
      if (doseFromApp[idx])
      {
        doseFromApp[idx] = false;
        setDoseStatus(idx, "rejected", "dihentikan sebelum selesai");
      }
      Serial.printf("[control] %s -> OFF (saklar manual)\n", name.c_str());
    }
    return;
  }

  // waterPump — sustained on/off (continuous circulation, not dosed)
  actSet(idx, on);
  Serial.printf("[control] %s -> %s\n", name.c_str(), on ? "ON" : "OFF");
}

// ============================================================================
//  App Dosing — control/<pump>/{start, volume, status, reason}
//  Flow: app writes volume=<mL> then start=true
//        -> device runs that pump for volume / cfg_pumpFlowMlMin[idx] minutes
//        -> status "running" while the pulse is live, "success" once actUpdate() ends it
//        -> any refusal or early abort writes "rejected" + a reason instead
//  Unlike the LCD buttons (which dose the global cfg_dosingMl via applyManual), each command
//  here carries its own volume. AUTO dosing still uses cfg_dosingMs[] — untouched.
// ============================================================================
void startAppDose(int idx)
{
  // Self-clearing trigger: ack immediately so the next true edge is always detectable,
  // even if validation below rejects this request. Mirrors startCalibration().
  if (idx >= 0 && idx <= 3 && fbInit && app.ready())
    Database.set<bool>(async_client, String(PATH_CONTROL) + "/" + actName[idx] + "/start",
                       false, processFirebaseData, "doseStartAck");

  if (idx < 0 || idx > 3)
    return; // only the 4 peristaltic dosing pumps are volume-dosed

  // Permintaan: dosis dari app SELALU bisa jalan. Guard kebijakan (mode AUTO & air rendah) DIHAPUS.
  // Yang tersisa hanya batas fisik/matematis: pompa sedang sibuk (kalibrasi / masih memompa) tak
  // bisa mulai pulsa kedua, dan volume/flow harus valid supaya durasi bisa dihitung.
  if (calibIdx == idx && (calibRunning || calibAwaitingResult))
  {
    setDoseStatus(idx, "rejected", "sedang kalibrasi");
    return;
  }
  if (pulseActive[idx])
  {
    setDoseStatus(idx, "rejected", "dosis masih berjalan");
    return;
  }

  float volumeMl = doseVolumeSel[idx];
  if (volumeMl <= 0)
  {
    setDoseStatus(idx, "rejected", "volume tidak valid");
    return;
  }
  if (cfg_pumpFlowMlMin[idx] <= 0)
  {
    setDoseStatus(idx, "rejected", "flow rate belum dikalibrasi");
    return;
  }

  uint32_t ms = (uint32_t)(volumeMl / cfg_pumpFlowMlMin[idx] * 60000.0f);
  if (ms == 0)
  {
    setDoseStatus(idx, "rejected", "durasi terlalu singkat");
    return;
  }

  actPulse(idx, ms);
  doseFromApp[idx] = true;
  // Keep AUTO's cooldown honest — same bookkeeping applyManual() does after a manual dose.
  if (idx <= 1)
    lastPhDose[idx] = millis();
  else
    lastNutrientDose = millis();

  setDoseStatus(idx, "running", "");
  Serial.printf("[dose] %s -> %.1fmL / %.1fmL/min = %lums\n",
                actName[idx], volumeMl, cfg_pumpFlowMlMin[idx], ms);
}

bool isValidDataType(int type)
{
  // realtime_database_data_type: 1=integer 2=float 3=double 4=boolean 5=string 6=json
  // (excludes -1=undefined, 0=null, 7=array)
  return type >= 1 && type <= 6;
}

void calcDosingMs(int i)
{
  if (cfg_pumpFlowMlMin[i] > 0)
    cfg_dosingMs[i] = (uint32_t)(cfg_dosingMl / cfg_pumpFlowMlMin[i] * 60000.0f);
  Serial.printf("[setting] dosing %s: %.1fmL / %.1fmL/min = %lums\n",
                actName[i], cfg_dosingMl, cfg_pumpFlowMlMin[i], cfg_dosingMs[i]);
}

// Applies a per-pump calibrated flow rate loaded from /setting/pumpCalibration/<pumpName>.
void applyCalibSetting(const String &pumpName, const String &val)
{
  int idx = actIndexFromName(pumpName);
  if (idx < 0 || idx > 3)
    return; // only the 4 peristaltic dosing pumps are calibrated (waterPump excluded)
  cfg_pumpFlowMlMin[idx] = atof(val.c_str());
  calcDosingMs(idx);
  Serial.printf("[setting] pumpCalibration/%s -> %.2f mL/min\n", pumpName.c_str(), cfg_pumpFlowMlMin[idx]);
}

// Applies a per-sensor debug toggle loaded from /setting/debugSensor/<sensor>.
// true  -> readSensor() injects a dummy value for that sensor (bench testing w/o hardware).
// false -> readSensor() goes back to reading the real sensor.
void applyDebugSetting(const String &sensor, const String &val)
{
  bool on = (val == "true" || val == "1");
  if (sensor == "ph")
    cfg_debugPh = on;
  else if (sensor == "tds")
    cfg_debugTds = on;
  else if (sensor == "aht")
    cfg_debugAht = on;
  else if (sensor == "ds18b20")
    cfg_debugDs18b20 = on;
  else if (sensor == "ultrasonic")
    cfg_debugUltrasonic = on;
  else
    return;
  Serial.printf("[debug] %s -> %s\n", sensor.c_str(), on ? "DUMMY" : "real");
}

// Sends the subset of sensor limits that actually exist as Firebase settings — type
// "update_sensorLimit", dataName "array_sensorLimit": "phMin;phMax;ppmMin;ppmMax;waterLowPct".
// tAir/tWater/humidity ranges have no server-side setting (yet), so the LCD keeps using its
// own local constants for those — only ph/ppm/waterLow are synced from here.
void sendSensorLimitToLCD()
{
  char buf[64];
  snprintf(buf, sizeof(buf), "%.1f;%.1f;%d;%d;%.0f",
           cfg_phMin, cfg_phMax, cfg_ppmMin, cfg_ppmMax, cfg_waterLowPct);
  ESP_LCD.send("update_sensorLimit", "array_sensorLimit", String(buf));
}

// Sends display units to LCD — type "sensor_unit", dataName "array_sensorUnit":
// "phUnit;tAirUnit;tWaterUnit;waterLvlUnit;ppmUnit;humidityUnit". Only ppmUnit/tempUnit are
// actually configurable (Firebase settings); the rest are fixed but still sent for completeness.
void sendSensorUnitToLCD()
{
  String out = "pH;" + cfg_tempUnit + ";" + cfg_tempUnit + ";%;" + cfg_ppmUnit + ";%";
  ESP_LCD.send("sensor_unit", "array_sensorUnit", out);
}

void applySetting(const String &key, const String &val)
{
  bool recalc = false;
  bool limitChanged = false;
  bool unitChanged = false;
  if (key == "phMin")
  {
    cfg_phMin = atof(val.c_str());
    limitChanged = true;
  }
  else if (key == "phMax")
  {
    cfg_phMax = atof(val.c_str());
    limitChanged = true;
  }
  else if (key == "ppmMin")
  {
    cfg_ppmMin = atoi(val.c_str());
    limitChanged = true;
  }
  else if (key == "ppmMax")
  {
    cfg_ppmMax = atoi(val.c_str());
    limitChanged = true;
  }
  else if (key == "intervalUpdate")
    cfg_dataPushMs = max((uint32_t)atoi(val.c_str()) * 1000UL, 5000UL); // min 5s — prevent write flood
  else if (key == "dosingMl")
  {
    cfg_dosingMl = atof(val.c_str());
    recalc = true;
  }
  else if (key == "mixingS")
    cfg_mixingMs = (uint32_t)atoi(val.c_str()) * 1000UL;
  else if (key == "pumpOnHour")
    cfg_pumpOnHr = (uint32_t)atoi(val.c_str());
  else if (key == "pumpOffHour")
    cfg_pumpOffHr = (uint32_t)atoi(val.c_str());
  else if (key == "tankDepthCm")
    cfg_tankDepthCm = atof(val.c_str());
  else if (key == "waterLowPct")
  {
    cfg_waterLowPct = atof(val.c_str());
    limitChanged = true;
  }
  else if (key == "waterGuard")
    cfg_waterGuard = (val == "true" || val == "1");
  else if (key == "phSlope")
    cfg_phSlope = atof(val.c_str());
  else if (key == "phOffset")
    cfg_phOffset = atof(val.c_str());
  else if (key == "tdsFactor")
    cfg_tdsFactor = atof(val.c_str());
  else if (key == "ppmUnit")
  {
    cfg_ppmUnit = val;
    unitChanged = true;
  }
  else if (key == "tempUnit")
  {
    cfg_tempUnit = val;
    unitChanged = true;
  }
  else
    return;
  Serial.printf("[setting] %s -> %s\n", key.c_str(), val.c_str());
  if (recalc)
    for (int i = 0; i < 4; i++)
      calcDosingMs(i); // dosingMl changed — every pump's pulse duration depends on it
  if (limitChanged)
    sendSensorLimitToLCD(); // onChanged — keep LCD's ph/ppm/waterLow ranges in sync
  if (unitChanged)
    sendSensorUnitToLCD(); // onChanged — keep LCD's unit labels in sync
}

void applySettings(JsonDocument &doc)
{
  for (JsonPair kv : doc.as<JsonObject>())
  {
    // /setting/pumpCalibration/{phUp,phDown,nutrisiA,nutrisiB} — nested per-pump calibration results
    if (String(kv.key().c_str()) == "pumpCalibration" && kv.value().is<JsonObject>())
    {
      for (JsonPair pk : kv.value().as<JsonObject>())
        applyCalibSetting(pk.key().c_str(), pk.value().as<String>());
      continue;
    }
    // /setting/debugSensor/{ph,aht,ds18b20,ultrasonic} — per-sensor dummy-data toggles
    if (String(kv.key().c_str()) == "debugSensor" && kv.value().is<JsonObject>())
    {
      for (JsonPair pk : kv.value().as<JsonObject>())
        applyDebugSetting(pk.key().c_str(), pk.value().as<String>());
      continue;
    }
    applySetting(kv.key().c_str(), kv.value().as<String>());
  }
  Serial.printf("[setting] loaded ph:%.1f-%.1f ppm:%d-%d interval:%lus mixing:%lus pump:%lu/%luhr\n",
                cfg_phMin, cfg_phMax, cfg_ppmMin, cfg_ppmMax,
                cfg_dataPushMs / 1000UL, cfg_mixingMs / 1000UL,
                cfg_pumpOnHr, cfg_pumpOffHr);
  sendSensorLimitToLCD(); // onBoot — full /setting sync just loaded, push current limits to LCD
  sendSensorUnitToLCD();  // onBoot — same, push current display units to LCD
  settingsLoaded = true;  // config now current — gates the control stream start in loop()
}

// ============================================================================
//  Settings persistence (NVS via Preferences)
//  Mirrors every cfg_ value that comes from /setting so an OFFLINE boot (no WiFi/Firebase)
//  restores the LAST known config — sensor slope/offset, limits, dosing, units — instead of
//  the compile-time defaults. saveSettingsToNVS() runs whenever a /setting payload is applied;
//  loadSettingsFromNVS() runs once at boot, before WiFi. Debug toggles are intentionally NOT
//  persisted (a device must never boot stuck in dummy-sensor mode). cfg_dosingMs is derived,
//  not stored — recomputed from dosingMl + flow after load.
// ============================================================================
Preferences settingPrefs;

void saveSettingsToNVS()
{
  settingPrefs.begin("setting", false); // read-write
  settingPrefs.putFloat("phMin", cfg_phMin);
  settingPrefs.putFloat("phMax", cfg_phMax);
  settingPrefs.putInt("ppmMin", cfg_ppmMin);
  settingPrefs.putInt("ppmMax", cfg_ppmMax);
  settingPrefs.putUInt("dataPushMs", cfg_dataPushMs);
  settingPrefs.putFloat("dosingMl", cfg_dosingMl);
  for (int i = 0; i < 4; i++)
    settingPrefs.putFloat((String("flow") + i).c_str(), cfg_pumpFlowMlMin[i]);
  settingPrefs.putUInt("mixingMs", cfg_mixingMs);
  settingPrefs.putUInt("pumpOnHr", cfg_pumpOnHr);
  settingPrefs.putUInt("pumpOffHr", cfg_pumpOffHr);
  settingPrefs.putFloat("tankDepthCm", cfg_tankDepthCm);
  settingPrefs.putFloat("waterLowPct", cfg_waterLowPct);
  settingPrefs.putBool("waterGuard", cfg_waterGuard);
  settingPrefs.putString("ppmUnit", cfg_ppmUnit);
  settingPrefs.putString("tempUnit", cfg_tempUnit);
  settingPrefs.putFloat("phSlope", cfg_phSlope);
  settingPrefs.putFloat("phOffset", cfg_phOffset);
  settingPrefs.putFloat("tdsFactor", cfg_tdsFactor);
  settingPrefs.end();
  Serial.println("[setting] saved to NVS");
}

void loadSettingsFromNVS()
{
  settingPrefs.begin("setting", true); // read-only
  // Each getX() falls back to the current cfg_ value, so a missing key keeps the compile-time
  // default and a first-ever boot (empty NVS) behaves exactly as before this feature existed.
  cfg_phMin = settingPrefs.getFloat("phMin", cfg_phMin);
  cfg_phMax = settingPrefs.getFloat("phMax", cfg_phMax);
  cfg_ppmMin = settingPrefs.getInt("ppmMin", cfg_ppmMin);
  cfg_ppmMax = settingPrefs.getInt("ppmMax", cfg_ppmMax);
  cfg_dataPushMs = settingPrefs.getUInt("dataPushMs", cfg_dataPushMs);
  cfg_dosingMl = settingPrefs.getFloat("dosingMl", cfg_dosingMl);
  for (int i = 0; i < 4; i++)
    cfg_pumpFlowMlMin[i] = settingPrefs.getFloat((String("flow") + i).c_str(), cfg_pumpFlowMlMin[i]);
  cfg_mixingMs = settingPrefs.getUInt("mixingMs", cfg_mixingMs);
  cfg_pumpOnHr = settingPrefs.getUInt("pumpOnHr", cfg_pumpOnHr);
  cfg_pumpOffHr = settingPrefs.getUInt("pumpOffHr", cfg_pumpOffHr);
  cfg_tankDepthCm = settingPrefs.getFloat("tankDepthCm", cfg_tankDepthCm);
  cfg_waterLowPct = settingPrefs.getFloat("waterLowPct", cfg_waterLowPct);
  cfg_waterGuard = settingPrefs.getBool("waterGuard", cfg_waterGuard);
  cfg_ppmUnit = settingPrefs.getString("ppmUnit", cfg_ppmUnit);
  cfg_tempUnit = settingPrefs.getString("tempUnit", cfg_tempUnit);
  cfg_phSlope = settingPrefs.getFloat("phSlope", cfg_phSlope);
  cfg_phOffset = settingPrefs.getFloat("phOffset", cfg_phOffset);
  cfg_tdsFactor = settingPrefs.getFloat("tdsFactor", cfg_tdsFactor);
  settingPrefs.end();

  for (int i = 0; i < 4; i++)
    calcDosingMs(i); // derived from dosingMl + flow — recompute after load

  Serial.printf("[setting] loaded from NVS ph:%.1f-%.1f ppm:%d-%d phCal:%.3f/%.3f tds:%.3f\n",
                cfg_phMin, cfg_phMax, cfg_ppmMin, cfg_ppmMax,
                cfg_phSlope, cfg_phOffset, cfg_tdsFactor);
}

// ============================================================================
//  Sensor calibration capture — calibration/{command,sensor,rawData}
//  The app flips calibration/command "idle" -> "start". We detect it by POLLING (no 4th SSL
//  stream client — heap is already tight with 3), then push CALIB_SAMPLE_COUNT sensor samples
//  one per second to calibration/rawData/data0..N, updating progress + currentData as we go.
//  When done we set command back to "stop". Which reading is sampled follows the `sensor`
//  field: "pH" -> data.pH, anything else -> data.ppm. Averaging/offset is left to the app.
// ============================================================================
#define CALIB_SAMPLE_COUNT 31
bool calibCaptureRunning = false;
bool calibStarting = false; // between "start" detected and sensor fetched — blocks re-trigger
int calibSampleIdx = 0;
uint32_t calibSampleLastMs = 0;
uint32_t calibPollLastMs = 0;
String calibSensorKind = "PPM";

void startCalibrationCapture(const String &sensorKind)
{
  calibStarting = false;
  if (calibCaptureRunning)
    return;
  calibSensorKind = sensorKind.length() ? sensorKind : "PPM";
  calibCaptureRunning = true;
  calibSampleIdx = 0;
  calibSampleLastMs = 0; // 0 => first sample fires immediately in updateCalibrationCapture()
  Serial.printf("[calibcap] start — sensor=%s, mengumpulkan %d sampel @1s\n",
                calibSensorKind.c_str(), CALIB_SAMPLE_COUNT);
  if (fbInit && app.ready())
    Database.set<int>(async_client, String(PATH_CALIBRATION) + "/progress", 0, processFirebaseData, "calibCap");
}

// Non-blocking stepper: one sample per second, called every loop() while a capture is running.
void updateCalibrationCapture()
{
  if (!calibCaptureRunning)
    return;
  uint32_t now = millis();
  if (calibSampleLastMs != 0 && now - calibSampleLastMs < 1000UL)
    return; // not yet time for the next sample
  calibSampleLastMs = now;

  // Sample the reading this calibration targets; data.pH/data.ppm are kept fresh by readSensor().
  bool isPh = (calibSensorKind == "pH" || calibSensorKind == "PH" || calibSensorKind == "ph");
  float sample = isPh ? data.pH : (float)data.ppm;
  int prec = isPh ? 2 : 0;

  if (fbInit && app.ready())
  {
    String base = String(PATH_CALIBRATION);
    Database.set<number_t>(async_client, base + "/rawData/data" + calibSampleIdx,
                           number_t(sample, prec), processFirebaseData, "calibCap");
    Database.set<number_t>(async_client, base + "/currentData", number_t(sample, prec), processFirebaseData, "calibCap");
    Database.set<int>(async_client, base + "/progress", calibSampleIdx + 1, processFirebaseData, "calibCap");
  }
  Serial.printf("[calibcap] data%d = %.*f\n", calibSampleIdx, prec, sample);

  calibSampleIdx++;
  if (calibSampleIdx >= CALIB_SAMPLE_COUNT)
  {
    calibCaptureRunning = false;
    if (fbInit && app.ready())
      Database.set<string_t>(async_client, String(PATH_CALIBRATION) + "/command",
                             string_t("stop"), processFirebaseData, "calibCap");
    Serial.printf("[calibcap] selesai — %d sampel terkirim, command -> stop\n", CALIB_SAMPLE_COUNT);
  }
}

// ============================================================================
//  Firebase callback (auth events, stream commands, write acks)
// ============================================================================
void processFirebaseData(AsyncResult &r)
{
  if (!r.isResult())
    return;

  if (r.isEvent())
  {
    Firebase.printf("[FB event] %s: %s (%d)\n", r.uid().c_str(),
                    r.eventLog().message().c_str(), r.eventLog().code());
    return;
  }
  if (r.isError())
  {
    Firebase.printf("[FB error] %s: %s (%d)\n", r.uid().c_str(),
                    r.error().message().c_str(), r.error().code());
    return;
  }
  if (!r.available())
    return;

  RealtimeDatabaseResult &rtdb = r.to<RealtimeDatabaseResult>();

  // One-shot polls of the calibration node — GET results are NOT streams, so they must be
  // handled before the isStream guard below (which drops non-stream results as write acks).
  // Two-step so the 3s poll only ever downloads the tiny `command` string, never rawData:
  // poll command -> on "start", fetch `sensor` -> start the capture with it.
  if (r.uid() == "calibCmdPoll")
  {
    if (rtdb.to<String>() == "start" && !calibCaptureRunning && !calibStarting)
    {
      calibStarting = true;
      Database.get(async_client, String(PATH_CALIBRATION) + "/sensor", processFirebaseData, false, "calibSensor");
    }
    return;
  }
  if (r.uid() == "calibSensor")
  {
    startCalibrationCapture(rtdb.to<String>());
    return;
  }

  if (!rtdb.isStream())
    return; // write ack — nothing to do
  if (!isValidDataType(rtdb.type()))
    return;

  String uid = r.uid();
  String node = rtdb.dataPath();
  if (node.startsWith("/"))
    node = node.substring(1);

  // ── Setting stream ──────────────────────────────────────────────────────────
  if (uid == "settingStream")
  {
    bool persist = true;
    if (node.length() == 0)
    {
      JsonDocument doc;
      if (!deserializeJson(doc, rtdb.to<String>()))
        applySettings(doc);
    }
    else if (node.startsWith("pumpCalibration/"))
    {
      applyCalibSetting(node.substring(16), rtdb.to<String>());
    }
    else if (node.startsWith("debugSensor/"))
    {
      applyDebugSetting(node.substring(12), rtdb.to<String>());
      persist = false; // debug toggles must not survive a reboot — never persist them
    }
    else
    {
      applySetting(node, rtdb.to<String>());
    }
    if (persist)
      saveSettingsToNVS(); // mirror the new config to NVS for the next offline boot
    return;
  }

  // ── Control stream ──────────────────────────────────────────────────────────
  // Initial full-node sync (dataPath "/"): parse the whole /control object.
  if (node.length() == 0)
  {
    JsonDocument doc;
    if (!deserializeJson(doc, rtdb.to<String>()))
    {
      if (doc["mode"].is<bool>())
        applyMode(doc["mode"].as<bool>());
      // Per-actuator booleans (phUp/phDown/nutrisiA/nutrisiB/waterPump) are intentionally
      // NOT replayed from this initial snapshot — same reasoning as the calibration
      // triggers below: a stale "true" left in /control (crash mid-dose, or a race with
      // syncInitialState()'s reset writes) must never auto-fire a dose on reconnect.
      // Live per-child changes further down still apply normally.

      // Calibration REFERENCE values are the exception, and must be replayed. They are inert data
      // (nothing fires on receipt) and they live only in RAM, so without this they silently revert
      // to their compile-time defaults on every reboot while Firebase still holds the real values.
      // The failure is invisible: the app sets refPpm=1000, the device reboots, the app writes
      // capture=true, and the device calibrates against the default 500 — saving a tdsFactor that
      // is off by 2x with no error anywhere. start/capture/measuredMl remain un-replayed.
      JsonVariant cal = doc["calibrate"];
      if (cal.is<JsonObject>())
      {
        JsonVariant refPpm = cal["tds"]["refPpm"];
        if (refPpm.is<float>())
          calibTdsRef = refPpm.as<float>();
        for (int i = 0; i < 4; i++)
        {
          JsonVariant durS = cal[actName[i]]["durationS"];
          if (durS.is<float>()) // is<float>() also matches integers in ArduinoJson
            calibDurationSel[i] = (uint32_t)durS.as<int>();
        }
        Serial.printf("[calib] refs restored — tdsRef %.0f durS %lu/%lu/%lu/%lu\n",
                      calibTdsRef, calibDurationSel[0],
                      calibDurationSel[1], calibDurationSel[2], calibDurationSel[3]);
      }

      // Dose volumes are replayed for exactly the same reason as the calibration refs: inert
      // data, RAM-only, and silently reverting to 0 on reboot would make the first dose after
      // a restart fail with "volume tidak valid" until the app happened to rewrite it.
      // `start` is NOT replayed — a stale trigger must never auto-fire a dose on reconnect.
      for (int i = 0; i < 4; i++)
      {
        JsonVariant vol = doc[actName[i]]["volume"];
        if (vol.is<float>())
          doseVolumeSel[i] = vol.as<float>();
      }
    }
    return;
  }

  // Per-child change
  if (node == "mode")
  {
    applyMode(rtdb.to<bool>());
    return;
  }

  // ── Calibration triggers — /control/calibrate/... ─────────────────────────────────────────────
  // NOTE: intentionally NOT handled in the full-node sync above, so any stale trigger left over
  // in Firebase from a previous session can never auto-fire on reconnect.
  if (node.startsWith("calibrate/"))
  {
    String rest = node.substring(10); // after "calibrate/"

    // pH is no longer calibrated on-device — cfg_phSlope/cfg_phOffset are read directly from
    // /setting (see applySetting). Any leftover calibrate/phSensor/* nodes are ignored.

    // ── TDS sensor 1-point calibration — calibrate/tds/{refPpm,capture} ──
    if (rest.startsWith("tds/"))
    {
      String field = rest.substring(4);
      if (field == "refPpm")
        calibTdsRef = rtdb.to<float>();
      else if (field == "capture" && rtdb.to<bool>())
        captureTdsCalib();
      return;
    }

    // ── Pump calibration — /control/calibrate/<pump>/{durationS,start,measuredMl} ──────────────
    // One independent node per pump (phUp/phDown/nutrisiA/nutrisiB), each with its own duration,
    // start trigger, and result field.
    int slash = rest.indexOf('/');
    if (slash < 0)
      return; // malformed path — ignore
    int idx = actIndexFromName(rest.substring(0, slash));
    if (idx < 0 || idx > 3)
      return; // only the 4 peristaltic dosing pumps are calibratable
    String field = rest.substring(slash + 1);

    if (field == "durationS")
      calibDurationSel[idx] = (uint32_t)rtdb.to<int>();
    else if (field == "start")
    {
      if (rtdb.to<bool>())
        startCalibration(idx);
    }
    else if (field == "measuredMl")
      submitCalibrationResult(idx, rtdb.to<float>());
    return;
  }

  // ── App dosing — /control/<pump>/{start,volume,status,reason} ────────────────────────────────
  // Dosing pumps (0-3) take the object form; waterPump stays a plain bool and falls through
  // to applyManual() below.
  {
    int slash = node.indexOf('/');
    int idx = actIndexFromName(slash < 0 ? node : node.substring(0, slash));
    if (idx >= 0 && idx <= 3)
    {
      if (slash < 0)
      {
        // Whole-node write: the app set() the object or update()d several fields at once, so
        // the stream delivers one event on the pump node instead of one per child. Apply
        // volume BEFORE start — read in the wrong order, the dose fires against whatever
        // volume the PREVIOUS command left behind.
        if (rtdb.type() != 6)
        {
          applyManual(node, rtdb.to<bool>()); // legacy bool command — keep working
          return;
        }
        JsonDocument doc;
        if (!deserializeJson(doc, rtdb.to<String>()))
        {
          if (doc["volume"].is<float>()) // is<float>() also matches integers in ArduinoJson
            doseVolumeSel[idx] = doc["volume"].as<float>();
          if (doc["start"].is<bool>() && doc["start"].as<bool>())
            startAppDose(idx);
        }
        return;
      }

      String field = node.substring(slash + 1);
      if (field == "volume")
        doseVolumeSel[idx] = rtdb.to<float>();
      else if (field == "start" && rtdb.to<bool>())
        startAppDose(idx);
      // status/reason are device-owned. They live under /control (which this stream watches),
      // so our own setDoseStatus() writes echo straight back here — ignoring them is what
      // keeps that from looping. Neither is a trigger, so there is nothing to act on.
      return;
    }
  }

  applyManual(node, rtdb.to<bool>());
}

// ============================================================================
//  Sensors — ONE function, all four sensors
// ============================================================================
// ============================================================================
//  Ultrasonic water level — HC-SR04 (trig/echo)
//  Non-blocking + noise-hardened: baca tiap US_READ_INTERVAL_MS, ambil US_SAMPLES ping (1 ping per
//  loop, jeda US_SAMPLE_GAP_MS), buang di luar rentang wajar, pakai MEDIAN. Nilai lama dipertahankan
//  saat gagal; "untrusted" via ultrasonicOK (dicek interlock controlAuto).
// ============================================================================
#define US_READ_INTERVAL_MS 2000UL // jeda antar burst (level air lambat)
#define US_SAMPLES 5               // ping per burst (median)
#define US_SAMPLE_GAP_MS 40UL      // jeda antar ping (echo settle)
#define US_MIN_CM 2.0f             // dead-zone
#define US_MAX_MARGIN_CM 5.0f      // toleransi di atas tankDepth (echo palsu > ini dibuang)

bool usSampling = false;
int usCount = 0;
float usSamples[US_SAMPLES];
uint32_t usLastSampleMs = 0;
uint32_t usLastReadMs = 0;

// Satu ping; kembalikan jarak (cm) bila valid & dalam rentang, atau -1 bila invalid.
float ultrasonicPingCm()
{
  int echoIdle = digitalRead(PIN_ECHO); // level ECHO sebelum trig — untuk diagnosa
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);
  // DIAGNOSTIK sementara — hapus setelah ultrasonik beres.
  if (dur <= 0)
  {
    Serial.printf("[us] ping: TIMEOUT (echo idle=%s). idle=LOW -> tak ada sinyal (power5V/kabel ECHO/GND); "
                  "idle=HIGH -> ECHO nyangkut HIGH (salah kabel/short/level)\n",
                  echoIdle ? "HIGH" : "LOW");
    return -1.0f; // timeout
  }
  float d = dur * 0.034f / 2.0f;
  if (d < US_MIN_CM || d > cfg_tankDepthCm + US_MAX_MARGIN_CM)
  {
    Serial.printf("[us] ping: dur=%ldus dist=%.1fcm DI LUAR rentang %.0f..%.0fcm\n",
                  dur, d, US_MIN_CM, cfg_tankDepthCm + US_MAX_MARGIN_CM);
    return -1.0f; // di luar rentang wajar — echo palsu
  }
  Serial.printf("[us] ping: dur=%ldus dist=%.1fcm OK\n", dur, d);
  return d;
}

// Selesai burst: median sampel valid -> waterLevel; atau hitung gagal.
void ultrasonicFinishBurst()
{
  float valid[US_SAMPLES];
  int n = 0;
  for (int i = 0; i < US_SAMPLES; i++)
    if (usSamples[i] > 0)
      valid[n++] = usSamples[i];

  static uint8_t ultrasonicFails = 0;
  if (n >= 3) // cukup sampel valid untuk dipercaya
  {
    for (int i = 0; i < n - 1; i++) // sort kecil untuk median
      for (int j = i + 1; j < n; j++)
        if (valid[j] < valid[i])
        {
          float t = valid[i];
          valid[i] = valid[j];
          valid[j] = t;
        }
    float distCm = valid[n / 2];
    ultrasonicFails = 0;
    waterLevelValid = true;
    ultrasonicOK = true;
    float heightCm = constrain(cfg_tankDepthCm - distCm, 0.0f, cfg_tankDepthCm);
    data.waterLevel = (heightCm / cfg_tankDepthCm) * 100.0f;
    Serial.printf("[us] median=%.1fcm (%d/%d valid) depth=%.1f -> level=%.0f%%\n",
                  distCm, n, US_SAMPLES, cfg_tankDepthCm, data.waterLevel);
  }
  else if (++ultrasonicFails >= 5) // ~5 burst gagal beruntun (~10 dtk) -> untrusted
  {
    ultrasonicOK = false; // controlAuto() menghentikan dosing/pompa berdasar flag ini
    ultrasonicFails = 5;  // clamp
    Serial.printf("[us] gagal — hanya %d/%d sampel valid, waterLevel dibekukan %.0f%% (nilai lama)\n",
                  n, US_SAMPLES, data.waterLevel);
  }
}

// Dipanggil tiap loop (self-throttle). Maks SATU pulseIn per pemanggilan → tak nge-blok lama.
void updateUltrasonic()
{
  if (!bootReady)
    return;
  uint32_t now = millis();

  // Mode debug: nilai dummy pada cadence yang sama.
  if (cfg_debugUltrasonic)
  {
    if (now - usLastReadMs < US_READ_INTERVAL_MS)
      return;
    usLastReadMs = now;
    data.waterLevel = 80.0f + (random(0, 2001) / 100.0f); // dummy, ~70-90%
    waterLevelValid = true;
    ultrasonicOK = true;
    return;
  }

  // tankDepth tak valid: cacat config, bukan sensor — jangan dihitung sebagai gagal sensor.
  if (cfg_tankDepthCm <= 0)
  {
    if (now - usLastReadMs < US_READ_INTERVAL_MS)
      return;
    usLastReadMs = now;
    ultrasonicOK = false;
    static uint32_t lastDepthWarn = 0;
    if (now - lastDepthWarn >= 30000UL)
    {
      lastDepthWarn = now;
      Serial.printf("[sensor] tankDepthCm tidak valid (%.1f) — waterLevel tak bisa dihitung, cek /setting\n",
                    cfg_tankDepthCm);
    }
    return;
  }

  // Mulai burst baru bila sudah waktunya.
  if (!usSampling)
  {
    if (now - usLastReadMs < US_READ_INTERVAL_MS)
      return;
    usSampling = true;
    usCount = 0;
    usLastSampleMs = 0; // ping pertama langsung
  }

  // Satu ping per US_SAMPLE_GAP_MS (jeda agar echo tak tumpang tindih).
  if (usLastSampleMs != 0 && now - usLastSampleMs < US_SAMPLE_GAP_MS)
    return;
  usLastSampleMs = now;
  usSamples[usCount++] = ultrasonicPingCm();

  if (usCount >= US_SAMPLES)
  {
    usSampling = false;
    usLastReadMs = now;
    ultrasonicFinishBurst();
  }
}

void readSensor()
{
  // pH (ADS1115 A0) and TDS (ADS1115 A1) — separate probes on the same chip, so each
  // gets its own debug toggle (one can be dummied while the other still reads real hardware).
  if (cfg_debugPh)
  {
    data.pH = 6.5f + (random(-50, 51) / 100.0f); // dummy, jitters ~6.0-7.0
  }
  else if (adsOK)
  {
    // Direct 2-point line straight from the A0 voltage: pH = slope * v0 + offset.
    // slope/offset come from /setting (phSlope/phOffset). Compute them from two buffers:
    //   slope  = (pH_b - pH_a) / (v_b - v_a)
    //   offset = pH_a - slope * v_a
    // No hardcoded probe assumptions — whatever the probe outputs, the two points define it.
    float v0 = ads.computeVolts(ads.readADC_SingleEnded(0));
    data.pH = constrain(v0 * cfg_phSlope + cfg_phOffset, 0.0f, 14.0f);
    // TEMP DEBUG — remove once pH is confirmed.
    // Serial.printf("[ph] v0=%.3fV slope=%.3f offset=%.3f -> pH=%.2f\n",
    //               v0, cfg_phSlope, cfg_phOffset, data.pH);
  }

  if (cfg_debugTds)
  {
    data.ppm = 600 + random(-50, 51); // dummy, jitters ~550-650
  }
  else if (adsOK)
  {
    float v1 = ads.computeVolts(ads.readADC_SingleEnded(1));
    data.ppm = (int)(v1 * 500.0f * cfg_tdsFactor);
  }

  // Air temp + humidity (AHT10)
  if (cfg_debugAht)
  {
    data.airTemp = 28.0f + (random(-30, 31) / 100.0f);  // dummy, jitters ~27.7-28.3
    data.humidity = 65.0f + (random(-50, 51) / 100.0f); // dummy, jitters ~64.5-65.5
  }
  else if (ahtOK)
  {
    sensors_event_t h, t;
    aht.getEvent(&h, &t);
    data.airTemp = isnan(t.temperature) ? 0 : t.temperature;
    data.humidity = isnan(h.relative_humidity) ? 0 : h.relative_humidity;
  }

  // Water temp (DS18B20) — non-blocking across the 1 s cadence:
  // read the PREVIOUS conversion (started last call, done after 750 ms),
  // then start the NEXT one. setWaitForConversion(false) => no 750 ms stall.
  if (cfg_debugDs18b20)
  {
    data.waterTemp = 25.0f + (random(-30, 31) / 100.0f); // dummy, jitters ~24.7-25.3
  }
  else if (dsOK)
  {
    if (dsStarted)
    {
      float t = ds.getTempCByIndex(0);
      if (t != DEVICE_DISCONNECTED_C)
        data.waterTemp = t;
    }
    ds.requestTemperatures();
    dsStarted = true;
  }

  // Water level (ultrasonic) DIPINDAH ke updateUltrasonic() — sampler non-blocking sendiri
  // (median-of-5 + gate rentang + cadence terpisah) karena sensornya berisik. Lihat loop().
}

// Pushes the live sensor snapshot to ESP32-LCD over UART — type "update_sensor",
// dataName "array_sensor", value "ph;tAir;tWater;waterLevel;nutrisi;humidity"
// (matches lcdBoard's parseSensor()). esp32_lcd has no Firebase connection of its
// own, so this UART push is its only source of live sensor data.
void sendSensorToLCD()
{
  char buf[80];
  snprintf(buf, sizeof(buf), "%.1f;%.1f;%.1f;%.0f;%d;%.0f",
           data.pH, data.airTemp, data.waterTemp, data.waterLevel, data.ppm, data.humidity);
  ESP_LCD.send("update_sensor", "array_sensor", String(buf));
}

// ============================================================================
//  Control (auto)
// ============================================================================
bool canDosePH(int i) { return millis() - lastPhDose[i] >= (cfg_dosingMs[i] + cfg_mixingMs); }
bool canDoseNutrient() { return millis() - lastNutrientDose >= (max(cfg_dosingMs[2], cfg_dosingMs[3]) + cfg_mixingMs); }
bool phPumpActive() { return actState[0] || actState[1]; }
bool nutrientPumpActive() { return actState[2] || actState[3]; }

void controlAuto()
{
  if (mode != AUTO)
    return;
  uint32_t now = millis();

  // Water level interlock — valid reading above threshold = tank too low, halt everything.
  // Also halts on !ultrasonicOK: after a sensor failure, data.waterLevel just freezes at its
  // last known-good value (see readSensor()), so the interlock can't rely on that number alone
  // to notice a dead sensor — it must check sensor health explicitly too.
  // cfg_waterGuard gates the whole interlock: set false in /setting to run auto WITHOUT any
  // water-level protection (escape hatch for an unreliable ultrasonic sensor). Off = pumps can
  // dry-run and dosing can over-concentrate a low tank, so only use it while the sensor is bad.
  static bool waterLowPrev = false;
  bool waterLow = cfg_waterGuard && (!ultrasonicOK || (data.waterLevel <= cfg_waterLowPct));
  if (waterLow != waterLowPrev)
  {
    waterLowPrev = waterLow;
    if (waterLow && !ultrasonicOK)
      Serial.println("[auto] WARNING: sensor ultrasonic tidak terpercaya — dosing & pompa dihentikan");
    else if (waterLow)
      Serial.printf("[auto] WARNING: water low (%.0f%% <= %.0f%%) — dosing & pump halted\n",
                    data.waterLevel, cfg_waterLowPct);
    else
    {
      // Restart the on/off cycle from now. The cycle is phase-based on (now - pumpCycleStart), and
      // the clock kept running through the halt — resuming without this drops the pump into an
      // arbitrary point of its cycle (possibly straight into a partial ON window).
      pumpCycleStart = millis();
      Serial.println("[auto] water level restored — resuming auto, pump cycle restarted");
    }
  }
  if (waterLow)
  {
    if (pumpOn)
    {
      pumpOn = false;
      actSet(4, false);
    }
    return;
  }

  // Sensor sanity — implausible readings mean disconnected/faulty sensor
  bool phOk = (data.pH > 1.0f && data.pH < 13.0f);
  bool ppmOk = (data.ppm > 10);

  // pH — skip if nutrient pumps are still running (interlock)
  if (phOk && !nutrientPumpActive())
  {
    if (data.pH < cfg_phMin && !actState[1] && canDosePH(0))
    {
      actPulse(0, cfg_dosingMs[0]);
      lastPhDose[0] = now;
    }
    else if (data.pH > cfg_phMax && !actState[0] && canDosePH(1))
    {
      actPulse(1, cfg_dosingMs[1]);
      lastPhDose[1] = now;
    }
  }

  // Nutrient — skip if pH pumps are still running (interlock)
  if (ppmOk && !phPumpActive())
  {
    if (data.ppm < cfg_ppmMin && canDoseNutrient())
    {
      actPulse(2, cfg_dosingMs[2]);
      actPulse(3, cfg_dosingMs[3]);
      lastNutrientDose = now;
    }
    if (data.ppm > cfg_ppmMax)
      Serial.printf("[auto] WARNING: ppm %d > cfg_ppmMax %d\n", data.ppm, cfg_ppmMax);
  }

  // Water pump — time-based cycle (independent of sensor quality)
  uint32_t cycleMs = (cfg_pumpOnHr + cfg_pumpOffHr) * 3600000UL;
  uint32_t onMs = cfg_pumpOnHr * 3600000UL;
  bool shouldOn = (cycleMs > 0) && (((now - pumpCycleStart) % cycleMs) < onMs);
  if (shouldOn != pumpOn)
  {
    pumpOn = shouldOn;
    actSet(4, pumpOn);
  }
}

// ============================================================================
//  WiFi + Firebase
// ============================================================================
// RSSI (dBm) -> 0-4 signal bars, matching the LCD's 4-bar indicator.
int rssiToBars(int32_t rssi)
{
  if (rssi >= -55)
    return 4;
  if (rssi >= -65)
    return 3;
  if (rssi >= -75)
    return 2;
  if (rssi >= -85)
    return 1;
  return 0;
}

// Scans nearby networks (blocking, ~2-4s) and sends up to 4 results to ESP32-LCD as
// type "scan_wifi", dataName "network", each entry "SSID;signalBar;isConnected".
void scanAndSendWifi()
{
  // While disconnected, handleWiFi() retries WiFi.begin() every 10s and that association
  // attempt owns the radio — esp_wifi_scan_start() then fails (-2) or returns 0, so the LCD's
  // network list comes back empty exactly when it's needed most: WiFi lost -> open WiFi
  // settings -> pick another network. Abort the in-flight attempt so the scan owns the radio.
  // Only when disconnected: an established connection survives a scan, and dropping it here
  // would knock the device offline every time someone opened the WiFi page.
  if (WiFi.status() != WL_CONNECTED)
    WiFi.disconnect();

  int n = WiFi.scanNetworks();
  bool isConnected = (WiFi.status() == WL_CONNECTED);
  String curSsid = WiFi.SSID();

  String netArr[4];
  int count = constrain(n, 0, 4); // WiFi.scanNetworks() can return negative (e.g. WIFI_SCAN_FAILED)
  for (int i = 0; i < count; i++)
  {
    String ssid = WiFi.SSID(i);
    bool thisConnected = isConnected && ssid == curSsid;
    netArr[i] = ssid + ";" + String(rssiToBars(WiFi.RSSI(i))) + ";" + String(thisConnected ? 1 : 0);
  }
  for (int i = count; i < 4; i++)
    netArr[i] = ";0;0"; // empty slot

  WiFi.scanDelete();
  ESP_LCD.send("scan_wifi", "network", netArr);
  Serial.printf("[wifi] scan -> %d network dikirim ke LCD\n", count);
}

// scanAndSendWifi() blocks 2-4s inside WiFi.scanNetworks(), which cannot be broken up. Calling it
// straight from handleUART() stalls actUpdate() for that whole window, so a dosing pulse that
// should have ended mid-scan keeps its peristaltic pump running until the scan returns — a 6s
// calibrated dose becomes a 10s one, and the watchdog never notices because the scan is healthy.
// All callers request a scan here instead; loop() runs it once the dosing pumps are idle.
bool wifiScanPending = false;
void requestWifiScan() { wifiScanPending = true; }

void handleWiFi()
{
  static uint32_t lastTry = 0;
  if (WiFi.status() == WL_CONNECTED)
    return;
  if (wifiUserDisconnected)
    return; // user explicitly disconnected — stay off until the next connect_wifi
  if (wifiScanPending)
    return; // scan queued this loop pass — starting an association now would make it fail
  if (millis() - lastTry >= 10000)
  {
    lastTry = millis();
    WiFi.disconnect();
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, IPAddress(8, 8, 8, 8));
    WiFi.begin(cfg_wifiSsid.c_str(), cfg_wifiPass.c_str());
  }
}

void firebaseInit()
{
  // SSL timeouts MUST stay well under WDT_TIMEOUT_S (30s), and the budget is PER app.loop()
  // call, not per connect: one app.loop() services all 3 SSL clients, so on a bad link it can
  // do up to 3 blocking connects back-to-back before returning. The WDT feed after app.loop()
  // is useless if app.loop() itself never returns in time. At 5s each, 3 failed connects =
  // ~15s < 30s, so the loop always returns and feeds the watchdog. 5s is still ample for a
  // real handshake/round-trip; anything slower on this link was failing anyway.
  ssl_client.setInsecure();
  ssl_client.setTimeout(5000);
  ssl_client.setHandshakeTimeout(5);
  stream_ssl_client.setInsecure();
  stream_ssl_client.setTimeout(5000);
  stream_ssl_client.setHandshakeTimeout(5);
  setting_ssl_client.setInsecure();
  setting_ssl_client.setTimeout(5000);
  setting_ssl_client.setHandshakeTimeout(5);

  configTime(7 * 3600, 0, NTP_SERVER1, NTP_SERVER2);
  Serial.println("NTP: sync started");

  initializeApp(async_client, app, getAuth(user_auth), processFirebaseData, "authTask");
  app.getApp<RealtimeDatabase>(Database);
  Database.url(FIREBASE_DATABASE_URL);

  fbInit = true;
  Serial.println("Firebase: init done, waiting for auth...");
}

void sendData()
{
  if (time(nullptr) < 1700000000UL)
  {
    Serial.println("[data] skip push — NTP not synced");
    return;
  }
  JsonDocument doc;
  doc["timestamp"] = (long long)time(nullptr);
  doc["online"] = true;
  doc["ph"] = data.pH; // lowercase to match live DB
  doc["ppm"] = data.ppm;
  doc["airTemp"] = data.airTemp;
  doc["humidity"] = data.humidity;
  doc["waterTemp"] = data.waterTemp;
  doc["waterLevel"] = data.waterLevel;
  // false => waterLevel is a frozen last-known-good value, not a live measurement (see readSensor()).
  // Every other field in this push is still live and trustworthy — one dead sensor must not suppress
  // the whole telemetry payload, so consumers get the reading plus a flag rather than nothing at all.
  doc["waterLevelValid"] = waterLevelValid && ultrasonicOK;

  String jsonStr;
  serializeJson(doc, jsonStr);
  object_t obj(jsonStr); // send as JSON object (NOT set<String>)
  Database.set<object_t>(async_client, PATH_SENSORS, obj, processFirebaseData, "pushData");

  // Device status — same interval as sensor push
  time_t now_t = time(nullptr);
  struct tm *t = localtime(&now_t);
  char timeBuf[9];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);

  JsonDocument dev;
  dev["connected"] = true;
  dev["lastUpdate"] = timeBuf;
  dev["ssid"] = WiFi.SSID();

  String devStr;
  serializeJson(dev, devStr);
  object_t devObj(devStr);
  Database.set<object_t>(async_client, PATH_DEVICE, devObj, processFirebaseData, "deviceInfo");
}

void sendHourlyHistory()
{
  // Tak ada sampel terkumpul (mis. sensor belum sempat baca) -> JANGAN rekam 0-semua.
  if (accCount == 0)
  {
    Serial.println("[history] skip — belum ada sampel jam ini");
    return;
  }

  // Rata-rata dari sampel VALID saja (0/error sudah dikecualikan per-field). Kalau suatu field tak
  // punya sampel valid sepanjang jam (sensor mati), nilainya 0 — itu memang menandakan tak ada data.
  float ph = cntPh ? (float)(accPh / cntPh) : 0.0f;
  int ppm = cntPpm ? (int)lround(accPpm / cntPpm) : 0;
  float airTemp = cntAirT ? (float)(accAirT / cntAirT) : 0.0f;
  float humidity = cntHum ? (float)(accHum / cntHum) : 0.0f;
  float waterTemp = cntWaterT ? (float)(accWaterT / cntWaterT) : 0.0f;
  bool levelValid = (cntLevel > 0);
  float waterLevel = levelValid ? (float)(accLevel / cntLevel) : data.waterLevel; // last-known bila tak ada sampel valid

  time_t now_t = time(nullptr);
  struct tm *t = localtime(&now_t);
  char path[64];
  snprintf(path, sizeof(path), "%s/%04d/%02d/%02d/%02d",
           PATH_HISTORY,
           t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour);

  JsonDocument doc;
  doc["ph"] = ph;
  doc["ppm"] = ppm;
  doc["airTemp"] = airTemp;
  doc["humidity"] = humidity;
  doc["waterTemp"] = waterTemp;
  doc["waterLevel"] = waterLevel;
  doc["waterLevelValid"] = levelValid; // sama makna dgn sendData(): true bila ada sampel ultrasonik valid

  String jsonStr;
  serializeJson(doc, jsonStr);
  object_t obj(jsonStr);
  Database.set<object_t>(async_client, path, obj, processFirebaseData, "historyPush");
  Serial.printf("[history] pushed -> %s (valid: ph%lu ppm%lu aT%lu hum%lu wT%lu lvl%lu dari %lu baca)\n",
                path, (unsigned long)cntPh, (unsigned long)cntPpm, (unsigned long)cntAirT,
                (unsigned long)cntHum, (unsigned long)cntWaterT, (unsigned long)cntLevel,
                (unsigned long)accCount);

  // Reset akumulator untuk jam berikutnya.
  accPh = accPpm = accAirT = accHum = accWaterT = accLevel = 0;
  cntPh = cntPpm = cntAirT = cntHum = cntWaterT = cntLevel = 0;
  accCount = 0;
}

// ============================================================================
//  Notifications — push alert to Firebase on rising edge of each condition
// ============================================================================
void sendNotification(const char *title, const char *message)
{
  if (!app.ready() || time(nullptr) < 1700000000UL)
    return;
  // time() only resolves to whole seconds, but checkNotifications() can raise several alerts in a
  // single pass (pH low AND ppm low, say) — keying on seconds alone made them collide on one path,
  // so all but the last were silently overwritten. millis()%1000 fills the sub-second field: not
  // wall-clock exact, but monotonic within a second, which is all the key needs. The nudge below
  // covers the remaining case of two alerts landing in the same millisecond.
  static long long lastTsMs = 0;
  long long tsMs = (long long)time(nullptr) * 1000LL + (long long)(millis() % 1000UL);
  if (tsMs <= lastTsMs)
    tsMs = lastTsMs + 1;
  lastTsMs = tsMs;
  char path[80];
  snprintf(path, sizeof(path), "%s/%lld", PATH_NOTIFICATIONS, tsMs);
  JsonDocument doc;
  doc["isRead"] = false;
  doc["title"] = title;
  doc["message"] = message;
  doc["time"] = tsMs;
  String jsonStr;
  serializeJson(doc, jsonStr);
  object_t obj(jsonStr);
  Database.set<object_t>(async_client, path, obj, processFirebaseData, "notifPush");
  Serial.printf("[notif] %s: %s\n", title, message);
}

void checkNotifications()
{
  if (!app.ready() || time(nullptr) < 1700000000UL)
    return;

  bool phOk = (data.pH > 1.0f && data.pH < 13.0f);
  bool ppmOk = (data.ppm > 10);

  static bool nPhHigh = false, nPhLow = false;
  static bool nPpmHigh = false, nPpmLow = false;
  static bool nWaterLow = false, nUltrasonicFail = false;

  bool phHigh = phOk && (data.pH > cfg_phMax);
  bool phLow = phOk && (data.pH < cfg_phMin);
  bool ppmHigh = ppmOk && (data.ppm > cfg_ppmMax);
  bool ppmLow = ppmOk && (data.ppm < cfg_ppmMin);
  // waterLevelValid stays true forever after the first good reading — data.waterLevel now just
  // freezes on sensor failure instead of resetting, so this notification stays tied to an
  // actually-measured low level. Sensor failure itself gets its own notification below.
  bool waterLow = waterLevelValid && ultrasonicOK && (data.waterLevel <= cfg_waterLowPct);
  bool ultrasonicFail = waterLevelValid && !ultrasonicOK;

  char m[64];
  if (phHigh && !nPhHigh)
  {
    snprintf(m, sizeof(m), "pH terlalu tinggi : %.1f", data.pH);
    sendNotification("pH Tinggi", m);
  }
  if (phLow && !nPhLow)
  {
    snprintf(m, sizeof(m), "pH terlalu rendah : %.1f", data.pH);
    sendNotification("pH Rendah", m);
  }
  if (ppmHigh && !nPpmHigh)
  {
    snprintf(m, sizeof(m), "Nutrisi terlalu tinggi : %d ppm", data.ppm);
    sendNotification("Nutrisi Tinggi", m);
  }
  if (ppmLow && !nPpmLow)
  {
    snprintf(m, sizeof(m), "Nutrisi terlalu rendah : %d ppm", data.ppm);
    sendNotification("Nutrisi Rendah", m);
  }
  if (waterLow && !nWaterLow)
  {
    snprintf(m, sizeof(m), "Level air rendah : %.0f%%", data.waterLevel);
    sendNotification("Air Rendah", m);
  }
  if (ultrasonicFail && !nUltrasonicFail)
  {
    sendNotification("Sensor Ultrasonic Error", "Pembacaan level air gagal 10x berturut-turut — dosing & pompa dihentikan sampai sensor pulih.");
  }
  else if (!ultrasonicFail && nUltrasonicFail)
  {
    sendNotification("Sensor Ultrasonic Pulih", "Pembacaan level air normal kembali.");
  }

  nPhHigh = phHigh;
  nPhLow = phLow;
  nPpmHigh = ppmHigh;
  nPpmLow = ppmLow;
  nWaterLow = waterLow;
  nUltrasonicFail = ultrasonicFail;
}

// ============================================================================
//  Pump calibration — per-pump peristaltic flow rate (mL/min)
//  Each of the 4 dosing pumps (phUp/phDown/nutrisiA/nutrisiB) has its own independent node:
//    control/calibrate/<pump>/{durationS, start, measuredMl}
//  Flow: app writes durationS then start=true
//        -> device runs that pump for durationS seconds
//        -> app measures the dispensed volume with a graduated cylinder
//        -> app writes measuredMl=<value>
//        -> device computes flowRate = measuredMl / (durationS/60) and saves it to
//           /Hics/hics2026/setting/pumpCalibration/<pump>, which also updates cfg_pumpFlowMlMin[]
//           immediately (and will reload from there on every future boot/reconnect).
//  Only one pump can be calibrated at a time — a second pump's "start" is rejected while
//  another calibration is running or awaiting its result.
// ============================================================================
void startCalibration(int idx)
{
  // Self-clearing trigger: ack immediately so the next true edge is always detectable,
  // even if validation below rejects this request.
  if (idx >= 0 && idx <= 3 && fbInit && app.ready())
    Database.set<bool>(async_client, String(PATH_CONTROL) + "/calibrate/" + actName[idx] + "/start",
                       false, processFirebaseData, "calibStartAck");

  if (idx < 0 || idx > 3)
    return; // only the 4 peristaltic dosing pumps are calibratable
  if (mode != MANUAL)
  {
    Serial.println("[calib] ditolak — pindah ke mode MANUAL dulu sebelum kalibrasi");
    return;
  }
  if (calibRunning || calibAwaitingResult)
  {
    Serial.printf("[calib] %s ditolak — %s sedang dikalibrasi/menunggu hasil\n", actName[idx], actName[calibIdx]);
    return;
  }
  uint32_t durationS = calibDurationSel[idx];
  if (durationS == 0 || durationS > 120)
  {
    Serial.println("[calib] durationS tidak valid (harus 1-120 detik)");
    return;
  }

  calibIdx = idx;
  calibDurationS = durationS;
  calibRunning = true;
  actPulse(idx, calibDurationS * 1000UL);

  char m[64];
  snprintf(m, sizeof(m), "Menjalankan %s selama %lu detik", actName[idx], calibDurationS);
  Serial.printf("[calib] %s\n", m);
  sendNotification("Kalibrasi Dimulai", m);
}

void submitCalibrationResult(int idx, float measuredMl)
{
  if (idx < 0 || idx > 3 || !calibAwaitingResult || idx != calibIdx)
  {
    Serial.printf("[calib] %s tidak sedang menunggu hasil — diabaikan\n",
                  (idx >= 0 && idx <= 3) ? actName[idx] : "?");
    return;
  }
  if (measuredMl <= 0)
  {
    Serial.println("[calib] volume terukur tidak valid (<=0) — kalibrasi dibatalkan");
    calibAwaitingResult = false;
    calibIdx = -1;
    return;
  }

  float flowRate = measuredMl / ((float)calibDurationS / 60.0f);
  cfg_pumpFlowMlMin[idx] = flowRate;
  calcDosingMs(idx);

  if (fbInit && app.ready())
    Database.set<number_t>(async_client, String(PATH_SETTING) + "/pumpCalibration/" + actName[idx],
                           number_t(flowRate, 2), processFirebaseData, "calibSave");

  char m[80];
  snprintf(m, sizeof(m), "%s: %.1f mL/menit (dari %.1f mL / %lu detik)", actName[idx], flowRate, measuredMl, calibDurationS);
  Serial.printf("[calib] %s\n", m);
  sendNotification("Kalibrasi Tersimpan", m);

  calibAwaitingResult = false;
  calibIdx = -1;
}

// ---- LCD/UART-driven pump calibration ----
// Same state machine as the Firebase path, but progress/result are reported over UART:
//   LCD "calib_pump_start" {idx,durationS} -> run pump -> "run;<durationS>"
//   pulse ends (actUpdate) -> "done" (LCD prompts for measured mL)
//   LCD "calib_pump_measured" {ml} -> compute+save -> "ok;<flowRate>"  (or "err;<reason>")
//   LCD "calib_pump_cancel" -> abort silently.
void startPumpCalibrationLcd(int idx, uint32_t durationS)
{
  if (idx < 0 || idx > 3)
  {
    ESP_LCD.send("calib_pump_result", "data", String("err;pompa tidak valid"));
    return;
  }
  if (mode != MANUAL)
  {
    ESP_LCD.send("calib_pump_result", "data", String("err;pindah ke MANUAL dulu"));
    return;
  }
  if (calibRunning || calibAwaitingResult)
  {
    ESP_LCD.send("calib_pump_result", "data", String("err;kalibrasi lain berjalan"));
    return;
  }
  if (durationS == 0 || durationS > 120)
  {
    ESP_LCD.send("calib_pump_result", "data", String("err;durasi 1-120 detik"));
    return;
  }

  calibIdx = idx;
  calibDurationS = durationS;
  calibDurationSel[idx] = durationS;
  calibRunning = true;
  calibViaLcd = true;
  actPulse(idx, calibDurationS * 1000UL);

  char b[24];
  snprintf(b, sizeof(b), "run;%lu", (unsigned long)durationS);
  ESP_LCD.send("calib_pump_result", "data", String(b)); // "run;<durationS>" -> LCD mulai countdown
  Serial.printf("[calib] LCD pump %s jalan %lus\n", actName[idx], (unsigned long)durationS);
}

void submitPumpCalibrationLcd(float measuredMl)
{
  if (!calibAwaitingResult || calibIdx < 0)
  {
    ESP_LCD.send("calib_pump_result", "data", String("err;tidak menunggu hasil"));
    return;
  }
  if (measuredMl <= 0)
  {
    ESP_LCD.send("calib_pump_result", "data", String("err;volume tidak valid"));
    return;
  }
  int idx = calibIdx;
  float flowRate = measuredMl / ((float)calibDurationS / 60.0f);
  cfg_pumpFlowMlMin[idx] = flowRate;
  calcDosingMs(idx);
  if (fbInit && app.ready())
    Database.set<number_t>(async_client, String(PATH_SETTING) + "/pumpCalibration/" + actName[idx],
                           number_t(flowRate, 2), processFirebaseData, "calibSave");
  saveSettingsToNVS(); // persist offline (flow0..3)

  char b[40];
  snprintf(b, sizeof(b), "ok;%.1f", flowRate);
  ESP_LCD.send("calib_pump_result", "data", String(b)); // "ok;<flowRate mL/menit>"
  Serial.printf("[calib] LCD pump %s -> %.1f mL/menit\n", actName[idx], flowRate);

  calibAwaitingResult = false;
  calibIdx = -1;
  calibViaLcd = false;
}

void cancelPumpCalibrationLcd()
{
  if (calibRunning && calibIdx >= 0)
  {
    pulseActive[calibIdx] = false;
    actSet(calibIdx, false);
  }
  calibRunning = false;
  calibAwaitingResult = false;
  calibIdx = -1;
  calibViaLcd = false;
  Serial.println("[calib] LCD pump dibatalkan");
}

// ============================================================================
//  Sensor calibration — TDS (1-point)
//  pH is NOT calibrated on-device: cfg_phSlope/cfg_phOffset are set directly in
//  /Hics/hics2026/setting/{phSlope,phOffset} and read via applySetting().
//  TDS: app writes control/calibrate/tds/refPpm (known solution ppm), dips the probe, writes
//       tds/capture=true -> device computes tdsFactor = refPpm / rawPpm and saves it to
//       /Hics/hics2026/setting/tdsFactor (applied immediately).
// ============================================================================
// Core TDS calibration shared by BOTH triggers (Firebase capture and LCD/UART). Averages a
// handful of raw reads to cut noise, computes tdsFactor = refPpm / rawPpm, and persists it to
// cfg + /setting/tdsFactor + NVS. Returns false with *errOut set on any failure.
bool computeTdsCalib(float refPpm, float &factorOut, float &rawPpmOut, const char **errOut)
{
  if (!adsOK)
  {
    *errOut = "sensor ADS tidak terdeteksi";
    return false;
  }
  if (refPpm <= 0)
  {
    *errOut = "referensi tidak valid";
    return false;
  }

  // Average 10 raw reads — a single instantaneous sample is noisy, and calibration accuracy
  // depends directly on this reading. readADC_SingleEnded already includes conversion time.
  float sum = 0;
  for (int i = 0; i < 10; i++)
    sum += ads.computeVolts(ads.readADC_SingleEnded(1));
  float rawPpm = (sum / 10.0f) * 500.0f; // uncalibrated formula, before tdsFactor
  if (rawPpm <= 0)
  {
    *errOut = "pembacaan TDS mentah tidak valid";
    return false;
  }

  float factor = refPpm / rawPpm;
  cfg_tdsFactor = factor;
  if (fbInit && app.ready())
    Database.set<number_t>(async_client, String(PATH_SETTING) + "/tdsFactor", number_t(factor, 4), processFirebaseData, "calibTdsSave");
  saveSettingsToNVS(); // persist so the new factor survives reboot / offline boot

  factorOut = factor;
  rawPpmOut = rawPpm;
  return true;
}

// Firebase-triggered TDS calibration (control/calibrate/tds/capture). Delegates the math to
// computeTdsCalib() and reports via Serial + notification.
void captureTdsCalib()
{
  // Self-clearing trigger — ack immediately so the next true edge is always detectable.
  if (fbInit && app.ready())
    Database.set<bool>(async_client, String(PATH_CONTROL) + "/calibrate/tds/capture", false, processFirebaseData, "calibTdsAck");

  float factor, rawPpm;
  const char *err = "";
  if (!computeTdsCalib(calibTdsRef, factor, rawPpm, &err))
  {
    Serial.printf("[calib] TDS gagal: %s\n", err);
    return;
  }

  char m[80];
  snprintf(m, sizeof(m), "tdsFactor=%.3f (raw %.0f -> ref %.0f ppm)", factor, rawPpm, calibTdsRef);
  Serial.printf("[calib] TDS selesai: %s\n", m);
  sendNotification("Kalibrasi TDS Tersimpan", m);
}

// ============================================================================
//  LCD-triggered windowed TDS calibration — non-blocking sampler with progress over UART.
//  Unlike captureTdsCalib()'s instant read, this samples over ~5s so the probe can settle and
//  the average is robust. Each tick sends "calib_tds_prog" (percent;liveRawPpm) to the LCD;
//  the final tick computes tdsFactor, persists it (cfg + /setting + NVS), and sends
//  "calib_tds_result" (ok;factor;avgRaw;ref  OR  err;reason).
// ============================================================================
#define TDS_CAL_SAMPLES 20
#define TDS_CAL_INTERVAL_MS 250 // 20 * 250ms ≈ 5s window
bool tdsCalRunning = false;
int tdsCalCount = 0;
float tdsCalSum = 0;
float tdsCalRef = 0;
uint32_t tdsCalLastMs = 0;
// Hasil ukur ditahan di sini sampai user menekan "Simpan" di LCD (calib_tds_save). Ukur tidak
// lagi langsung commit — user melihat preview faktor dulu, lalu Simpan atau Batal.
bool tdsCalHasPending = false;
float tdsCalPendingFactor = 0;

void startTdsCalibration(float refPpm)
{
  // Calibration requires MANUAL mode: the probe is dipped in reference solution (out of the
  // tank), so leaving AUTO on would let controlAuto() dose against the calibration reading.
  if (mode != MANUAL)
  {
    ESP_LCD.send("calib_tds_result", "data", String("err;pindah ke MANUAL dulu"));
    return;
  }
  if (!adsOK)
  {
    ESP_LCD.send("calib_tds_result", "data", String("err;sensor ADS tidak terdeteksi"));
    return;
  }
  if (refPpm <= 0)
  {
    ESP_LCD.send("calib_tds_result", "data", String("err;referensi tidak valid"));
    return;
  }
  tdsCalRef = refPpm;
  tdsCalSum = 0;
  tdsCalCount = 0;
  tdsCalLastMs = 0; // first sample fires immediately
  tdsCalRunning = true;
  Serial.printf("[calib] TDS sampling mulai — ref %.0f, %d sampel @%dms\n",
                refPpm, TDS_CAL_SAMPLES, TDS_CAL_INTERVAL_MS);
}

void updateTdsCalibration()
{
  if (!tdsCalRunning)
    return;
  uint32_t now = millis();
  if (tdsCalLastMs != 0 && now - tdsCalLastMs < TDS_CAL_INTERVAL_MS)
    return;
  tdsCalLastMs = now;

  float rawPpm = ads.computeVolts(ads.readADC_SingleEnded(1)) * 500.0f; // uncalibrated, before factor
  tdsCalSum += rawPpm;
  tdsCalCount++;

  int pct = (tdsCalCount * 100) / TDS_CAL_SAMPLES;
  char prog[32];
  snprintf(prog, sizeof(prog), "%d;%.0f", pct, rawPpm); // percent ; live raw ppm
  ESP_LCD.send("calib_tds_prog", "data", String(prog));

  if (tdsCalCount < TDS_CAL_SAMPLES)
    return;

  // Window complete — compute + persist.
  tdsCalRunning = false;
  float avgRaw = tdsCalSum / tdsCalCount;
  if (avgRaw <= 0)
  {
    ESP_LCD.send("calib_tds_result", "data", String("err;pembacaan TDS tidak valid"));
    return;
  }
  // Hanya hitung + tahan sebagai pending; belum menyentuh cfg/NVS/Firebase. Commit menunggu
  // "calib_tds_save" dari LCD (tombol Simpan).
  float factor = tdsCalRef / avgRaw;
  tdsCalPendingFactor = factor;
  tdsCalHasPending = true;

  char buf[48];
  snprintf(buf, sizeof(buf), "ok;%.3f;%.0f;%.0f", factor, avgRaw, tdsCalRef);
  ESP_LCD.send("calib_tds_result", "data", String(buf));
  Serial.printf("[calib] TDS terukur (pending): factor=%.3f (avg raw %.0f -> ref %.0f)\n", factor, avgRaw, tdsCalRef);
}

// Commit hasil ukur yang tertahan ke cfg + NVS + Firebase (dipicu tombol Simpan di LCD).
void commitTdsCalibration()
{
  if (!tdsCalHasPending)
  {
    ESP_LCD.send("calib_tds_saved", "data", String("err;belum ada hasil ukur"));
    return;
  }
  cfg_tdsFactor = tdsCalPendingFactor;
  if (fbInit && app.ready())
    Database.set<number_t>(async_client, String(PATH_SETTING) + "/tdsFactor", number_t(cfg_tdsFactor, 4), processFirebaseData, "calibTdsSave");
  saveSettingsToNVS();
  tdsCalHasPending = false;

  char buf[32];
  snprintf(buf, sizeof(buf), "ok;%.3f", cfg_tdsFactor);
  ESP_LCD.send("calib_tds_saved", "data", String(buf));
  Serial.printf("[calib] TDS disimpan: factor=%.3f\n", cfg_tdsFactor);
}

// Batalkan: hentikan sampling yang sedang jalan dan buang hasil pending tanpa menyimpan.
void cancelTdsCalibration()
{
  tdsCalRunning = false;
  tdsCalHasPending = false;
  Serial.println("[calib] TDS dibatalkan (tak disimpan)");
}

// ============================================================================
//  LCD-triggered pH calibration (2-point) — non-blocking sampler over UART.
//  Ukur tegangan A0 di dua larutan buffer, lalu turunkan slope/offset:
//    slope = (ref2-ref1)/(v2-v1),  offset = ref1 - slope*v1   (pH = v*slope + offset)
//  Alur: LCD "calib_ph_point"{which;ref} -> sampling ~5s (progress "calib_ph_prog"=pct;pHnow)
//        -> "calib_ph_result"="pt;which;v" (titik selesai)  atau "err;alasan"
//        ulang untuk titik 2, lalu LCD "calib_ph_save" -> hitung+simpan -> "calib_ph_saved"=ok;slope;offset
//        "calib_ph_cancel" -> buang.
// ============================================================================
#define PH_CAL_SAMPLES 20
#define PH_CAL_INTERVAL_MS 250 // ~5s window
bool phCalRunning = false;
int phCalCount = 0;
float phCalSum = 0;
uint32_t phCalLastMs = 0;
int phCalWhich = 0;          // titik yang sedang diukur (1 atau 2)
float phCalRef[2] = {0, 0};  // nilai buffer titik 1/2
float phCalVolt[2] = {0, 0}; // tegangan rata-rata terukur titik 1/2
bool phCalHas[2] = {false, false};

void startPhCalPoint(int which, float ref)
{
  if (mode != MANUAL)
  {
    ESP_LCD.send("calib_ph_result", "data", String("err;pindah ke MANUAL dulu"));
    return;
  }
  if (!adsOK)
  {
    ESP_LCD.send("calib_ph_result", "data", String("err;sensor ADS tidak terdeteksi"));
    return;
  }
  if (which != 1 && which != 2)
  {
    ESP_LCD.send("calib_ph_result", "data", String("err;titik tidak valid"));
    return;
  }
  if (ref <= 0 || ref > 14)
  {
    ESP_LCD.send("calib_ph_result", "data", String("err;buffer tidak valid"));
    return;
  }
  phCalWhich = which;
  phCalRef[which - 1] = ref;
  phCalSum = 0;
  phCalCount = 0;
  phCalLastMs = 0; // first sample fires immediately
  phCalRunning = true;
  Serial.printf("[calib] pH titik %d ref %.2f — sampling mulai\n", which, ref);
}

void updatePhCalibration()
{
  if (!phCalRunning)
    return;
  uint32_t now = millis();
  if (phCalLastMs != 0 && now - phCalLastMs < PH_CAL_INTERVAL_MS)
    return;
  phCalLastMs = now;

  float v = ads.computeVolts(ads.readADC_SingleEnded(0)); // pH di A0
  phCalSum += v;
  phCalCount++;

  int pct = (phCalCount * 100) / PH_CAL_SAMPLES;
  float pHnow = constrain(v * cfg_phSlope + cfg_phOffset, 0.0f, 14.0f); // pH dgn kalibrasi saat ini
  char prog[32];
  snprintf(prog, sizeof(prog), "%d;%.2f", pct, pHnow);
  ESP_LCD.send("calib_ph_prog", "data", String(prog));

  if (phCalCount < PH_CAL_SAMPLES)
    return;

  phCalRunning = false;
  float avgV = phCalSum / phCalCount;
  phCalVolt[phCalWhich - 1] = avgV;
  phCalHas[phCalWhich - 1] = true;

  char b[32];
  snprintf(b, sizeof(b), "pt;%d;%.3f", phCalWhich, avgV);
  ESP_LCD.send("calib_ph_result", "data", String(b));
  Serial.printf("[calib] pH titik %d selesai: %.3fV (ref %.2f)\n", phCalWhich, avgV, phCalRef[phCalWhich - 1]);
}

void savePhCalibration()
{
  if (!phCalHas[0] || !phCalHas[1])
  {
    ESP_LCD.send("calib_ph_saved", "data", String("err;ukur 2 titik dulu"));
    return;
  }
  float dV = phCalVolt[1] - phCalVolt[0];
  float dRef = phCalRef[1] - phCalRef[0];
  if (fabsf(dV) < 0.01f)
  {
    ESP_LCD.send("calib_ph_saved", "data", String("err;tegangan 2 titik sama"));
    return;
  }
  if (fabsf(dRef) < 0.01f)
  {
    ESP_LCD.send("calib_ph_saved", "data", String("err;buffer 2 titik sama"));
    return;
  }
  float slope = dRef / dV;
  float offset = phCalRef[0] - slope * phCalVolt[0];
  cfg_phSlope = slope;
  cfg_phOffset = offset;
  if (fbInit && app.ready())
  {
    Database.set<number_t>(async_client, String(PATH_SETTING) + "/phSlope", number_t(slope, 4), processFirebaseData, "calibPhSave");
    Database.set<number_t>(async_client, String(PATH_SETTING) + "/phOffset", number_t(offset, 4), processFirebaseData, "calibPhSave");
  }
  saveSettingsToNVS();

  char b[48];
  snprintf(b, sizeof(b), "ok;%.3f;%.2f", slope, offset);
  ESP_LCD.send("calib_ph_saved", "data", String(b));
  Serial.printf("[calib] pH disimpan slope=%.4f offset=%.4f\n", slope, offset);
  phCalHas[0] = phCalHas[1] = false;
}

void cancelPhCalibration()
{
  phCalRunning = false;
  phCalHas[0] = phCalHas[1] = false;
  Serial.println("[calib] pH dibatalkan (tak disimpan)");
}

// Beri tahu LCD status WiFi (untuk status bar atas): "1;<ssid>" tersambung, "0;" tidak.
// LCD tak tahu SSID sampai buka menu WiFi (scan); ini mengisinya sejak boot/awal konek.
void sendWifiStatusToLCD()
{
  bool up = (WiFi.status() == WL_CONNECTED);
  String s = String(up ? "1;" : "0;") + (up ? WiFi.SSID() : String(""));
  ESP_LCD.send("wifi_status", "data", s);
}

// Kirim status boot ke LCD (splash progress). Payload "pct;teks"; terminal "ready"/"offline"
// dikirim langsung di boot gate. LCD hanya menggambar — main yang menghitung pct & teks.
void sendBootProgress(bool wifiUp, bool ntpOk, bool fbOk)
{
  int pct = 10 + (wifiUp ? 30 : 0) + (ntpOk ? 30 : 0) + (fbOk ? 30 : 0);
  if (pct > 100)
    pct = 100;
  const char *txt;
  if (!wifiUp)
    txt = "Menyambung WiFi...";
  else if (!fbOk && !ntpOk)
    txt = "Sinkron waktu & Firebase...";
  else if (!fbOk)
    txt = "Menyambung Firebase...";
  else if (!ntpOk)
    txt = "Sinkron waktu...";
  else
    txt = "Siap!";
  char b[56];
  snprintf(b, sizeof(b), "%d;%s", pct, txt);
  ESP_LCD.send("boot", "data", String(b));
}

// ============================================================================
//  UART — terima mode dari esp32_lcd
// ============================================================================
void handleUART()
{
  if (!ESP_LCD.update())
    return;

  String type = ESP_LCD.recvType();
  Serial.println(type);

  if (type == "mode")
  {
    bool autoOn = ESP_LCD.recv<bool>("mode", "value");
    applyMode(autoOn);
    Serial.printf("[uart] recv mode -> %s\n", autoOn ? "AUTO" : "MANUAL");
  }
  else if (type == "scan_wifi")
  {
    Serial.println("[uart] scan_wifi diminta LCD");
    requestWifiScan(); // deferred until pumps idle — see requestWifiScan()
  }
  else if (type == "connect_wifi")
  {
    // credential format: "SSID;PASSWORD" (matches lcdBoard's connect_wifi sender)
    String credential = ESP_LCD.recv<String>("connect_wifi", "credential");
    int sep = credential.indexOf(';');
    if (sep < 0)
    {
      Serial.println("[uart] connect_wifi ditolak — format kredensial tidak valid");
    }
    else
    {
      String ssid = credential.substring(0, sep);
      String pass = credential.substring(sep + 1);
      Serial.printf("[uart] connect_wifi -> mencoba konek ke '%s'\n", ssid.c_str());

      wifiUserDisconnected = false; // explicit connect request cancels any prior disconnect
      WiFi.disconnect();
      WiFi.begin(ssid.c_str(), pass.c_str());

      // This wait blocks the main loop for up to 8s, so it has to do actUpdate()'s job itself —
      // otherwise a dosing pulse in flight runs until the connect resolves. 50ms keeps pulse
      // resolution tight enough that the overrun stays within a pump's own mechanical slop.
      uint32_t startMs = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startMs < 8000UL)
      {
        esp_task_wdt_reset();
        actUpdate();
        delay(50);
      }
      bool ok = WiFi.status() == WL_CONNECTED;
      Serial.printf("[uart] connect_wifi -> %s\n", ok ? "berhasil" : "gagal/timeout");
      if (ok)
        saveWifiCredentials(ssid, pass); // persist to NVS — handleWiFi() + next boot use this network
    }
    requestWifiScan(); // report the resulting state either way
  }
  else if (type == "disconnect_wifi")
  {
    Serial.println("[uart] disconnect_wifi");
    wifiUserDisconnected = true; // stop handleWiFi() from auto-reconnecting
    WiFi.disconnect();
    requestWifiScan();
  }
  else if (type == "restart")
  {
    ESP.restart();
  }
  else if (type == "pump_btn")
  {
    // waterPump is a sustained on/off toggle, not a dose — pass the state through as-is.
    bool state = ESP_LCD.recv<bool>("pump_btn", "state");
    applyManual("waterPump", state);
  }
  else if (type == "phUp_btn")
  {
    // Toggle saklar: state ON = pompa berputar terus, OFF = mati (applyManual sustained utk idx<=3).
    applyManual("phUp", ESP_LCD.recv<bool>("phUp_btn", "state"));
  }
  else if (type == "phDown_btn")
  {
    applyManual("phDown", ESP_LCD.recv<bool>("phDown_btn", "state"));
  }
  else if (type == "nutrisiA_btn")
  {
    applyManual("nutrisiA", ESP_LCD.recv<bool>("nutrisiA_btn", "state"));
  }
  else if (type == "nutrisiB_btn")
  {
    applyManual("nutrisiB", ESP_LCD.recv<bool>("nutrisiB_btn", "state"));
  }
  else if (type == "selector_switch")
  {
    // Local switch on the LCD panel — apply the mode the same as a Firebase/app change.
    bool selector = ESP_LCD.recv<bool>("selector_switch", "state");
    applyMode(selector);
  }
  else if (type == "calib_tds_start")
  {
    // LCD started a windowed TDS calibration. refPpm arrives as a string; kick off the
    // non-blocking sampler (updateTdsCalibration() streams progress + sends the final result).
    float refPpm = ESP_LCD.recv<String>("calib_tds_start", "refPpm").toFloat();
    Serial.printf("[uart] calib_tds_start -> refPpm %.0f\n", refPpm);
    startTdsCalibration(refPpm);
  }
  else if (type == "calib_tds_save")
  {
    // Tombol Simpan di LCD — commit hasil ukur yang tertahan.
    Serial.println("[uart] calib_tds_save");
    commitTdsCalibration();
  }
  else if (type == "calib_tds_cancel")
  {
    // Tombol Batal di LCD — buang hasil/ hentikan sampling tanpa menyimpan.
    Serial.println("[uart] calib_tds_cancel");
    cancelTdsCalibration();
  }
  else if (type == "calib_pump_start")
  {
    // LCD memulai kalibrasi pompa: payload "idx;durationS" (satu field, seperti pola TDS).
    String d = ESP_LCD.recv<String>("calib_pump_start", "data");
    int sep = d.indexOf(';');
    int idx = d.substring(0, sep).toInt();
    uint32_t durationS = (uint32_t)d.substring(sep + 1).toInt();
    Serial.printf("[uart] calib_pump_start -> idx %d, %lus\n", idx, (unsigned long)durationS);
    startPumpCalibrationLcd(idx, durationS);
  }
  else if (type == "calib_pump_measured")
  {
    // Volume terukur dari LCD — hitung flow rate & simpan.
    float ml = ESP_LCD.recv<String>("calib_pump_measured", "ml").toFloat();
    Serial.printf("[uart] calib_pump_measured -> %.1f mL\n", ml);
    submitPumpCalibrationLcd(ml);
  }
  else if (type == "calib_pump_cancel")
  {
    Serial.println("[uart] calib_pump_cancel");
    cancelPumpCalibrationLcd();
  }
  else if (type == "calib_ph_point")
  {
    // Ukur satu titik pH: payload "which;ref" (which=1/2).
    String d = ESP_LCD.recv<String>("calib_ph_point", "data");
    int sep = d.indexOf(';');
    int which = d.substring(0, sep).toInt();
    float ref = d.substring(sep + 1).toFloat();
    Serial.printf("[uart] calib_ph_point -> titik %d, ref %.2f\n", which, ref);
    startPhCalPoint(which, ref);
  }
  else if (type == "calib_ph_save")
  {
    Serial.println("[uart] calib_ph_save");
    savePhCalibration();
  }
  else if (type == "calib_ph_cancel")
  {
    Serial.println("[uart] calib_ph_cancel");
    cancelPhCalibration();
  }
  else if (type == "req_boot")
  {
    // LCD baru startup / reset sendiri — minta status boot terkini. Kalau main sudah selesai
    // boot, balas terminal; kalau masih proses, kirim progress saat ini.
    if (bootReady)
      ESP_LCD.send("boot", "data", String((fbInit && app.ready()) ? "ready" : "offline"));
    else
      sendBootProgress(WiFi.status() == WL_CONNECTED, time(nullptr) > 1700000000UL, fbInit && app.ready());
    sendWifiStatusToLCD();                              // sekalian kirim status WiFi terkini
    ESP_LCD.send("device_id", "id", String(DEVICE_ID)); // dan ID device (untuk halaman Info)
  }
}

// ============================================================================
//  Setup / Loop
// ============================================================================
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n== ESP32-MAIN ==");

  // This board never uses Bluetooth, but Arduino-ESP32 reserves ~50KB of heap for the BT
  // controller by default. mem_release() reclaims it WITHOUT linking the BT stack (unlike
  // btStop(), which costs ~128KB flash just to link the stop/teardown path — measured).
  // The freed heap matters because 3 concurrent WiFiClientSecure/TLS sessions (auth +
  // control stream + setting stream) each need ~40KB+ to handshake, and plain esp32dev has
  // no PSRAM — without this, that's exactly when "SSL - Memory allocation failed" happens.
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  Serial.printf("BT: memory released, free heap %u bytes\n", ESP.getFreeHeap());

  esp_task_wdt_init(WDT_TIMEOUT_S, true); // panic + reboot if loop hangs > 30s
  esp_task_wdt_add(NULL);
  Serial.printf("WDT: %ds timeout enabled\n", WDT_TIMEOUT_S);

  for (int i = 0; i < ACT_COUNT; i++)
  {
    // Preload the OFF level into the output latch BEFORE switching to OUTPUT, so the pin never
    // passes through a transient active level as it flips to output — otherwise the actuator would
    // click ON for an instant at boot. OFF level is per-actuator (relayLevel): HIGH for the
    // active-LOW dosing relays, LOW for the active-HIGH waterPump SSR.
    digitalWrite(actPin[i], relayLevel(i, false));
    pinMode(actPin[i], OUTPUT);
    digitalWrite(actPin[i], relayLevel(i, false));
  }
  Serial.println(ACTUATORS_ENABLED ? "Actuators: enabled, all OFF" : "Actuators: DISABLED (bench mode)");

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Wire.begin(); // SDA 21, SCL 22

  if (ads.begin(0x48))
  {
    ads.setGain(GAIN_ONE);
    adsOK = true;
  }
  Serial.printf("PH & TDS    : %s\n", adsOK ? "OK" : "NOT FOUND (0x48)");

  ahtOK = aht.begin();
  Serial.printf("AHT10       : %s\n", ahtOK ? "OK" : "NOT FOUND (0x38)");

  ds.begin();
  ds.setWaitForConversion(false);
  dsOK = ds.getDeviceCount() > 0;
  Serial.printf("DS18B20     : %s\n", dsOK ? "OK" : "NOT FOUND (32)");

  Serial2.begin(115200, SERIAL_8N1, PIN_RXD2, PIN_TXD2);
  ESP_LCD.begin(Serial2);
  Serial.println("UART: Serial2 ready (RX=16, TX=17)");

  // Restore last-known config from NVS BEFORE WiFi/Firebase — so an offline boot runs on real
  // saved values (sensor slope/offset, limits, dosing) instead of compile-time defaults. If
  // Firebase later connects, its /setting payload overwrites these and re-persists.
  loadSettingsFromNVS();

  // Migrasi satu-kali: NVS lama menyimpan tankDepthCm dari default lama (50). Set 160 sekali saja
  // (dijaga flag td160done) — aman ditinggal, tak melawan perubahan Firebase berikutnya.
  {
    Preferences p;
    p.begin("setting", false);
    if (!p.getBool("td160done", false))
    {
      p.putFloat("tankDepthCm", 160.0f);
      p.putBool("td160done", true);
      cfg_tankDepthCm = 160.0f;
      Serial.println("[setting] migrasi tankDepthCm -> 160 (sekali)");
    }
    p.end();
  }
  sendSensorLimitToLCD(); // push restored limits/units to the LCD immediately
  sendSensorUnitToLCD();

  loadWifiCredentials();
  Serial.printf("WiFi: credentials loaded -> '%s'\n", cfg_wifiSsid.c_str());

  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, IPAddress(8, 8, 8, 8));
  WiFi.begin(cfg_wifiSsid.c_str(), cfg_wifiPass.c_str());
  Serial.println("WiFi: connecting...");

  bootStartMs = millis();
  pumpCycleStart = millis();

  // Status boot awal ke LCD (umpan balik langsung, sebelum heartbeat pertama di loop).
  ESP_LCD.send("boot", "data", String("10;Menyalakan sistem..."));
  ESP_LCD.send("device_id", "id", String(DEVICE_ID)); // ID device untuk halaman Info di LCD

  Serial.println("MAIN ready\n");
}

void loop()
{
  esp_task_wdt_reset();
  handleWiFi();
  handleUART();
  actUpdate();
  controlAuto();
  updateTdsCalibration(); // step the windowed TDS calibration sampler (no-op unless running)
  updatePhCalibration();  // step the pH 2-point sampler (no-op unless running)
  updateUltrasonic();     // step the non-blocking ultrasonic sampler (median burst @2s)

  // Beri tahu LCD tiap kali status WiFi berubah (connect/disconnect) supaya status bar atas
  // langsung benar tanpa harus buka menu WiFi dulu.
  static bool lastWifiUpReported = false;
  bool wifiUpNow = (WiFi.status() == WL_CONNECTED);
  if (wifiUpNow != lastWifiUpReported)
  {
    lastWifiUpReported = wifiUpNow;
    sendWifiStatusToLCD();
  }

  uint32_t now = millis();

  // Boot gate: hold sensor reads until BOTH WiFi and Firebase are ready — NTP gets synced
  // as part of firebaseInit() once WiFi connects. Only the boot timeout elapsing forces a
  // fall back to OFFLINE (sensors-only, no cloud) so the device doesn't stall forever.
  if (!bootReady)
  {
    bool netUp = (WiFi.status() == WL_CONNECTED);
    if (netUp && !fbInit)
      firebaseInit(); // starts NTP sync + Firebase auth as soon as WiFi is up

    bool ntpOk = time(nullptr) > 1700000000UL;
    bool fbOk = fbInit && app.ready();

    // Heartbeat status boot ke LCD tiap ~1s — juga menolong LCD yang baru reset (langsung dapat
    // status terkini karena kita terus mengirim ulang selama boot belum selesai).
    if (now - lastBootMsgMs >= 1000)
    {
      lastBootMsgMs = now;
      sendBootProgress(netUp, ntpOk, fbOk);
    }

    if (netUp && fbOk)
    {
      bootReady = true;
      lastSensorRead = now; // first real sensor read happens 1s from here, not from cold boot
      ESP_LCD.send("boot", "data", String("ready"));
      Serial.println("[boot] WiFi + Firebase ready -> resuming sensor reads");
    }
    else if (now - bootStartMs >= WIFI_BOOT_TIMEOUT_MS)
    {
      bootReady = true;
      bootOffline = true;
      lastSensorRead = now;
      ESP_LCD.send("boot", "data", String("offline"));
      Serial.println("[boot] WiFi/Firebase timeout -> continuing OFFLINE, sensor reads resumed");
    }
  }

  // Deferred WiFi scan — blocks 2-4s, so it only runs with the dosing pumps idle (requestWifiScan()).
  if (wifiScanPending && !anyPulseActive())
  {
    wifiScanPending = false;
    scanAndSendWifi();
    esp_task_wdt_reset(); // scan blocked ~2-4s — feed WDT before the SSL work below
  }

  if (bootReady && now - lastSensorRead >= 1000)
  {
    lastSensorRead = now;
    readSensor();
    sensorsRead = true;

    // Akumulasi untuk rata-rata history per jam (lihat sendHourlyHistory()).
    // Nilai 0 = error sensor -> tak ikut dijumlah (per-field), supaya tak menyeret rata-rata.
    accCount++;
    if (data.pH > 0)
    {
      accPh += data.pH;
      cntPh++;
    }
    if (data.ppm > 0)
    {
      accPpm += data.ppm;
      cntPpm++;
    }
    if (data.airTemp > 0)
    {
      accAirT += data.airTemp;
      cntAirT++;
    }
    if (data.humidity > 0)
    {
      accHum += data.humidity;
      cntHum++;
    }
    if (data.waterTemp > 0)
    {
      accWaterT += data.waterTemp;
      cntWaterT++;
    }
    if (ultrasonicOK) // waterLevel: 0 sah (tangki kosong), cukup digate ultrasonicOK
    {
      accLevel += data.waterLevel;
      cntLevel++;
    }
  }

  // LCD refresh — throttled to LCD_PUSH_MS, independent of the read cadence above.
  // lastLcdPush == 0 forces the push right after the first read, so the panel populates on boot
  // instead of showing nothing until LCD_PUSH_MS of uptime has elapsed.
  if (sensorsRead && (lastLcdPush == 0 || now - lastLcdPush >= LCD_PUSH_MS))
  {
    lastLcdPush = now;
    sendSensorToLCD();
  }

  if (!fbInit && WiFi.status() == WL_CONNECTED)
    firebaseInit();
  if (fbInit)
  {
    app.loop();           // pumps auth, the write client, AND the stream client
    esp_task_wdt_reset(); // a (re)connect handshake here can block up to 10s — feed WDT after

    // Detect Firebase drop so streams can be restarted on reconnect
    static bool fbWasReady = false;
    bool fbReady = app.ready();
    if (fbWasReady && !fbReady)
    {
      streamInit = false;
      settingStreamStarted = false;
      settingsLoaded = false;
      Serial.println("[FB] connection lost — will reload configuration on reconnect");
    }
    if (!fbWasReady && fbReady)
    {
      // Reconnected — re-assert every dose status so any that was frozen while offline
      // (e.g. a "success" that never got written) gets corrected now, not left stale.
      for (int i = 0; i < 4; i++)
        doseStatusDirty[i] = true;
    }
    fbWasReady = fbReady;

    flushDoseStatus(); // push any pending dose status once the link is up

    // Step 1: as soon as Firebase auth is ready, start the /setting stream — its first
    // payload is the full config node, applied by applySettings() (sets settingsLoaded).
    if (app.ready() && !settingStreamStarted)
    {
      Database.get(setting_client, PATH_SETTING, processFirebaseData, true, "settingStream");
      settingStreamStarted = true;
      Serial.println("Firebase: setting stream started — loading configuration...");
    }

    // Step 2: only once configuration has actually loaded, sync actuator state and start
    // the control stream — control commands should never race ahead of current config.
    if (app.ready() && settingsLoaded && !streamInit)
    {
      syncInitialState();
      Database.get(stream_client, PATH_CONTROL, processFirebaseData, true, "controlStream");
      streamInit = true;
      Serial.println("Firebase: configuration loaded — control stream started");
    }
    // Pushed unconditionally: a dead ultrasonic used to gate this, which meant an unplugged level
    // sensor also blocked pH/ppm/temp telemetry AND /device/connected — the app showed the whole
    // device offline while every other sensor read fine. waterLevelValid in the payload carries
    // that signal now instead.
    if (app.ready() && now - lastDataPush >= cfg_dataPushMs)
    {
      lastDataPush = now;
      sendData();
    }

    // Hourly history — push AVERAGE saat jam berganti (NTP tersinkron + minimal 1x baca sensor).
    static int lastHistoryHour = -1;
    if (app.ready() && sensorsRead && time(nullptr) > 1700000000UL)
    {
      time_t now_t = time(nullptr);
      struct tm *t = localtime(&now_t);
      if (lastHistoryHour == -1)
      {
        // Pass pertama: seed jam sekarang — JANGAN push saat boot (bisa data belum matang).
        // Push pertama menyusul di pergantian jam berikutnya, dengan rata-rata sampel sejak boot.
        lastHistoryHour = t->tm_hour;
      }
      else if (t->tm_hour != lastHistoryHour)
      {
        lastHistoryHour = t->tm_hour;
        sendHourlyHistory();
      }
    }

    // Notifications — only after at least one sensor read to avoid boot spurious alerts
    if (sensorsRead)
      checkNotifications();

    // Sensor calibration capture: step an in-progress run every loop; when idle, poll the
    // calibration node (one-shot GET on the write client — no dedicated stream) for a
    // command="start". Polling pauses while a capture runs so command="start" can't re-trigger.
    updateCalibrationCapture();
    if (streamInit && !calibCaptureRunning && !calibStarting && now - calibPollLastMs >= 3000UL)
    {
      calibPollLastMs = now;
      Database.get(async_client, String(PATH_CALIBRATION) + "/command", processFirebaseData, false, "calibCmdPoll");
    }
  }

  if (now - lastHeartbeat >= 1000)
  {
    lastHeartbeat = now;
    Serial.printf("pH:%.2f ppm:%d air:%.1fC hum:%.0f%% water:%.1fC level:%.0f%% | WiFi:%s FB:%s NTP:%s mode:%s | act:%s%s%s%s%s | calib:%s | heap:%u\n",
                  data.pH, data.ppm, data.airTemp, data.humidity, data.waterTemp, data.waterLevel,
                  WiFi.status() == WL_CONNECTED ? "ok" : "..",
                  (fbInit && app.ready()) ? "ready" : "..",
                  time(nullptr) > 1700000000UL ? "ok" : "..",
                  mode == AUTO ? "auto" : "manual",
                  actState[0] ? "phUp " : "", actState[1] ? "phDn " : "",
                  actState[2] ? "nA " : "", actState[3] ? "nB " : "",
                  actState[4] ? "pump" : "----",
                  calibRunning ? actName[calibIdx] : (calibAwaitingResult ? "waiting-mL" : "idle"),
                  ESP.getFreeHeap());
  }
}