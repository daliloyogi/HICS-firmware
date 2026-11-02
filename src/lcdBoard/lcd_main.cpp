
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <VoltX_UART.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>

#include "Splash.h"
#include "Display_Touch.h"

#define TFT_CS 5
#define TFT_DC 2
#define TFT_RST 15
#define TFT_BL 4
#define TOUCH_CS 21
#define TOUCH_IRQ 22

#define SELECTOR_MAN 27
#define SELECTOR_AUT 14
#define BUZZER 13

#define TOUCH_X_MIN 224
#define TOUCH_X_MAX 3896
#define TOUCH_Y_MIN 396
#define TOUCH_Y_MAX 3899

#define RXD2 16
#define TXD2 17

#define EEPROM_SIZE 128

VoltX_UART espMaster;
XPT2046_Touchscreen touch(TOUCH_CS);
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

enum Page
{
	PAGE_BOOT,
	PAGE_DASHBOARD,
	PAGE_SETTING,
	PAGE_WIFI,
	PAGE_WIFI_PASSWORD,
	PAGE_SENSOR_LIMIT,
	PAGE_CALIBRATION,
	PAGE_CALIB_PH,
	PAGE_CALIB_TDS,
	PAGE_CALIB_PUMP,
	PAGE_SCREEN_TIMEOUT,
	PAGE_RESET,
	PAGE_ABOUT
};

enum KbMode
{
	KB_UPPER,
	KB_LOWER,
	KB_NUM,
	KB_SYM
};

Page currentPage = PAGE_DASHBOARD;
KbMode kbMode = KB_UPPER;

struct Wifi
{
	String ssid;
	int signal;
	bool connected;
};

String wifiRaw[4];
Wifi wifiList[4];

int NET_COUNT = 4;

void setActiveButtons(Button **btns, int count);
void showPage(Page page);

char wifiInputSSID[32] = "";
char wifiPassword[64] = "";
int wifiPasswordLen = 0;

String wifiSSID = "";
String deviceId = "-"; // ID device dari main board (UART "device_id"), tampil di halaman Info
bool waitRespons = false;
bool scanWifi = false;
unsigned long wifiTimeout = 0;

bool wifiOK = false;
int t9LastBtn = -1;
int t9CharIdx = 0;

// ---- Kalibrasi TDS ----
char calibRefBuf[10] = ""; // nilai referensi PPM yang diketik (mis. "1382")
int calibRefLen = 0;
String calibTdsStatus = "";		   // detail hasil/pesan terakhir (mis. "faktor = 0.923")
bool tdsCalibInProgress = false;   // true selama jendela sampling berjalan (progress bar aktif)
bool tdsCalibSuccess = false;	   // warna kotak hasil: hijau (sukses) vs merah (gagal/peringatan)
bool tdsCalibHasResult = false;	   // ada faktor hasil ukur yang belum disimpan (tombol Simpan aktif)
unsigned long tdsCalibStartMs = 0; // waktu mulai — untuk timeout kalau main tak membalas

// ---- Kalibrasi pompa peristaltik ----
// Alur: pilih pompa + durasi -> Jalankan (pompa berputar, hitung mundur) -> ukur volume dengan
// gelas ukur -> ketik mL -> Simpan (main hitung flow rate mL/menit) atau Batal.
int pumpCalibSel = -1;		// pompa terpilih 0..3 (-1 = belum pilih)
uint32_t pumpCalibDur = 10; // durasi uji (detik): 10/20/30
char pumpMlBuf[8] = "";		// volume terukur yang diketik (mL)
int pumpMlLen = 0;
String pumpCalibStatus = "";	  // teks status/hasil terakhir
bool pumpCalibSuccess = false;	  // warna kotak status: hijau vs merah
bool pumpCalibRunning = false;	  // pompa sedang berputar (hitung mundur)
bool pumpCalibMeasure = false;	  // pompa selesai, menunggu input volume
unsigned long pumpRunStartMs = 0; // waktu mulai putaran — untuk hitung mundur & timeout
uint32_t pumpRunDurMs = 0;		  // durasi putaran (ms)
int pumpLastCountShown = -1;	  // detik terakhir yang tergambar (agar tak redraw tiap loop)
static const uint32_t PUMP_DUR_OPT[3] = {10, 20, 30};
static const char *PUMP_NAMES[4] = {"pH+", "pH-", "Nut A", "Nut B"};

// ---- Kalibrasi pH (2 titik) ----
// Alur: pilih buffer standar untuk titik 1 -> Ukur (main sampling ~5s tegangan) -> ulang titik 2
// -> Simpan (main hitung slope/offset) atau Batal. Pakai preset buffer, tanpa keypad.
int phPointsDone = 0;			// jumlah titik yang selesai diukur (0,1,2)
float phBufSel = 0;				// buffer terpilih utk titik saat ini (0 = belum)
bool phCalInProgress = false;	// sampling titik berjalan
bool phCalSuccess = false;		// warna kotak status
bool phCalHasResult = false;	// 2 titik siap -> tombol Simpan aktif
String phCalStatus = "";		// teks status/hasil
float phRef[2] = {0, 0};		// buffer titik 1/2 (tampilan ringkasan)
float phVolt[2] = {0, 0};		// tegangan hasil titik 1/2 (tampilan ringkasan)
unsigned long phCalStartMs = 0; // waktu mulai — timeout
int phLastPct = -1;				// pct terakhir tergambar
static const float PH_BUFFERS[4] = {4.00f, 6.86f, 7.00f, 9.18f};

#define LED_BRIGHT 1023
#define LED_DIM 10

int SCREEN_TIMEOUT_END = 10000;
int SCREEN_TO[5] = {15000, 30000, 60000, 120000, 500};
int SCREEN_TIMEOUT = 30000; // default 30s sampai EEPROM dibaca
unsigned long lastTouchTime = 0;
unsigned long lastTouchTimeEnd = 0;

bool screenTimeoutOptions[5] = {false, true, false, false, false};

bool ledOn = true;
bool ledEnd = false;
bool pumpOn = false; // status pompa air; boot = mati (disinkron dari main lewat "wifi_status"/kontrol)
bool autoMode = false;
bool lastPump = false;
// Status ON/OFF 4 pompa dosing manual (phUp, phDown, nutrisiA, nutrisiB). Tombolnya kini SAKLAR:
// tekan = nyala (pompa berputar terus), tekan lagi = mati. Reset ke false saat pindah ke AUTO.
bool doseOn[4] = {false, false, false, false};

// ---- Layar boot (progress dari main lewat UART "boot") ----
int bootPct = 10;                    // persentase progress
String bootText = "Menyalakan sistem..."; // teks langkah sekarang
bool bootOfflineShown = false;       // sudah menampilkan tombol "Lanjut Offline"
bool bootMsgReceived = false;        // sudah pernah menerima pesan boot (main hidup)
unsigned long bootPageEnterMs = 0;   // waktu masuk PAGE_BOOT (safety no-signal)
unsigned long bootOfflineMs = 0;     // waktu tombol offline muncul (auto-lanjut)
// Auto-lanjut ke dashboard bila layar offline tak ditekan (antisipasi auto-restart tanpa penjaga),
// dan bila main sama sekali bisu (UART putus) supaya tak hang selamanya.
#define BOOT_OFFLINE_AUTO_MS 30000UL
#define BOOT_NO_SIGNAL_MS 25000UL

struct SensorData
{
	float ph = 6.8;
	float tempAir = 28.0;
	float tempWater = 24.0;
	float waterLvl = 82; // persen
	float nutrisi = 1.4; // EC
	float humidity = 65; // %
};

SensorData sensor;

float phMin = 6.0, phMax = 7.5;
float tAirMin = 20.0, tAirMax = 35.0;
float tWaterMin = 18.0, tWaterMax = 30.0;
float nutrisiMin = 400, nutrisiMax = 1200; // PPM — synced from Firebase ppmMin/ppmMax via update_sensorLimit
float waterMin = 20.0, waterMax = 90.0;
float humidityMin = 40, humidityMax = 80;

// Display units — synced from ESP32-MAIN via "sensor_unit" (see handleMaster()).
// Only unitTempAir/unitTempWater/unitNutrisi are actually configurable server-side;
// unitPh/unitWaterLvl/unitHumidity are fixed but kept as variables for consistency.
String unitPh = "pH";
String unitTempAir = "C";
String unitTempWater = "C";
String unitWaterLvl = "%";
String unitNutrisi = "PPM";
String unitHumidity = "%";

// ── Waktu (simulasi)
unsigned long lastUpdate = 0;

Button *activeButtons[10];
int activeButtonCount = 0;

void setActiveButtons(Button **btns, int count)
{
	activeButtonCount = count;
	for (int i = 0; i < count; i++)
		activeButtons[i] = btns[i];
}

int dashSubPage = 1;

// Dashboard
Button btnSetting;
Button btnPump;

// Manual control buttons
Button btnToSensor;
Button btnToControl;
Button btnPhUp;
Button btnPhDown;
Button btnNutA;
Button btnNutB;

// Setting menu → navigasi
Button btnBackFromSetting;
Button btnMenuWifi;
Button btnMenuLimit;
Button btnMenuCalib;
Button btnMenuScreenTimeout;
Button btnMenuReset;
Button btnMenuAbout;
Button btnBackFromCalib;
Button btnCalibPh;
Button btnCalibTds;
Button btnCalibPump;
Button btnBackFromCalibSub; // shared back button for the calibration sub-pages (one shown at a time)
Button btnCalibTdsGo;		// "Mulai" — jalankan jendela sampling
Button btnCalibTdsSave;		// "Simpan" — commit faktor hasil ukur
Button btnCalibTdsCancel;	// "Batal" — buang hasil & kembali
Button btnCalibPumpRun;		// "Jalankan" — putar pompa selama durasi terpilih
Button btnCalibPumpSave;	// "Simpan" — kirim volume terukur
Button btnCalibPumpCancel;	// "Batal" — batalkan & kembali
Button btnCalibPhMeasure;	// "Ukur" — sampling tegangan titik saat ini
Button btnCalibPhSave;		// "Simpan" — hitung & simpan slope/offset
Button btnCalibPhCancel;	// "Batal" — batalkan & kembali
Button btnBootOffline;		// "Lanjut Offline" di layar boot (muncul saat mode offline)

// WiFi
Button btnBackFromWifi;
Button btnWifiItem[4];
Button btnKbBack;
Button btnRefreshWifi;
Button btnDisconnect;

// Sensor Limit
Button btnBackFromLimit;

// Set Screen Timeout
Button btnBackFromSetScreen;
Button btnScreen15;
Button btnScreen30;
Button btnScreen60;
Button btnScreen120;
Button btnScreenOff;

// Reset
Button btnRestart;
Button btnBackFromReset;

// About
Button btnBackFromAbout;

unsigned long lastPongTime = 0;
bool waitingPong = false;

void showPage(Page page);
void drawDashboard();
void drawDashboardControl();
void drawDashboardSensor();
void updateDashboardValues();
void updateControlValues();
void drawControlBlock(int x, int y, int w, int h, const char *title, const char *valStr, const char *unit, bool isWarn, const char *statusText, uint16_t statusColor, bool hasBtns, Button *btnUp, Button *btnDn, const char *upLabel, const char *dnLabel);
void drawSetting();
void drawWifi();
void waitWifi(bool mode);
void connectingWifi(String ssidName);
void drawWifiPassword();
void setupEEPROM();
void handleMaster();
void handleKbTouch(int tx, int ty);
void refreshDashboardValues();
void drawSensorLimit();
void drawCalibration();
void drawCalibPh();
void drawCalibTds();
void drawCalibPump();
void redrawCalibRefField();
void redrawCalibTdsStatus();
void handleCalibTdsTouch(int tx, int ty);
void drawCalibTdsProgress(int pct);
void handleCalibPumpTouch(int tx, int ty);
void drawPumpSelectors();
void drawPumpMlField();
void redrawPumpStatus();
void handleCalibPhTouch(int tx, int ty);
void drawPhBuffers();
void redrawPhStatus();
void redrawPhSummary();
void redrawPhLive();
void drawBoot();
void redrawBootProgress();
void drawScreenTimeout();
void drawAbout();
void drawRestart();
void redrawPasswordField();
void drawKeyboard();
void setMode(uint8_t value);
uint8_t readModeFromEEPROM();

void setup()
{
	delay(1000);
	Serial.begin(115200);
	Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // must match ESP32-MAIN's Serial2 baud
	espMaster.begin(Serial2);

	pinMode(TFT_CS, OUTPUT);
	digitalWrite(TFT_CS, HIGH);
	pinMode(TOUCH_CS, OUTPUT);
	digitalWrite(TOUCH_CS, HIGH);

	pinMode(TFT_BL, OUTPUT);
	analogWrite(TFT_BL, LED_BRIGHT);
	pinMode(TOUCH_IRQ, INPUT_PULLUP);

	pinMode(BUZZER, OUTPUT);
	digitalWrite(BUZZER, LOW);

	pinMode(SELECTOR_MAN, INPUT_PULLUP);
	pinMode(SELECTOR_AUT, INPUT_PULLUP);

	setupEEPROM();

	uint8_t savedMode = readModeFromEEPROM(); // sesuaikan dengan fungsi EEPROM-mu yang sebenarnya
	autoMode = (savedMode == 0);
	lastPump = autoMode;

	SPI.begin();
	tft.begin();
	tft.setRotation(1); // landscape

	tft.fillScreen(CLR_BG);
	tft.drawRGBBitmap(0, 0, Splash, 320, 240);

	touch.begin();
	touch.setRotation(3); // harus sama dengan tft.setRotation(3)
	touch.touched();

	randomSeed(analogRead(A0));
	delay(1500);

	// Mulai di layar boot; tampilkan progres main (WiFi/NTP/Firebase) sampai "ready"/"offline".
	showPage(PAGE_BOOT);
	espMaster.sendType("req_boot"); // minta status terkini (jika main sudah selesai boot)

	lastTouchTime = millis();
}

void loop()
{
	handleMaster();
	if (waitingPong && millis() - lastPongTime >= 1000)
	{
		waitingPong = false;
		espMaster.sendType("pong");
	}

	// Guard layar boot — jangan pernah hang (antisipasi auto-restart tanpa penjaga):
	//  - layar offline tak ditekan dalam BOOT_OFFLINE_AUTO_MS  -> lanjut sendiri
	//  - main sama sekali bisu (UART putus) dalam BOOT_NO_SIGNAL_MS -> lanjut sendiri
	if (currentPage == PAGE_BOOT)
	{
		if (bootOfflineShown && millis() - bootOfflineMs > BOOT_OFFLINE_AUTO_MS)
			showPage(PAGE_DASHBOARD);
		else if (!bootMsgReceived && millis() - bootPageEnterMs > BOOT_NO_SIGNAL_MS)
			showPage(PAGE_DASHBOARD);
	}

	// Timeout kalibrasi TDS: window ~5s; kalau tak ada respons main dalam 12s, jangan
	// menggantung di progress — tampilkan pesan dan reset supaya bisa dicoba lagi.
	if (tdsCalibInProgress && millis() - tdsCalibStartMs > 12000UL)
	{
		tdsCalibInProgress = false;
		tdsCalibSuccess = false;
		tdsCalibHasResult = false;
		calibTdsStatus = "Timeout, coba lagi";
		if (currentPage == PAGE_CALIB_TDS)
			redrawCalibTdsStatus();
	}

	// Hitung mundur kalibrasi pompa + timeout kalau main tak mengirim "done".
	if (pumpCalibRunning)
	{
		unsigned long elapsed = millis() - pumpRunStartMs;
		if (elapsed > pumpRunDurMs + 6000UL)
		{
			pumpCalibRunning = false;
			pumpCalibSuccess = false;
			pumpCalibStatus = "Timeout, coba lagi";
			if (currentPage == PAGE_CALIB_PUMP)
				redrawPumpStatus();
		}
		else
		{
			int remain = (elapsed >= pumpRunDurMs) ? 0 : (int)((pumpRunDurMs - elapsed + 999) / 1000);
			if (remain != pumpLastCountShown)
			{
				pumpLastCountShown = remain;
				pumpCalibStatus = (remain > 0) ? ("Berputar " + String(remain) + "s") : "Selesai, tunggu...";
				if (currentPage == PAGE_CALIB_PUMP)
					redrawPumpStatus();
			}
		}
	}

	// Timeout sampling pH (mirip TDS).
	if (phCalInProgress && millis() - phCalStartMs > 12000UL)
	{
		phCalInProgress = false;
		phCalSuccess = false;
		phCalStatus = "Timeout, coba lagi";
		if (currentPage == PAGE_CALIB_PH)
			redrawPhStatus();
	}

	bool isTouching = touch.touched();
	int tx = 0, ty = 0;

	static bool manPrev = HIGH;
	bool manNow = digitalRead(SELECTOR_MAN);
	if (manPrev == HIGH && manNow == LOW)
	{
		setMode(1);
	}
	manPrev = manNow;

	static bool autPrev = HIGH;
	bool autNow = digitalRead(SELECTOR_AUT);
	if (autPrev == HIGH && autNow == LOW)
	{
		setMode(0);
	}
	autPrev = autNow;

	if (digitalRead(TOUCH_IRQ) == LOW)
	{
		isTouching = touch.touched();
		if (isTouching)
		{
			lastTouchTime = millis();

			if (!ledOn)
			{
				ledOn = true;
				ledEnd = false;
				analogWrite(TFT_BL, LED_BRIGHT);
				delay(50);
				return;
			}

			TS_Point p = touch.getPoint();
			tx = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, SCREEN_W);
			ty = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, SCREEN_H);
			tx = constrain(tx, 0, SCREEN_W);
			ty = constrain(ty, 0, SCREEN_H);
		}
	}

	if (!screenTimeoutOptions[4] && ledOn && millis() - lastTouchTime > SCREEN_TIMEOUT)
	{
		ledOn = false;
		ledEnd = true;
		lastTouchTimeEnd = millis();
		Serial.println("Layar redup karena timeout");
		analogWrite(TFT_BL, LED_DIM);
	}

	if (ledEnd && millis() - lastTouchTimeEnd > SCREEN_TIMEOUT_END)
	{
		ledEnd = false;
		Serial.println("Layar mati karena tidak aktif");
		analogWrite(TFT_BL, 0);
	}

	static bool kbTapped = false;
	static bool wifiTapped = false;
	static bool calibTapped = false;
	static int lastPage = -1;

	if (currentPage != lastPage)
	{
		lastPage = currentPage;
		kbTapped = false;
		wifiTapped = false;
	}

	if (!isTouching)
	{
		kbTapped = false;
		wifiTapped = false;
		calibTapped = false;
	}

	if (ledOn)
	{
		for (int i = 0; i < activeButtonCount; i++)
		{
			activeButtons[i]->handleTouch(tx, ty, isTouching);
		}

		if (currentPage == PAGE_WIFI_PASSWORD && isTouching && !kbTapped)
		{
			kbTapped = true;
			handleKbTouch(tx, ty);
		}

		if (currentPage == PAGE_CALIB_TDS && isTouching && !calibTapped)
		{
			calibTapped = true;
			handleCalibTdsTouch(tx, ty);
		}

		if (currentPage == PAGE_CALIB_PUMP && isTouching && !calibTapped)
		{
			calibTapped = true;
			handleCalibPumpTouch(tx, ty);
		}

		if (currentPage == PAGE_CALIB_PH && isTouching && !calibTapped)
		{
			calibTapped = true;
			handleCalibPhTouch(tx, ty);
		}

		if (currentPage == PAGE_WIFI)
		{
			if (waitRespons && millis() - wifiTimeout > 15000)
			{
				waitRespons = false;
				waitWifi(false);
			}

			if (isTouching && !wifiTapped && !scanWifi)
			{
				for (int i = 0; i < NET_COUNT; i++)
				{
					int ry = 100 + i * 30;
					// Empty slots (blank SSID) are tappable rows too — on the "no networks"
					// screen that would open the password keyboard for a nameless network.
					if (ty >= ry && ty <= ry + 29 && !wifiList[i].connected &&
						wifiList[i].ssid.length() > 0)
					{
						wifiTapped = true;
						strncpy(wifiInputSSID, wifiList[i].ssid.c_str(), 31);
						wifiInputSSID[31] = '\0';
						wifiPassword[0] = '\0';
						wifiPasswordLen = 0;
						t9LastBtn = -1;
						t9CharIdx = 0;
						showPage(PAGE_WIFI_PASSWORD);
						break;
					}
				}
			}
		}
	}

	if (currentPage == PAGE_DASHBOARD &&
		millis() - lastUpdate > 10000)
	{
		lastUpdate = millis();
		// updateSensors() (random simulation) is no longer called here — ESP32-MAIN now
		// pushes real sensor data over UART every ~10s (its LCD_PUSH_MS), and handleMaster()'s
		// "update_sensor" case already redraws on arrival. This periodic block just re-draws
		// with whatever `sensor` currently holds, as a fallback heartbeat for a missed push.
		// It shares that 10s period, so in the healthy case it is redundant by design — it is
		// a backstop, not a second source of truth.
		refreshDashboardValues();
	}

	delay(20);
}

// ── Layar boot: logo UNNES + judul + progress bar (atau tombol Lanjut Offline) ──
// Diperbarui oleh pesan UART "boot" dari main. Pindah ke dashboard saat "ready", saat user
// menekan "Lanjut Offline", atau saat timeout keamanan (lihat guard di loop()).
void redrawBootProgress()
{
	// Area di bawah judul (progress / offline). Bersihkan dulu.
	tft.fillRect(0, 150, SCREEN_W, 70, CLR_SURFACE);

	if (bootOfflineShown)
	{
		const char *msg = "Mode Offline - WiFi tak tersambung";
		tft.setFont(&Poppins_Medium5pt7b);
		tft.setTextColor(CLR_WARN);
		tft.setCursor((SCREEN_W - (int)strlen(msg) * 6) / 2, 165);
		tft.print(msg);
		tft.setFont();
		btnBootOffline = Button(90, 182, 140, 30, "Lanjut Offline >", CLR_OK, CLR_HEADER_TEXT, CLR_OK);
		btnBootOffline.setOnClickListener([]()
										  { showPage(PAGE_DASHBOARD); });
		btnBootOffline.draw();
		Button *btns[] = {&btnBootOffline};
		setActiveButtons(btns, 1);
		return;
	}

	// Teks langkah sekarang (tengah).
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_TEXT);
	tft.setCursor((SCREEN_W - (int)bootText.length() * 6) / 2, 165);
	tft.print(bootText);
	tft.setFont();

	// Progress bar.
	const int x = 40, y = 185, w = SCREEN_W - 80, h = 16;
	tft.drawRoundRect(x, y, w, h, 4, CLR_BORDER);
	tft.fillRect(x + 1, y + 1, w - 2, h - 2, CLR_CARD);
	int fillW = (w - 2) * bootPct / 100;
	if (fillW > 0)
		tft.fillRect(x + 1, y + 1, fillW, h - 2, CLR_OK);
	char pctBuf[8];
	snprintf(pctBuf, sizeof(pctBuf), "%d%%", bootPct);
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_HINT);
	tft.setCursor((SCREEN_W - (int)strlen(pctBuf) * 6) / 2, y + h + 12);
	tft.print(pctBuf);
	tft.setFont();
}

void drawBoot()
{
	bootPageEnterMs = millis();
	tft.fillScreen(CLR_SURFACE);
	tft.drawRGBBitmap(130, 30, unnes, 60, 80); // logo UNNES (sama seperti halaman Info)

	const char *title = "Hydro-IoT Control System";
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_TEXT);
	tft.setCursor((SCREEN_W - (int)strlen(title) * 6) / 2, 128);
	tft.print(title);
	tft.setFont();

	setActiveButtons(nullptr, 0); // belum ada tombol sampai mode offline
	redrawBootProgress();
}

// Pages
void showPage(Page page)
{
	currentPage = page;
	switch (page)
	{
	case PAGE_BOOT:
		drawBoot();
		break;
	case PAGE_DASHBOARD:
		drawDashboard();
		break;
	case PAGE_SETTING:
		drawSetting();
		break;
	case PAGE_WIFI:
		waitWifi(true);
		break;
	case PAGE_WIFI_PASSWORD:
		drawWifiPassword();
		break;
	case PAGE_SENSOR_LIMIT:
		drawSensorLimit();
		break;
	case PAGE_CALIBRATION:
		drawCalibration();
		break;
	case PAGE_CALIB_PH:
		drawCalibPh();
		break;
	case PAGE_CALIB_TDS:
		drawCalibTds();
		break;
	case PAGE_CALIB_PUMP:
		drawCalibPump();
		break;
	case PAGE_SCREEN_TIMEOUT:
		drawScreenTimeout();
		break;
	case PAGE_ABOUT:
		drawAbout();
		break;
	case PAGE_RESET:
		drawRestart();
		break;
	}
}

// ── Gambar sub-page sensor (dipakai di auto & manual) ────────
void drawDashboardSensor()
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Hydro-IoT Control System");

	btnSetting = Button(SCREEN_W - 58, 22, 54, 22, "Setting",
						0x14BF, CLR_HEADER_TEXT, 0x14BF);
	btnSetting.setOnClickListener([]()
								  { showPage(PAGE_SETTING); });
	btnSetting.draw();

	int cw = 100, ch = 54, gap = 5;
	int startX = 4, startY = 50;
	char buf[12];

	dtostrf(sensor.ph, 4, 1, buf);
	bool phWarn = sensor.ph < phMin || sensor.ph > phMax;
	drawSensorCard(startX, startY, cw, ch, "pH Air", buf, unitPh.c_str(),
				   constrain((int)((sensor.ph - phMin) / (phMax - phMin) * 100), 0, 100), CLR_OK, phWarn);

	dtostrf(sensor.tempAir, 4, 1, buf);
	bool taWarn = sensor.tempAir > tAirMax;
	drawSensorCard(startX + cw + gap, startY, cw, ch, "Suhu Udara", buf, unitTempAir.c_str(),
				   constrain((int)((sensor.tempAir - tAirMin) / (tAirMax - tAirMin) * 100), 0, 100), CLR_AMBER, taWarn);

	dtostrf(sensor.tempWater, 4, 1, buf);
	bool twWarn = sensor.tempWater > tWaterMax;
	drawSensorCard(startX + (cw + gap) * 2, startY, cw, ch, "Suhu Air", buf, unitTempWater.c_str(),
				   constrain((int)((sensor.tempWater - tWaterMin) / (tWaterMax - tWaterMin) * 100), 0, 100), CLR_BLUE, twWarn);

	dtostrf(sensor.waterLvl, 4, 1, buf);
	bool wlWarn = sensor.waterLvl > waterMax || sensor.waterLvl < waterMin;
	drawSensorCard(startX, startY + ch + gap, cw, ch, "Kapasitas Air", buf, unitWaterLvl.c_str(),
				   constrain((int)sensor.waterLvl, 0, 100), CLR_WARN, wlWarn);

	dtostrf(sensor.nutrisi, 4, 0, buf);
	bool nuWarn = sensor.nutrisi < nutrisiMin || sensor.nutrisi > nutrisiMax;
	drawSensorCard(startX + cw + gap, startY + ch + gap, cw, ch, "Nutrisi", buf, unitNutrisi.c_str(),
				   constrain((int)((sensor.nutrisi / nutrisiMax) * 100), 0, 100), CLR_OK, nuWarn);

	dtostrf(sensor.humidity, 3, 0, buf);
	bool huWarn = sensor.humidity < humidityMin || sensor.humidity > humidityMax;
	drawSensorCard(startX + (cw + gap) * 2, startY + ch + gap, cw, ch, "Humidity", buf, unitHumidity.c_str(),
				   constrain((int)sensor.humidity, 0, 100), CLR_AMBER, huWarn);

	if (!autoMode)
	{
		// Mode manual: di halaman sensor, tampilkan tombol balik ke kontrol
		// Gunakan status strip sebagai area tombol kembali
		tft.fillRect(0, SCREEN_H - 25, SCREEN_W, 16, CLR_STATUS_BG);
		tft.drawFastHLine(0, SCREEN_H - 25, SCREEN_W, CLR_BORDER);

		drawStatusStrip(pumpOn, wifiOK, autoMode);
		// Tombol ke kontrol di kanan
		tft.fillRoundRect(SCREEN_W - 76, SCREEN_H - 22, 74, 19, 3, CLR_HEADER);
		tft.setTextColor(CLR_HEADER_TEXT);
		tft.setCursor(SCREEN_W - 62, SCREEN_H - 16);
		tft.print("< Kontrol");
		tft.setFont();

		btnToControl = Button(SCREEN_W - 76, SCREEN_H - 24, 78, 22, "");
		btnToControl.setOnClickListener([]()
										{
      dashSubPage = 0;
      drawDashboard(); });

		Button *btns[] = {&btnSetting, &btnToControl};
		setActiveButtons(btns, 2);
	}
	else
	{
		drawStatusStrip(pumpOn, wifiOK, autoMode);
		Button *btns[] = {&btnSetting};
		setActiveButtons(btns, 1);
	}
}

// ── Update nilai sensor tanpa redraw penuh (sub-page sensor) ─
void updateDashboardValues()
{
	int cw = 100, ch = 54, gap = 5;
	int startX = 4, startY = 50;
	char buf[12];

	struct
	{
		float val;
		float minV, maxV;
		uint16_t barColor;
		bool warn;
		int decimals;
		int cx, cy;
	} cards[6] = {
		{sensor.ph, phMin, phMax, CLR_OK,
		 sensor.ph < phMin || sensor.ph > phMax, 1, startX, startY},
		{sensor.tempAir, tAirMin, tAirMax, CLR_AMBER,
		 sensor.tempAir > tAirMax, 1, startX + cw + gap, startY},
		{sensor.tempWater, tWaterMin, tWaterMax, CLR_BLUE,
		 sensor.tempWater > tWaterMax, 1, startX + (cw + gap) * 2, startY},
		{sensor.waterLvl, waterMin, waterMax, CLR_WARN,
		 sensor.waterLvl > waterMax || sensor.waterLvl < waterMin,
		 1, startX, startY + ch + gap},
		{sensor.nutrisi, nutrisiMin, nutrisiMax, CLR_OK,
		 sensor.nutrisi < nutrisiMin || sensor.nutrisi > nutrisiMax,
		 0, startX + cw + gap, startY + ch + gap},
		{sensor.humidity, humidityMin, humidityMax, CLR_AMBER,
		 sensor.humidity < humidityMin || sensor.humidity > humidityMax, 0, startX + (cw + gap) * 2, startY + ch + gap},
	};

	for (int i = 0; i < 6; i++)
	{
		int x = cards[i].cx, y = cards[i].cy;
		tft.fillRect(x + 4, y + 14, cw - 8, 18, CLR_CARD);
		tft.setFont(&Poppins_SemiBold10pt7b);
		tft.setTextColor(CLR_TEXT);
		tft.setTextSize(1);
		dtostrf(cards[i].val, 1, cards[i].decimals, buf);
		tft.setCursor(x + 6, y + 16 + 13);
		tft.print(buf);
		tft.setFont();

		int barX = x + 5, barY = y + ch - 7, barW = cw - 10;
		tft.fillRoundRect(barX, barY, barW, 4, 2, CLR_BORDER);
		int pct = (i == 3) ? constrain((int)cards[i].val, 0, 100)
						   : constrain((int)((cards[i].val - cards[i].minV) / (cards[i].maxV - cards[i].minV) * 100), 0, 100);
		int fill = (pct * barW) / 100;
		if (fill > 0)
			tft.fillRoundRect(barX, barY, fill, 4, 2, cards[i].barColor);
		tft.fillRect(x, y + 4 - 4, 3, ch - 8 + 8, cards[i].warn ? CLR_WARN_STRIP : CLR_OK);
	}
}

// ── Gambar satu blok kontrol (pompa / pH / nutrisi) ──────────
void drawControlBlock(int x, int y, int w, int h,
					  const char *title, const char *valStr, const char *unit,
					  bool isWarn, const char *statusText,
					  uint16_t statusColor, bool hasBtns,
					  Button *btnUp, Button *btnDn,
					  const char *upLabel, const char *dnLabel)
{
	tft.fillRoundRect(x, y, w, h, 5, CLR_CARD);
	tft.drawRoundRect(x, y, w, h, 5, CLR_BORDER);

	// Title
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_MUTED);
	tft.setTextSize(1);
	tft.setCursor(x + 7, y + 8 + 5);
	tft.print(title);

	if (hasBtns)
	{
		// Nilai di tengah kiri
		tft.setFont(&Poppins_SemiBold10pt7b);
		tft.setTextColor(isWarn ? CLR_WARN : CLR_TEXT);
		tft.setTextSize(1);
		tft.setCursor(x + 7, y + 26 + 5);
		tft.print(valStr);

		// Satuan
		tft.setFont(&Poppins_Medium5pt7b);
		tft.setTextColor(CLR_HINT);
		int vw = strlen(valStr) * 12 + 10;
		tft.setCursor(x + 7 + vw, y + 22);
		tft.print(unit);

		// Status
		tft.setTextColor(statusColor);
		tft.setCursor(x + 7, y + h - 10);
		tft.print(statusText);
		tft.setFont();

		// Tombol UP kanan atas
		int bx = x + w - 52;
		int bw = 46, bh = (h - 10) / 2;
		tft.fillRoundRect(bx, y + 4, bw, bh, 4, rgb565(235, 245, 255));
		tft.drawRoundRect(bx, y + 4, bw, bh, 4, CLR_BLUE);
		tft.fillRoundRect(bx, y + 4 + bh + 2, bw, bh, 4, rgb565(255, 242, 235));
		tft.drawRoundRect(bx, y + 4 + bh + 2, bw, bh, 4, CLR_WARN);

		tft.setFont();
		tft.setTextSize(1);
		tft.setTextColor(CLR_BLUE);
		tft.setCursor(bx + (bw - strlen(upLabel) * 6) / 2, y + 4 + (bh - 8) / 2);
		tft.print(upLabel);
		tft.setTextColor(CLR_WARN);
		tft.setCursor(bx + (bw - strlen(dnLabel) * 6) / 2, y + 4 + bh + 2 + (bh - 8) / 2);
		tft.print(dnLabel);

		if (btnUp)
		{
			*btnUp = Button(bx, y + 4, bw, bh, "");
			*btnDn = Button(bx, y + 4 + bh + 2, bw, bh, "");
		}
	}
}

// Repaint a dosing-pump toggle button to show ON (green) / OFF (neutral). Uses the button's own
// geometry so it overpaints exactly what drawControlBlock laid out.
void drawDoseButton(Button &b, const char *label, bool on)
{
	uint16_t bg = on ? CLR_OK : CLR_CARD;
	uint16_t fg = on ? CLR_HEADER_TEXT : CLR_TEXT;
	uint16_t br = on ? CLR_OK : CLR_BORDER;
	tft.fillRoundRect(b.getX(), b.getY(), b.getW(), b.getH(), 4, bg);
	tft.drawRoundRect(b.getX(), b.getY(), b.getW(), b.getH(), 4, br);
	tft.setFont();
	tft.setTextSize(1);
	tft.setTextColor(fg);
	tft.setCursor(b.getX() + (b.getW() - (int)strlen(label) * 6) / 2, b.getY() + (b.getH() - 8) / 2);
	tft.print(label);
}

// ── Update nilai pH & nutrisi di halaman kontrol ─────────────
// drawControlBlock(4, y1, hw, 70,
void updateControlValues()
{
	// pH blok: x=4, y=84, w=152, h=56
	int y0 = 48;
	int fullW = SCREEN_W - 8;
	int y1 = y0 + 46 + 4;
	int hw = (fullW - 4) / 2;

	char buf[10];
	bool phWarn = sensor.ph < phMin || sensor.ph > phMax;

	tft.fillRect(11, y1 + 10 + 5, 55, 20, CLR_CARD);
	tft.setFont(&Poppins_SemiBold10pt7b);
	tft.setTextColor(phWarn ? CLR_WARN : CLR_TEXT);
	tft.setTextSize(1);
	dtostrf(sensor.ph, 4, 1, buf);
	tft.setCursor(11, y1 + 26 + 5);
	tft.print(buf);

	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(phWarn ? CLR_WARN : CLR_OK);
	tft.fillRect(10, y1 + 50, 95, 12, CLR_CARD);
	tft.setCursor(10, y1 + 70 - 10);
	tft.print(phWarn ? "Warning!!" : "Normal");
	tft.setFont();

	// Nutrisi blok: x=162, y=84, w=152, h=56
	bool nuWarn = sensor.nutrisi < nutrisiMin || sensor.nutrisi > nutrisiMax;

	tft.fillRect(168, y1 + 10 + 5, 55, 20, CLR_CARD);
	tft.setFont(&Poppins_SemiBold10pt7b);
	tft.setTextColor(nuWarn ? CLR_WARN : CLR_TEXT);
	tft.setTextSize(1);
	dtostrf(sensor.nutrisi, 4, 0, buf);
	tft.setCursor(168, y1 + 26 + 5);
	tft.print(buf);

	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(nuWarn ? CLR_WARN : CLR_OK);
	tft.fillRect(168, y1 + 50, 95, 12, CLR_CARD);
	tft.setCursor(168, y1 + 70 - 10);
	tft.print(nuWarn ? "Warning!!" : "Normal");
	tft.setFont();
}

// ── Gambar sub-page kontrol (mode manual) ────────────────────
void drawDashboardControl()
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Hydro-IoT Control System");

	btnSetting = Button(SCREEN_W - 58, 22, 54, 22, "Setting",
						0x14BF, CLR_HEADER_TEXT, 0x14BF);
	btnSetting.setOnClickListener([]()
								  { showPage(PAGE_SETTING); });
	btnSetting.draw();

	// ── Baris 1: Pompa Air (full width, tinggi 46px) ──────────
	int y0 = 48;
	int fullW = SCREEN_W - 8;

	tft.fillRoundRect(4, y0, fullW, 46, 5, CLR_CARD);
	tft.drawRoundRect(4, y0, fullW, 46, 5, CLR_BORDER);

	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_MUTED);
	tft.setTextSize(1);
	tft.setCursor(12, y0 + 15);
	tft.print("Pompa Air");

	// Status pompa
	tft.setTextColor(pumpOn ? CLR_OK : CLR_HINT);
	tft.setCursor(12, y0 + 27);
	tft.print(pumpOn ? "Menyala" : "Mati");
	tft.setFont();

	// Toggle switch pompa
	pumpButton(fullW - 60, y0, pumpOn, "");
	btnPump = Button(fullW - 60, y0 - 8, 50, 30, "");
	btnPump.setOnClickListener([]()
							   {
    pumpOn = !pumpOn;
    espMaster.send("pump_btn", "state", pumpOn);
    // Update tampilan pompa saja tanpa redraw penuh
    tft.fillRoundRect(4, 48, SCREEN_W-8, 46, 5, CLR_CARD);
    tft.drawRoundRect(4, 48, SCREEN_W-8, 46, 5, CLR_BORDER);
    tft.setFont(&Poppins_Medium5pt7b);
    tft.setTextColor(CLR_MUTED); tft.setTextSize(1);
    tft.setCursor(12, 48 + 15);    tft.print("Pompa Air");
    tft.setTextColor(pumpOn ? CLR_OK : CLR_HINT);
    tft.setCursor(12, 48 + 27);    tft.print(pumpOn ? "Menyala" : "Mati");
    tft.setFont();
    pumpButton(SCREEN_W-68, 48, pumpOn, "");
    drawStatusStrip(pumpOn, wifiOK, autoMode);
    tft.fillRoundRect(SCREEN_W-76, SCREEN_H-22, 74, 19, 3, CLR_HEADER);
    tft.setTextColor(CLR_HEADER_TEXT);
    tft.setCursor(SCREEN_W-62, SCREEN_H-16);
    tft.print("Sensor >");
    tft.setFont(); });

	// ── Baris 2: pH Air (setengah kiri) ──────────────────────
	int y1 = y0 + 46 + 4;
	int hw = (fullW - 4) / 2; // half width ~154px

	char phBuf[10], nuBuf[10];
	dtostrf(sensor.ph, 4, 1, phBuf);
	dtostrf(sensor.nutrisi, 4, 0, nuBuf);
	bool phWarn = sensor.ph < phMin || sensor.ph > phMax;
	bool nuWarn = sensor.nutrisi < nutrisiMin || sensor.nutrisi > nutrisiMax;

	// pH blok kiri
	drawControlBlock(4, y1, hw, 70,
					 "pH Air", phBuf, unitPh.c_str(),
					 phWarn, phWarn ? "Warning!" : "Normal",
					 phWarn ? CLR_WARN : CLR_OK,
					 true, &btnPhUp, &btnPhDown, "pH+", "pH-");

	// Tombol kini SAKLAR: tekan = nyala terus, tekan lagi = mati (kirim state ON/OFF ke main).
	btnPhUp.setOnClickListener([]()
							   {
    doseOn[0] = !doseOn[0];
    espMaster.send("phUp_btn", "state", doseOn[0]);
    drawDoseButton(btnPhUp, "pH+", doseOn[0]); });
	btnPhDown.setOnClickListener([]()
								 {
    doseOn[1] = !doseOn[1];
    espMaster.send("phDown_btn", "state", doseOn[1]);
    drawDoseButton(btnPhDown, "pH-", doseOn[1]); });
	drawDoseButton(btnPhUp, "pH+", doseOn[0]);
	drawDoseButton(btnPhDown, "pH-", doseOn[1]);

	// Nutrisi blok kanan — dua pompa terpisah (Nutrisi A & B), bukan naik/turun
	drawControlBlock(4 + hw + 4, y1, hw, 70,
					 "Nutrisi", nuBuf, unitNutrisi.c_str(),
					 nuWarn, nuWarn ? "Di luar batas!" : "Normal",
					 nuWarn ? CLR_WARN : CLR_OK,
					 true, &btnNutA, &btnNutB, "Nut A", "Nut B");

	btnNutA.setOnClickListener([]()
							   {
    doseOn[2] = !doseOn[2];
    espMaster.send("nutrisiA_btn", "state", doseOn[2]);
    drawDoseButton(btnNutA, "Nut A", doseOn[2]); });
	btnNutB.setOnClickListener([]()
							   {
    doseOn[3] = !doseOn[3];
    espMaster.send("nutrisiB_btn", "state", doseOn[3]);
    drawDoseButton(btnNutB, "Nut B", doseOn[3]); });
	drawDoseButton(btnNutA, "Nut A", doseOn[2]);
	drawDoseButton(btnNutB, "Nut B", doseOn[3]);

	drawStatusStrip(pumpOn, wifiOK, autoMode);

	// Tombol ke sensor di kanan
	tft.fillRoundRect(SCREEN_W - 76, SCREEN_H - 22, 74, 19, 3, CLR_HEADER);
	tft.setTextColor(CLR_HEADER_TEXT);
	tft.setCursor(SCREEN_W - 62, SCREEN_H - 16);
	tft.print("Sensor >");
	tft.setFont();

	btnToSensor = Button(SCREEN_W - 76, SCREEN_H - 24, 78, 22, "");
	btnToSensor.setOnClickListener([]()
								   {
    dashSubPage = 1;
    drawDashboard(); });

	Button *btns[] = {
		&btnSetting, &btnPump,
		&btnPhUp, &btnPhDown,
		&btnNutA, &btnNutB,
		&btnToSensor};
	setActiveButtons(btns, 7);
}

// Dashboard: dispatcher berdasarkan mode & subpage
void drawDashboard()
{
	if (autoMode)
	{
		dashSubPage = 0; // auto selalu sensor
		drawDashboardSensor();
	}
	else
	{
		// Manual: pilih sub-page
		if (dashSubPage == 0)
		{
			drawDashboardControl();
		}
		else
		{
			drawDashboardSensor();
		}
	}
}

// Setting
void drawSetting()
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Pengaturan", true);

	btnBackFromSetting = Button(4, 22, 60, 22, "< Kembali",
								CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromSetting.setOnClickListener([]()
										  { 
    dashSubPage = 1;
    showPage(PAGE_DASHBOARD); });

	// 6 rows now (added Kalibrasi) — shorter cards (h=30, pitch=31) so all fit on the 240px screen.
	const int y0 = 46, dy = 31, ih = 30;
	drawMenuItem(y0 + dy * 0, wifi, "WiFi", "Kelola koneksi WiFi", ih);
	drawMenuItem(y0 + dy * 1, sensorLimit, "Batas Sensor", "pH, suhu, ketinggian, PPM, Humidity", ih);
	drawMenuItem(y0 + dy * 2, sensorLimit, "Kalibrasi", "Kalibrasi sensor pH & PPM", ih);
	drawMenuItem(y0 + dy * 3, screenTimeout, "Layar Timeout", "Pengaturan timeout layar", ih);
	drawMenuItem(y0 + dy * 4, restart, "Restart", "Mulai ulang sistem", ih);
	drawMenuItem(y0 + dy * 5, info, "Info", "Info tentang alat dan pembuatnya", ih);

	// Invisible touch targets tile the rows contiguously (height = pitch, no dead gaps).
	btnMenuWifi = Button(4, y0 + dy * 0, SCREEN_W - 8, dy, "", CLR_BG, CLR_BG, CLR_BG);
	btnMenuLimit = Button(4, y0 + dy * 1, SCREEN_W - 8, dy, "", CLR_BG, CLR_BG, CLR_BG);
	btnMenuCalib = Button(4, y0 + dy * 2, SCREEN_W - 8, dy, "", CLR_BG, CLR_BG, CLR_BG);
	btnMenuScreenTimeout = Button(4, y0 + dy * 3, SCREEN_W - 8, dy, "", CLR_BG, CLR_BG, CLR_BG);
	btnMenuReset = Button(4, y0 + dy * 4, SCREEN_W - 8, dy, "", CLR_BG, CLR_BG, CLR_BG);
	btnMenuAbout = Button(4, y0 + dy * 5, SCREEN_W - 8, dy, "", CLR_BG, CLR_BG, CLR_BG);

	btnMenuWifi.setOnClickListener([]()
								   {
    showPage(PAGE_WIFI);

    waitRespons = true;
    scanWifi    = true;
    espMaster.sendType("scan_wifi");
    wifiTimeout = millis(); });
	btnMenuLimit.setOnClickListener([]()
									{ showPage(PAGE_SENSOR_LIMIT); });
	btnMenuCalib.setOnClickListener([]()
									{ showPage(PAGE_CALIBRATION); });
	btnMenuScreenTimeout.setOnClickListener([]()
											{ showPage(PAGE_SCREEN_TIMEOUT); });
	btnMenuReset.setOnClickListener([]()
									{ showPage(PAGE_RESET); });
	btnMenuAbout.setOnClickListener([]()
									{ showPage(PAGE_ABOUT); });
	Button *btns[] = {
		&btnBackFromSetting,
		&btnMenuWifi, &btnMenuLimit, &btnMenuCalib,
		&btnMenuScreenTimeout, &btnMenuReset, &btnMenuAbout};
	setActiveButtons(btns, 7);
}

// WiFi
void drawWifi()
{
	wifiOK = wifiList[0].connected;
	wifiSSID = wifiList[0].ssid;

	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Pengaturan WiFi", true);

	btnBackFromWifi = Button(4, 22, 60, 22, "< Kembali", CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromWifi.setOnClickListener([]()
									   { showPage(PAGE_SETTING); });

	// ── Status koneksi saat ini ──────────────────────────────
	tft.setFont(&Poppins_Medium5pt7b);
	tft.fillRect(0, 46, SCREEN_W, 28, CLR_OK_BG);
	tft.drawFastHLine(0, 46, SCREEN_W, CLR_OK_BORDER);
	tft.drawFastHLine(0, 46 + 28, SCREEN_W, CLR_OK_BORDER);

	if (wifiOK)
	{
		tft.setTextColor(CLR_OK);
		tft.setTextSize(1);
		tft.setCursor(8, 52 + 5);
		tft.print("TERHUBUNG");
		tft.setTextColor(CLR_TEXT);
		tft.setCursor(8, 63 + 5);
		tft.print(wifiSSID.c_str());
		tft.setTextColor(CLR_HINT);
		tft.setCursor(255, 63 + 5);
		tft.print("192.168.1.105");
	}
	else
	{
		// Tidak terhubung → tampilkan placeholder
		tft.setTextColor(CLR_WARN);
		tft.setTextSize(1);
		tft.setCursor(8, 52 + 5);
		tft.print("TIDAK TERHUBUNG");
		tft.setTextColor(CLR_HINT);
		tft.setCursor(8, 63 + 5);
		tft.print("Pilih jaringan di bawah untuk terhubung ke internet");
	}

	// ── Judul daftar jaringan ────────────────────────────────
	tft.setTextColor(CLR_HINT);
	tft.setTextSize(1);
	int lx = (SCREEN_W - (int)strlen("JARINGAN TERSEDIA") * 6) / 2;
	tft.setCursor(lx, 87 + 5);
	tft.print("JARINGAN TERSEDIA");
	hLine(98);

	// ── Daftar jaringan ──────────────────────────────────────
	for (int i = 0; i < NET_COUNT; i++)
	{
		int ry = 100 + i * 30;
		tft.fillRect(0, ry, SCREEN_W, 29, i % 2 == 0 ? CLR_CARD : CLR_SURFACE);
		hLine(ry + 29);

		// SSID
		tft.setTextColor(CLR_TEXT);
		tft.setTextSize(1);
		tft.setCursor(8, ry + 8 + 5);
		tft.print(wifiList[i].ssid);

		if (wifiList[i].connected)
		{
			// ── Jaringan yang terhubung ──────────────────────────
			tft.setTextColor(CLR_HEADER);
			tft.setTextSize(1);
			tft.setCursor(8, ry + 8 + 5);
			tft.print(wifiList[i].ssid);

			tft.setTextColor(CLR_HINT);
			tft.setCursor(8, ry + 19 + 5);
			tft.print("WPA2");

			// Tombol "Disconnect" di kiri, sinyal di kanan-nya
			btnDisconnect = Button(8 + 215, ry + 5, 68, 18, "", 0, 0, 0);
			btnDisconnect.setOnClickListener([]()
											 {
        espMaster.sendType("disconnect_wifi");
        wifiTimeout = millis();
        waitRespons = true;
        scanWifi    = true; });
			tft.fillRoundRect(8 + 215, ry + 5, 68, 18, 3, rgb565(255, 235, 235));
			tft.drawRoundRect(8 + 215, ry + 5, 68, 18, 3, CLR_WARN);
			tft.setTextColor(CLR_WARN);
			tft.setTextSize(1);
			tft.setCursor(14 + 211, ry + 15);
			tft.print("Disconnect");

			// Sinyal di kanan tombol disconnect
			for (int b = 0; b < 4; b++)
			{
				int bh = (b + 1) * 3;
				int bx = SCREEN_W - 22 + b * 5;
				int by = ry + 26 - bh;
				tft.fillRect(bx, by, 3, bh, b < wifiList[i].signal ? (uint16_t)CLR_BLUE : CLR_BORDER);
			}
		}
		else
		{
			// ── Jaringan tidak terhubung ─────────────────────────
			tft.setTextColor(CLR_HINT);
			tft.setCursor(8, ry + 19 + 5);
			tft.print("WPA2");

			// Sinyal di kanan
			for (int b = 0; b < 4; b++)
			{
				int bh = (b + 1) * 3;
				int bx = SCREEN_W - 22 + b * 5;
				int by = ry + 26 - bh;
				tft.fillRect(bx, by, 3, bh, b < wifiList[i].signal ? (uint16_t)CLR_BLUE : CLR_BORDER);
			}
		}

		// Tombol invisible per baris (hanya untuk yang belum konek)
		if (!wifiList[i].connected)
		{
			btnWifiItem[i] = Button(0, ry, SCREEN_W, 29, "", 0, 0, 0);
		}
	}

	Button *btns[] = {
		&btnBackFromWifi, &btnDisconnect,
		&btnWifiItem[0], &btnWifiItem[1], &btnWifiItem[2], &btnWifiItem[3]
		// index 0 (MyNetwork) tidak perlu — sudah konek
	};
	setActiveButtons(btns, 6);
}

void waitWifi(bool mode)
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Pengaturan WiFi", true);

	btnBackFromWifi = Button(4, 22, 60, 22, "< Kembali", CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromWifi.setOnClickListener([]()
									   { showPage(PAGE_SETTING); });

	if (!mode)
	{
		btnRefreshWifi = Button(4, 50, 300, 150, "", 0, 0, 0);
		btnRefreshWifi.setOnClickListener([]()
										  {
      showPage(PAGE_WIFI);
      // Arm the 15s timeout like every other scan requester (btnMenuWifi/btnKbBack/
      // btnDisconnect) — without it, a rescan that also comes back empty leaves the page
      // stuck on "Memindai jaringan..." with nothing to return it to this prompt.
      waitRespons = true;
      scanWifi    = true;
      espMaster.sendType("scan_wifi");
      wifiTimeout = millis(); });
	}

	String msg = mode ? "Memindai jaringan Wi-Fi . . ." : "Tidak ada jaringan, klik untuk memindai ulang";
	int x = (SCREEN_W - (int)strlen(msg.c_str()) * 6) / 2;
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_HINT);
	tft.setCursor(x + 20, 140);
	tft.print(msg);
	tft.setFont();

	Button *btns[] = {
		&btnBackFromWifi, &btnRefreshWifi};
	setActiveButtons(btns, 2);
}

void connectingWifi(String ssidName)
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Pengaturan WiFi", true);

	btnBackFromWifi = Button(4, 22, 60, 22, "< Kembali", CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromWifi.setOnClickListener([]()
									   { showPage(PAGE_SETTING); });

	String msg = "Menghubungkan ke jaringan " + ssidName;
	int x = (SCREEN_W - (int)strlen(msg.c_str()) * 6) / 2;
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_HINT);
	tft.setCursor(x + 5, 140);
	tft.print(msg);
	tft.setFont();

	Button *btns[] = {
		&btnBackFromWifi, &btnRefreshWifi};
	setActiveButtons(btns, 2);
}

void drawWifiPassword()
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Pengaturan WiFi", true);

	// Header
	tft.setFont(&Poppins_Medium5pt7b);
	tft.fillRect(0, 20, SCREEN_W, 26, CLR_HEADER);
	tft.setTextColor(CLR_HEADER_SUB);
	tft.setTextSize(1);
	tft.setCursor(6, 30 + 5);
	tft.print("< Kembali");
	tft.setTextColor(CLR_HEADER_TEXT);
	int tx = (SCREEN_W - (int)strlen(wifiInputSSID) * 6) / 2;
	tft.setCursor(tx, 30 + 5);
	tft.print(wifiInputSSID);
	tft.setFont();

	btnKbBack = Button(0, 20, 70, 26, "", CLR_HEADER, CLR_HEADER, CLR_HEADER);
	btnKbBack.setOnClickListener([]()
								 { 
    waitRespons = true;
    scanWifi    = true;
    espMaster.sendType("scan_wifi");
    wifiTimeout = millis();
    wifiPassword[0] = '\0';
    wifiPasswordLen = 0;
    t9LastBtn = -1;
    showPage(PAGE_WIFI); });

	redrawPasswordField();
	drawKeyboard();

	Button *btns[] = {&btnKbBack};
	setActiveButtons(btns, 1);
}

void parseWifi(String data, Wifi &wifi)
{
	int p1 = data.indexOf(';');
	int p2 = data.indexOf(';', p1 + 1);

	wifi.ssid = data.substring(0, p1);
	wifi.signal = data.substring(p1 + 1, p2).toInt();
	wifi.connected = data.substring(p2 + 1).toInt();
}

void sortNetworks()
{
	for (int i = 0; i < NET_COUNT - 1; i++)
	{
		for (int j = i + 1; j < NET_COUNT; j++)
		{

			bool swapNeeded = false;

			if (wifiList[j].connected && !wifiList[i].connected)
			{
				swapNeeded = true;
			}
			else if (wifiList[i].connected == wifiList[j].connected &&
					 wifiList[j].signal > wifiList[i].signal)
			{
				swapNeeded = true;
			}

			if (swapNeeded)
			{
				Wifi temp = wifiList[i];
				wifiList[i] = wifiList[j];
				wifiList[j] = temp;
			}
		}
	}
}

// Sensor Limit
void drawSensorLimit()
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Batas Sensor (Edit via App)", true);

	btnBackFromLimit = Button(4, 22, 60, 22, "< Kembali",
							  CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromLimit.setOnClickListener([]()
										{ showPage(PAGE_SETTING); });

	struct
	{
		const char *label;
		float minV, maxV, curV;
		const char *unit;
		uint16_t color;
	} limits[] = {
		{"pH Air", phMin, phMax, sensor.ph, unitPh.c_str(), CLR_OK},
		{"Suhu Udara", tAirMin, tAirMax, sensor.tempAir, unitTempAir.c_str(), CLR_AMBER},
		{"Suhu Air", tWaterMin, tWaterMax, sensor.tempWater, unitTempWater.c_str(), CLR_BLUE},
		{"Ketinggian", waterMin, waterMax, sensor.waterLvl, unitWaterLvl.c_str(), CLR_WARN},
		{"Nutrisi PPM", nutrisiMin, nutrisiMax, sensor.nutrisi, unitNutrisi.c_str(), CLR_OK},
		{"Humidity", humidityMin, humidityMax, sensor.humidity, unitHumidity.c_str(), CLR_AMBER},
	};

	char buf[20];
	for (int i = 0; i < 6; i++)
	{
		int ry = 48 + i * 30;
		tft.fillRect(0, ry, SCREEN_W, 29, i % 2 == 0 ? CLR_CARD : CLR_SURFACE);
		hLine(ry + 29);

		tft.setFont(&Poppins_Medium5pt7b);
		tft.setTextColor(CLR_TEXT);
		tft.setTextSize(1);
		tft.setCursor(8, ry + 6 + 5);
		tft.print(limits[i].label);

		// Min - Max
		tft.setFont(&Poppins_Medium5pt7b);
		tft.setTextColor(CLR_MUTED);
		snprintf(buf, sizeof(buf), "Min:%.1f  Max:%.1f %s",
				 limits[i].minV, limits[i].maxV, limits[i].unit);
		tft.setCursor(8, ry + 17 + 7);
		tft.print(buf);

		// Nilai sekarang
		bool warn = limits[i].curV < limits[i].minV || limits[i].curV > limits[i].maxV;
		tft.setFont(&Poppins_Medium5pt7b);
		tft.setTextColor(warn ? CLR_WARN : CLR_OK);
		snprintf(buf, sizeof(buf), "%.1f", limits[i].curV);
		tft.setCursor(SCREEN_W - strlen(buf) * 6 - 8, ry + 10 + 5);
		tft.print(buf);
		tft.setFont();
	}

	Button *btns[] = {&btnBackFromLimit};
	setActiveButtons(btns, 1);
}

void menuScreenTimeOut()
{
	int y = 50;
	drawMenuScreen(0, y, screenTimeoutOptions[0], "15 detik");
	drawMenuScreen(0, y + 30, screenTimeoutOptions[1], "30 detik");
	drawMenuScreen(0, y + 60, screenTimeoutOptions[2], "60 detik");
	drawMenuScreen(0, y + 90, screenTimeoutOptions[3], "120 detik");
	drawMenuScreen(0, y + 120, screenTimeoutOptions[4], "Selalu Nyala");
}

// Kalibrasi — submenu: pilih jenis kalibrasi (pH / TDS / Pompa peristaltik).
// Tiap pilihan membuka halaman kalibrasi masing-masing (isi detail menyusul).
void drawCalibration()
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Kalibrasi", true);

	btnBackFromCalib = Button(4, 22, 60, 22, "< Kembali",
							  CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromCalib.setOnClickListener([]()
										{ showPage(PAGE_SETTING); });

	// 3 pilihan — muat lega, pakai kartu tinggi normal (34).
	const int y0 = 54, dy = 40;
	drawMenuItem(y0 + dy * 0, sensorLimit, "Sensor pH", "Kalibrasi 2 titik (buffer pH)");
	drawMenuItem(y0 + dy * 1, sensorLimit, "Sensor TDS", "Kalibrasi 1 titik (larutan PPM)");
	drawMenuItem(y0 + dy * 2, screenTimeout, "Pompa Peristaltik", "Kalibrasi laju alir (mL/menit)");

	btnCalibPh = Button(4, y0 + dy * 0, SCREEN_W - 8, 34, "", CLR_BG, CLR_BG, CLR_BG);
	btnCalibTds = Button(4, y0 + dy * 1, SCREEN_W - 8, 34, "", CLR_BG, CLR_BG, CLR_BG);
	btnCalibPump = Button(4, y0 + dy * 2, SCREEN_W - 8, 34, "", CLR_BG, CLR_BG, CLR_BG);

	btnCalibPh.setOnClickListener([]()
								  { showPage(PAGE_CALIB_PH); });
	btnCalibTds.setOnClickListener([]()
								   { showPage(PAGE_CALIB_TDS); });
	btnCalibPump.setOnClickListener([]()
									{ showPage(PAGE_CALIB_PUMP); });

	Button *btns[] = {&btnBackFromCalib, &btnCalibPh, &btnCalibTds, &btnCalibPump};
	setActiveButtons(btns, 4);
}

// Helper: header + tombol kembali (ke submenu Kalibrasi) + teks placeholder yang dipusatkan.
// Dipakai ketiga halaman kalibrasi sampai isinya dibangun.
static void drawCalibPlaceholder(const char *title)
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader(title, true);

	btnBackFromCalibSub = Button(4, 22, 60, 22, "< Kembali",
								 CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromCalibSub.setOnClickListener([]()
										   { showPage(PAGE_CALIBRATION); });

	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_HINT);
	const char *msg = "Halaman kalibrasi akan ditambahkan";
	int x = (SCREEN_W - (int)strlen(msg) * 6) / 2;
	tft.setCursor(x, 130);
	tft.print(msg);
	tft.setFont();

	Button *btns[] = {&btnBackFromCalibSub};
	setActiveButtons(btns, 1);
}

// ── Kalibrasi pH (2 titik) ───────────────────────────────────────────────────
// pH sekarang (live) + pilih buffer standar + Ukur + ringkasan 2 titik + Simpan/Batal.
// Titik aktif = phPointsDone+1. Setelah 2 titik, main hitung slope/offset saat Simpan.
static const int PHB_X[4] = {4, 80, 156, 232};
#define PHB_Y 82
#define PHB_W 74
#define PHB_H 24

void drawPhBuffers()
{
	// Label titik aktif + baris tombol buffer (highlight yang terpilih).
	int which = (phPointsDone >= 2) ? 2 : phPointsDone + 1;
	tft.fillRect(4, 66, 312, 12, CLR_SURFACE);
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_HINT);
	tft.setCursor(6, 76);
	char lbl[40];
	snprintf(lbl, sizeof(lbl), "Titik %d/2 - pilih buffer:", which);
	tft.print(lbl);
	tft.setFont();
	for (int i = 0; i < 4; i++)
	{
		bool sel = (phBufSel == PH_BUFFERS[i]);
		uint16_t bg = sel ? CLR_BLUE : CLR_CARD;
		uint16_t fg = sel ? CLR_HEADER_TEXT : CLR_TEXT;
		tft.fillRoundRect(PHB_X[i], PHB_Y, PHB_W, PHB_H, 4, bg);
		tft.drawRoundRect(PHB_X[i], PHB_Y, PHB_W, PHB_H, 4, sel ? CLR_BLUE : CLR_BORDER);
		char b[8];
		snprintf(b, sizeof(b), "%.2f", PH_BUFFERS[i]);
		tft.setTextSize(1);
		tft.setTextColor(fg);
		tft.setCursor(PHB_X[i] + (PHB_W - strlen(b) * 6) / 2, PHB_Y + (PHB_H - 8) / 2);
		tft.print(b);
	}
}

void redrawPhLive()
{
	tft.fillRect(4, 50, 312, 12, CLR_SURFACE);
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_TEXT);
	tft.setCursor(6, 60);
	char b[28];
	snprintf(b, sizeof(b), "pH sekarang: %.2f", sensor.ph);
	tft.print(b);
	tft.setFont();
}

void redrawPhSummary()
{
	tft.fillRect(4, 144, 312, 12, CLR_SURFACE);
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_HINT);
	tft.setCursor(6, 153);
	char b[48];
	char t1[20], t2[20];
	if (phPointsDone >= 1)
		snprintf(t1, sizeof(t1), "T1 %.2f=%.2fV", phRef[0], phVolt[0]);
	else
		strcpy(t1, "T1 --");
	if (phPointsDone >= 2)
		snprintf(t2, sizeof(t2), "T2 %.2f=%.2fV", phRef[1], phVolt[1]);
	else
		strcpy(t2, "T2 --");
	snprintf(b, sizeof(b), "%s   %s", t1, t2);
	tft.print(b);
	tft.setFont();
}

void redrawPhStatus()
{
	tft.fillRect(4, 158, 312, 26, CLR_SURFACE);
	if (phCalStatus.length() == 0)
		return;
	uint16_t bg, accent;
	if (phCalInProgress)
	{
		bg = rgb565(225, 235, 255);
		accent = CLR_BLUE;
	}
	else
	{
		bg = phCalSuccess ? rgb565(225, 245, 225) : rgb565(255, 235, 235);
		accent = phCalSuccess ? CLR_OK : CLR_WARN;
	}
	tft.fillRoundRect(4, 158, 312, 24, 4, bg);
	tft.drawRoundRect(4, 158, 312, 24, 4, accent);
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_TEXT);
	tft.setCursor(10, 174);
	tft.print(phCalStatus);
	tft.setFont();
}

void drawCalibPh()
{
	phPointsDone = 0;
	phBufSel = 0;
	phCalInProgress = false;
	phCalSuccess = false;
	phCalHasResult = false;
	phCalStatus = "";
	phRef[0] = phRef[1] = 0;
	phVolt[0] = phVolt[1] = 0;

	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Kalibrasi pH", true);

	btnBackFromCalibSub = Button(4, 22, 60, 22, "< Kembali",
								 CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromCalibSub.setOnClickListener([]()
										   {
    if (phCalInProgress || phCalHasResult || phPointsDone > 0) espMaster.send("calib_ph_cancel", "state", true);
    phCalInProgress = false; phCalHasResult = false;
    showPage(PAGE_CALIBRATION); });

	redrawPhLive();
	drawPhBuffers();

	// Tombol Ukur
	btnCalibPhMeasure = Button(4, 112, 200, 24, "Ukur", CLR_OK, CLR_HEADER_TEXT, CLR_OK);
	btnCalibPhMeasure.setOnClickListener([]()
										 {
    if (phCalInProgress) return;
    if (autoMode) { phCalSuccess = false; phCalStatus = "Pindah ke MANUAL dulu"; redrawPhStatus(); return; }
    if (phPointsDone >= 2) { phCalSuccess = false; phCalStatus = "2 titik sudah, Simpan/Batal"; redrawPhStatus(); return; }
    if (phBufSel <= 0) { phCalSuccess = false; phCalStatus = "Pilih buffer dulu"; redrawPhStatus(); return; }
    int which = phPointsDone + 1;
    phRef[which - 1] = phBufSel;
    phCalInProgress = true;
    phCalStartMs = millis();
    phLastPct = -1;
    phCalStatus = "Mengukur titik " + String(which) + "...";
    redrawPhStatus();
    espMaster.send("calib_ph_point", "data", String(which) + ";" + String(phBufSel, 2)); });
	btnCalibPhMeasure.draw();

	redrawPhSummary();

	// Simpan / Batal
	btnCalibPhSave = Button(4, 188, 150, 26, "Simpan", CLR_OK, CLR_HEADER_TEXT, CLR_OK);
	btnCalibPhSave.setOnClickListener([]()
									  {
    if (phCalInProgress) return;
    if (!phCalHasResult) { phCalSuccess = false; phCalStatus = "Ukur 2 titik dulu"; redrawPhStatus(); return; }
    espMaster.send("calib_ph_save", "state", true); });
	btnCalibPhSave.draw();

	btnCalibPhCancel = Button(162, 188, 150, 26, "Batal", CLR_CARD, CLR_TEXT, CLR_BORDER);
	btnCalibPhCancel.setOnClickListener([]()
										{
    espMaster.send("calib_ph_cancel", "state", true);
    phCalInProgress = false; phCalHasResult = false;
    showPage(PAGE_CALIBRATION); });
	btnCalibPhCancel.draw();

	redrawPhStatus();

	Button *btns[] = {&btnBackFromCalibSub, &btnCalibPhMeasure, &btnCalibPhSave, &btnCalibPhCancel};
	setActiveButtons(btns, 4);
}

// Sentuh koordinat halaman pH: baris tombol buffer (selain itu ditangani objek Button).
void handleCalibPhTouch(int tx, int ty)
{
	if (phCalInProgress)
		return;
	if (ty < PHB_Y || ty > PHB_Y + PHB_H)
		return;
	for (int i = 0; i < 4; i++)
		if (tx >= PHB_X[i] && tx <= PHB_X[i] + PHB_W)
		{
			phBufSel = PH_BUFFERS[i];
			drawPhBuffers();
			return;
		}
}

// ── Kalibrasi Pompa Peristaltik ──────────────────────────────────────────────
// Kiri: pilih pompa (2x2) + durasi (10/20/30s) + Jalankan + status + Simpan/Batal.
// Kanan: field volume + keypad numerik (untuk mengetik mL hasil ukur).
// Alur: pilih pompa & durasi -> Jalankan (pompa berputar, hitung mundur) -> ukur volume keluar
// dengan gelas ukur -> ketik mL -> Simpan (main hitung mL/menit) -> tersimpan.
#define PUMP_KP_X 162
#define PUMP_KP_Y 70
#define PUMP_KP_W 51
#define PUMP_KP_H 40
// Label keypad numerik dipakai bersama halaman TDS & pompa ('<' = hapus).
static const char *CAL_KP[4] = {"789", "456", "123", ".0<"};
static const int PUMP_GX[4] = {4, 81, 4, 81};
static const int PUMP_GY[4] = {48, 48, 74, 74};
static const int PUMP_DX[3] = {4, 56, 108};
static const int PUMP_DW[3] = {48, 48, 46};

static void drawPumpTile(int x, int y, int w, int h, const char *label, bool sel)
{
	uint16_t bg = sel ? CLR_BLUE : CLR_CARD;
	uint16_t fg = sel ? CLR_HEADER_TEXT : CLR_TEXT;
	tft.fillRoundRect(x, y, w, h, 4, bg);
	tft.drawRoundRect(x, y, w, h, 4, sel ? CLR_BLUE : CLR_BORDER);
	tft.setFont();
	tft.setTextSize(1);
	tft.setTextColor(fg);
	int tw = strlen(label) * 6;
	tft.setCursor(x + (w - tw) / 2, y + (h - 8) / 2);
	tft.print(label);
}

void drawPumpSelectors()
{
	for (int i = 0; i < 4; i++)
		drawPumpTile(PUMP_GX[i], PUMP_GY[i], 73, 22, PUMP_NAMES[i], pumpCalibSel == i);
	for (int i = 0; i < 3; i++)
	{
		char lbl[6];
		snprintf(lbl, sizeof(lbl), "%lus", (unsigned long)PUMP_DUR_OPT[i]);
		drawPumpTile(PUMP_DX[i], 102, PUMP_DW[i], 22, lbl, pumpCalibDur == PUMP_DUR_OPT[i]);
	}
}

void drawPumpMlField()
{
	tft.fillRect(162, 46, 152, 20, CLR_CARD);
	tft.drawRoundRect(162, 46, 152, 20, 4, CLR_BLUE);
	tft.setFont();
	tft.setTextSize(1);
	tft.setTextColor(pumpMlLen == 0 ? CLR_HINT : CLR_TEXT);
	tft.setCursor(168, 53);
	tft.print(pumpMlLen == 0 ? "Volume mL" : pumpMlBuf);
}

void redrawPumpStatus()
{
	tft.fillRect(4, 160, 150, 32, CLR_SURFACE);
	if (pumpCalibRunning)
	{
		tft.fillRoundRect(4, 160, 150, 30, 4, rgb565(225, 235, 255));
		tft.drawRoundRect(4, 160, 150, 30, 4, CLR_BLUE);
		tft.setFont(&Poppins_Medium5pt7b);
		tft.setTextColor(CLR_TEXT);
		tft.setCursor(10, 179);
		tft.print(pumpCalibStatus);
		tft.setFont();
		return;
	}
	if (pumpCalibStatus.length() == 0)
		return;
	uint16_t bg = pumpCalibSuccess ? rgb565(225, 245, 225) : rgb565(255, 235, 235);
	uint16_t accent = pumpCalibSuccess ? CLR_OK : CLR_WARN;
	tft.fillRoundRect(4, 160, 150, 30, 4, bg);
	tft.drawRoundRect(4, 160, 150, 30, 4, accent);
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_TEXT);
	tft.setCursor(10, 179);
	tft.print(pumpCalibStatus);
	tft.setFont();
}

void drawCalibPump()
{
	// Reset state tiap masuk halaman.
	pumpCalibSel = -1;
	pumpCalibDur = 10;
	pumpMlBuf[0] = '\0';
	pumpMlLen = 0;
	pumpCalibStatus = "";
	pumpCalibSuccess = false;
	pumpCalibRunning = false;
	pumpCalibMeasure = false;

	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Kalibrasi Pompa", true);

	btnBackFromCalibSub = Button(4, 22, 60, 22, "< Kembali",
								 CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromCalibSub.setOnClickListener([]()
										   {
    // Keluar juga membersihkan state kalibrasi di main (kalau tak, run/awaiting tersangkut
    // dan Jalankan berikutnya ditolak "kalibrasi lain berjalan").
    if (pumpCalibRunning || pumpCalibMeasure) espMaster.send("calib_pump_cancel", "state", true);
    pumpCalibRunning = false;
    pumpCalibMeasure = false;
    showPage(PAGE_CALIBRATION); });

	drawPumpSelectors();

	// Tombol Jalankan
	btnCalibPumpRun = Button(4, 130, 150, 24, "Jalankan", CLR_OK, CLR_HEADER_TEXT, CLR_OK);
	btnCalibPumpRun.setOnClickListener([]()
									   {
    if (pumpCalibRunning) return;
    if (autoMode) { pumpCalibSuccess = false; pumpCalibStatus = "Pindah ke MANUAL dulu"; redrawPumpStatus(); return; }
    if (pumpCalibSel < 0) { pumpCalibSuccess = false; pumpCalibStatus = "Pilih pompa dulu"; redrawPumpStatus(); return; }
    // Ada hasil ukur lama yang belum disimpan -> bersihkan state 'menunggu hasil' di main dulu,
    // supaya start berikutnya tidak ditolak "kalibrasi lain berjalan".
    if (pumpCalibMeasure) espMaster.send("calib_pump_cancel", "state", true);
    pumpCalibMeasure = false;
    pumpMlBuf[0] = '\0'; pumpMlLen = 0; drawPumpMlField();
    pumpCalibRunning = true;
    pumpRunStartMs = millis();
    pumpRunDurMs = pumpCalibDur * 1000UL;
    pumpLastCountShown = -1;
    pumpCalibSuccess = false;
    pumpCalibStatus = "Berputar " + String(pumpCalibDur) + "s";
    redrawPumpStatus();
    espMaster.send("calib_pump_start", "data", String(pumpCalibSel) + ";" + String(pumpCalibDur)); });
	btnCalibPumpRun.draw();

	// Tombol Simpan (kirim volume terukur)
	btnCalibPumpSave = Button(4, 196, 73, 28, "Simpan", CLR_OK, CLR_HEADER_TEXT, CLR_OK);
	btnCalibPumpSave.setOnClickListener([]()
										{
    if (pumpCalibRunning) return;
    if (!pumpCalibMeasure) { pumpCalibSuccess = false; pumpCalibStatus = "Jalankan dulu"; redrawPumpStatus(); return; }
    float ml = String(pumpMlBuf).toFloat();
    if (ml <= 0) { pumpCalibSuccess = false; pumpCalibStatus = "Isi volume dulu"; redrawPumpStatus(); return; }
    espMaster.send("calib_pump_measured", "ml", String(pumpMlBuf)); });
	btnCalibPumpSave.draw();

	// Tombol Batal
	btnCalibPumpCancel = Button(81, 196, 73, 28, "Batal", CLR_CARD, CLR_TEXT, CLR_BORDER);
	btnCalibPumpCancel.setOnClickListener([]()
										  {
    espMaster.send("calib_pump_cancel", "state", true);
    pumpCalibRunning = false;
    pumpCalibMeasure = false;
    showPage(PAGE_CALIBRATION); });
	btnCalibPumpCancel.draw();

	// Field volume + keypad numerik (kanan)
	drawPumpMlField();
	for (int r = 0; r < 4; r++)
		for (int c = 0; c < 3; c++)
		{
			char lbl[2] = {CAL_KP[r][c], '\0'};
			int kx = PUMP_KP_X + c * PUMP_KP_W;
			int ky = PUMP_KP_Y + r * PUMP_KP_H;
			const char *shown = (lbl[0] == '<') ? "Del" : lbl;
			drawKbKey(kx, ky, PUMP_KP_W - 2, PUMP_KP_H - 2, shown,
					  (lbl[0] == '<') ? rgb565(255, 235, 235) : CLR_CARD,
					  (lbl[0] == '<') ? CLR_WARN : CLR_TEXT);
		}

	redrawPumpStatus();

	Button *btns[] = {&btnBackFromCalibSub, &btnCalibPumpRun, &btnCalibPumpSave, &btnCalibPumpCancel};
	setActiveButtons(btns, 4);
}

// Sentuh koordinat halaman pompa: tile pompa, tile durasi, keypad volume.
void handleCalibPumpTouch(int tx, int ty)
{
	// Saat pompa berputar, kunci pemilihan agar tak mengubah target di tengah jalan.
	if (!pumpCalibRunning)
	{
		for (int i = 0; i < 4; i++)
			if (tx >= PUMP_GX[i] && tx <= PUMP_GX[i] + 73 && ty >= PUMP_GY[i] && ty <= PUMP_GY[i] + 22)
			{
				pumpCalibSel = i;
				drawPumpSelectors();
				return;
			}
		for (int i = 0; i < 3; i++)
			if (tx >= PUMP_DX[i] && tx <= PUMP_DX[i] + PUMP_DW[i] && ty >= 102 && ty <= 124)
			{
				pumpCalibDur = PUMP_DUR_OPT[i];
				drawPumpSelectors();
				return;
			}
	}

	// Keypad volume (aktif setelah pompa selesai berputar).
	if (tx < PUMP_KP_X || ty < PUMP_KP_Y)
		return;
	int c = (tx - PUMP_KP_X) / PUMP_KP_W;
	int r = (ty - PUMP_KP_Y) / PUMP_KP_H;
	if (c < 0 || c > 2 || r < 0 || r > 3)
		return;
	char ch = CAL_KP[r][c];
	if (ch == '<')
	{
		if (pumpMlLen > 0)
			pumpMlBuf[--pumpMlLen] = '\0';
	}
	else if (ch == '.')
	{
		if (pumpMlLen > 0 && pumpMlLen < 6 && strchr(pumpMlBuf, '.') == NULL)
		{
			pumpMlBuf[pumpMlLen++] = '.';
			pumpMlBuf[pumpMlLen] = '\0';
		}
	}
	else if (pumpMlLen < 6)
	{
		pumpMlBuf[pumpMlLen++] = ch;
		pumpMlBuf[pumpMlLen] = '\0';
	}
	drawPumpMlField();
}

// ── Kalibrasi TDS ────────────────────────────────────────────────────────────
// Kiri: PPM live + field referensi + tombol Kalibrasi + status. Kanan: keypad numerik 3x4.
// Alur: ketik referensi (mis. 1382) -> "Kalibrasi" -> kirim UART "calib_tds" -> main hitung
// tdsFactor & balas "calib_tds_result" -> status diperbarui di redrawCalibTdsStatus().
#define CAL_KP_X 162 // kolom kiri keypad
#define CAL_KP_Y 50	 // baris atas keypad
#define CAL_KP_W 51	 // lebar tombol
#define CAL_KP_H 43	 // tinggi tombol
// (CAL_KP[] didefinisikan di atas, bagian konstanta kalibrasi pompa — dipakai bersama.)

void redrawCalibRefField()
{
	tft.fillRect(4, 92, 150, 24, CLR_CARD);
	tft.drawRoundRect(4, 92, 150, 24, 4, CLR_BLUE);
	tft.setFont();
	tft.setTextSize(1);
	tft.setTextColor(calibRefLen == 0 ? CLR_HINT : CLR_TEXT);
	tft.setCursor(10, 101);
	tft.print(calibRefLen == 0 ? "cth: 1382" : calibRefBuf);
}

void redrawCalibTdsStatus()
{
	tft.fillRect(4, 150, 150, 44, CLR_SURFACE); // bersihkan area hasil (dipakai bersama progress bar)
	if (calibTdsStatus.length() == 0)
		return;
	uint16_t bg = tdsCalibSuccess ? rgb565(225, 245, 225) : rgb565(255, 235, 235);
	uint16_t accent = tdsCalibSuccess ? CLR_OK : CLR_WARN;
	const char *title = tdsCalibSuccess ? "Berhasil!" : "Perhatian";
	tft.fillRoundRect(4, 150, 150, 40, 4, bg);
	tft.drawRoundRect(4, 150, 150, 40, 4, accent);
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(accent);
	tft.setCursor(10, 166);
	tft.print(title);
	tft.setTextColor(CLR_TEXT);
	tft.setCursor(10, 183);
	tft.print(calibTdsStatus);
	tft.setFont();
}

// Progress bar jendela sampling (0..100%). Digambar di area kiri-bawah, di atas area status.
void drawCalibTdsProgress(int pct)
{
	const int x = 4, y = 152, w = 150, h = 14;
	tft.drawRoundRect(x, y, w, h, 3, CLR_BORDER);
	tft.fillRect(x + 1, y + 1, w - 2, h - 2, CLR_CARD); // bersihkan isi
	int fillW = (w - 2) * pct / 100;
	if (fillW > 0)
		tft.fillRect(x + 1, y + 1, fillW, h - 2, CLR_OK);
	tft.fillRect(x, y + h + 2, w, 12, CLR_SURFACE);
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_TEXT);
	tft.setCursor(x + 2, y + h + 11);
	char b[24];
	snprintf(b, sizeof(b), "Mengukur %d%%", pct);
	tft.print(b);
	tft.setFont();
}

void drawCalibTds()
{
	// Mulai bersih tiap masuk halaman.
	calibRefBuf[0] = '\0';
	calibRefLen = 0;
	calibTdsStatus = "";
	tdsCalibInProgress = false;
	tdsCalibHasResult = false;

	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Kalibrasi TDS", true);

	btnBackFromCalibSub = Button(4, 22, 60, 22, "< Kembali",
								 CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromCalibSub.setOnClickListener([]()
										   { showPage(PAGE_CALIBRATION); });

	// PPM live saat ini (dari sensor terakhir; diperbarui saat update_sensor tiba)
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_HINT);
	tft.setCursor(6, 58);
	tft.print("PPM sekarang");
	tft.setTextColor(CLR_TEXT);
	tft.setCursor(6, 74);
	char liveBuf[24];
	snprintf(liveBuf, sizeof(liveBuf), "%d PPM", (int)sensor.nutrisi);
	tft.print(liveBuf);

	tft.setTextColor(CLR_HINT);
	tft.setCursor(6, 90);
	tft.print("Referensi (PPM)");
	tft.setFont();
	redrawCalibRefField();

	// Tombol Mulai — memulai jendela sampling di main board
	btnCalibTdsGo = Button(4, 120, 150, 24, "Mulai", CLR_OK, CLR_HEADER_TEXT, CLR_OK);
	btnCalibTdsGo.setOnClickListener([]()
									 {
    if (tdsCalibInProgress) return; // sedang mengukur — abaikan tap
    if (autoMode) { tdsCalibSuccess = false; calibTdsStatus = "Pindah ke MANUAL dulu"; redrawCalibTdsStatus(); return; }
    float ref = String(calibRefBuf).toFloat();
    if (ref <= 0) { tdsCalibSuccess = false; calibTdsStatus = "Isi referensi dulu"; redrawCalibTdsStatus(); return; }
    tdsCalibInProgress = true;
    tdsCalibHasResult = false; // ukur baru membatalkan hasil lama yang belum disimpan
    tdsCalibStartMs = millis();
    calibTdsStatus = "";
    redrawCalibTdsStatus();
    espMaster.send("calib_tds_start", "refPpm", String(calibRefBuf));
    drawCalibTdsProgress(0); });
	btnCalibTdsGo.draw(); // setActiveButtons hanya mendaftarkan sentuh — gambar tombol harus eksplisit

	// Tombol Simpan — commit faktor hasil ukur (hanya berlaku bila sudah ada hasil).
	btnCalibTdsSave = Button(4, 196, 73, 28, "Simpan", CLR_OK, CLR_HEADER_TEXT, CLR_OK);
	btnCalibTdsSave.setOnClickListener([]()
									   {
    if (tdsCalibInProgress) return; // tunggu selesai mengukur
    if (!tdsCalibHasResult) { tdsCalibSuccess = false; calibTdsStatus = "Ukur dulu"; redrawCalibTdsStatus(); return; }
    espMaster.send("calib_tds_save", "state", true); });
	btnCalibTdsSave.draw();

	// Tombol Batal — buang hasil (dan hentikan sampling di main) lalu kembali ke submenu.
	btnCalibTdsCancel = Button(81, 196, 73, 28, "Batal", CLR_CARD, CLR_TEXT, CLR_BORDER);
	btnCalibTdsCancel.setOnClickListener([]()
										 {
    espMaster.send("calib_tds_cancel", "state", true);
    tdsCalibInProgress = false;
    tdsCalibHasResult = false;
    showPage(PAGE_CALIBRATION); });
	btnCalibTdsCancel.draw();

	// Keypad numerik 3x4
	for (int r = 0; r < 4; r++)
		for (int c = 0; c < 3; c++)
		{
			char lbl[2] = {CAL_KP[r][c], '\0'};
			int kx = CAL_KP_X + c * CAL_KP_W;
			int ky = CAL_KP_Y + r * CAL_KP_H;
			const char *shown = (lbl[0] == '<') ? "Del" : lbl;
			drawKbKey(kx, ky, CAL_KP_W - 2, CAL_KP_H - 2, shown,
					  (lbl[0] == '<') ? rgb565(255, 235, 235) : CLR_CARD,
					  (lbl[0] == '<') ? CLR_WARN : CLR_TEXT);
		}

	redrawCalibTdsStatus();

	Button *btns[] = {&btnBackFromCalibSub, &btnCalibTdsGo, &btnCalibTdsSave, &btnCalibTdsCancel};
	setActiveButtons(btns, 4);
}

// Keypad TDS via koordinat (bukan objek Button) — dipanggil dari loop touch saat PAGE_CALIB_TDS.
void handleCalibTdsTouch(int tx, int ty)
{
	if (tx < CAL_KP_X || ty < CAL_KP_Y)
		return;
	int c = (tx - CAL_KP_X) / CAL_KP_W;
	int r = (ty - CAL_KP_Y) / CAL_KP_H;
	if (c < 0 || c > 2 || r < 0 || r > 3)
		return;
	char ch = CAL_KP[r][c];

	if (ch == '<') // hapus
	{
		if (calibRefLen > 0)
			calibRefBuf[--calibRefLen] = '\0';
	}
	else if (ch == '.')
	{
		if (calibRefLen > 0 && calibRefLen < 9 && strchr(calibRefBuf, '.') == NULL)
		{
			calibRefBuf[calibRefLen++] = '.';
			calibRefBuf[calibRefLen] = '\0';
		}
	}
	else if (calibRefLen < 9) // digit
	{
		calibRefBuf[calibRefLen++] = ch;
		calibRefBuf[calibRefLen] = '\0';
	}
	redrawCalibRefField();
}

// Screen Timeout
void drawScreenTimeout()
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Layar Timeout", true);

	btnBackFromSetScreen = Button(4, 22, 60, 22, "< Kembali",
								  CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromSetScreen.setOnClickListener([]()
											{ showPage(PAGE_SETTING); });

	int y = 50;

	btnScreen15 = Button(0, y, SCREEN_W, 25, "", CLR_BG, CLR_SURFACE, CLR_SURFACE);
	btnScreen30 = Button(0, y + 30, SCREEN_W, 25, "", CLR_BG, CLR_SURFACE, CLR_SURFACE);
	btnScreen60 = Button(0, y + 60, SCREEN_W, 25, "", CLR_BG, CLR_SURFACE, CLR_SURFACE);
	btnScreen120 = Button(0, y + 90, SCREEN_W, 25, "", CLR_BG, CLR_SURFACE, CLR_SURFACE);
	btnScreenOff = Button(0, y + 120, SCREEN_W, 25, "", CLR_BG, CLR_SURFACE, CLR_SURFACE);

	btnScreen15.setOnClickListener([]()
								   {
    for (int i = 0; i < 5; i++) screenTimeoutOptions[i] = false;
    screenTimeoutOptions[0] = true;
    for (int i = 0; i < 5; i++) {
      EEPROM.write(i, screenTimeoutOptions[i]);
      EEPROM.commit();
    }
    menuScreenTimeOut();
    SCREEN_TIMEOUT = SCREEN_TO[0]; });
	btnScreen15.draw();

	btnScreen30.setOnClickListener([]()
								   {
    for (int i = 0; i < 5; i++) screenTimeoutOptions[i] = false;
    screenTimeoutOptions[1] = true;
    for (int i = 0; i < 5; i++) {
      EEPROM.write(i, screenTimeoutOptions[i]);
      EEPROM.commit();
    }
    menuScreenTimeOut();
    SCREEN_TIMEOUT = SCREEN_TO[1]; });
	btnScreen30.draw();

	btnScreen60.setOnClickListener([]()
								   {
    for (int i = 0; i < 5; i++) screenTimeoutOptions[i] = false;
    screenTimeoutOptions[2] = true;
    for (int i = 0; i < 5; i++) {
      EEPROM.write(i, screenTimeoutOptions[i]);
      EEPROM.commit();
    }
    menuScreenTimeOut();
    SCREEN_TIMEOUT = SCREEN_TO[2]; });
	btnScreen60.draw();

	btnScreen120.setOnClickListener([]()
									{
    for (int i = 0; i < 5; i++) screenTimeoutOptions[i] = false;
    screenTimeoutOptions[3] = true;
    for (int i = 0; i < 5; i++) {
      EEPROM.write(i, screenTimeoutOptions[i]);
      EEPROM.commit();
    }
    menuScreenTimeOut();
    SCREEN_TIMEOUT = SCREEN_TO[3]; });
	btnScreen120.draw();

	btnScreenOff.setOnClickListener([]()
									{
    for (int i = 0; i < 5; i++) screenTimeoutOptions[i] = false;
    screenTimeoutOptions[4] = true;
    for (int i = 0; i < 5; i++) {
      EEPROM.write(i, screenTimeoutOptions[i]);
      EEPROM.commit();
    }
    menuScreenTimeOut(); });
	btnScreenOff.draw();

	Button *btns[] = {
		&btnBackFromSetScreen,
		&btnScreen15, &btnScreen30, &btnScreen60, &btnScreen120, &btnScreenOff};
	setActiveButtons(btns, 6);

	menuScreenTimeOut();
}

// Reset
void drawRestart()
{
	tft.fillRoundRect(40, 90, 240, 100, 10, CLR_CARD);
	tft.drawRoundRect(40, 90, 240, 100, 10, CLR_HEADER);

	int x = (SCREEN_W - strlen("Restart Perangkat?") * 6) / 2;
	tft.setFont(&Poppins_Medium5pt7b);
	tft.setTextColor(CLR_TEXT);
	tft.setCursor(x, 130);
	tft.print("Restart Perangkat?");
	tft.setFont();

	btnBackFromReset = Button(44, 160, 80, 22, "Batal",
							  CLR_OK, CLR_HEADER_TEXT, CLR_CARD);
	btnBackFromReset.setOnClickListener([]()
										{ showPage(PAGE_SETTING); });

	btnRestart = Button(196, 160, 80, 22, "Restart",
						CLR_WARN, CLR_HEADER_TEXT, CLR_CARD);
	btnRestart.setOnClickListener([]()
								  { 
		espMaster.sendType("restart");
		delay(500);
		ESP.restart(); });

	btnBackFromReset.draw();
	btnRestart.draw();

	Button *btns[] = {&btnBackFromReset, &btnRestart};
	setActiveButtons(btns, 2);
}

// Page About
void drawAbout()
{
	tft.fillScreen(CLR_SURFACE);
	drawStatusBar(wifiOK, wifiSSID.c_str());
	drawHeader("Info Alat", true);

	btnBackFromAbout = Button(4, 22, 60, 22, "< Kembali",
							  CLR_HEADER, CLR_HEADER_SUB, CLR_HEADER);
	btnBackFromAbout.setOnClickListener([]()
										{ showPage(PAGE_SETTING); });

	tft.drawRGBBitmap(130, 50, unnes, 60, 80);

	tft.setTextColor(CLR_TEXT);
	tft.setFont(&Poppins_Medium5pt7b);
	int x = (SCREEN_W - strlen("Hydro-Iot Control System") * 6) / 2;
	tft.setCursor(x, 150);
	tft.print("Hydro-Iot Control System");

	x = (SCREEN_W - strlen("v1.0.0") * 6) / 2;
	tft.setCursor(x, 165);
	tft.print("v1.0.0");

	// ID device (dikirim main board via UART "device_id").
	String idLine = "ID: " + deviceId;
	x = (SCREEN_W - (int)idLine.length() * 6) / 2;
	tft.setTextColor(CLR_BLUE);
	tft.setCursor(x, 178);
	tft.print(idLine);
	tft.setTextColor(CLR_TEXT);

	x = (SCREEN_W - strlen("by: Tim VoltX") * 6) / 2;
	tft.setCursor(x, 190);
	tft.print("by: Tim VoltX");

	x = (SCREEN_W - strlen("Universitas Negeri Semarang") * 6) / 2;
	tft.setCursor(x, 205);
	tft.print("Universitas Negeri Semarang");

	x = (SCREEN_W - strlen("2026") * 6) / 2;
	tft.setCursor(x, 220);
	tft.print("2026");
	tft.setFont();

	Button *btns[] = {&btnBackFromAbout};
	setActiveButtons(btns, 1);
}

// Keyboard
void drawKeyboard()
{
	// ── Baris mode tab ───────────────────────────────────────
	int tabW = (SCREEN_W - 8) / 4;
	int tabY = KB_Y_START;
	drawKbModeTab(4, tabY, tabW, KB_KEY_H, "ABC", kbMode == KB_UPPER);
	drawKbModeTab(4 + tabW, tabY, tabW, KB_KEY_H, "abc", kbMode == KB_LOWER);
	drawKbModeTab(4 + tabW * 2, tabY, tabW, KB_KEY_H, "123", kbMode == KB_NUM);
	drawKbModeTab(4 + tabW * 3, tabY, tabW, KB_KEY_H, "!@#", kbMode == KB_SYM);

	const char **rows = (kbMode == KB_UPPER) ? ROWS_UPPER : (kbMode == KB_LOWER) ? ROWS_LOWER
														: (kbMode == KB_NUM)	 ? ROWS_NUM
																				 : ROWS_SYM;

	// ── Baris 1: 10 tombol ───────────────────────────────────
	int row1Y = KB_Y_START + KB_H;
	int kw1 = (SCREEN_W - 8) / 10;
	for (int i = 0; i < 10; i++)
	{
		char lbl[3] = {rows[0][i], '\0'};
		drawKbKey(4 + i * kw1, row1Y, kw1 - 1, KB_KEY_H, lbl);
	}

	// ── Baris 2: 9 tombol (sedikit indent) ───────────────────
	int row2Y = KB_Y_START + KB_H * 2;
	int kw2 = (SCREEN_W - 8) / 10; // sama lebar
	int indent = kw2 / 2;		   // geser setengah tombol
	for (int i = 0; i < 9; i++)
	{
		char lbl[3] = {rows[1][i], '\0'};
		drawKbKey(4 + indent + i * kw2, row2Y, kw2 - 1, KB_KEY_H, lbl);
	}

	// ── Baris 3: Caps | 7 huruf | Del ────────────────────────
	int row3Y = KB_Y_START + KB_H * 3;
	int kw3 = (SCREEN_W - 8) / 10;
	int capsW = kw3 + kw3 / 2; // caps lebih lebar
	int delW = capsW;

	// Caps
	const char *capsLabel = (kbMode == KB_UPPER) ? "v abc" : "^ ABC";
	bool isCaps = (kbMode == KB_UPPER);
	drawKbKey(4, row3Y, capsW, KB_KEY_H, capsLabel,
			  isCaps ? rgb565(220, 235, 255) : CLR_CARD,
			  isCaps ? CLR_HEADER : CLR_MUTED);

	// 7 huruf tengah
	int chars3 = strlen(rows[2]);
	int startX3 = 4 + capsW + 2;
	int availW = SCREEN_W - 4 - capsW - 2 - delW - 2 - 4;
	int kw3m = availW / chars3;
	for (int i = 0; i < chars3; i++)
	{
		char lbl[3] = {rows[2][i], '\0'};
		drawKbKey(startX3 + i * kw3m, row3Y, kw3m - 1, KB_KEY_H, lbl);
	}

	// Del
	drawKbKey(SCREEN_W - 4 - delW, row3Y, delW, KB_KEY_H, "Del",
			  rgb565(255, 235, 235), CLR_WARN);

	// ── Baris 4: Spasi + Hubungkan ────────────────────────────
	int row4Y = KB_Y_START + KB_H * 4;
	int konekW = 90;
	int spasiW = SCREEN_W - 8 - konekW - 4;
	drawKbKey(4, row4Y, spasiW, KB_KEY_H, "SPASI", CLR_CARD, CLR_MUTED);
	drawKbKey(4 + spasiW + 4, row4Y, konekW, KB_KEY_H, "HUBUNGKAN", CLR_OK, CLR_HEADER_TEXT);
}

void redrawPasswordField()
{
	tft.fillRect(4, 50, SCREEN_W - 8, 24, CLR_CARD);
	tft.drawRoundRect(4, 50, SCREEN_W - 8, 24, 4, CLR_BLUE);
	tft.setFont();
	tft.setTextColor(wifiPasswordLen == 0 ? CLR_HINT : CLR_TEXT);
	tft.setTextSize(1);
	tft.setCursor(10, 59);
	if (wifiPasswordLen == 0)
	{
		tft.print("Masukkan password...");
	}
	else
	{
		int maxChar = (SCREEN_W - 20) / 6;
		int start = wifiPasswordLen > maxChar ? wifiPasswordLen - maxChar : 0;
		for (int i = start; i < wifiPasswordLen; i++)
			tft.print(wifiPassword[i]);
	}
	// Kursor
	int shown = min(wifiPasswordLen, (SCREEN_W - 20) / 6);
	tft.fillRect(10 + shown * 6, 53, 1, 14, CLR_BLUE);
}

void handleKbTouch(int tx, int ty)
{
	if (ty < KB_Y_START)
		return;

	// ── Tab mode ─────────────────────────────────────────────
	if (ty < KB_Y_START + KB_H)
	{
		int tabW = (SCREEN_W - 8) / 4;
		if (tx >= 4 && tx < 4 + tabW)
		{
			kbMode = KB_UPPER;
			drawKeyboard();
			return;
		}
		if (tx >= 4 + tabW && tx < 4 + tabW * 2)
		{
			kbMode = KB_LOWER;
			drawKeyboard();
			return;
		}
		if (tx >= 4 + tabW * 2 && tx < 4 + tabW * 3)
		{
			kbMode = KB_NUM;
			drawKeyboard();
			return;
		}
		if (tx >= 4 + tabW * 3)
		{
			kbMode = KB_SYM;
			drawKeyboard();
			return;
		}
	}

	const char **rows = (kbMode == KB_UPPER) ? ROWS_UPPER : (kbMode == KB_LOWER) ? ROWS_LOWER
														: (kbMode == KB_NUM)	 ? ROWS_NUM
																				 : ROWS_SYM;

	// ── Baris 1 ──────────────────────────────────────────────
	int row1Y = KB_Y_START + KB_H;
	if (ty >= row1Y && ty < row1Y + KB_KEY_H)
	{
		int kw = (SCREEN_W - 8) / 10;
		int idx = (tx - 4) / kw;
		if (idx >= 0 && idx < 10)
		{
			char c = rows[0][idx];
			if (wifiPasswordLen < 63)
			{
				wifiPassword[wifiPasswordLen++] = c;
				wifiPassword[wifiPasswordLen] = '\0';
			}
			redrawPasswordField();
		}
		return;
	}

	// ── Baris 2 ──────────────────────────────────────────────
	int row2Y = KB_Y_START + KB_H * 2;
	if (ty >= row2Y && ty < row2Y + KB_KEY_H)
	{
		int kw = (SCREEN_W - 8) / 10;
		int indent = kw / 2;
		int idx = (tx - 4 - indent) / kw;
		if (idx >= 0 && idx < 9)
		{
			char c = rows[1][idx];
			if (wifiPasswordLen < 63)
			{
				wifiPassword[wifiPasswordLen++] = c;
				wifiPassword[wifiPasswordLen] = '\0';
			}
			redrawPasswordField();
		}
		return;
	}

	// ── Baris 3 ──────────────────────────────────────────────
	int row3Y = KB_Y_START + KB_H * 3;
	if (ty >= row3Y && ty < row3Y + KB_KEY_H)
	{
		int kw3 = (SCREEN_W - 8) / 10;
		int capsW = kw3 + kw3 / 2;
		int delW = capsW;

		// Caps toggle
		if (tx >= 4 && tx < 4 + capsW)
		{
			kbMode = (kbMode == KB_UPPER) ? KB_LOWER : KB_UPPER;
			drawKeyboard();
			return;
		}

		// Del
		if (tx >= SCREEN_W - 4 - delW)
		{
			if (wifiPasswordLen > 0)
				wifiPassword[--wifiPasswordLen] = '\0';
			redrawPasswordField();
			return;
		}

		// Huruf tengah
		int chars3 = strlen(rows[2]);
		int startX3 = 4 + capsW + 2;
		int availW = SCREEN_W - 4 - capsW - 2 - delW - 2 - 4;
		int kw3m = availW / chars3;
		int idx = (tx - startX3) / kw3m;
		if (idx >= 0 && idx < chars3)
		{
			char c = rows[2][idx];
			if (wifiPasswordLen < 63)
			{
				wifiPassword[wifiPasswordLen++] = c;
				wifiPassword[wifiPasswordLen] = '\0';
			}
			redrawPasswordField();
		}
		return;
	}

	// ── Baris 4: Spasi / Hubungkan ────────────────────────────
	int row4Y = KB_Y_START + KB_H * 4;
	if (ty >= row4Y && ty < row4Y + KB_KEY_H)
	{
		int konekW = 90;
		int spasiW = SCREEN_W - 8 - konekW - 4;

		if (tx >= 4 && tx < 4 + spasiW)
		{
			// Spasi
			if (wifiPasswordLen < 63)
			{
				wifiPassword[wifiPasswordLen++] = ' ';
				wifiPassword[wifiPasswordLen] = '\0';
			}
			redrawPasswordField();
		}
		else if (tx >= 4 + spasiW + 4)
		{

			String credential = String(wifiInputSSID) + ";" + String(wifiPassword);
			espMaster.send("connect_wifi", "credential", credential);
			showPage(PAGE_WIFI);
			waitRespons = true;
			scanWifi = true;
			wifiTimeout = millis();
			connectingWifi(String(wifiInputSSID));
		}
		return;
	}
}

void parseSensor(String data, SensorData &sens)
{
	int p1 = data.indexOf(';');
	int p2 = data.indexOf(';', p1 + 1);
	int p3 = data.indexOf(';', p2 + 1);
	int p4 = data.indexOf(';', p3 + 1);
	int p5 = data.indexOf(';', p4 + 1);

	sens.ph = data.substring(0, p1).toFloat();
	sens.tempAir = data.substring(p1 + 1, p2).toFloat();
	sens.tempWater = data.substring(p2 + 1, p3).toFloat();
	sens.waterLvl = data.substring(p3 + 1, p4).toInt();
	sens.nutrisi = data.substring(p4 + 1, p5).toInt();
	sens.humidity = data.substring(p5 + 1).toInt();
}

// Lightweight value refresh for the dashboard — repaints the numbers/bars in place, no full
// redraw, so no flicker. No-op on every other page. Everything that changes a displayed VALUE
// (new sensor reading, new limit) goes through here; only a change to something drawn once per
// page (unit labels) needs a full showPage().
void refreshDashboardValues()
{
	if (currentPage != PAGE_DASHBOARD)
		return;
	if (autoMode || dashSubPage == 1)
		updateDashboardValues();
	if (!autoMode && dashSubPage == 0)
		updateControlValues();
}

// Komunikasi
void handleMaster()
{
	if (!espMaster.update())
		return;

	String type = espMaster.recvType();
	Serial.println(type);

	if (type == "ping")
	{
		Serial.println("Dapat ping dari ESP_Sensor " + String(espMaster.recv<int>("ping", "data")));
	}
	else if (type == "device_id")
	{
		deviceId = espMaster.recv<String>("device_id", "id");
		if (currentPage == PAGE_ABOUT)
			drawAbout(); // segarkan bila sedang di halaman Info
	}
	else if (type == "wifi_status")
	{
		// Status WiFi dari main: "1;<ssid>" tersambung, "0;" tidak. Isi status bar atas tanpa
		// harus buka menu WiFi dulu.
		String data = espMaster.recv<String>("wifi_status", "data");
		int sep = data.indexOf(';');
		wifiOK = (data.substring(0, sep).toInt() == 1);
		wifiSSID = data.substring(sep + 1);
		if (currentPage != PAGE_BOOT)
			drawStatusBar(wifiOK, wifiSSID.c_str());
	}
	else if (type == "boot")
	{
		// Status boot dari main: "ready" | "offline" | "<pct>;<teks>".
		String data = espMaster.recv<String>("boot", "data");
		bootMsgReceived = true;
		if (data == "ready")
		{
			if (currentPage == PAGE_BOOT)
				showPage(PAGE_DASHBOARD);
		}
		else if (data == "offline")
		{
			if (currentPage == PAGE_BOOT && !bootOfflineShown)
			{
				bootOfflineShown = true;
				bootOfflineMs = millis();
				redrawBootProgress(); // ganti bar dengan tombol "Lanjut Offline"
			}
		}
		else if (!bootOfflineShown) // "<pct>;<teks>"
		{
			int sep = data.indexOf(';');
			if (sep > 0)
			{
				bootPct = data.substring(0, sep).toInt();
				bootText = data.substring(sep + 1);
				if (currentPage == PAGE_BOOT)
					redrawBootProgress();
			}
		}
	}
	else if (type == "scan_wifi")
	{
		waitRespons = false;
		scanWifi = false;
		espMaster.recv("network", wifiRaw);

		for (int i = 0; i < 4; i++)
		{
			parseWifi(wifiRaw[i], wifiList[i]);
		}

		sortNetworks();
		wifiOK = wifiList[0].connected;
		wifiSSID = wifiList[0].ssid;

		drawStatusBar(wifiOK, wifiSSID.c_str());

		if (currentPage == PAGE_WIFI)
		{
			// Empty slots arrive as ";0;0" (scan failed, or genuinely nothing in range) and
			// parse to a blank SSID. Handing those to drawWifi() paints blank rows under
			// "JARINGAN TERSEDIA"; show the rescan prompt instead so the page is actionable.
			int found = 0;
			for (int i = 0; i < NET_COUNT; i++)
				if (wifiList[i].ssid.length() > 0)
					found++;

			if (found == 0)
				waitWifi(false);
			else
				drawWifi();
		}
	}
	else if (type == "update_sensor")
	{
		String sensorRaw = espMaster.recv<String>("array_sensor");
		parseSensor(sensorRaw, sensor);
		refreshDashboardValues();
		// Refresh the live PPM shown on the TDS calibration page as new readings arrive.
		// Skip while a sampling window is running — "calib_tds_prog" owns that value then.
		if (currentPage == PAGE_CALIB_TDS && !tdsCalibInProgress)
		{
			tft.fillRect(4, 66, 150, 12, CLR_SURFACE);
			tft.setFont(&Poppins_Medium5pt7b);
			tft.setTextColor(CLR_TEXT);
			tft.setCursor(6, 74);
			char liveBuf[24];
			snprintf(liveBuf, sizeof(liveBuf), "%d PPM", (int)sensor.nutrisi);
			tft.print(liveBuf);
			tft.setFont();
		}
		// pH live di halaman kalibrasi pH (kecuali saat sampling — "calib_ph_prog" yang pegang).
		if (currentPage == PAGE_CALIB_PH && !phCalInProgress)
			redrawPhLive();
	}
	else if (type == "calib_tds_prog")
	{
		// Progress jendela sampling: "<pct>;<rawPpm>". Perbarui progress bar + PPM live (raw).
		String data = espMaster.recv<String>("calib_tds_prog", "data");
		int sep = data.indexOf(';');
		int pct = data.substring(0, sep).toInt();
		int livePpm = data.substring(sep + 1).toInt();
		if (currentPage == PAGE_CALIB_TDS)
		{
			drawCalibTdsProgress(pct);
			tft.fillRect(4, 66, 150, 12, CLR_SURFACE);
			tft.setFont(&Poppins_Medium5pt7b);
			tft.setTextColor(CLR_TEXT);
			tft.setCursor(6, 74);
			char liveBuf[24];
			snprintf(liveBuf, sizeof(liveBuf), "%d PPM", livePpm);
			tft.print(liveBuf);
			tft.setFont();
		}
	}
	else if (type == "calib_tds_result")
	{
		// Hasil kalibrasi TDS dari main board: "ok;<factor>;<rawPpm>;<refPpm>" atau "err;<alasan>".
		String data = espMaster.recv<String>("calib_tds_result", "data");
		tdsCalibInProgress = false;
		if (data.startsWith("ok;"))
		{
			int p1 = data.indexOf(';');
			int p2 = data.indexOf(';', p1 + 1);
			tdsCalibSuccess = true;
			tdsCalibHasResult = true; // faktor terukur, tombol Simpan siap dipakai
			calibTdsStatus = "faktor = " + data.substring(p1 + 1, p2) + " (Simpan?)";
		}
		else
		{
			tdsCalibSuccess = false;
			tdsCalibHasResult = false;
			calibTdsStatus = data.substring(data.indexOf(';') + 1);
		}
		if (currentPage == PAGE_CALIB_TDS)
			redrawCalibTdsStatus(); // sudah membersihkan seluruh area (termasuk progress bar)
	}
	else if (type == "calib_tds_saved")
	{
		// Konfirmasi commit dari main board: "ok;<factor>" atau "err;<alasan>".
		String data = espMaster.recv<String>("calib_tds_saved", "data");
		if (data.startsWith("ok;"))
		{
			tdsCalibSuccess = true;
			tdsCalibHasResult = false; // sudah tersimpan, tak perlu Simpan lagi
			calibTdsStatus = "Tersimpan (faktor " + data.substring(data.indexOf(';') + 1) + ")";
		}
		else
		{
			tdsCalibSuccess = false;
			calibTdsStatus = data.substring(data.indexOf(';') + 1);
		}
		if (currentPage == PAGE_CALIB_TDS)
			redrawCalibTdsStatus();
	}
	else if (type == "calib_pump_result")
	{
		// Status kalibrasi pompa dari main: "run;<dur>" | "done" | "ok;<flow>" | "err;<alasan>".
		String data = espMaster.recv<String>("calib_pump_result", "data");
		if (data == "done")
		{
			// Pompa selesai berputar — minta operator mengukur & mengetik volume.
			pumpCalibRunning = false;
			pumpCalibMeasure = true;
			pumpCalibSuccess = true;
			pumpCalibStatus = "Ukur volume, isi mL";
		}
		else if (data.startsWith("ok;"))
		{
			pumpCalibRunning = false;
			pumpCalibMeasure = false;
			pumpCalibSuccess = true;
			pumpCalibStatus = "Tersimpan " + data.substring(3) + " mL/mnt";
		}
		else if (data.startsWith("err;"))
		{
			pumpCalibRunning = false;
			pumpCalibMeasure = false;
			pumpCalibSuccess = false;
			pumpCalibStatus = data.substring(4);
		}
		// "run;<dur>" tidak perlu ditangani: LCD sudah memulai hitung mundur sendiri saat menekan Jalankan.
		if (currentPage == PAGE_CALIB_PUMP && !data.startsWith("run"))
		{
			pumpLastCountShown = -1; // paksa redraw countdown berikutnya
			redrawPumpStatus();
		}
	}
	else if (type == "calib_ph_prog")
	{
		// Progress sampling titik pH: "<pct>;<pHnow>".
		String data = espMaster.recv<String>("calib_ph_prog", "data");
		int sep = data.indexOf(';');
		int pct = data.substring(0, sep).toInt();
		if (currentPage == PAGE_CALIB_PH && pct != phLastPct)
		{
			phLastPct = pct;
			int which = phPointsDone + 1;
			phCalStatus = "Mengukur titik " + String(which) + " " + String(pct) + "%";
			redrawPhStatus();
			// pH live (dgn kalibrasi lama) tetap diperbarui dari nilai prog.
			tft.fillRect(4, 50, 312, 12, CLR_SURFACE);
			tft.setFont(&Poppins_Medium5pt7b);
			tft.setTextColor(CLR_TEXT);
			tft.setCursor(6, 60);
			tft.print("pH sekarang: " + data.substring(sep + 1));
			tft.setFont();
		}
	}
	else if (type == "calib_ph_result")
	{
		// Titik selesai: "pt;<which>;<v>" atau "err;<alasan>".
		String data = espMaster.recv<String>("calib_ph_result", "data");
		phCalInProgress = false;
		if (data.startsWith("pt;"))
		{
			int p1 = data.indexOf(';');
			int p2 = data.indexOf(';', p1 + 1);
			int which = data.substring(p1 + 1, p2).toInt();
			float v = data.substring(p2 + 1).toFloat();
			if (which >= 1 && which <= 2)
			{
				phVolt[which - 1] = v;
				phPointsDone = which;
			}
			phBufSel = 0; // reset pilihan buffer utk titik berikutnya
			if (phPointsDone >= 2)
			{
				phCalHasResult = true;
				phCalSuccess = true;
				phCalStatus = "2 titik siap - Simpan?";
			}
			else
			{
				phCalSuccess = true;
				phCalStatus = "Titik 1 OK - ukur titik 2";
			}
		}
		else // err;...
		{
			phCalSuccess = false;
			phCalStatus = data.substring(data.indexOf(';') + 1);
		}
		if (currentPage == PAGE_CALIB_PH)
		{
			drawPhBuffers(); // perbarui label titik & reset highlight
			redrawPhSummary();
			redrawPhStatus();
		}
	}
	else if (type == "calib_ph_saved")
	{
		// Konfirmasi simpan: "ok;<slope>;<offset>" atau "err;<alasan>".
		String data = espMaster.recv<String>("calib_ph_saved", "data");
		if (data.startsWith("ok;"))
		{
			int p1 = data.indexOf(';');
			int p2 = data.indexOf(';', p1 + 1);
			phCalSuccess = true;
			phCalHasResult = false;
			phCalStatus = "Tersimpan s=" + data.substring(p1 + 1, p2) + " o=" + data.substring(p2 + 1);
		}
		else
		{
			phCalSuccess = false;
			phCalStatus = data.substring(data.indexOf(';') + 1);
		}
		if (currentPage == PAGE_CALIB_PH)
			redrawPhStatus();
	}
	else if (type == "mode")
	{
		// ESP32-MAIN syncs mode here after a Firebase/app change (see applyMode() on that side).
		// Mirrors setMode()'s state update but does NOT echo "selector_switch" back — this is a
		// sync FROM master, not a local button press, so no round-trip is needed.
		bool autoOn = espMaster.recv<bool>("mode", "value");
		if (autoOn != autoMode)
		{
			autoMode = autoOn;
			lastPump = autoMode;
			if (autoMode)
				for (int i = 0; i < 4; i++)
					doseOn[i] = false; // main mematikan semua pompa saat masuk AUTO

			EEPROM.write(5, autoOn ? 0 : 1); // 0=AUTO 1=MANUAL, matches setMode()/readModeFromEEPROM()
			EEPROM.commit();

			if (currentPage == PAGE_DASHBOARD)
			{
				drawDashboard();
			}

			Serial.printf("[uart] mode dari ESP_Sensor -> %s\n", autoMode ? "AUTO" : "MANUAL");
		}
	}
	else if (type == "update_sensorLimit")
	{
		// "phMin;phMax;ppmMin;ppmMax;waterLowPct" — only the limits that actually exist as
		// Firebase settings on ESP32-MAIN. tAir/tWater/humidity/waterMax stay local constants.
		String raw = espMaster.recv<String>("array_sensorLimit");
		int p1 = raw.indexOf(';');
		int p2 = raw.indexOf(';', p1 + 1);
		int p3 = raw.indexOf(';', p2 + 1);
		int p4 = raw.indexOf(';', p3 + 1);
		if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0)
			return;

		phMin = raw.substring(0, p1).toFloat();
		phMax = raw.substring(p1 + 1, p2).toFloat();
		nutrisiMin = raw.substring(p2 + 1, p3).toFloat();
		nutrisiMax = raw.substring(p3 + 1, p4).toFloat();
		waterMin = raw.substring(p4 + 1).toFloat();

		Serial.printf("[uart] limit -> pH %.1f-%.1f PPM %.0f-%.0f waterLow %.0f\n",
					  phMin, phMax, nutrisiMin, nutrisiMax, waterMin);

		// Limits feed two things: the warn thresholds on the dashboard (a value refresh) and the
		// Min/Max labels on the sensor-limit page (drawn once per page, so a full redraw).
		if (currentPage == PAGE_SENSOR_LIMIT)
			showPage(currentPage);
		else
			refreshDashboardValues();
	}
	else if (type == "sensor_unit")
	{
		// "phUnit;tAirUnit;tWaterUnit;waterLvlUnit;ppmUnit;humidityUnit"
		String raw = espMaster.recv<String>("array_sensorUnit");
		int p1 = raw.indexOf(';');
		int p2 = raw.indexOf(';', p1 + 1);
		int p3 = raw.indexOf(';', p2 + 1);
		int p4 = raw.indexOf(';', p3 + 1);
		int p5 = raw.indexOf(';', p4 + 1);
		if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0)
			return;

		unitPh = raw.substring(0, p1);
		unitTempAir = raw.substring(p1 + 1, p2);
		unitTempWater = raw.substring(p2 + 1, p3);
		unitWaterLvl = raw.substring(p3 + 1, p4);
		unitNutrisi = raw.substring(p4 + 1, p5);
		unitHumidity = raw.substring(p5 + 1);

		Serial.printf("[uart] unit -> pH:%s tAir:%s tWater:%s water:%s ppm:%s hum:%s\n",
					  unitPh.c_str(), unitTempAir.c_str(), unitTempWater.c_str(),
					  unitWaterLvl.c_str(), unitNutrisi.c_str(), unitHumidity.c_str());

		// Unit labels are only drawn on the full page draw (drawSensorCard/drawControlBlock), not
		// by the lightweight refresh — so the whole page has to be re-drawn. Restricted to the two
		// pages that actually render units, because showPage() is NOT a neutral repaint: on
		// PAGE_WIFI it routes to waitWifi(true), a "Memindai jaringan..." screen nobody asked for
		// that never clears (the 15s recovery needs waitRespons, which is false here). ESP32-MAIN
		// re-sends sensor_unit on every Firebase reconnect — i.e. exactly when the user is most
		// likely to be standing on the WiFi page, having just joined a network from it.
		if (currentPage == PAGE_DASHBOARD || currentPage == PAGE_SENSOR_LIMIT)
			showPage(currentPage);
	}
}

// EEPROM
void setupEEPROM()
{
	EEPROM.begin(EEPROM_SIZE);

	for (int i = 0; i < 5; i++)
	{
		screenTimeoutOptions[i] = EEPROM.read(i);
	}

	for (int i = 0; i < 5; i++)
	{
		if (screenTimeoutOptions[i])
		{
			SCREEN_TIMEOUT = SCREEN_TO[i];
		}
	}
}

uint8_t readModeFromEEPROM()
{
	return EEPROM.read(5);
}

void beep()
{
	digitalWrite(BUZZER, HIGH);
	delay(50);
	digitalWrite(BUZZER, LOW);
}

void setMode(uint8_t value)
{
	bool newAutoMode = (value == 0);

	if (newAutoMode != autoMode)
	{
		autoMode = newAutoMode;
		lastPump = autoMode;
		if (autoMode)
			for (int i = 0; i < 4; i++)
				doseOn[i] = false; // main mematikan semua pompa saat masuk AUTO

		EEPROM.write(5, value);
		EEPROM.commit();

		espMaster.send("selector_switch", "state", autoMode);

		if (currentPage == PAGE_DASHBOARD)
		{
			drawDashboard();
		}

		Serial.printf("Mode berubah jadi: %s\n", autoMode ? "AUTO" : "MANUAL");
	}
}

/*
ESP_LCD.send("mode", "value", autoOn);
*/