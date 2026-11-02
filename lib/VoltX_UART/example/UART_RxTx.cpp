#include <Arduino.h>
#include <VoltX_UART.h>

#define RXD2 16
#define TXD2 17

VoltX_UART ESP_Panel;

// ---- Struct & globals ----
struct Wifi
{
    String ssid;
    int signal;
    bool connected;
};

Wifi wifiList[] = {
    {"SSID_1", 4, false},
    {"SSID_2", 3, true},
    {"SSID_3", 2, false},
    {"SSID_4", 1, false}};

String wifiString[4];

// ---- Forward declarations ----
void receiveESP();
void caraKirim();
void caraTerima();
void convertWifitoString();
void parseWifi(String raw);

// ============================================================================

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    ESP_Panel.begin(Serial2);
}

void loop()
{
    receiveESP();
}

void receiveESP()
{
    if (!ESP_Panel.update())
        return;

    String type = ESP_Panel.recvType();

    if (type == "ping")
    {
        Serial.println("Dapat ping dari Panel!");
    }
    else if (type == "selector")
    {
        bool dataSelector = ESP_Panel.recv<bool>("selector", "data");
    }
}

void caraKirim()
{
    // data type only
    ESP_Panel.sendType("ping");

    // data type dan value tunggal
    ESP_Panel.send("selector", "data", true);
    ESP_Panel.send("sensor", "pH", 5.6);

    // data struct (di-String-kan dulu)
    convertWifitoString();
    ESP_Panel.send("list_wifi", "network", wifiString);
}

void caraTerima()
{
    // ambil value di dalam if type (tanpa parameter type)
    String type = ESP_Panel.recvType();
    if (type == "sensor")
    {
        float pH = ESP_Panel.recv<float>("pH");
    }

    // ambil value di luar if type (harus sertakan type)
    bool selector = ESP_Panel.recv<bool>("selector", "data");
    float pH = ESP_Panel.recv<float>("sensor", "pH");

    // data struct
    String listWifi = ESP_Panel.recv<String>("list_wifi", "network");
    parseWifi(listWifi);
}

void convertWifitoString()
{
    for (int i = 0; i < 4; i++)
    {
        wifiString[i] = wifiList[i].ssid + ";" +
                        String(wifiList[i].signal) + ";" +
                        String(wifiList[i].connected);
    }
}

void parseWifi(String raw)
{
    // contoh parse: "SSID_1;4;0"
    int sep1 = raw.indexOf(';');
    int sep2 = raw.lastIndexOf(';');
    if (sep1 < 0 || sep2 < 0 || sep1 == sep2) return;

    String ssid      = raw.substring(0, sep1);
    int    signal    = raw.substring(sep1 + 1, sep2).toInt();
    bool   connected = raw.substring(sep2 + 1).toInt();

    Serial.printf("SSID: %s | Signal: %d | Connected: %s\n",
                  ssid.c_str(), signal, connected ? "yes" : "no");
}
