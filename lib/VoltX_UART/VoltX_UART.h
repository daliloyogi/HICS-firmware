#ifndef VOLTX_UART_H
#define VOLTX_UART_H

#include <Arduino.h>
#include <ArduinoJson.h>

#if defined(ARDUINO_ARCH_AVR)
    #define VOLTX_LINE_SIZE 256
#elif defined(ESP8266)
    #define VOLTX_LINE_SIZE 2048
#elif defined(ESP32)
    #define VOLTX_LINE_SIZE 4096
#else
    #define VOLTX_LINE_SIZE 512
#endif

class VoltX_UART {

private:

    Stream* serial;
    JsonDocument rxDoc;
    String currentType;

public:

    VoltX_UART();
    void begin(Stream &port);
    bool update();
    String recvType();

    // ==================================================
    // SEND TYPE ONLY
    void sendType(String type);

    // ==================================================
    // SEND SINGLE DATA
    template<typename T>
    void send(String type, String name, T value) {

        JsonDocument doc;

        doc["type"]  = type;
        doc["name"]  = name;
        doc["value"] = value;

        String output;
        serializeJson(doc, output);

        serial->println(output);
        serial->flush();
    }

    // ==================================================
    // SEND ARRAY 1D
    template<typename T, size_t N>
    void send(String type, String name, T (&arr)[N]) {

        JsonDocument doc;

        doc["type"] = type;
        doc["name"] = name;

        JsonArray value = doc["value"].to<JsonArray>();

        for(size_t i = 0; i < N; i++) {
            value.add(arr[i]);
        }

        String output;
        serializeJson(doc, output);

        serial->println(output);
        serial->flush();
    }

    // ==================================================
    // SEND ARRAY 2D
    template<typename T, size_t ROW, size_t COL>
    void send(String type, String name, T (&arr)[ROW][COL]) {

        JsonDocument doc;

        doc["type"] = type;
        doc["name"] = name;

        JsonArray value = doc["value"].to<JsonArray>();

        for(size_t r = 0; r < ROW; r++) {
            JsonArray row = value.add<JsonArray>();

            for(size_t c = 0; c < COL; c++) {
                row.add(arr[r][c]);
            }
        }

        String output;
        serializeJson(doc, output);

        serial->println(output);
        serial->flush();
    }

    // ==================================================
    // RECEIVE SINGLE DATA
    template<typename T>
    T recv(String type, String name) {

        if(rxDoc["type"].as<String>() != type) return T();
        if(rxDoc["name"].as<String>() != name) return T();

        return rxDoc["value"].as<T>();
    }

    // ==================================================
    // RECEIVE SINGLE DATA TANPA TYPE
    template<typename T>
    T recv(String name) {

        if(rxDoc["name"].as<String>() != name) return T();

        return rxDoc["value"].as<T>();
    }

    // ==================================================
    // RECEIVE ARRAY 1D
    template<typename T, size_t N>
    void recv(String type, String name, T (&arr)[N]) {

        if(rxDoc["type"].as<String>() != type) return;
        if(rxDoc["name"].as<String>() != name) return;

        JsonArray value = rxDoc["value"].as<JsonArray>();

        for(size_t i = 0; i < N && i < value.size(); i++) {
            arr[i] = value[i].as<T>();
        }
    }

    // ==================================================
    // RECEIVE ARRAY 1D TANPA TYPE
    template<typename T, size_t N>
    void recv(String name, T (&arr)[N]) {

        if(rxDoc["name"].as<String>() != name) return;

        JsonArray value = rxDoc["value"].as<JsonArray>();

        for(size_t i = 0; i < N && i < value.size(); i++) {
            arr[i] = value[i].as<T>();
        }
    }

    // ==================================================
    // RECEIVE ARRAY 2D
    template<typename T, size_t ROW, size_t COL>
    void recv(String type, String name, T (&arr)[ROW][COL]) {

        if(rxDoc["type"].as<String>() != type) return;
        if(rxDoc["name"].as<String>() != name) return;

        JsonArray value = rxDoc["value"].as<JsonArray>();

        for(size_t r = 0; r < ROW && r < value.size(); r++) {
            JsonArray row = value[r];

            for(size_t c = 0; c < COL && c < row.size(); c++) {
                arr[r][c] = row[c].as<T>();
            }
        }
    }

    // ==================================================
    // RECEIVE ARRAY 2D TANPA TYPE
    template<typename T, size_t ROW, size_t COL>
    void recv(String name, T (&arr)[ROW][COL]) {

        if(rxDoc["name"].as<String>() != name) return;

        JsonArray value = rxDoc["value"].as<JsonArray>();

        for(size_t r = 0; r < ROW && r < value.size(); r++) {
            JsonArray row = value[r];

            for(size_t c = 0; c < COL && c < row.size(); c++) {
                arr[r][c] = row[c].as<T>();
            }
        }
    }
};

#endif
