#include "VoltX_UART.h"

VoltX_UART::VoltX_UART() {

}

void VoltX_UART::begin(Stream &port) {
    serial = &port;
}

bool VoltX_UART::update() {

    currentType = "";

    if(!serial->available()) return false;

    char line[VOLTX_LINE_SIZE];

    size_t len = serial->readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';
    if(len > 0 && line[len - 1] == '\r') line[--len] = '\0';

    rxDoc.clear();

    DeserializationError error = deserializeJson(rxDoc, line);

    if(error) return false;

    currentType = rxDoc["type"].as<String>();

    return true;
}

String VoltX_UART::recvType() {
    return currentType;
}

void VoltX_UART::sendType(String type) {
    JsonDocument doc;

    doc["type"] = type;

    String output;

    serializeJson(doc, output);

    serial->println(output);

    serial->flush();
}