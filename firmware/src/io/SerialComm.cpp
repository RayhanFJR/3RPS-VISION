//==================================================================
// FILE : firmware/src/io/SerialComm.cpp
//==================================================================

#include "SerialComm.h"

SerialComm::SerialComm() {
    buffer = "";
}

SerialComm::~SerialComm() {
}

void SerialComm::begin(long baud_rate) {
    Serial.begin(baud_rate);
    
    // Wait for serial connection
    while (!Serial && millis() < 3000) {
        delay(10);
    }
    
    delay(100);
    Serial.println("[SerialComm] Initialized");
}

void SerialComm::sendCommand(const String& cmd) {
    Serial.print(cmd);
    Serial.println();  // Add newline
}

void SerialComm::sendData(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    Serial.println(buffer);
}

void SerialComm::sendTrajectoryData(float pos1, float pos2, float pos3,
                                   float velo1, float velo2, float velo3,
                                   float fc1, float fc2, float fc3) {
    Serial.print("TRAJ:");
    Serial.print(pos1, 3);
    Serial.print(",");
    Serial.print(pos2, 3);
    Serial.print(",");
    Serial.print(pos3, 3);
    Serial.print(",");
    Serial.print(velo1, 3);
    Serial.print(",");
    Serial.print(velo2, 3);
    Serial.print(",");
    Serial.print(velo3, 3);
    Serial.print(",");
    Serial.print(fc1, 3);
    Serial.print(",");
    Serial.print(fc2, 3);
    Serial.print(",");
    Serial.print(fc3, 3);
    Serial.println();
}

void SerialComm::sendStatus(const String& status, const String& mode,
                           float load, float scale, float p1, float p2, float p3) {
    Serial.print("status:");
    Serial.print(status);
    Serial.print(",mode:");
    Serial.print(mode);
    Serial.print(",load:");
    Serial.print(load, 2);
    Serial.print(",scale:");
    Serial.print(scale, 2);
    Serial.print(",pos:");
    Serial.print(p1, 2);
    Serial.print(",");
    Serial.print(p2, 2);
    Serial.print(",");
    Serial.print(p3, 2);
    Serial.println();
}

void SerialComm::sendRetreatCommand() {
    Serial.println("RETREAT");
}

void SerialComm::sendCalibrationMessage() {
    Serial.println("[System] Calibration complete");
}

void SerialComm::sendErrorMessage(const char* error) {
    Serial.print("[ERROR] ");
    Serial.println(error);
}

bool SerialComm::hasData() {
    return Serial.available() > 0;
}

String SerialComm::readLine() {
    String line = "";
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (line.length() > 0) {
                return line;
            }
        } else {
            line += c;
        }
    }
    return "";
}

String SerialComm::readUntilChar(char delimiter) {
    String data = "";
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == delimiter) {
            return data;
        }
        data += c;
    }
    return data;
}

void SerialComm::clearBuffer() {
    buffer = "";
    while (Serial.available() > 0) {
        Serial.read();
    }
}

int SerialComm::availableBytes() {
    return Serial.available();
}

void SerialComm::printHexDump(const String& data) {
    Serial.print("[HEX] ");
    for (int i = 0; i < data.length(); i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
