//==================================================================
// FILE : firmware/src/io/SerialComm.h
//==================================================================

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <Arduino.h>
#include <string.h>

class SerialComm {
    
public:
    SerialComm();
    ~SerialComm();
    
    // Initialization
    void begin(long baud_rate = 115200);
    
    // Sending data
    void sendCommand(const String& cmd);
    void sendData(const char* format, ...);
    void sendTrajectoryData(float pos1, float pos2, float pos3,
                           float velo1, float velo2, float velo3,
                           float fc1, float fc2, float fc3);
    
    void sendStatus(const String& status, const String& mode,
                   float load, float scale, float p1, float p2, float p3);
    
    void sendRetreatCommand();
    void sendCalibrationMessage();
    void sendErrorMessage(const char* error);
    
    // Receiving data
    bool hasData();
    String readLine();
    String readUntilChar(char delimiter);
    
    // Buffer management
    void clearBuffer();
    int availableBytes();
    
    // Debug
    void printHexDump(const String& data);
    
private:
    String buffer;
    const int BUFFER_MAX = 256;
};

#endif  // SERIAL_COMM_H
