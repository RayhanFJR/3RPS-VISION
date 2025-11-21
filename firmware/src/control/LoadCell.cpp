//==================================================================
// FILE : firmware/src/io/LoadCell.cpp
//==================================================================

#include "LoadCell.h"

LoadCell::LoadCell() {
    offset = 0;
    latest_load = 0.0;
    smoothed_load = 0.0;
    threshold1 = THRESHOLD1_DEFAULT;
    threshold2 = THRESHOLD2_DEFAULT;
}

LoadCell::~LoadCell() {
}

void LoadCell::begin() {
    pinMode(LOADCELL_DOUT_PIN, INPUT);
    pinMode(LOADCELL_SCK_PIN, OUTPUT);
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    
    delay(100);
    calibrate();  // Auto-calibrate at startup
    
    Serial.println("[LoadCell] Initialized");
}

void LoadCell::calibrate() {
    // Read multiple times to get stable offset
    long sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += readRaw();
        delay(10);
    }
    offset = sum / 10;
    smoothed_load = 0.0;
    
    Serial.print("[LoadCell] Calibrated - Offset: ");
    Serial.println(offset);
}

long LoadCell::readRaw() {
    // Wait for data ready
    while (digitalRead(LOADCELL_DOUT_PIN));
    
    long result = 0;
    
    // Read 24 bits
    for (int i = 0; i < 24; i++) {
        digitalWrite(LOADCELL_SCK_PIN, HIGH);
        delayMicroseconds(1);
        result = result << 1;
        
        if (digitalRead(LOADCELL_DOUT_PIN)) {
            result++;
        }
        
        digitalWrite(LOADCELL_SCK_PIN, LOW);
        delayMicroseconds(1);
    }
    
    // Pulse clock for next conversion
    digitalWrite(LOADCELL_SCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    delayMicroseconds(1);
    
    // Handle 24-bit two's complement
    if (result & 0x800000) {
        result |= ~0xFFFFFF;
    }
    
    return result;
}

float LoadCell::readLoad() {
    long raw_value = readRaw();
    long effective_value = raw_value - offset;
    
    // Convert to Newton (calibration factor)
    latest_load = effective_value / 10000.0;
    
    // Exponential smoothing
    smoothed_load = (LOAD_ALPHA * latest_load) + 
                    ((1.0 - LOAD_ALPHA) * smoothed_load);
    
    return smoothed_load;
}

float LoadCell::getSmoothedLoad() {
    return smoothed_load;
}

void LoadCell::setOffset(long new_offset) {
    offset = new_offset;
}

long LoadCell::getOffset() {
    return offset;
}

void LoadCell::setThreshold1(int threshold) {
    threshold1 = threshold;
}

void LoadCell::setThreshold2(int threshold) {
    threshold2 = threshold;
}

int LoadCell::getThreshold1() {
    return threshold1;
}

int LoadCell::getThreshold2() {
    return threshold2;
}

bool LoadCell::isDataReady() {
    return digitalRead(LOADCELL_DOUT_PIN) == LOW;
}


//==================================================================
// FILE 3: firmware/src/io/CommandParser.h
//==================================================================

#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>
#include <string.h>

// ============ COMMAND STRUCTURE ============
struct Command {
    char command_type;          // 'S', 'R', 'X', 'E', 'T', 'K', 'P', etc.
    float param1, param2, param3;
    float param4, param5, param6;
    float param7, param8, param9;
    bool valid;
};

class CommandParser {
    
public:
    CommandParser();
    ~CommandParser();
    
    // Parse incoming string into command
    Command parseCommand(String data);
    
    // Trajectory command parsing
    void parseTrajectoryCommand(Command& cmd, String data, bool is_retreat);
    
    // Threshold command parsing
    void parseThresholdCommand(Command& cmd, String data);
    
    // Gain command parsing
    void parseOuterGainCommand(Command& cmd, String data);
    void parseInnerGainCommand(Command& cmd, String data);
    
    // Utility
    bool isValidCommand(char type);
    void printCommand(Command& cmd);
    
private:
    // Helper parsing functions
    float extractFloat(String& data, int start_idx, int end_idx);
    int countCommas(String data);
};

#endif  // COMMAND_PARSER_H


//==================================================================
// FILE 4: firmware/src/io/CommandParser.cpp
//==================================================================

#include "CommandParser.h"

CommandParser::CommandParser() {
}

CommandParser::~CommandParser() {
}

Command CommandParser::parseCommand(String data) {
    Command cmd;
    cmd.valid = false;
    cmd.command_type = data.charAt(0);
    
    if (!isValidCommand(cmd.command_type)) {
        return cmd;
    }
    
    // Route to appropriate parser
    switch (cmd.command_type) {
        case 'S':  // Trajectory forward
            parseTrajectoryCommand(cmd, data, false);
            cmd.valid = true;
            break;
            
        case 'R':  // Trajectory retreat
            parseTrajectoryCommand(cmd, data, true);
            cmd.valid = true;
            break;
            
        case 'T':  // Thresholds
            parseThresholdCommand(cmd, data);
            cmd.valid = true;
            break;
            
        case 'K':  // Outer loop gains
            parseOuterGainCommand(cmd, data);
            cmd.valid = true;
            break;
            
        case 'P':  // Inner loop gains
            parseInnerGainCommand(cmd, data);
            cmd.valid = true;
            break;
            
        case 'X':  // Calibrate
        case 'E':  // Emergency stop
        case '0':  // Manual stop
        case '1':  // Manual forward
        case '2':  // Manual backward
            cmd.valid = true;
            break;
            
        default:
            cmd.valid = false;
    }
    
    return cmd;
}

void CommandParser::parseTrajectoryCommand(Command& cmd, String data, bool is_retreat) {
    // Remove command character
    data.remove(0, 1);
    
    // Parse: pos1,pos2,pos3,velo1,velo2,velo3,fc1,fc2,fc3
    int idx = 0;
    int comma_idx = 0;
    
    // pos1
    comma_idx = data.indexOf(',', idx);
    cmd.param1 = data.substring(idx, comma_idx).toFloat();
    idx = comma_idx + 1;
    
    // pos2
    comma_idx = data.indexOf(',', idx);
    cmd.param2 = data.substring(idx, comma_idx).toFloat();
    idx = comma_idx + 1;
    
    // pos3
    comma_idx = data.indexOf(',', idx);
    cmd.param3 = data.substring(idx, comma_idx).toFloat();
    idx = comma_idx + 1;
    
    // velo1
    comma_idx = data.indexOf(',', idx);
    cmd.param4 = data.substring(idx, comma_idx).toFloat();
    idx = comma_idx + 1;
    
    // velo2
    comma_idx = data.indexOf(',', idx);
    cmd.param5 = data.substring(idx, comma_idx).toFloat();
    idx = comma_idx + 1;
    
    // velo3
    comma_idx = data.indexOf(',', idx);
    cmd.param6 = data.substring(idx, comma_idx).toFloat();
    idx = comma_idx + 1;
    
    // fc1
    comma_idx = data.indexOf(',', idx);
    cmd.param7 = data.substring(idx, comma_idx).toFloat();
    idx = comma_idx + 1;
    
    // fc2
    comma_idx = data.indexOf(',', idx);
    cmd.param8 = data.substring(idx, comma_idx).toFloat();
    idx = comma_idx + 1;
    
    // fc3
    cmd.param9 = data.substring(idx).toFloat();
    
    // Scale velocity for retreat
    if (is_retreat) {
        cmd.param4 *= RETREAT_VELOCITY_SCALE;
        cmd.param5 *= RETREAT_VELOCITY_SCALE;
        cmd.param6 *= RETREAT_VELOCITY_SCALE;
    }
}

void CommandParser::parseThresholdCommand(Command& cmd, String data) {
    // Format: Tthreshold1,threshold2
    data.remove(0, 1);  // Remove 'T'
    
    int comma_idx = data.indexOf(',');
    cmd.param1 = data.substring(0, comma_idx).toInt();
    cmd.param2 = data.substring(comma_idx + 1).toInt();
}

void CommandParser::parseOuterGainCommand(Command& cmd, String data) {
    // Format: Kmotor_id,kp,kd
    data.remove(0, 1);  // Remove 'K'
    
    int comma1 = data.indexOf(',');
    cmd.param1 = data.substring(0, comma1).toInt();
    
    int comma2 = data.indexOf(',', comma1 + 1);
    cmd.param2 = data.substring(comma1 + 1, comma2).toFloat();
    
    cmd.param3 = data.substring(comma2 + 1).toFloat();
}

void CommandParser::parseInnerGainCommand(Command& cmd, String data) {
    // Format: Pmotor_id,kpc,kdc
    data.remove(0, 1);  // Remove 'P'
    
    int comma1 = data.indexOf(',');
    cmd.param1 = data.substring(0, comma1).toInt();
    
    int comma2 = data.indexOf(',', comma1 + 1);
    cmd.param2 = data.substring(comma1 + 1, comma2).toFloat();
    
    cmd.param3 = data.substring(comma2 + 1).toFloat();
}

bool CommandParser::isValidCommand(char type) {
    return (type == 'S' || type == 'R' || type == 'X' || type == 'E' ||
            type == 'T' || type == 'K' || type == 'P' ||
            type == '0' || type == '1' || type == '2');
}

void CommandParser::printCommand(Command& cmd) {
    Serial.print("[Parser] Type:");
    Serial.print(cmd.command_type);
    Serial.print(" | P1:");
    Serial.print(cmd.param1, 2);
    Serial.print(" | P2:");
    Serial.print(cmd.param2, 2);
    Serial.print(" | Valid:");
    Serial.println(cmd.valid ? "YES" : "NO");
}