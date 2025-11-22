//==================================================================
// FILE: server/src/modbus/ModbusServer.h
// Modbus register address definitions
//==================================================================

#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H

#include <modbus/modbus.h>
#include <string>
#include <cstdint>

// ========== MODBUS REGISTER MAP ==========
namespace ModbusAddr {
    // Control buttons (Coils)
    constexpr int START = 0;
    constexpr int STOP = 1;
    constexpr int EMERGENCY = 2;
    constexpr int RESET = 3;
    constexpr int CALIBRATE = 4;
    constexpr int MANUAL_MAJU = 5;
    constexpr int MANUAL_MUNDUR = 6;
    constexpr int MANUAL_STOP = 7;
    
    // Trajectory selection (Holding Registers)
    constexpr int TRAJ_SELECT_1 = 100;
    constexpr int TRAJ_SELECT_2 = 101;
    constexpr int TRAJ_SELECT_3 = 102;
    constexpr int TRAJ_SELECT_4 = 103;
    constexpr int TRAJ_SELECT_5 = 104;
    
    // Configuration
    constexpr int JUMLAH_CYCLE = 200;      // Number of cycles to run
    constexpr int CYCLE_COUNTER = 201;     // Current cycle number
    constexpr int CYCLE_COMPLETE = 202;    // Flag: all cycles done
    
    // System status
    constexpr int SYSTEM_STATUS = 300;     // 0=OK, 0xFF=Emergency
    constexpr int ARDUINO_CONNECTED = 301; // 1=connected, 0=disconnected
    
    // ========== SENSOR DATA (NEW!) ==========
    constexpr int LOADCELL_1 = 400;        // Load cell 1 (0.01 N units)
    constexpr int LOADCELL_2 = 401;        // Load cell 2 (0.01 N units)
    constexpr int LOADCELL_3 = 402;        // Load cell 3 (0.01 N units)
    
    constexpr int ENCODER_1 = 410;         // Encoder 1 position
    constexpr int ENCODER_2 = 411;         // Encoder 2 position
    constexpr int ENCODER_3 = 412;         // Encoder 3 position
    
    constexpr int MOTOR_CURRENT_1 = 420;   // Motor 1 current (mA)
    constexpr int MOTOR_CURRENT_2 = 421;   // Motor 2 current (mA)
    constexpr int MOTOR_CURRENT_3 = 422;   // Motor 3 current (mA)
    
    // Trajectory data (for HMI display)
    constexpr int TRAJ_DATA_START = 1000;  // Start of trajectory array
    constexpr int GRAPH_DATA_START = 5000; // Start of graph data
    constexpr int ANIMATION_INDEX = 9000;  // Current animation point
}

class ModbusServer {
public:
    ModbusServer();
    ~ModbusServer();
    
    // Connection management
    bool initialize(const std::string& ip, int port);
    bool acceptConnection();
    void closeConnection();
    bool isConnected() const;
    
    // Modbus operations
    int receiveQuery(uint8_t* query, int max_length);
    void sendReply(const uint8_t* query, int query_length);
    
    // Register access
    uint16_t readRegister(int address);
    void writeRegister(int address, uint16_t value);
    bool readCoil(int address);
    void writeCoil(int address, bool value);
    
    // Direct access to mapping (for batch operations)
    modbus_mapping_t* getMapping() { return mb_mapping; }

private:
    modbus_t* ctx;
    modbus_mapping_t* mb_mapping;
    int server_socket;
    bool connected;
};

#endif // MODBUS_SERVER_H