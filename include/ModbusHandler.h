#ifndef MODBUSHANDLER_H
#define MODBUSHANDLER_H

#include <modbus/modbus.h>
#include <cstdint>

class ModbusHandler {
public:
    ModbusHandler();
    ~ModbusHandler();
    
    // Initialization
    bool initialize(const char* host = "0.0.0.0", int port = 5020, int slaveId = 1);
    void close();
    
    // Communication
    int receive(uint8_t* query, int maxLength);
    int reply(const uint8_t* query, int queryLength);
    
    // Register access
    modbus_mapping_t* getMapping() { return mb_mapping; }
    uint16_t readRegister(int address);
    void writeRegister(int address, uint16_t value);
    void writeFloat(int address, float value);
    float readFloat(int address);
    
    // Connection management
    bool acceptConnection();
    bool isConnected() const { return ctx != nullptr && mb_mapping != nullptr; }

private:
    modbus_t* ctx;
    modbus_mapping_t* mb_mapping;
    int server_socket;
    bool connected;
};

#endif // MODBUSHANDLER_H

