#include "ModbusHandler.h"
#include "Config.h"
#include <iostream>
#include <cerrno>

ModbusHandler::ModbusHandler() : ctx(nullptr), mb_mapping(nullptr), server_socket(-1), connected(false) {
}

ModbusHandler::~ModbusHandler() {
    close();
}

bool ModbusHandler::initialize(const char* host, int port, int slaveId) {
    ctx = modbus_new_tcp(host, port);
    if (ctx == nullptr) {
        std::cerr << "Error creating Modbus context" << std::endl;
        return false;
    }
    
    modbus_set_slave(ctx, slaveId);
    server_socket = modbus_tcp_listen(ctx, 1);
    
    if (server_socket == -1) {
        std::cerr << "Error listening on Modbus port" << std::endl;
        modbus_free(ctx);
        ctx = nullptr;
        return false;
    }
    
    mb_mapping = modbus_mapping_new(0, 0, MODBUS_REGISTER_COUNT, 0);
    if (mb_mapping == nullptr) {
        std::cerr << "Error creating Modbus mapping" << std::endl;
        modbus_close(ctx);
        modbus_free(ctx);
        ctx = nullptr;
        return false;
    }
    
    std::cout << "Modbus server initialized. Waiting for HMI connection..." << std::endl;
    return acceptConnection();
}

bool ModbusHandler::acceptConnection() {
    if (ctx == nullptr) return false;
    
    if (modbus_tcp_accept(ctx, &server_socket) == -1) {
        return false;
    }
    
    connected = true;
    std::cout << "HMI connection accepted." << std::endl;
    return true;
}

void ModbusHandler::close() {
    if (mb_mapping != nullptr) {
        modbus_mapping_free(mb_mapping);
        mb_mapping = nullptr;
    }
    
    if (ctx != nullptr) {
        modbus_close(ctx);
        modbus_free(ctx);
        ctx = nullptr;
    }
    
    connected = false;
}

int ModbusHandler::receive(uint8_t* query, int maxLength) {
    if (!connected || ctx == nullptr) return -1;
    
    modbus_set_response_timeout(ctx, 0, 10000);
    int rc = modbus_receive(ctx, query);
    
    if (rc == -1 && errno != EWOULDBLOCK && errno != EAGAIN) {
        std::cerr << "\nHMI connection lost: " << modbus_strerror(errno) 
                  << ". Waiting for new connection..." << std::endl;
        modbus_close(ctx);
        connected = false;
        
        // Try to reconnect
        if (acceptConnection()) {
            std::cout << "HMI connection re-established." << std::endl;
        }
    }
    
    return rc;
}

int ModbusHandler::reply(const uint8_t* query, int queryLength) {
    if (!connected || ctx == nullptr || mb_mapping == nullptr) return -1;
    return modbus_reply(ctx, query, queryLength, mb_mapping);
}

uint16_t ModbusHandler::readRegister(int address) {
    if (mb_mapping == nullptr || address < 0 || address >= MODBUS_REGISTER_COUNT) {
        return 0;
    }
    return mb_mapping->tab_registers[address];
}

void ModbusHandler::writeRegister(int address, uint16_t value) {
    if (mb_mapping == nullptr || address < 0 || address >= MODBUS_REGISTER_COUNT) {
        return;
    }
    mb_mapping->tab_registers[address] = value;
}

void ModbusHandler::writeFloat(int address, float value) {
    if (mb_mapping == nullptr || address < 0 || address >= MODBUS_REGISTER_COUNT - 1) {
        return;
    }
    modbus_set_float_dcba(value, mb_mapping->tab_registers + address);
}

float ModbusHandler::readFloat(int address) {
    if (mb_mapping == nullptr || address < 0 || address >= MODBUS_REGISTER_COUNT - 1) {
        return 0.0f;
    }
    return modbus_get_float_dcba(mb_mapping->tab_registers + address);
}

