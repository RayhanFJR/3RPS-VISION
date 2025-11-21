//==================================================================
// FILE : server/src/modbus/ModbusServer.cpp
//==================================================================
#include "ModbusServer.h"
#include <iostream>
#include <cstring>
ModbusServer::ModbusServer() {
mb_ctx = nullptr;
mb_mapping = nullptr;
server_socket = -1;
}
ModbusServer::~ModbusServer() {
if (mb_mapping) {
modbus_mapping_free(mb_mapping);
}
if (mb_ctx) {
modbus_close(mb_ctx);
modbus_free(mb_ctx);
}
}
bool ModbusServer::initialize(const char* ip, int port) {
// Create TCP context
mb_ctx = modbus_new_tcp(ip, port);
if (!mb_ctx) {
std::cerr << "Error: Failed to create Modbus TCP context" << std::endl;
return false;
}
// Allocate memory mapping
mb_mapping = modbus_mapping_new(0, 0, NUM_REGISTERS, 0);
if (!mb_mapping) {
    std::cerr << "Error: Failed to allocate Modbus mapping" << std::endl;
    modbus_free(mb_ctx);
    mb_ctx = nullptr;
    return false;
}

// Set slave ID
modbus_set_slave(mb_ctx, 1);

// Listen for connections
server_socket = modbus_tcp_listen(mb_ctx, 1);
if (server_socket == -1) {
    std::cerr << "Error: Failed to listen on Modbus port" << std::endl;
    modbus_mapping_free(mb_mapping);
    modbus_free(mb_ctx);
    return false;
}

std::cout << "[ModbusServer] Listening on " << ip << ":" << port << std::endl;
return true;
}
void ModbusServer::acceptConnection() {
if (!mb_ctx) return;
std::cout << "[ModbusServer] Waiting for HMI connection..." << std::endl;

if (modbus_tcp_accept(mb_ctx, &server_socket) == -1) {
    std::cerr << "[ModbusServer] Connection accept failed" << std::endl;
} else {
    std::cout << "[ModbusServer] HMI connected successfully" << std::endl;
}
}
void ModbusServer::closeConnection() {
if (mb_ctx) {
modbus_close(mb_ctx);
}
}
uint16_t ModbusServer::readRegister(int address) {
if (!mb_mapping || address < 0 || address >= NUM_REGISTERS) {
return 0;
}
return mb_mapping->tab_registers[address];
}
void ModbusServer::writeRegister(int address, uint16_t value) {
if (!mb_mapping || address < 0 || address >= NUM_REGISTERS) {
return;
}
mb_mapping->tab_registers[address] = value;
}
void ModbusServer::writeFloat(int address, float value) {
if (!mb_mapping || address + 1 >= NUM_REGISTERS) {
return;
}
modbus_set_float_dcba(value, mb_mapping->tab_registers + address);
}
float ModbusServer::readFloat(int address) {
if (!mb_mapping || address + 1 >= NUM_REGISTERS) {
return 0.0f;
}
return modbus_get_float_dcba(mb_mapping->tab_registers + address);
}
int ModbusServer::receiveQuery(uint8_t* query, int max_size) {
if (!mb_ctx) return -1;
modbus_set_response_timeout(mb_ctx, MODBUS_TIMEOUT_SEC, MODBUS_TIMEOUT_USEC);
return modbus_receive(mb_ctx, query);
}
void ModbusServer::sendReply(uint8_t* query, int query_size) {
if (!mb_ctx) return;
modbus_reply(mb_ctx, query, query_size, mb_mapping);
}
void ModbusServer::allocateMemory(int num_registers) {
if (mb_mapping) {
modbus_mapping_free(mb_mapping);
}
mb_mapping = modbus_mapping_new(0, 0, num_registers, 0);
}
void ModbusServer::resetRegisters() {
if (!mb_mapping) return;
for (int i = 0; i < NUM_REGISTERS; i++) {
    mb_mapping->tab_registers[i] = 0;
}
}
bool ModbusServer::isConnected() {
return mb_ctx != nullptr;
}
void ModbusServer::printStatus() {
std::cout << "\n========== MODBUS SERVER STATUS ==========" << std::endl;
std::cout << "Context: " << (mb_ctx ? "OK" : "NONE") << std::endl;
std::cout << "Mapping: " << (mb_mapping ? "OK" : "NONE") << std::endl;
std::cout << "Socket: " << server_socket << std::endl;
std::cout << "=========================================\n" << std::endl;
}