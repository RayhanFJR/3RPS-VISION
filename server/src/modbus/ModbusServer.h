//==================================================================
// FILE : server/src/modbus/ModbusServer.h
//==================================================================
#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H
#include <modbus/modbus.h>
#include <cstdint>
#include <string>
// ============ MODBUS REGISTER ADDRESSES ============
namespace ModbusAddr {
// Control Registers
const int MANUAL_MAJU = 99;
const int MANUAL_STOP = 100;
const int MANUAL_MUNDUR = 101;
const int CALIBRATE = 102;
const int START = 103;
const int EMERGENCY = 104;
const int RESET = 105;
// Trajectory Selection
const int TRAJEKTORI_1 = 106;
const int TRAJEKTORI_2 = 107;
const int TRAJEKTORI_3 = 108;

// Thresholds
const int THRESHOLD_1 = 130;
const int THRESHOLD_2 = 131;

// Cycle Counter
const int JUMLAH_CYCLE = 132;

// Graph Commands
const int COMMAND_REG = 120;
const int NUM_OF_DATA_CH0 = 121;
const int NUM_OF_DATA_CH1 = 122;
const int REALTIME_LOAD_CELL = 126;

// Data Registers
const int X_DATA_CH0_START = 200;
const int Y_DATA_CH0_START = 2000;
const int X_DATA_CH1_START = 4000;
const int Y_DATA_CH1_START = 6000;
}
// ============ CLASS DEFINITION ============
class ModbusServer {
public:
ModbusServer();
~ModbusServer();
// ========== INITIALIZATION ==========
bool initialize(const char* ip, int port);
void acceptConnection();
void closeConnection();

// ========== REGISTER READ/WRITE ==========
uint16_t readRegister(int address);
void writeRegister(int address, uint16_t value);
void writeFloat(int address, float value);
float readFloat(int address);

// ========== HANDLE REQUESTS ==========
int receiveQuery(uint8_t* query, int max_size);
void sendReply(uint8_t* query, int query_size);

// ========== DATA MANAGEMENT ==========
void allocateMemory(int num_registers);
void resetRegisters();

// ========== STATUS ==========
bool isConnected();
void printStatus();
private:
modbus_t* mb_ctx;
modbus_mapping_t* mb_mapping;
int server_socket;
const int NUM_REGISTERS = 8000;
const int MODBUS_TIMEOUT_SEC = 0;
const int MODBUS_TIMEOUT_USEC = 10000;
};
#endif  // MODBUS_SERVER_H