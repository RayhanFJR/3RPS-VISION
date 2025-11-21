//==================================================================
// FILE : server/src/modbus/DataHandler.cpp
//==================================================================

#include "DataHandler.h"
#include <iostream>
#include <cstring>

DataHandler::DataHandler(ModbusServer* modbus, TrajectoryManager* traj_mgr) {
    modbus_server = modbus;
    trajectory_manager = traj_mgr;
}

DataHandler::~DataHandler() {
}

void DataHandler::loadTrajectoryToHMI(int trajectory_id) {
    if (!modbus_server || !trajectory_manager) return;
    
    trajectory_manager->switchTrajectory(trajectory_id);
    
    // Get trajectory data
    int graph_start = trajectory_manager->getGraphStartIndex();
    int graph_end = trajectory_manager->getGraphEndIndex();
    int graph_count = graph_end - graph_start;
    
    float* graph_x = trajectory_manager->getGraphDataX();
    float* graph_y = trajectory_manager->getGraphDataY();
    
    if (!graph_x || !graph_y) {
        std::cerr << "[DataHandler] Failed to get graph data" << std::endl;
        return;
    }
    
    // Write to Modbus registers
    for (int i = 0; i < graph_count; i++) {
        modbus_server->writeFloat(
            ModbusAddr::X_DATA_CH0_START + (i * 2),
            graph_x[graph_start + i]
        );
        modbus_server->writeFloat(
            ModbusAddr::Y_DATA_CH0_START + (i * 2),
            graph_y[graph_start + i]
        );
    }
    
    // Update point count
    modbus_server->writeRegister(ModbusAddr::NUM_OF_DATA_CH0, graph_count);
    
    // Set command to start display
    modbus_server->writeRegister(ModbusAddr::COMMAND_REG, 1);
    
    std::cout << "[DataHandler] Loaded trajectory " << trajectory_id 
              << " to HMI: " << graph_count << " points" << std::endl;
}

void DataHandler::clearAnimationData() {
    if (!modbus_server) return;
    
    for (int i = 0; i < 1400; i++) {
        modbus_server->writeRegister(ModbusAddr::X_DATA_CH1_START + i, 0);
        modbus_server->writeRegister(ModbusAddr::Y_DATA_CH1_START + i, 0);
    }
    
    modbus_server->writeRegister(ModbusAddr::NUM_OF_DATA_CH1, 0);
}

void DataHandler::updateAnimationPoint(int point_index, float x, float y) {
    if (!modbus_server) return;
    
    modbus_server->writeFloat(
        ModbusAddr::X_DATA_CH1_START + (point_index * 2),
        x
    );
    modbus_server->writeFloat(
        ModbusAddr::Y_DATA_CH1_START + (point_index * 2),
        y
    );
    
    // Increment animation counter
    uint16_t current_count = modbus_server->readRegister(ModbusAddr::NUM_OF_DATA_CH1);
    modbus_server->writeRegister(ModbusAddr::NUM_OF_DATA_CH1, current_count + 1);
    
    // Set animate command
    modbus_server->writeRegister(ModbusAddr::COMMAND_REG, 3);
}

void DataHandler::setRealTimeLoadCell(float load_value) {
    if (!modbus_server) return;
    modbus_server->writeFloat(ModbusAddr::REALTIME_LOAD_CELL, load_value);
}

void DataHandler::setCycleCounter(int current, int target) {
    if (!modbus_server) return;
    
    // Can write both to same register or split
    // For now, just write target (can be improved)
    modbus_server->writeRegister(ModbusAddr::JUMLAH_CYCLE, target);
}

void DataHandler::setGraphCommand(int command) {
    if (!modbus_server) return;
    modbus_server->writeRegister(ModbusAddr::COMMAND_REG, command);
}

int DataHandler::getThreshold1() {
    if (!modbus_server) return 20;
    return modbus_server->readRegister(ModbusAddr::THRESHOLD_1);
}

int DataHandler::getThreshold2() {
    if (!modbus_server) return 40;
    return modbus_server->readRegister(ModbusAddr::THRESHOLD_2);
}

void DataHandler::updateThresholds(int t1, int t2) {
    if (!modbus_server) return;
    modbus_server->writeRegister(ModbusAddr::THRESHOLD_1, t1);
    modbus_server->writeRegister(ModbusAddr::THRESHOLD_2, t2);
}

bool DataHandler::isButtonPressed(int button_address) {
    if (!modbus_server) return false;
    uint16_t value = modbus_server->readRegister(button_address);
    return value == 1;
}

void DataHandler::clearButton(int button_address) {
    if (!modbus_server) return;
    modbus_server->writeRegister(button_address, 0);
}

int DataHandler::checkTrajectorySelection() {
    if (!modbus_server) return 0;
    
    if (isButtonPressed(ModbusAddr::TRAJEKTORI_1)) {
        clearButton(ModbusAddr::TRAJEKTORI_1);
        return 1;
    }
    if (isButtonPressed(ModbusAddr::TRAJEKTORI_2)) {
        clearButton(ModbusAddr::TRAJEKTORI_2);
        return 2;
    }
    if (isButtonPressed(ModbusAddr::TRAJEKTORI_3)) {
        clearButton(ModbusAddr::TRAJEKTORI_3);
        return 3;
    }
    
    return 0;
}

void DataHandler::printHMIStatus() {
    std::cout << "\n========== HMI STATUS ==========" << std::endl;
    std::cout << "Threshold 1: " << getThreshold1() << " N" << std::endl;
    std::cout << "Threshold 2: " << getThreshold2() << " N" << std::endl;
    std::cout << "Load Cell: " << modbus_server->readFloat(ModbusAddr::REALTIME_LOAD_CELL) << " N" << std::endl;
    std::cout << "Animation Counter: " << modbus_server->readRegister(ModbusAddr::NUM_OF_DATA_CH1) << std::endl;
    std::cout << "================================\n" << std::endl;
}