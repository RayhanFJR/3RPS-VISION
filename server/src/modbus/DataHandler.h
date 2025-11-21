//==================================================================
// FILE : server/src/modbus/DataHandler.h
//==================================================================

#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include "ModbusServer.h"
#include "../trajectory/TrajectoryManager.h"
#include <cstdint>

class DataHandler {
    
public:
    DataHandler(ModbusServer* modbus, TrajectoryManager* traj_mgr);
    ~DataHandler();
    
    // ========== GRAPH DATA MANAGEMENT ==========
    void loadTrajectoryToHMI(int trajectory_id);
    void clearAnimationData();
    void updateAnimationPoint(int point_index, float x, float y);
    
    // ========== REGISTER OPERATIONS ==========
    void setRealTimeLoadCell(float load_value);
    void setCycleCounter(int current, int target);
    void setGraphCommand(int command);  // 1=start, 2=clear, 3=animate
    
    // ========== THRESHOLD OPERATIONS ==========
    int getThreshold1();
    int getThreshold2();
    void updateThresholds(int t1, int t2);
    
    // ========== CONTROL BUTTON CHECKS ==========
    bool isButtonPressed(int button_address);
    void clearButton(int button_address);
    
    int checkTrajectorySelection();  // Returns 1, 2, 3, or 0 if none
    
    // ========== STATUS UPDATES ==========
    void printHMIStatus();
    
private:
    ModbusServer* modbus_server;
    TrajectoryManager* trajectory_manager;
    
    // Helper functions
    void writeFloatToGraph(int start_addr, float* data, int count);
};

#endif  // DATA_HANDLER_H