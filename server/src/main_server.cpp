//==================================================================
// FILE : server/src/main_server.cpp - INTEGRATION
//==================================================================

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>

#include "trajectory/TrajectoryManager.h"
#include "modbus/ModbusServer.h"
#include "modbus/DataHandler.h"
#include "serial/SerialPort.h"
#include "state_machine/StateMachine.h"

using boost::asio::io_context;

// ============ GLOBAL VARIABLES ============
TrajectoryManager traj_manager;
ModbusServer modbus_server;
DataHandler data_handler(&modbus_server, &traj_manager);
io_context io_ctx;
SerialPort serial_port(io_ctx);
StateMachine state_machine;

// ========== TIMING ============
const int CONTROLLER_INTERVAL_MS = 100;
const int GRAFIK_INTERVAL_MS = 100;
const int POST_REHAB_DELAY_SEC = 5;

int trajectory_index = 0;
int animation_counter = 0;

// ============ HELPER FUNCTIONS ============

void handleIdleState() {
    // Check for start button
    if (data_handler.isButtonPressed(ModbusAddr::START)) {
        data_handler.clearButton(ModbusAddr::START);
        
        // Get number of cycles from HMI
        int num_cycles = modbus_server.readRegister(ModbusAddr::JUMLAH_CYCLE);
        if (num_cycles < 1) num_cycles = 1;
        
        state_machine.startRehabCycle(num_cycles);
        
        std::cout << "\n=== STARTING REHABILITATION ===" << std::endl;
        std::cout << "Target cycles: " << num_cycles << std::endl;
    }
    
    // Check for trajectory selection
    int selected_traj = data_handler.checkTrajectorySelection();
    if (selected_traj > 0) {
        data_handler.loadTrajectoryToHMI(selected_traj);
    }
    
    // Check for manual controls
    if (data_handler.isButtonPressed(ModbusAddr::MANUAL_MAJU)) {
        data_handler.clearButton(ModbusAddr::MANUAL_MAJU);
        serial_port.sendManualCommand('1');
    } else if (data_handler.isButtonPressed(ModbusAddr::MANUAL_MUNDUR)) {
        data_handler.clearButton(ModbusAddr::MANUAL_MUNDUR);
        serial_port.sendManualCommand('2');
    } else if (data_handler.isButtonPressed(ModbusAddr::MANUAL_STOP)) {
        data_handler.clearButton(ModbusAddr::MANUAL_STOP);
        serial_port.sendManualCommand('0');
    }
    
    // Check for calibration
    if (data_handler.isButtonPressed(ModbusAddr::CALIBRATE)) {
        data_handler.clearButton(ModbusAddr::CALIBRATE);
        serial_port.sendCalibrate();
        data_handler.clearAnimationData();
    }
}

void handleAutoRehabState() {
    int gait_start = traj_manager.getGaitStartIndex();
    int gait_end = traj_manager.getGaitEndIndex();
    int graph_start = traj_manager.getGraphStartIndex();
    int graph_end = traj_manager.getGraphEndIndex();
    
    // Send trajectory point to Arduino
    TrajectoryPoint point = traj_manager.getPoint(gait_start + trajectory_index);
    serial_port.sendControlData(
        point.pos1, point.pos2, point.pos3,
        point.velo1, point.velo2, point.velo3,
        point.fc1, point.fc2, point.fc3
    );
    
    // Update HMI animation
    if (graph_start + animation_counter < graph_end) {
        TrajectoryPoint graph_point = traj_manager.getPoint(graph_start + animation_counter);
        
        // Get graph coordinates (if available)
        float* graph_x = traj_manager.getGraphDataX();
        float* graph_y = traj_manager.getGraphDataY();
        
        if (graph_x && graph_y) {
            data_handler.updateAnimationPoint(
                animation_counter,
                graph_x[graph_start + animation_counter],
                graph_y[graph_start + animation_counter]
            );
        }
        
        animation_counter++;
    }
    
    // Check if trajectory finished
    if (trajectory_index >= (gait_end - gait_start)) {
        std::cout << "\n=== TRAJECTORY COMPLETE ===" << std::endl;
        
        if (state_machine.hasCycleRemaining()) {
            state_machine.setState(SystemState::POST_REHAB_DELAY);
            state_machine.startDelayTimer();
        } else {
            state_machine.setState(SystemState::IDLE);
            serial_port.sendManualCommand('0');
        }
    } else {
        trajectory_index++;
    }
}

void handlePostRehabDelayState() {
    if (state_machine.isDelayTimerExpired(POST_REHAB_DELAY_SEC)) {
        state_machine.incrementCycle();
        trajectory_index = 0;
        animation_counter = 0;
        data_handler.clearAnimationData();
        
        std::cout << "\n=== CYCLE " << state_machine.getCurrentCycle() 
                  << "/" << state_machine.getTargetCycle() << " ===" << std::endl;
        
        state_machine.setState(SystemState::AUTO_REHAB);
    }
}

void handleArduinoFeedback() {
    if (serial_port.hasData()) {
        std::string msg = serial_port.readLine();
        
        if (msg.find("RETREAT") != std::string::npos) {
            std::cout << "\n!!! RETREAT TRIGGERED !!!" << std::endl;
            state_machine.setRetreatTriggered(true);
        }
        
        if (msg.find("status:") != std::string::npos) {
            auto status = serial_port.parseStatusMessage(msg);
            data_handler.setRealTimeLoadCell(status.load);
        }
    }
}

// ============ MAIN PROGRAM ============

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "  SISTEM KONTROL REHABILITASI" << std::endl;
    std::cout << "  Multi-Trajectory + Cycle Counter" << std::endl;
    std::cout << "==========================================\n" << std::endl;
    
    // Initialize Modbus Server
    std::cout << "Initializing Modbus Server..." << std::endl;
    if (!modbus_server.initialize("0.0.0.0", 5020)) {
        std::cerr << "Failed to initialize Modbus server" << std::endl;
        return 1;
    }
    modbus_server.acceptConnection();
    
    // Load Trajectory Data
    std::cout << "\nLoading trajectory data..." << std::endl;
    if (!traj_manager.loadAllTrajectories()) {
        std::cerr << "Failed to load trajectories" << std::endl;
        return 1;
    }
    
    // Initialize Serial Port to Arduino
    std::cout << "\nConnecting to Arduino..." << std::endl;
    if (!serial_port.open("/dev/ttyACM0", 115200)) {
        std::cerr << "Warning: Could not connect to Arduino" << std::endl;
    }
    
    // Load default trajectory to HMI
    data_handler.loadTrajectoryToHMI(1);
    
    std::cout << "\n=== SYSTEM READY ===" << std::endl;
    std::cout << "Waiting for HMI commands..." << std::endl;
    
    // ============ MAIN CONTROL LOOP ============
    auto last_controller_time = std::chrono::steady_clock::now();
    auto last_grafik_time = std::chrono::steady_clock::now();
    
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        
        // ========== MODBUS COMMUNICATION ==========
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        int rc = modbus_server.receiveQuery(query, sizeof(query));
        
        if (rc > 0) {
            modbus_server.sendReply(query, rc);
        }
        
        // ========== STATE MACHINE ==========
        switch (state_machine.getState()) {
            case SystemState::IDLE:
                handleIdleState();
                break;
                
            case SystemState::AUTO_REHAB:
                // Controller interval check
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - last_controller_time).count() >= CONTROLLER_INTERVAL_MS) {
                    handleAutoRehabState();
                    last_controller_time = current_time;
                }
                break;
                
            case SystemState::POST_REHAB_DELAY:
                handlePostRehabDelayState();
                break;
                
            case SystemState::EMERGENCY_STOP:
                serial_port.sendEmergencyStop();
                break;
                
            default:
                break;
        }
        
        // ========== ARDUINO FEEDBACK ==========
        handleArduinoFeedback();
        
        // ========== SMALL DELAY ==========
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    // Cleanup
    serial_port.close();
    modbus_server.closeConnection();
    
    return 0;
}