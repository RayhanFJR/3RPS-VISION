//==================================================================
// FILE: server/include/state_machine/StateHandlers.h
// DESCRIPTION:
//  Handles the logic for each system state (IDLE, AUTO_REHAB,
//  AUTO_RETREAT, POST_REHAB_DELAY, EMERGENCY_STOP, RESETTING)
//==================================================================

#ifndef STATE_HANDLERS_H
#define STATE_HANDLERS_H

#include "StateMachine.h"
#include <chrono>
#include <string>

class ModbusServer;
class DataHandler;
class TrajectoryManager;
class SerialPort;

// Timing constants
constexpr int CONTROLLER_INTERVAL_MS = 100;   // 10 Hz control
constexpr int POST_REHAB_DELAY_SEC   = 3;     // 3 second cool-down

class StateHandlers {
public:

    StateHandlers(ModbusServer* modbus,
                  DataHandler* data_handler,
                  TrajectoryManager* traj_mgr,
                  SerialPort* serial);

    ~StateHandlers() = default;

    //================= STATE CALLBACK HANDLERS =================//

    void handleIdle(
        StateMachine& state_machine,
        int& t_controller,
        int& t_grafik
    );

    void handleAutoRehab(
        StateMachine& state_machine,
        int& t_controller,
        int& t_grafik,
        std::chrono::steady_clock::time_point& last_controller_time,
        bool& animasi_grafik_berjalan,
        std::string& arduino_feedback_state
    );

    void handleAutoRetreat(
        StateMachine& state_machine,
        SerialPort& serial,
        std::string& arduino_feedback_state,
        std::chrono::steady_clock::time_point& last_retreat_time
    );

    void handlePostRehabDelay(
        StateMachine& state_machine,
        int& t_controller,
        int& t_grafik,
        bool& animasi_grafik_berjalan,
        std::chrono::steady_clock::time_point& delay_start_time
    );

    void handleEmergencyStop(
        StateMachine& state_machine,
        SerialPort& serial,
        bool& animasi_grafik_berjalan
    );

    void handleResetting(
        StateMachine& state_machine,
        SerialPort& serial,
        bool& animasi_grafik_berjalan
    );

    //================= ARDUINO FEEDBACK =================//

    void readArduinoFeedback();
    void parseArduinoFeedback(const std::string& feedback);

    std::string getArduinoState() const { return arduino_state; }
    bool isRetreatTriggered() const;
    void clearRetreatTrigger();

private:

    //============ DEPENDENCIES ============//
    ModbusServer*     modbus_server;
    DataHandler*      data_handler;
    TrajectoryManager* trajectory_manager;
    SerialPort*       serial_port;

    //============ INTERNAL STATE ============//
    std::chrono::steady_clock::time_point last_idle_print;
    std::chrono::steady_clock::time_point last_feedback_read;

    std::string arduino_state;
    bool arduino_retreat_triggered = false;
};

#endif // STATE_HANDLERS_H
