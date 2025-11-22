#include "StateHandlers.h"
#include "../modbus/ModbusServer.h"
#include "../modbus/DataHandler.h"
#include "../trajectory/TrajectoryManager.h"
#include "../serial/SerialPort.h"
#include <iostream>

StateHandlers::StateHandlers(
    ModbusServer* modbus,
    DataHandler* data_handler,
    TrajectoryManager* traj_mgr,
    SerialPort* serial
)
: modbus_server(modbus),
  data_handler(data_handler),
  trajectory_manager(traj_mgr),
  serial_port(serial),
  arduino_retreat_triggered(false)
{
    last_idle_print    = std::chrono::steady_clock::now();
    last_feedback_read = std::chrono::steady_clock::now();
}


//==================================================
// IDLE STATE
//==================================================
void StateHandlers::handleIdle(
    StateMachine& sm,
    int& t_controller,
    int& t_grafik
){
    // -------- Manual Commands --------
    if (data_handler->isButtonPressed(ModbusAddr::MANUAL_MAJU)) {
        serial_port->send("1");
        data_handler->clearButton(ModbusAddr::MANUAL_MAJU);
    }
    if (data_handler->isButtonPressed(ModbusAddr::MANUAL_MUNDUR)) {
        serial_port->send("2");
        data_handler->clearButton(ModbusAddr::MANUAL_MUNDUR);
    }
    if (data_handler->isButtonPressed(ModbusAddr::MANUAL_STOP)) {
        serial_port->send("0");
        data_handler->clearButton(ModbusAddr::MANUAL_STOP);
    }

    // -------- Select Trajectory --------
    int traj = data_handler->checkTrajectorySelection();
    if (traj > 0) {
        trajectory_manager->switchTrajectory(traj);
        data_handler->loadTrajectoryToHMI(traj);
    }

    // -------- Calibration --------
    if (data_handler->isButtonPressed(ModbusAddr::CALIBRATE)) {
        serial_port->send("X");
        data_handler->setGraphCommand(2);
        data_handler->clearButton(ModbusAddr::CALIBRATE);
    }

    // -------- START REHAB --------
    if (data_handler->isButtonPressed(ModbusAddr::START)) {

        sm.startRehabCycle(
            modbus_server->readRegister(ModbusAddr::JUMLAH_CYCLE)
        );

        t_controller = 0;
        t_grafik = trajectory_manager->getGraphStartIndex();

        data_handler->clearButton(ModbusAddr::START);
        sm.setState(SystemState::AUTO_REHAB);
        return;
    }
}



//==================================================
// AUTO REHAB
//==================================================
void StateHandlers::handleAutoRehab(
    StateMachine& sm,
    int& t_controller,
    int& t_grafik,
    std::chrono::steady_clock::time_point& last_controller_time,
    bool& anim,
    std::string& arduino_state
){
    auto now = std::chrono::steady_clock::now();

    // ------------------------------------------------
    // Controller Output Every 100ms
    // ------------------------------------------------
    if (now - last_controller_time >= std::chrono::milliseconds(CONTROLLER_INTERVAL_MS)) {

        int gaitStart = trajectory_manager->getGaitStartIndex();
        int gaitEnd   = trajectory_manager->getGaitEndIndex();

        if (t_controller >= (gaitEnd - gaitStart)) {
            sm.startDelayTimer();
            sm.setState(SystemState::POST_REHAB_DELAY);
            return;
        }

        int real_index = gaitStart + t_controller;
        auto p = trajectory_manager->getPoint(real_index);

        serial_port->send(
            "S" +
            std::to_string(p.pos1) + "," +
            std::to_string(p.pos2) + "," +
            std::to_string(p.pos3) + "," +
            std::to_string(p.velo1) + "," +
            std::to_string(p.velo2) + "," +
            std::to_string(p.velo3) + "," +
            std::to_string(p.fc1)   + "," +
            std::to_string(p.fc2)   + "," +
            std::to_string(p.fc3)
        );

        t_controller++;
        last_controller_time = now;
    }

    // ------------------------------------------------
    - GRAPH UPDATES
    // ------------------------------------------------
    if (anim && t_grafik < trajectory_manager->getGraphEndIndex()) {

        auto gx = trajectory_manager->getGraphDataX();
        auto gy = trajectory_manager->getGraphDataY();

        data_handler->updateAnimationPoint(
            t_grafik - trajectory_manager->getGraphStartIndex(),
            gx[t_grafik],
            gy[t_grafik]
        );

        data_handler->setGraphCommand(3);
        t_grafik++;
    }

    // ------------------------------------------------
    // Arduino RETREAT feedback
    // ------------------------------------------------
    readArduinoFeedback();
    if (isRetreatTriggered()) {
        clearRetreatTrigger();
        sm.setState(SystemState::AUTO_RETREAT);
        return;
    }
}



//==================================================
// AUTO RETREAT
//==================================================
void StateHandlers::handleAutoRetreat(
    StateMachine& sm,
    SerialPort& serial,
    std::string& arduino_state,
    std::chrono::steady_clock::time_point& last_time
){
    static int retreat_index = 0;
    static bool retreat_active = false;

    auto now = std::chrono::steady_clock::now();

    if (!retreat_active) {
        retreat_index = sm.getTrajectoryIndex() - 1;
        retreat_active = true;
        std::cout << "[RETREAT] BEGIN\n";
    }

    if (retreat_active && now - last_time >= std::chrono::milliseconds(100)) {

        if (retreat_index >= 0) {
            auto pt = trajectory_manager->getPoint(retreat_index);

            serial.send(
                "R" +
                std::to_string(pt.pos1) + "," +
                std::to_string(pt.pos2) + "," +
                std::to_string(pt.pos3)
            );

            retreat_index--;
        }
        else {
            serial.send("RETREAT_COMPLETE");
            retreat_active = false;
        }

        last_time = now;
    }

    // RESET → Move to RESETTING
    if (data_handler->isButtonPressed(ModbusAddr::RESET)) {
        serial.send("R");
        data_handler->clearButton(ModbusAddr::RESET);
        sm.setState(SystemState::RESETTING);
    }
}



//==================================================
// POST REHAB DELAY
//==================================================
void StateHandlers::handlePostRehabDelay(
    StateMachine& sm,
    int& t_controller,
    int& t_graph,
    bool& anim,
    std::chrono::steady_clock::time_point& delay_start
){
    if (sm.isDelayTimerExpired(POST_REHAB_DELAY_SEC)) {

        if (sm.hasCycleRemaining()) {
            sm.incrementCycle();

            t_controller = 0;
            t_graph = trajectory_manager->getGraphStartIndex();
            anim = true;

            sm.setState(SystemState::AUTO_REHAB);
        }
        else {
            serial_port->send("0");
            sm.setState(SystemState::IDLE);
        }
    }
}



//==================================================
// EMERGENCY STOP
//==================================================
void StateHandlers::handleEmergencyStop(
    StateMachine& sm,
    SerialPort& serial,
    bool& anim
){
    serial.send("E");
    anim = false;

    if (data_handler->isButtonPressed(ModbusAddr::RESET)) {
        serial.send("R");
        data_handler->clearButton(ModbusAddr::RESET);
        sm.setState(SystemState::RESETTING);
    }
}



//==================================================
// RESETTING
//==================================================
void StateHandlers::handleResetting(
    StateMachine& sm,
    SerialPort& serial,
    bool& anim
){
    if (data_handler->isButtonPressed(ModbusAddr::CALIBRATE)) {
        data_handler->clearButton(ModbusAddr::CALIBRATE);
        sm.setState(SystemState::IDLE);
    }
}



//==================================================
// ARDUINO FEEDBACK
//==================================================
void StateHandlers::readArduinoFeedback() {
    if (!serial_port) return;

    std::string data = serial_port->readLine();
    if (!data.empty()) {
        parseArduinoFeedback(data);
    }
}

void StateHandlers::parseArduinoFeedback(const std::string& feedback) {

    arduino_state = feedback;

    if (feedback == "RETREAT") {
        arduino_retreat_triggered = true;
    }
}

bool StateHandlers::isRetreatTriggered() const {
    return arduino_retreat_triggered;
}

void StateHandlers::clearRetreatTrigger() {
    arduino_retreat_triggered = false;
}
