//==================================================================
// FILE : server/src/state_machine/StateMachine.cpp
//==================================================================

#include "StateMachine.h"
#include <iostream>
#include <iomanip>

StateMachine::StateMachine() {
    current_state = SystemState::IDLE;
    previous_state = SystemState::IDLE;
    current_cycle = 0;
    target_cycle = 1;
    trajectory_index = 0;
    retreat_triggered = false;
    emergency_flag = false;
}

StateMachine::~StateMachine() {
}

void StateMachine::setState(SystemState new_state) {
    if (current_state != new_state) {
        previous_state = current_state;
        current_state = new_state;
        
        std::cout << "[StateMachine] Transition: " 
                  << getStateString() << std::endl;
    }
}

SystemState StateMachine::getState() {
    return current_state;
}

std::string StateMachine::getStateString() {
    switch (current_state) {
        case SystemState::IDLE:
            return "IDLE";
        case SystemState::AUTO_REHAB:
            return "AUTO_REHAB";
        case SystemState::AUTO_RETREAT:
            return "AUTO_RETREAT";
        case SystemState::POST_REHAB_DELAY:
            return "POST_REHAB_DELAY";
        case SystemState::EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        case SystemState::RESETTING:
            return "RESETTING";
        default:
            return "UNKNOWN";
    }
}

void StateMachine::startRehabCycle(int target_cycles) {
    target_cycle = (target_cycles > 0) ? target_cycles : 1;
    current_cycle = 1;
    trajectory_index = 0;
    retreat_triggered = false;
    
    setState(SystemState::AUTO_REHAB);
    
    std::cout << "[StateMachine] Starting rehab cycle" << std::endl;
    std::cout << "  Target cycles: " << target_cycle << std::endl;
    std::cout << "  Current cycle: " << current_cycle << "/" << target_cycle << std::endl;
}

void StateMachine::incrementCycle() {
    current_cycle++;
    trajectory_index = 0;
    retreat_triggered = false;
}

int StateMachine::getCurrentCycle() {
    return current_cycle;
}

int StateMachine::getTargetCycle() {
    return target_cycle;
}

bool StateMachine::hasCycleRemaining() {
    return current_cycle < target_cycle;
}

void StateMachine::setTrajectoryIndex(int index) {
    trajectory_index = index;
}

void StateMachine::incrementTrajectoryIndex() {
    trajectory_index++;
}

int StateMachine::getTrajectoryIndex() {
    return trajectory_index;
}

void StateMachine::startDelayTimer() {
    delay_start_time = std::chrono::steady_clock::now();
}

bool StateMachine::isDelayTimerExpired(int delay_seconds) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - delay_start_time).count();
    
    return elapsed >= delay_seconds;
}

void StateMachine::setRetreatTriggered(bool value) {
    retreat_triggered = value;
}

bool StateMachine::isRetreatTriggered() {
    return retreat_triggered;
}

void StateMachine::setEmergencyFlag(bool value) {
    emergency_flag = value;
}

bool StateMachine::isEmergencyFlagSet() {
    return emergency_flag;
}

void StateMachine::printState() {
    std::cout << "\n========== STATE MACHINE ==========" << std::endl;
    std::cout << "State: " << getStateString() << std::endl;
    std::cout << "Cycle: " << current_cycle << "/" << target_cycle << std::endl;
    std::cout << "Traj Index: " << trajectory_index << std::endl;
    std::cout << "Retreat: " << (retreat_triggered ? "YES" : "NO") << std::endl;
    std::cout << "Emergency: " << (emergency_flag ? "YES" : "NO") << std::endl;
    std::cout << "===================================\n" << std::endl;
}