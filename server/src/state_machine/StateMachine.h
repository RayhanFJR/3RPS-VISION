//==================================================================
// FILE : server/src/state_machine/StateMachine.h
//==================================================================

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <chrono>
#include <string>

// ============ STATE ENUMERATION ============
enum class SystemState {
    IDLE,
    AUTO_REHAB,
    AUTO_RETREAT,
    POST_REHAB_DELAY,
    EMERGENCY_STOP,
    RESETTING
};

// ============ CLASS DEFINITION ============

class StateMachine {
    
public:
    StateMachine();
    ~StateMachine();
    
    // ========== STATE MANAGEMENT ==========
    void setState(SystemState new_state);
    SystemState getState();
    std::string getStateString();
    
    // ========== CYCLE MANAGEMENT ==========
    void startRehabCycle(int target_cycles);
    void incrementCycle();
    int getCurrentCycle();
    int getTargetCycle();
    bool hasCycleRemaining();
    
    // ========== TRAJECTORY INDEX ==========
    void setTrajectoryIndex(int index);
    void incrementTrajectoryIndex();
    int getTrajectoryIndex();
    
    // ========== TIMING ==========
    void startDelayTimer();
    bool isDelayTimerExpired(int delay_seconds);
    
    // ========== FLAGS ==========
    void setRetreatTriggered(bool value);
    bool isRetreatTriggered();
    
    void setEmergencyFlag(bool value);
    bool isEmergencyFlagSet();
    
    // ========== DEBUG ==========
    void printState();
    
private:
    SystemState current_state;
    SystemState previous_state;
    
    // Cycle tracking
    int current_cycle;
    int target_cycle;
    
    // Trajectory tracking
    int trajectory_index;
    
    // Timing
    std::chrono::steady_clock::time_point delay_start_time;
    
    // Flags
    bool retreat_triggered;
    bool emergency_flag;
};

#endif  // STATE_MACHINE_H