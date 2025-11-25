#ifndef STATEMACHINE_H
#define STATEMACHINE_H

//==================================================================
// STATE MACHINE DEFINITIONS
//==================================================================

enum class SystemState {
    IDLE,
    AUTO_REHAB,
    POST_REHAB_DELAY,
    EMERGENCY_STOP,
    RESETTING,
    AUTO_RETREAT
};

#endif // STATEMACHINE_H

