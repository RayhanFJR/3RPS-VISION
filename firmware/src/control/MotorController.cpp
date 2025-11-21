//==================================================================
// FILE : firmware/src/control/MotorController.cpp
//==================================================================

#include "MotorController.h"

// ============ CONSTRUCTOR ============

MotorController::MotorController() {
    // Initialize Motor 1
    m1.ref_pos = 0;
    m1.ref_velo = 0;
    m1.ref_current = 0;
    m1.ref_force = 0;
    m1.ref_accel = 0;
    m1.encoder_count = 0;
    m1.actual_pos = 0;
    m1.actual_velo = 0;
    m1.actual_current = 0;
    m1.error_pos = 0;
    m1.error_velo = 0;
    m1.prev_error = 0;
    m1.pwm_value = 0;
    m1.direction = 0;
    m1.kp = MOTOR1_KP;
    m1.kd = MOTOR1_KD;
    m1.kpc = MOTOR1_KPC;
    m1.kdc = MOTOR1_KDC;
    m1.prev_pos = 0;
    m1.prev_ref_velo = 0;
    m1.prev_state = 0;
    
    // Initialize Motor 2
    m2.ref_pos = 0;
    m2.ref_velo = 0;
    m2.ref_current = 0;
    m2.ref_force = 0;
    m2.ref_accel = 0;
    m2.encoder_count = 0;
    m2.actual_pos = 0;
    m2.actual_velo = 0;
    m2.actual_current = 0;
    m2.error_pos = 0;
    m2.error_velo = 0;
    m2.prev_error = 0;
    m2.pwm_value = 0;
    m2.direction = 0;
    m2.kp = MOTOR2_KP;
    m2.kd = MOTOR2_KD;
    m2.kpc = MOTOR2_KPC;
    m2.kdc = MOTOR2_KDC;
    m2.prev_pos = 0;
    m2.prev_ref_velo = 0;
    m2.prev_state = 0;
    
    // Initialize Motor 3
    m3.ref_pos = 0;
    m3.ref_velo = 0;
    m3.ref_current = 0;
    m3.ref_force = 0;
    m3.ref_accel = 0;
    m3.encoder_count = 0;
    m3.actual_pos = 0;
    m3.actual_velo = 0;
    m3.actual_current = 0;
    m3.error_pos = 0;
    m3.error_velo = 0;
    m3.prev_error = 0;
    m3.pwm_value = 0;
    m3.direction = 0;
    m3.kp = MOTOR3_KP;
    m3.kd = MOTOR3_KD;
    m3.kpc = MOTOR3_KPC;
    m3.kdc = MOTOR3_KDC;
    m3.prev_pos = 0;
    m3.prev_ref_velo = 0;
    m3.prev_state = 0;
    
    // Load tracking
    current_load = 0.0;
    smoothed_load = 0.0;
    
    // Timing
    last_encoder_time = 0;
    last_velocity_time = 0;
}

MotorController::~MotorController() {
    stopAllMotors();
}

// ============ INITIALIZATION ============

void MotorController::begin() {
    // Setup motor PWM pins
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM2, OUTPUT);
    pinMode(RPWM3, OUTPUT);
    pinMode(LPWM3, OUTPUT);
    
    // Setup encoder input pins
    pinMode(ENC1, INPUT);
    pinMode(ENC2, INPUT);
    pinMode(ENC3, INPUT);
    
    // Setup current sensor pins
    pinMode(CURRENT_SEN1, INPUT);
    pinMode(CURRENT_SEN2, INPUT);
    pinMode(CURRENT_SEN3, INPUT);
    
    stopAllMotors();
    
    Serial.println("[MotorController] Initialized successfully");
}

void MotorController::calibrate() {
    m1.encoder_count = 0;
    m2.encoder_count = 0;
    m3.encoder_count = 0;
    
    m1.actual_pos = 0;
    m2.actual_pos = 0;
    m3.actual_pos = 0;
    
    m1.prev_pos = 0;
    m2.prev_pos = 0;
    m3.prev_pos = 0;
    
    stopAllMotors();
    
    Serial.println("[MotorController] Calibration complete - encoders zeroed");
}

void MotorController::stopAllMotors() {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, 0);
    
    m1.pwm_value = 0;
    m2.pwm_value = 0;
    m3.pwm_value = 0;
    
    m1.direction = 0;
    m2.direction = 0;
    m3.direction = 0;
}

// ============ SET REFERENCE VALUES ============

void MotorController::setReferencePosition(int motor_id, float position) {
    MotorState* motor = getMotorState(motor_id);
    if (motor) motor->ref_pos = position;
}

void MotorController::setReferenceVelocity(int motor_id, float velocity) {
    MotorState* motor = getMotorState(motor_id);
    if (motor) motor->ref_velo = velocity;
}

void MotorController::setReferenceCurrent(int motor_id, float current) {
    MotorState* motor = getMotorState(motor_id);
    if (motor) motor->ref_current = current;
}

void MotorController::setReferenceForce(int motor_id, float force) {
    MotorState* motor = getMotorState(motor_id);
    if (motor) motor->ref_force = force;
}

// ============ GET ACTUAL VALUES ============

float MotorController::getActualPosition(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    return motor ? motor->actual_pos : 0.0;
}

float MotorController::getActualVelocity(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    return motor ? motor->actual_velo : 0.0;
}

float MotorController::getActualCurrent(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    return motor ? motor->actual_current : 0.0;
}

int MotorController::getEncoderCount(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    return motor ? motor->encoder_count : 0;
}

// ============ UPDATE ENCODERS ============

void MotorController::updateEncoders() {
    updateSingleEncoder(1, ENC1, m1);
    updateSingleEncoder(2, ENC2, m2);
    updateSingleEncoder(3, ENC3, m3);
}

void MotorController::updateSingleEncoder(int motor_id, int pin, MotorState& motor) {
    int current_state = digitalRead(pin);
    
    if (current_state > motor.prev_state) {
        if (motor.pwm_value > 0) {
            motor.encoder_count++;
        } else if (motor.pwm_value < 0) {
            motor.encoder_count--;
        }
    }
    
    motor.actual_pos = motor.encoder_count * POSITION_SCALE;
    motor.prev_state = current_state;
}

// ============ UPDATE VELOCITIES ============

void MotorController::updateVelocities() {
    // dt = 10ms
    m1.actual_velo = (m1.actual_pos - m1.prev_pos) / 0.01;
    m2.actual_velo = (m2.actual_pos - m2.prev_pos) / 0.01;
    m3.actual_velo = (m3.actual_pos - m3.prev_pos) / 0.01;
    
    m1.prev_pos = m1.actual_pos;
    m2.prev_pos = m2.actual_pos;
    m3.prev_pos = m3.actual_pos;
}

// ============ CALCULATE CTC (Outer Loop) ============

void MotorController::calculateCTCControl() {
    // Error calculation
    m1.error_pos = m1.ref_pos - m1.actual_pos;
    m2.error_pos = m2.ref_pos - m2.actual_pos;
    m3.error_pos = m3.ref_pos - m3.actual_pos;
    
    // Reference current (with load-based Kp scaling)
    float kp1_scaled = m1.kp;  // Can add load scaling here
    float kp2_scaled = m2.kp;
    float kp3_scaled = m3.kp;
    
    m1.ref_current = calculateCTC(
        m1.error_pos, kp1_scaled,
        (m1.ref_velo - m1.actual_velo), m1.kd,
        m1.ref_force, GEAR_RATIO, MOTOR_KT
    );
    
    m2.ref_current = calculateCTC(
        m2.error_pos, kp2_scaled,
        (m2.ref_velo - m2.actual_velo), m2.kd,
        m2.ref_force, GEAR_RATIO, MOTOR_KT
    );
    
    m3.ref_current = calculateCTC(
        m3.error_pos, kp3_scaled,
        (m3.ref_velo - m3.actual_velo), m3.kd,
        m3.ref_force, GEAR_RATIO, MOTOR_KT
    );
}

// ============ CALCULATE PD (Inner Loop) ============

void MotorController::calculatePDControl() {
    // Current error derivative
    float error1_rate = (m1.ref_current - m1.actual_current) / 0.1;
    float error2_rate = (m2.ref_current - m2.actual_current) / 0.1;
    float error3_rate = (m3.ref_current - m3.actual_current) / 0.1;
    
    // PD control
    m1.pwm_value = calculatePD(
        (m1.ref_current - m1.actual_current), m1.kpc,
        error1_rate, m1.kdc
    );
    m2.pwm_value = calculatePD(
        (m2.ref_current - m2.actual_current), m2.kpc,
        error2_rate, m2.kdc
    );
    m3.pwm_value = calculatePD(
        (m3.ref_current - m3.actual_current), m3.kpc,
        error3_rate, m3.kdc
    );
    
    // Constrain PWM
    m1.pwm_value = constrainPWM(m1.pwm_value);
    m2.pwm_value = constrainPWM(m2.pwm_value);
    m3.pwm_value = constrainPWM(m3.pwm_value);
    
    // Determine direction based on position error
    m1.direction = (m1.error_pos > 0) ? 1 : ((m1.error_pos < 0) ? -1 : 0);
    m2.direction = (m2.error_pos > 0) ? 1 : ((m2.error_pos < 0) ? -1 : 0);
    m3.direction = (m3.error_pos > 0) ? 1 : ((m3.error_pos < 0) ? -1 : 0);
}

// ============ APPLY MOTOR CONTROL ============

void MotorController::applyMotorControl() {
    // Motor 1
    if (m1.direction > 0) {
        analogWrite(RPWM1, m1.pwm_value);
        analogWrite(LPWM1, 0);
    } else if (m1.direction < 0) {
        analogWrite(RPWM1, 0);
        analogWrite(LPWM1, m1.pwm_value);
    } else {
        analogWrite(RPWM1, 0);
        analogWrite(LPWM1, 0);
    }
    
    // Motor 2
    if (m2.direction > 0) {
        analogWrite(RPWM2, m2.pwm_value);
        analogWrite(LPWM2, 0);
    } else if (m2.direction < 0) {
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, m2.pwm_value);
    } else {
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, 0);
    }
    
    // Motor 3
    if (m3.direction > 0) {
        analogWrite(RPWM3, m3.pwm_value);
        analogWrite(LPWM3, 0);
    } else if (m3.direction < 0) {
        analogWrite(RPWM3, 0);
        analogWrite(LPWM3, m3.pwm_value);
    } else {
        analogWrite(RPWM3, 0);
        analogWrite(LPWM3, 0);
    }
}

// ============ GAINS SETTING ============

void MotorController::setOuterLoopGains(int motor_id, float kp, float kd) {
    MotorState* motor = getMotorState(motor_id);
    if (motor) {
        motor->kp = kp;
        motor->kd = kd;
    }
}

void MotorController::setInnerLoopGains(int motor_id, float kpc, float kdc) {
    MotorState* motor = getMotorState(motor_id);
    if (motor) {
        motor->kpc = kpc;
        motor->kdc = kdc;
    }
}

float MotorController::getKp(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    return motor ? motor->kp : 0.0;
}

float MotorController::getKd(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    return motor ? motor->kd : 0.0;
}

float MotorController::getKpc(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    return motor ? motor->kpc : 0.0;
}

float MotorController::getKdc(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    return motor ? motor->kdc : 0.0;
}

// ============ LOAD SCALING ============

void MotorController::setLoadScaling(float load_value) {
    current_load = load_value;
    smoothed_load = (LOAD_ALPHA * current_load) + ((1.0 - LOAD_ALPHA) * smoothed_load);
}

float MotorController::getLoadScaling() {
    if (smoothed_load < THRESHOLD1_DEFAULT) {
        return LOAD_SCALE_MAX;
    } else if (smoothed_load >= THRESHOLD2_DEFAULT) {
        return LOAD_SCALE_MIN;
    } else {
        float ratio = (smoothed_load - THRESHOLD1_DEFAULT) / 
                     (float)(THRESHOLD2_DEFAULT - THRESHOLD1_DEFAULT);
        return LOAD_SCALE_MAX - (ratio * (LOAD_SCALE_MAX - LOAD_SCALE_MIN));
    }
}

// ============ HELPER FUNCTIONS ============

float MotorController::constrainPWM(float pwm) {
    if (pwm > MAX_PWM) return MAX_PWM;
    if (pwm < -MAX_PWM) return -MAX_PWM;
    return pwm;
}

float MotorController::calculatePD(float error, float kpc, float error_rate, float kdc) {
    return (error * kpc) + (error_rate * kdc);
}

float MotorController::calculateCTC(float pos_err, float kp, float velo_err, float kd,
                                    float force, float gr, float kt) {
    return ((pos_err * kp) + (velo_err * kd) + force) * gr * kt;
}

MotorState* MotorController::getMotorState(int motor_id) {
    if (motor_id == 1) return &m1;
    if (motor_id == 2) return &m2;
    if (motor_id == 3) return &m3;
    return nullptr;
}

// ============ STATUS & DEBUG ============

void MotorController::printStatus() {
    Serial.println("\n========== MOTOR STATUS ==========");
    printMotorStatus(1);
    printMotorStatus(2);
    printMotorStatus(3);
    Serial.println("==================================\n");
}

void MotorController::printMotorStatus(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    if (!motor) return;
    
    Serial.print("M");
    Serial.print(motor_id);
    Serial.print(" | Pos:");
    Serial.print(motor->actual_pos, 2);
    Serial.print(" | Ref:");
    Serial.print(motor->ref_pos, 2);
    Serial.print(" | Err:");
    Serial.print(motor->error_pos, 2);
    Serial.print(" | PWM:");
    Serial.print(motor->pwm_value);
    Serial.print(" | Dir:");
    Serial.println(motor->direction);
}