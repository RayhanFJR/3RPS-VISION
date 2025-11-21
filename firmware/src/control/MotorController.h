//==================================================================
// FILE : firmware/src/control/MotorController.h
//==================================================================

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <stdint.h>
#include "../config/pins.h"
#include "../config/constants.h"

// ============ MOTOR STATE STRUCTURE ============
struct MotorState {
    // Reference values (dari PC/Server)
    float ref_pos;
    float ref_velo;
    float ref_current;
    float ref_force;
    float ref_accel;
    
    // Actual values (dari sensor)
    int encoder_count;
    float actual_pos;
    float actual_velo;
    float actual_current;
    
    // Error signals
    float error_pos;
    float error_velo;
    float prev_error;
    
    // Control output
    int pwm_value;
    int direction;  // 1=forward, -1=backward, 0=stop
    
    // Gains
    float kp, kd;    // Outer loop (position control)
    float kpc, kdc;  // Inner loop (current control)
    
    // Timing & history
    float prev_pos;
    float prev_ref_velo;
    int prev_state;  // Previous encoder state
};

// ============ CLASS DEFINITION ============

class MotorController {
    
public:
    // ========== CONSTRUCTOR & DESTRUCTOR ==========
    MotorController();
    ~MotorController();
    
    // ========== INITIALIZATION ==========
    void begin();                          // Setup pins & initialize
    void calibrate();                      // Reset encoders to zero
    void stopAllMotors();                  // Emergency stop
    
    // ========== REFERENCE SETTING ==========
    // Set reference values for trajectory execution
    void setReferencePosition(int motor_id, float position);
    void setReferenceVelocity(int motor_id, float velocity);
    void setReferenceCurrent(int motor_id, float current);
    void setReferenceForce(int motor_id, float force);
    
    // ========== SENSOR READING ==========
    // Get actual sensor values
    float getActualPosition(int motor_id);
    float getActualVelocity(int motor_id);
    float getActualCurrent(int motor_id);
    int getEncoderCount(int motor_id);
    
    // ========== CONTROL CALCULATION ==========
    void updateEncoders();                 // Read encoder pins
    void updateVelocities();               // Calculate velocity
    void calculateCTCControl();            // Calculate reference current
    void calculatePDControl();             // Calculate PWM output
    void applyMotorControl();              // Send PWM to H-bridge
    
    // ========== GAINS TUNING ==========
    void setOuterLoopGains(int motor_id, float kp, float kd);
    void setInnerLoopGains(int motor_id, float kpc, float kdc);
    
    float getKp(int motor_id);
    float getKd(int motor_id);
    float getKpc(int motor_id);
    float getKdc(int motor_id);
    
    // ========== LOAD SCALING ==========
    void setLoadScaling(float load_value);
    float getLoadScaling();
    
    // ========== STATUS & DEBUG ==========
    void printStatus();                    // Print all motors status
    void printMotorStatus(int motor_id);   // Print single motor
    
private:
    // ========== INTERNAL STATE - 3 MOTORS ==========
    MotorState m1, m2, m3;
    
    // ========== LOAD TRACKING ==========
    float current_load;
    float smoothed_load;
    
    // ========== HELPER FUNCTIONS ==========
    float constrainPWM(float pwm);
    float calculatePD(float error, float kpc, float error_rate, float kdc);
    float calculateCTC(float pos_err, float kp, float velo_err, float kd, 
                      float force, float gr, float kt);
    
    // Get motor state by ID
    MotorState* getMotorState(int motor_id);
    
    // Update single encoder
    void updateSingleEncoder(int motor_id, int pin, MotorState& motor);
    
    // ========== TIMING ==========
    long last_encoder_time;
    long last_velocity_time;
};

#endif  // MOTOR_CONTROLLER_H