//==================================================================
// FILE : firmware/src/config/constants.h
//==================================================================

#ifndef CONSTANTS_H
#define CONSTANTS_H

// ============== MOTOR CONTROL CONSTANTS ==============
// System parameters (from datasheet & calibration)
const float GEAR_RATIO = 0.2786;        // Reduction ratio
const float MOTOR_KT = 0.0663;          // Torque constant (Nm/A)
const float POSITION_SCALE = 0.245;     // Encoder to position (mm/pulse)

// PWM limits
const int MAX_PWM = 255;
const int MIN_PWM = 0;

// ============== SPEED SETTINGS ==============
// Manual control speeds
const int MANUAL_SPEED = 125;           // PWM value for manual mode
const int RETREAT_SPEED = 150;          // PWM value for retreat
const float RETREAT_VELOCITY_SCALE = 1.5;  // Speed multiplier for retreat

// ============== LOAD CELL SETTINGS ==============
// HX711 conversion parameters
const int ADC_MAX = 1023;               // Arduino ADC resolution
const int NUM_SAMPLES = 3;              // Moving average samples
const float LOAD_ALPHA = 0.3;           // Exponential smoothing factor
const long LOAD_CELL_OFFSET_DEFAULT = 0;  // Initial offset (will be calibrated)

// Load thresholds (in Newton)
// These can be changed via serial command "T..."
const int THRESHOLD1_DEFAULT = 20;      // Warning threshold
const int THRESHOLD2_DEFAULT = 40;      // Retreat trigger threshold

// ============== ADAPTIVE CONTROL ==============
// Load-based motor speed scaling
const float LOAD_SCALE_MIN = 0.15;      // Min speed (15% of max)
const float LOAD_SCALE_MAX = 1.0;       // Max speed (100% of max)
const float KP_DAMPING_FACTOR = 0.6;    // Kp reduction at high load

// ============== TIMING INTERVALS (milliseconds) ==============
const int LOAD_CELL_INTERVAL = 100;     // Read load every 100ms
const int ENCODER_INTERVAL = 1;         // Read encoder every 1ms
const int VELO_INTERVAL = 10000;        // Calculate velocity every 10 sec
const int CTC_CALC_INTERVAL = 100;      // CTC control calc every 100ms
const int PD_CALC_INTERVAL = 100;       // PD control calc every 100ms
const int PRINT_INTERVAL = 100;         // Print status every 100ms
const int SERIAL_TIMEOUT = 1000;        // Serial read timeout

// ============== MOTOR GAINS (TUNABLE) ==============
// These are default values - can be overwritten via serial commands

// Motor 1 Outer Loop (Position Control)
const float MOTOR1_KP = 110.0;
const float MOTOR1_KD = 0.1;

// Motor 1 Inner Loop (Current Control)
const float MOTOR1_KPC = 30.0;
const float MOTOR1_KDC = 0.1;

// Motor 2 Outer Loop
const float MOTOR2_KP = 142.0;
const float MOTOR2_KD = 0.6;

// Motor 2 Inner Loop
const float MOTOR2_KPC = 33.0;
const float MOTOR2_KDC = 0.1;

// Motor 3 Outer Loop
const float MOTOR3_KP = 150.0;
const float MOTOR3_KD = 0.3;

// Motor 3 Inner Loop
const float MOTOR3_KPC = 38.0;
const float MOTOR3_KDC = 0.1;

// ============== SERIAL COMMUNICATION ==============
const long BAUD_RATE = 115200;
const char TRAJECTORY_COMMAND = 'S';    // Trajectory forward
const char RETREAT_COMMAND = 'R';       // Trajectory backward
const char CALIBRATE_COMMAND = 'X';     // Calibrate/Reset
const char EMERGENCY_COMMAND = 'E';     // Emergency stop
const char THRESHOLD_COMMAND = 'T';     // Set thresholds
const char GAIN_OUTER_COMMAND = 'K';    // Set outer loop gains
const char GAIN_INNER_COMMAND = 'P';    // Set inner loop gains

// Manual commands
const char MANUAL_FORWARD = '1';
const char MANUAL_BACKWARD = '2';
const char MANUAL_STOP = '0';

// ============== ADAPTIVE CONTROL FLAGS ==============
const bool ENABLE_ADAPTIVE_MANUAL = true;   // Scale manual speed by load
const bool ENABLE_LOAD_DAMPING = true;      // Reduce Kp at high load

#endif  // CONSTANTS_H
