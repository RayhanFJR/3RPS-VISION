//==================================================================
// FIRMWARE UTAMA - SISTEM KONTROL REHABILITASI 3-RPS
// File: firmware/src/main.cpp
// 
// Features:
// - Motor control (3 DC motors)
// - Sensor reading (encoders, load cell, current)
// - Adaptive CTC control
// - Load-based adaptive scaling
// - Trajectory execution
// - Retreat mechanism
//==================================================================

// ============== INCLUDES ==============
#include <Arduino.h>
#include <stdint.h>

// Include custom headers (nanti bisa split ke file terpisah)
#include "config/pins.h"
#include "config/constants.h"

// ============== PIN DEFINITIONS ==============
// Motor PWM pins
const int RPWM1 = 3,  LPWM1 = 5;
const int RPWM2 = 6,  LPWM2 = 9;
const int RPWM3 = 10, LPWM3 = 11;

// Encoder pins
const int ENC1 = 4, ENC2 = 2, ENC3 = 8;

// Current sensor pins
const int CurrSen1 = A0, CurrSen2 = A1, CurrSen3 = A2;

// Load cell pins
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;

// ============== CONSTANTS ==============
const float GEAR_RATIO = 0.2786;
const float MOTOR_KT = 0.0663;
const float POSITION_SCALE = 0.245;
const int ADC_MAX = 1023;
const int NUM_SAMPLES = 3;
const float LOAD_ALPHA = 0.3;
const float RETREAT_VELOCITY_SCALE = 1.5;

const int MANUAL_SPEED = 125;
const int RETREAT_SPEED = 150;

const float LOAD_SCALE_MIN = 0.15;
const float LOAD_SCALE_MAX = 1.0;
const float KP_DAMPING_FACTOR = 0.6;

// ============== TIMING CONSTANTS ==============
const int LOAD_CELL_INTERVAL = 100;      // ms
const int ENCODER_INTERVAL = 1;          // ms
const int VELO_INTERVAL = 10000;         // 10 seconds
const int CTC_CALC_INTERVAL = 100;       // ms
const int PD_CALC_INTERVAL = 100;        // ms
const int PRINT_INTERVAL = 100;          // ms

// ============== MOTOR STATE STRUCTURE ==============
struct MotorState {
    // Reference values (dari PC)
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
    float kp, kd;    // Outer loop
    float kpc, kdc;  // Inner loop
    
    // Timing
    float prev_pos;
    float prev_ref_velo;
};

// ============== GLOBAL VARIABLES ==============

// Motor states
MotorState m1, m2, m3;

// Control parameters
int operating_mode = 0;    // 0=Manual, 1=Forward, 2=Retreat
int manual_command = 0;
int manipulator_state = 0; // 0=Running, 1=Paused

// Load cell
long load_cell_offset = 0;
float latest_valid_load = 0.0;
float smoothed_load = 0.0;
int threshold1 = 20;
int threshold2 = 40;

// Retreat control
bool retreat_has_been_triggered = false;
bool retreat_request_sent = false;

// Timing
long last_load_time = 0;
long last_enc_time = 0;
long last_velo_time = 0;
long last_ctc_calc_time = 0;
long last_pd_calc_time = 0;
long last_prn_time = 0;

// Serial
String received_data = "";

// Adaptive control
bool adaptive_control_enabled = true;

// ============== HELPER FUNCTIONS ==============

float error_calculation(float ref, float act) {
    return ref - act;
}

float constraint_pwm(float pwm) {
    if (pwm > 255) return 255;
    if (pwm < -255) return -255;
    return pwm;
}

float calculate_pd(float error, float kpc, float error_rate, float kdc) {
    return (error * kpc) + (error_rate * kdc);
}

float calculate_ctc(float pos_err, float kp, float velo_err, float kd, 
                    float force, float gr, float kt) {
    return ((pos_err * kp) + (velo_err * kd) + force) * gr * kt;
}

// ============== LOAD CELL FUNCTIONS ==============

long read_hx711() {
    long result = 0;
    while (digitalRead(LOADCELL_DOUT_PIN));
    
    for (int i = 0; i < 24; i++) {
        digitalWrite(LOADCELL_SCK_PIN, HIGH);
        delayMicroseconds(1);
        result = result << 1;
        if (digitalRead(LOADCELL_DOUT_PIN)) {
            result++;
        }
        digitalWrite(LOADCELL_SCK_PIN, LOW);
        delayMicroseconds(1);
    }
    
    digitalWrite(LOADCELL_SCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    delayMicroseconds(1);
    
    if (result & 0x800000) {
        result |= ~0xFFFFFF;
    }
    
    return result;
}

float get_load_scaling() {
    smoothed_load = (LOAD_ALPHA * latest_valid_load) + 
                    ((1.0 - LOAD_ALPHA) * smoothed_load);
    
    if (smoothed_load < threshold1) {
        return LOAD_SCALE_MAX;
    } else if (smoothed_load >= threshold2) {
        return LOAD_SCALE_MIN;
    } else {
        float load_ratio = (smoothed_load - threshold1) / 
                          (float)(threshold2 - threshold1);
        return LOAD_SCALE_MAX - 
               (load_ratio * (LOAD_SCALE_MAX - LOAD_SCALE_MIN));
    }
}

float get_adaptive_kp(float base_kp) {
    if (smoothed_load < threshold1) {
        return base_kp;
    } else if (smoothed_load >= threshold2) {
        return base_kp * (1.0 - KP_DAMPING_FACTOR);
    } else {
        float load_ratio = (smoothed_load - threshold1) / 
                          (float)(threshold2 - threshold1);
        float damping = 1.0 - (load_ratio * KP_DAMPING_FACTOR);
        return base_kp * damping;
    }
}

// ============== CURRENT SENSOR FUNCTIONS ==============

float avg_current_1() {
    float val = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        val += analogRead(CurrSen1);
        delay(1);
    }
    return val / ADC_MAX / NUM_SAMPLES;
}

float avg_current_2() {
    float val = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        val += analogRead(CurrSen2);
        delay(1);
    }
    return val / ADC_MAX / NUM_SAMPLES;
}

float avg_current_3() {
    float val = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        val += analogRead(CurrSen3);
        delay(1);
    }
    return val / ADC_MAX / NUM_SAMPLES;
}

// ============== MOTOR CONTROL FUNCTIONS ==============

void update_encoders() {
    // Motor 1
    int state1 = digitalRead(ENC1);
    static int prev_state1 = 0;
    if (state1 > prev_state1) {
        if (m1.pwm_value > 0) m1.encoder_count++;
        else if (m1.pwm_value < 0) m1.encoder_count--;
    }
    m1.actual_pos = m1.encoder_count * POSITION_SCALE;
    prev_state1 = state1;
    
    // Motor 2
    int state2 = digitalRead(ENC2);
    static int prev_state2 = 0;
    if (state2 > prev_state2) {
        if (m2.pwm_value > 0) m2.encoder_count++;
        else if (m2.pwm_value < 0) m2.encoder_count--;
    }
    m2.actual_pos = m2.encoder_count * POSITION_SCALE;
    prev_state2 = state2;
    
    // Motor 3
    int state3 = digitalRead(ENC3);
    static int prev_state3 = 0;
    if (state3 > prev_state3) {
        if (m3.pwm_value > 0) m3.encoder_count++;
        else if (m3.pwm_value < 0) m3.encoder_count--;
    }
    m3.actual_pos = m3.encoder_count * POSITION_SCALE;
    prev_state3 = state3;
}

void update_velocities() {
    m1.actual_velo = (m1.actual_pos - m1.prev_pos) / 0.01;
    m2.actual_velo = (m2.actual_pos - m2.prev_pos) / 0.01;
    m3.actual_velo = (m3.actual_pos - m3.prev_pos) / 0.01;
    
    m1.prev_pos = m1.actual_pos;
    m2.prev_pos = m2.actual_pos;
    m3.prev_pos = m3.actual_pos;
}

void calculate_ctc_control() {
    m1.error_pos = error_calculation(m1.ref_pos, m1.actual_pos);
    m2.error_pos = error_calculation(m2.ref_pos, m2.actual_pos);
    m3.error_pos = error_calculation(m3.ref_pos, m3.actual_pos);
    
    float kp1_adaptive = get_adaptive_kp(m1.kp);
    float kp2_adaptive = get_adaptive_kp(m2.kp);
    float kp3_adaptive = get_adaptive_kp(m3.kp);
    
    m1.ref_current = calculate_ctc(
        m1.error_pos, kp1_adaptive,
        (m1.ref_velo - m1.actual_velo), m1.kd,
        m1.ref_force, GEAR_RATIO, MOTOR_KT
    );
    m2.ref_current = calculate_ctc(
        m2.error_pos, kp2_adaptive,
        (m2.ref_velo - m2.actual_velo), m2.kd,
        m2.ref_force, GEAR_RATIO, MOTOR_KT
    );
    m3.ref_current = calculate_ctc(
        m3.error_pos, kp3_adaptive,
        (m3.ref_velo - m3.actual_velo), m3.kd,
        m3.ref_force, GEAR_RATIO, MOTOR_KT
    );
}

void calculate_pd_control() {
    float error1_rate = (m1.ref_current - m1.actual_current) / 0.1;
    float error2_rate = (m2.ref_current - m2.actual_current) / 0.1;
    float error3_rate = (m3.ref_current - m3.actual_current) / 0.1;
    
    m1.pwm_value = calculate_pd(
        (m1.ref_current - m1.actual_current), m1.kpc,
        error1_rate, m1.kdc
    );
    m2.pwm_value = calculate_pd(
        (m2.ref_current - m2.actual_current), m2.kpc,
        error2_rate, m2.kdc
    );
    m3.pwm_value = calculate_pd(
        (m3.ref_current - m3.actual_current), m3.kpc,
        error3_rate, m3.kdc
    );
    
    m1.pwm_value = constraint_pwm(m1.pwm_value);
    m2.pwm_value = constraint_pwm(m2.pwm_value);
    m3.pwm_value = constraint_pwm(m3.pwm_value);
    
    // Apply load scaling in forward mode
    if (operating_mode == 1) {
        float load_scale = get_load_scaling();
        m1.pwm_value = constraint_pwm(m1.pwm_value * load_scale);
        m2.pwm_value = constraint_pwm(m2.pwm_value * load_scale);
        m3.pwm_value = constraint_pwm(m3.pwm_value * load_scale);
    }
    
    // Determine direction
    if (m1.error_pos > 0) m1.direction = 1;
    else if (m1.error_pos < 0) m1.direction = -1;
    else m1.direction = 0;
    
    if (m2.error_pos > 0) m2.direction = 1;
    else if (m2.error_pos < 0) m2.direction = -1;
    else m2.direction = 0;
    
    if (m3.error_pos > 0) m3.direction = 1;
    else if (m3.error_pos < 0) m3.direction = -1;
    else m3.direction = 0;
}

void apply_motor_control() {
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

void stop_all_motors() {
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
    analogWrite(RPWM3, 0);
    analogWrite(LPWM3, 0);
}

void manual_mode_control() {
    int effective_speed = MANUAL_SPEED;
    float load_scale = get_load_scaling();
    effective_speed = MANUAL_SPEED * load_scale;
    
    if (manual_command == 1) {
        // Forward
        analogWrite(RPWM1, effective_speed);
        analogWrite(LPWM1, 0);
        analogWrite(RPWM2, effective_speed);
        analogWrite(LPWM2, 0);
        analogWrite(RPWM3, effective_speed);
        analogWrite(LPWM3, 0);
    } else if (manual_command == 2) {
        // Backward
        analogWrite(RPWM1, 0);
        analogWrite(LPWM1, effective_speed);
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, effective_speed);
        analogWrite(RPWM3, 0);
        analogWrite(LPWM3, effective_speed);
    } else {
        stop_all_motors();
    }
}

// ============== COMMAND PARSING ==============

void parse_trajectory_command(String data, bool is_retreat) {
    data.replace("S", "");
    data.replace("R", "");
    
    // Simple parsing (bisa improve dengan strtok)
    int idx = 0;
    
    m1.ref_pos = data.toFloat();
    idx = data.indexOf(',');
    
    m2.ref_pos = atof(data.substring(idx + 1).c_str());
    idx = data.indexOf(',', idx + 1);
    
    m3.ref_pos = atof(data.substring(idx + 1).c_str());
    idx = data.indexOf(',', idx + 1);
    
    m1.ref_velo = atof(data.substring(idx + 1).c_str());
    idx = data.indexOf(',', idx + 1);
    
    m2.ref_velo = atof(data.substring(idx + 1).c_str());
    idx = data.indexOf(',', idx + 1);
    
    m3.ref_velo = atof(data.substring(idx + 1).c_str());
    idx = data.indexOf(',', idx + 1);
    
    m1.ref_force = atof(data.substring(idx + 1).c_str());
    idx = data.indexOf(',', idx + 1);
    
    m2.ref_force = atof(data.substring(idx + 1).c_str());
    idx = data.indexOf(',', idx + 1);
    
    m3.ref_force = atof(data.substring(idx + 1).c_str());
    
    if (is_retreat) {
        m1.ref_velo *= RETREAT_VELOCITY_SCALE;
        m2.ref_velo *= RETREAT_VELOCITY_SCALE;
        m3.ref_velo *= RETREAT_VELOCITY_SCALE;
    }
}

void parse_thresholds(String data) {
    data.replace("T", "");
    int comma = data.indexOf(',');
    threshold1 = data.substring(0, comma).toInt();
    threshold2 = data.substring(comma + 1).toInt();
    
    Serial.print("Thresholds: T1=");
    Serial.print(threshold1);
    Serial.print(", T2=");
    Serial.println(threshold2);
}

void reset_system() {
    operating_mode = 0;
    manual_command = 0;
    manipulator_state = 0;
    retreat_has_been_triggered = false;
    retreat_request_sent = false;
    
    stop_all_motors();
    
    m1.encoder_count = 0;
    m2.encoder_count = 0;
    m3.encoder_count = 0;
    
    m1.actual_pos = 0;
    m2.actual_pos = 0;
    m3.actual_pos = 0;
    
    load_cell_offset = read_hx711();
    smoothed_load = 0.0;
    
    Serial.println("System Reset: Tare & Zero OK\n");
}

void emergency_stop() {
    operating_mode = 0;
    manual_command = 0;
    retreat_has_been_triggered = false;
    stop_all_motors();
    Serial.println("EMERGENCY_STOP");
}

// ============== SETUP ==============

void setup() {
    Serial.begin(115200);
    
    // Setup pins
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM2, OUTPUT);
    pinMode(RPWM3, OUTPUT);
    pinMode(LPWM3, OUTPUT);
    
    pinMode(ENC1, INPUT);
    pinMode(ENC2, INPUT);
    pinMode(ENC3, INPUT);
    
    pinMode(CurrSen1, INPUT);
    pinMode(CurrSen2, INPUT);
    pinMode(CurrSen3, INPUT);
    
    pinMode(LOADCELL_DOUT_PIN, INPUT);
    pinMode(LOADCELL_SCK_PIN, OUTPUT);
    
    // Initialize motor states
    m1.kp = 110.0;  m1.kd = 0.1;  m1.kpc = 30.0;  m1.kdc = 0.1;
    m2.kp = 142.0;  m2.kd = 0.6;  m2.kpc = 33.0;  m2.kdc = 0.1;
    m3.kp = 150.0;  m3.kd = 0.3;  m3.kpc = 38.0;  m3.kdc = 0.1;
    
    reset_system();
    
    Serial.println("===========================================");
    Serial.println("  3-RPS Parallel Robot Control System");
    Serial.println("  Adaptive CTC + Load-based Control");
    Serial.println("===========================================\n");
}

// ============== MAIN LOOP ==============

void loop() {
    long current_time = millis();
    
    // ========== SERIAL COMMAND PROCESSING ==========
    if (Serial.available() > 0) {
        char received_char = Serial.read();
        received_data += received_char;
        
        if (received_char == '\n') {
            received_data.trim();
            
            if (received_data.startsWith("S")) {
                operating_mode = 1;
                retreat_has_been_triggered = false;
                retreat_request_sent = false;
                manipulator_state = 0;
                parse_trajectory_command(received_data, false);
            }
            else if (received_data.startsWith("R") && received_data.indexOf(',') > 0) {
                operating_mode = 2;
                manipulator_state = 0;
                parse_trajectory_command(received_data, true);
            }
            else if (received_data == "X") {
                reset_system();
            }
            else if (received_data == "E") {
                emergency_stop();
            }
            else if (received_data == "1") {
                operating_mode = 0;
                manual_command = 1;
                retreat_has_been_triggered = false;
            }
            else if (received_data == "2") {
                operating_mode = 0;
                manual_command = 2;
                retreat_has_been_triggered = false;
            }
            else if (received_data == "0") {
                operating_mode = 0;
                manual_command = 0;
            }
            else if (received_data.startsWith("T")) {
                parse_thresholds(received_data);
            }
            
            received_data = "";
        }
    }
    
    // ========== AUTO MODE EXECUTION ==========
    if (operating_mode == 1 || operating_mode == 2) {
        
        // Load cell monitoring
        if (operating_mode == 1 && current_time - last_load_time >= LOAD_CELL_INTERVAL) {
            if (retreat_has_been_triggered) {
                manipulator_state = 1;
            } else {
                long effective_value = read_hx711() - load_cell_offset;
                latest_valid_load = effective_value / 10000.0;
                int round_value = round(latest_valid_load);
                
                if (round_value >= threshold2) {
                    manipulator_state = 1;
                    retreat_has_been_triggered = true;
                    
                    if (!retreat_request_sent) {
                        Serial.println("RETREAT");
                        retreat_request_sent = true;
                    }
                } else if (round_value >= threshold1) {
                    manipulator_state = 1;
                } else {
                    manipulator_state = 0;
                }
            }
            last_load_time = current_time;
        }
        
        // Encoder reading
        if (current_time - last_enc_time >= ENCODER_INTERVAL && manipulator_state != 1) {
            update_encoders();
            last_enc_time = current_time;
        }
        
        // Control calculations
        if (manipulator_state == 0) {
            if (current_time - last_velo_time >= VELO_INTERVAL) {
                update_velocities();
                last_velo_time = current_time;
            }
            
            if (current_time - last_ctc_calc_time >= CTC_CALC_INTERVAL) {
                calculate_ctc_control();
                last_ctc_calc_time = current_time;
            }
            
            if (current_time - last_pd_calc_time >= PD_CALC_INTERVAL) {
                calculate_pd_control();
                last_pd_calc_time = current_time;
            }
        }
        
        // Motor control execution
        if (manipulator_state == 0) {
            apply_motor_control();
        } else {
            stop_all_motors();
        }
        
        // Status reporting
        if (current_time - last_prn_time >= PRINT_INTERVAL) {
            String status_str = (manipulator_state == 0) ? "running" : "paused";
            String mode_str = (operating_mode == 1) ? "forward" : "retreat";
            
            float current_scale = get_load_scaling();
            Serial.print("status:");
            Serial.print(status_str);
            Serial.print(",mode:");
            Serial.print(mode_str);
            Serial.print(",load:");
            Serial.print(latest_valid_load, 2);
            Serial.print(",scale:");
            Serial.print(current_scale, 2);
            Serial.print(",pos:");
            Serial.print(m1.actual_pos, 2);
            Serial.print(",");
            Serial.print(m2.actual_pos, 2);
            Serial.print(",");
            Serial.print(m3.actual_pos, 2);
            Serial.println("");
            
            last_prn_time = current_time;
        }
    }
    
    // ========== MANUAL MODE ==========
    else {
        if (current_time - last_load_time >= LOAD_CELL_INTERVAL) {
            long effective_value = read_hx711() - load_cell_offset;
            latest_valid_load = effective_value / 10000.0;
            last_load_time = current_time;
        }
        
        manual_mode_control();
    }
}