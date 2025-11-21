//==================================================================
// FILE : firmware/src/config/pins.h
//==================================================================

#ifndef PINS_H
#define PINS_H

// ============== MOTOR PWM PINS ==============
// Motor 1 (3-axis parallel robot)
const int RPWM1 = 3;   // Right PWM (forward)
const int LPWM1 = 5;   // Left PWM (backward)

// Motor 2
const int RPWM2 = 6;   // Right PWM (forward)
const int LPWM2 = 9;   // Left PWM (backward)

// Motor 3
const int RPWM3 = 10;  // Right PWM (forward)
const int LPWM3 = 11;  // Left PWM (backward)

// ============== ENCODER PINS ==============
// Quadrature encoder input (digital pins)
const int ENC1 = 4;    // Encoder motor 1
const int ENC2 = 2;    // Encoder motor 2
const int ENC3 = 8;    // Encoder motor 3

// ============== CURRENT SENSOR PINS ==============
// Analog input pins (0-1023 ADC)
const int CURRENT_SEN1 = A0;  // Current feedback motor 1
const int CURRENT_SEN2 = A1;  // Current feedback motor 2
const int CURRENT_SEN3 = A2;  // Current feedback motor 3

// ============== LOAD CELL PINS ==============
// HX711 load cell amplifier (SPI-like)
const int LOADCELL_DOUT_PIN = 12;  // Data out
const int LOADCELL_SCK_PIN = 13;   // Serial clock

// ============== EXPANSION PINS (RESERVED) ==============
// Available for future sensors
const int RESERVED_PIN1 = 7;
const int RESERVED_PIN2 = 8;
const int RESERVED_ANALOG1 = A3;
const int RESERVED_ANALOG2 = A4;
const int RESERVED_ANALOG3 = A5;

#endif  // PINS_H
