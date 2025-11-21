//==================================================================
// FILE : firmware/src/io/CurrentSensor.cpp
//==================================================================

#include "CurrentSensor.h"

CurrentSensor::CurrentSensor(int pin) {
    analog_pin = pin;
    zero_offset = 0.0;
    smoothed_current = 0.0;
    smoothing_alpha = 0.2;  // Default smoothing
    sampling_count = NUM_SAMPLES;
}

CurrentSensor::~CurrentSensor() {
}

void CurrentSensor::begin() {
    pinMode(analog_pin, INPUT);
    calibrate();
}

void CurrentSensor::calibrate() {
    // Average multiple readings at zero current
    float sum = 0.0;
    for (int i = 0; i < 10; i++) {
        sum += readAverageCurrent(sampling_count);
        delay(10);
    }
    zero_offset = sum / 10.0;
    
    Serial.print("[CurrentSensor] Pin ");
    Serial.print(analog_pin);
    Serial.print(" calibrated - Offset: ");
    Serial.println(zero_offset, 3);
}

float CurrentSensor::readCurrent() {
    int adc_value = readRawADC();
    float current = convertADCToCurrent(adc_value);
    current -= zero_offset;
    
    // Apply exponential smoothing
    smoothed_current = (smoothing_alpha * current) + 
                      ((1.0 - smoothing_alpha) * smoothed_current);
    
    return smoothed_current;
}

float CurrentSensor::readAverageCurrent(int n) {
    float sum = 0.0;
    for (int i = 0; i < n; i++) {
        int adc_value = readRawADC();
        float current = convertADCToCurrent(adc_value);
        sum += current;
        delayMicroseconds(100);
    }
    return sum / n;
}

float CurrentSensor::getSmoothedCurrent() {
    return smoothed_current;
}

void CurrentSensor::setOffset(float offset) {
    zero_offset = offset;
}

void CurrentSensor::setSamplingCount(int count) {
    sampling_count = constrain(count, 1, 10);
}

void CurrentSensor::setSmoothingFactor(float alpha) {
    smoothing_alpha = constrain(alpha, 0.0, 1.0);
}

int CurrentSensor::readRawADC() {
    return analogRead(analog_pin);
}

float CurrentSensor::convertADCToCurrent(int adc_value) {
    // Convert ADC (0-1023) to voltage (0-5V)
    float voltage = (adc_value / 1023.0) * ADC_REFERENCE;
    
    // Convert voltage to current using sensitivity
    // Assuming Hall-effect sensor with linear output
    float current = (voltage - (ADC_REFERENCE / 2.0)) / CURRENT_SENSITIVITY;
    
    return current;
}