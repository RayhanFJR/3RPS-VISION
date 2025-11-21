//==================================================================
// FILE : firmware/src/io/CurrentSensor.h
//==================================================================

#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <Arduino.h>
#include "../config/pins.h"
#include "../config/constants.h"

class CurrentSensor {
    
public:
    CurrentSensor(int pin);
    ~CurrentSensor();
    
    // Initialize
    void begin();
    
    // Reading
    float readCurrent();              // Read single value
    float readAverageCurrent(int n);  // Read and average n samples
    float getSmoothedCurrent();       // Get exponentially smoothed value
    
    // Calibration
    void setOffset(float offset);
    void calibrate();                 // Measure zero-current offset
    
    // Configuration
    void setSamplingCount(int count);
    void setSmoothingFactor(float alpha);
    
    // Utility
    int readRawADC();
    float convertADCToCurrent(int adc_value);
    
private:
    int analog_pin;
    float zero_offset;
    float smoothed_current;
    float smoothing_alpha;
    int sampling_count;
    
    const float CURRENT_SENSITIVITY = 0.04;  // A/V (example: 40mV per Amp)
    const float ADC_REFERENCE = 5.0;         // 5V reference
};

#endif  // CURRENT_SENSOR_H

