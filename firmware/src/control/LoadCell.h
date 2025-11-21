//==================================================================
// FILE : firmware/src/io/LoadCell.h
//==================================================================

#ifndef LOAD_CELL_H
#define LOAD_CELL_H

#include <Arduino.h>
#include "../config/pins.h"
#include "../config/constants.h"

class LoadCell {
    
public:
    // Constructor & Destructor
    LoadCell();
    ~LoadCell();
    
    // Initialization
    void begin();
    void calibrate();  // Tare the load cell
    
    // Reading
    long readRaw();           // Read 24-bit ADC value
    float readLoad();         // Read smoothed load value (Newton)
    float getSmoothedLoad();  // Get current smoothed value
    
    // Calibration
    void setOffset(long offset);
    long getOffset();
    
    // Thresholds
    void setThreshold1(int threshold);
    void setThreshold2(int threshold);
    int getThreshold1();
    int getThreshold2();
    
    // Status check
    bool isDataReady();
    
private:
    long offset;
    float latest_load;
    float smoothed_load;
    int threshold1;
    int threshold2;
};

#endif  // LOAD_CELL_H

