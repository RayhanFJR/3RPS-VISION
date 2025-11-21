# rehabilitation-system
3RPS+VISION (TA) 


# Firmware Module Guide

## 📁 Folder Structure

```
firmware/
├── src/
│   ├── main.cpp                    # Main program entry point
│   │
│   ├── config/                     # Configuration files
│   │   ├── pins.h                  # Pin definitions
│   │   └── constants.h             # System constants & tunable parameters
│   │
│   ├── control/                    # Motor control modules
│   │   ├── MotorController.h/.cpp  # Multi-motor management
│   │   └── AdaptiveControl.h/.cpp  # (Future) Adaptive CTC
│   │
│   ├── io/                         # Input/Output modules
│   │   ├── LoadCell.h/.cpp         # HX711 load cell interface
│   │   ├── CurrentSensor.h/.cpp    # Current sensing (ACS758 or similar)
│   │   ├── SerialComm.h/.cpp       # Serial communication wrapper
│   │   └── CommandParser.h/.cpp    # Parse commands from PC
│   │
│   └── ...
│
└── platformio.ini                  # PlatformIO build config
```

---

## 🔧 Module Descriptions

### **1. MotorController** - Motor Control Management
**Files:** `control/MotorController.h/.cpp`

**Fungsi:**
- Manage 3 DC motors dengan H-bridge driver
- Implement cascaded PD + CTC control
- Track motor position, velocity, current
- Load-based adaptive scaling

**Usage Example:**
```cpp
#include "control/MotorController.h"

MotorController motor;

void setup() {
    motor.begin();
    motor.calibrate();
}

void loop() {
    // Set reference trajectory
    motor.setReferencePosition(1, 100.0);  // Motor 1, 100mm
    motor.setReferenceVelocity(1, 50.0);   // 50mm/s
    motor.setReferenceForce(1, 0.5);       // 0.5N feedforward
    
    // Update sensors
    motor.updateEncoders();
    motor.updateVelocities();
    
    // Calculate & apply control
    motor.calculateCTCControl();
    motor.calculatePDControl();
    motor.applyMotorControl();
    
    // Debug
    Serial.println(motor.getActualPosition(1));
}
```

**Key Functions:**
```cpp
// Set reference trajectory
setReferencePosition(int motor_id, float position);
setReferenceVelocity(int motor_id, float velocity);
setReferenceForce(int motor_id, float force);

// Get actual values
float getActualPosition(int motor_id);
float getActualVelocity(int motor_id);

// Tune gains online
setOuterLoopGains(int motor_id, float kp, float kd);
setInnerLoopGains(int motor_id, float kpc, float kdc);

// Load scaling
float getLoadScaling();

// Debug
printStatus();
printMotorStatus(int motor_id);
```

---

### **2. LoadCell** - Load Cell Interface
**Files:** `io/LoadCell.h/.cpp`

**Fungsi:**
- Read HX711 24-bit ADC (analog-to-digital converter)
- Calibration & tare functionality
- Exponential smoothing untuk noise reduction
- Threshold detection untuk retreat trigger

**Usage Example:**
```cpp
#include "io/LoadCell.h"

LoadCell load_cell;

void setup() {
    load_cell.begin();      // Auto-calibrate at startup
    load_cell.setThreshold1(20);  // Warning: 20N
    load_cell.setThreshold2(40);  // Retreat: 40N
}

void loop() {
    float load = load_cell.readLoad();  // Read & smooth
    
    if (load > load_cell.getThreshold2()) {
        Serial.println("RETREAT");  // Trigger retreat
    }
}
```

**HX711 Pin Connection:**
```
HX711 DOUT → Arduino D12 (LOADCELL_DOUT_PIN)
HX711 CLK  → Arduino D13 (LOADCELL_SCK_PIN)
```

**Key Functions:**
```cpp
void calibrate();                      // Tare & offset
float readLoad();                      // Read smoothed value (Newton)
float getSmoothedLoad();               // Get current value
void setThreshold1(int threshold);     // Warning level
void setThreshold2(int threshold);     // Retreat level
```

---

### **3. CurrentSensor** - Current Feedback
**Files:** `io/CurrentSensor.h/.cpp`

**Fungsi:**
- Read analog current sensor (ACS758 atau sejenisnya)
- Calibration offset (zero-current measurement)
- Exponential smoothing
- Used for inner loop (current control)

**Usage Example:**
```cpp
#include "io/CurrentSensor.h"

CurrentSensor current1(A0);  // Motor 1 current on A0
CurrentSensor current2(A1);  // Motor 2 current on A1
CurrentSensor current3(A2);  // Motor 3 current on A2

void setup() {
    current1.begin();
    current2.begin();
    current3.begin();
    // Calibration automatic at startup
}

void loop() {
    float i1 = current1.readCurrent();
    float i2 = current2.readCurrent();
    float i3 = current3.readCurrent();
    
    Serial.print("I1: ");
    Serial.println(i1, 3);  // Print with 3 decimals
}
```

**Current Sensor Pin Connection:**
```
ACS758 OUT → Arduino A0 (CURRENT_SEN1)
ACS758 OUT → Arduino A1 (CURRENT_SEN2)
ACS758 OUT → Arduino A2 (CURRENT_SEN3)
```

**Key Functions:**
```cpp
float readCurrent();              // Single reading (smoothed)
float readAverageCurrent(int n);  // Average n samples
void calibrate();                 // Measure zero-offset
void setSmoothingFactor(float alpha);  // Adjust smoothing (0-1)
```

---

### **4. CommandParser** - Serial Command Parsing
**Files:** `io/CommandParser.h/.cpp`

**Fungsi:**
- Parse commands dari PC/Server
- Support trajectory (S), retreat (R), calibrate (X), emergency (E), etc.
- Extract parameters (positions, velocities, forces, gains, thresholds)
- Validation & error handling

**Supported Commands:**

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| S | S`pos1,pos2,pos3,v1,v2,v3,f1,f2,f3` | S100,110,120,50,51,52,0.5,0.5,0.5 | Trajectory (forward) |
| R | R`pos1,pos2,pos3,v1,v2,v3,f1,f2,f3` | R50,55,60,25,25,25,0.3,0.3,0.3 | Retreat (backward) |
| X | X | X | Calibrate/Reset |
| E | E | E | Emergency stop |
| T | T`thresh1,thresh2` | T20,40 | Set thresholds |
| K | K`motor_id,kp,kd` | K1,110,0.1 | Outer loop gains |
| P | P`motor_id,kpc,kdc` | P1,30,0.1 | Inner loop gains |
| 1 | 1 | 1 | Manual forward |
| 2 | 2 | 2 | Manual backward |
| 0 | 0 | 0 | Manual stop |

**Usage Example:**
```cpp
#include "io/CommandParser.h"

CommandParser parser;
String received_line = "";

void setup() {
    Serial.begin(115200);
}

void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        received_line += c;
        
        if (c == '\n') {
            Command cmd = parser.parseCommand(received_line);
            
            if (cmd.valid) {
                switch (cmd.command_type) {
                    case 'S':
                        // param1-3: positions
                        // param4-6: velocities
                        // param7-9: forces
                        break;
                    case 'T':
                        // param1: threshold1
                        // param2: threshold2
                        break;
                }
                parser.printCommand(cmd);
            }
            
            received_line = "";
        }
    }
}
```

**Key Functions:**
```cpp
Command parseCommand(String data);           // Main parser
void parseTrajectoryCommand(Command& cmd, String data, bool is_retreat);
void parseThresholdCommand(Command& cmd, String data);
void parseOuterGainCommand(Command& cmd, String data);
void parseInnerGainCommand(Command& cmd, String data);
bool isValidCommand(char type);
void printCommand(Command& cmd);             // Debug
```

---

### **5. SerialComm** - Serial Communication Wrapper
**Files:** `io/SerialComm.h/.cpp`

**Fungsi:**
- Simplified serial communication interface
- Send formatted status messages
- Receive & buffer incoming data
- Debug utilities

**Usage Example:**
```cpp
#include "io/SerialComm.h"

SerialComm comm;

void setup() {
    comm.begin(115200);
}

void loop() {
    // Send status
    comm.sendStatus("running", "forward", 25.5, 0.95, 100.5, 105.2, 98.7);
    // Output: status:running,mode:forward,load:25.50,scale:0.95,pos:100.50,105.20,98.70
    
    // Send formatted data
    comm.sendData("Motor1: pos=%f, vel=%f", 100.5, 50.2);
    
    // Receive command
    if (comm.hasData()) {
        String cmd = comm.readLine();
        Serial.print("Received: ");
        Serial.println(cmd);
    }
}
```

**Key Functions:**
```cpp
void sendCommand(const String& cmd);        // Send raw command
void sendData(const char* format, ...);     // printf-style sending
void sendStatus(...);                       // Send status message
void sendRetreatCommand();                  // Send "RETREAT"

bool hasData();                             // Check if data available
String readLine();                          // Read until newline
String readUntilChar(char delimiter);       // Read custom delimiter
void clearBuffer();                         // Flush buffer
```

---

### **6. config/pins.h** - Pin Definitions
**Centralized pin mapping untuk semua komponen**

```cpp
// Motor PWM (H-bridge control)
RPWM1, LPWM1, RPWM2, LPWM2, RPWM3, LPWM3

// Encoder (digital input)
ENC1, ENC2, ENC3

// Current Sensor (analog input)
CURRENT_SEN1, CURRENT_SEN2, CURRENT_SEN3

// Load Cell (HX711)
LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN
```

**Mengapa centralized?**
- Easy to change pins untuk different hardware variants
- No need to edit multiple files
- Clear documentation

---

### **7. config/constants.h** - Tunable Parameters
**Semua magic numbers di satu tempat**

**Motor Gains:**
```cpp
MOTOR1_KP, MOTOR1_KD      // Outer loop
MOTOR1_KPC, MOTOR1_KDC    // Inner loop
// (same for M2, M3)
```

**System Parameters:**
```cpp
GEAR_RATIO = 0.2786
MOTOR_KT = 0.0663
POSITION_SCALE = 0.245
```

**Thresholds:**
```cpp
THRESHOLD1_DEFAULT = 20   // N
THRESHOLD2_DEFAULT = 40   // N
```

**Timing:**
```cpp
LOAD_CELL_INTERVAL = 100   // ms
ENCODER_INTERVAL = 1       // ms
CTC_CALC_INTERVAL = 100    // ms
```

---

## 🚀 Integration into main.cpp

```cpp
#include "config/pins.h"
#include "config/constants.h"
#include "control/MotorController.h"
#include "io/LoadCell.h"
#include "io/CurrentSensor.h"
#include "io/SerialComm.h"
#include "io/CommandParser.h"

// Create instances
MotorController motor;
LoadCell load_cell;
CurrentSensor current1(CURRENT_SEN1);
CurrentSensor current2(CURRENT_SEN2);
CurrentSensor current3(CURRENT_SEN3);
SerialComm comm;
CommandParser parser;

void setup() {
    motor.begin();
    load_cell.begin();
    current1.begin();
    current2.begin();
    current3.begin();
    comm.begin(115200);
}

void loop() {
    // 1. Receive commands
    if (comm.hasData()) {
        String cmd_str = comm.readLine();
        Command cmd = parser.parseCommand(cmd_str);
        
        if (cmd.valid && cmd.command_type == 'S') {
            motor.setReferencePosition(1, cmd.param1);
            motor.setReferenceVelocity(1, cmd.param4);
            motor.setReferenceForce(1, cmd.param7);
        }
    }
    
    // 2. Update sensors
    motor.updateEncoders();
    motor.updateVelocities();
    float load = load_cell.readLoad();
    
    // 3. Calculate control
    motor.calculateCTCControl();
    motor.calculatePDControl();
    motor.applyMotorControl();
    
    // 4. Send status
    comm.sendStatus("running", "forward", load, 0.9,
                   motor.getActualPosition(1),
                   motor.getActualPosition(2),
                   motor.getActualPosition(3));
}
```

---

## 📊 Data Flow Diagram

```
Serial Input (PC/Server)
    ↓
CommandParser
    ↓
MotorController (set reference)
    ↓
Encoder/CurrentSensor (read feedback)
    ↓
CTC + PD Control (calculate PWM)
    ↓
Motor Driver → Motors
    ↓
LoadCell (monitor force)
    ↓
SerialComm (send status)
    ↓
Serial Output (to PC/Server)
```

---

## ✅ Compilation Checklist

- [ ] All `.h` files in correct directories
- [ ] All `.cpp` files in correct directories
- [ ] `#include` paths correct (use relative paths)
- [ ] No circular includes
- [ ] All `#ifndef` guards present
- [ ] Arduino.h included where needed
- [ ] No undefined functions/variables

---

## 🔍 Debugging Tips

### Serial Monitor Output Format:

```
[MotorController] Initialized successfully
[LoadCell] Calibrated - Offset: 12345
[CurrentSensor] Pin 14 calibrated - Offset: 2.450

status:running,mode:forward,load:25.50,scale:0.95,pos:100.50,105.20,98.70
status:running,mode:forward,load:26.10,scale:0.92,pos:101.20,106.05,99.40
```

### Common Issues:

| Issue | Cause | Solution |
|-------|-------|----------|
| Motor not moving | Wrong PWM pin | Check `pins.h` |
| Encoder not reading | Rising edge not detected | Add debouncing |
| Load cell always 0 | Offset not calibrated | Call `calibrate()` |
| Current feedback wrong | Sensor offset incorrect | Recalibrate sensor |
| Serial garbled | Baud rate mismatch | Check 115200 |

---

## 📚 Next Steps

1. Copy all files ke `firmware/src/` folder
2. Update `main.cpp` untuk include semua modules
3. Verify compilation dengan `pio run`
4. Upload dengan `pio run -t upload`
5. Monitor dengan `pio device monitor`

**Good luck! 🚀**