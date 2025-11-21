# Alternatif: Arduino IDE + Manual Setup

Jika tidak mau pakai **PlatformIO**, bisa pakai **Arduino IDE tradisional**. Tapi agak lebih ribet.

---

## 🚨 Masalah dengan Arduino IDE

| Feature | Arduino IDE | PlatformIO |
|---------|------------|-----------|
| Support `.cpp` + `.h` | ❌ Susah | ✅ Native |
| Include paths | ❌ Manual | ✅ Automatic |
| Multi-file | ❌ Ribet | ✅ Easy |
| Library management | ⚠️ Limited | ✅ Excellent |
| Compilation | ❌ Slow | ✅ Fast |

---

## 📝 Cara 1: Arduino IDE dengan Concatenation

### Konsep
Combine semua `.h` + `.cpp` menjadi 1 file `.ino`

### Step 1: Buat File `.ino`

**File:** `firmware/firmware.ino`

```cpp
//==================================================================
// FIRMWARE MAIN - All in one file
// Combine all modules: config, control, io
//==================================================================

// ============ INCLUDES ============
#include <Arduino.h>
#include <stdint.h>

// ============ CONFIGURATIONS ============
// Include semua content dari pins.h
#define RPWM1 3
#define LPWM1 5
#define RPWM2 6
#define LPWM2 9
#define RPWM3 10
#define LPWM3 11
#define ENC1 4
#define ENC2 2
#define ENC3 8
#define CURRENT_SEN1 A0
#define CURRENT_SEN2 A1
#define CURRENT_SEN3 A2
#define LOADCELL_DOUT_PIN 12
#define LOADCELL_SCK_PIN 13

// Include semua content dari constants.h
const float GEAR_RATIO = 0.2786;
const float MOTOR_KT = 0.0663;
const float POSITION_SCALE = 0.245;
// ... dst semua constants ...

// ============ MOTOR STATE STRUCTURE ============
struct MotorState {
    float ref_pos, ref_velo, ref_current, ref_force, ref_accel;
    int encoder_count;
    float actual_pos, actual_velo, actual_current;
    float error_pos, error_velo, prev_error;
    int pwm_value, direction;
    float kp, kd, kpc, kdc;
    float prev_pos, prev_ref_velo;
    int prev_state;
};

// ============ GLOBAL VARIABLES ============
MotorState m1, m2, m3;
long load_cell_offset = 0;
float latest_valid_load = 0.0;
// ... dst variables ...

// ============ HELPER FUNCTIONS ============
float error_calculation(float ref, float act) {
    return ref - act;
}

float constraint_pwm(float pwm) {
    if (pwm > 255) return 255;
    if (pwm < -255) return -255;
    return pwm;
}

// ... dst semua functions ...

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    
    // Setup pins
    pinMode(RPWM1, OUTPUT);
    // ... dst pin setup ...
    
    // Initialize motor states
    m1.kp = 110.0;
    m1.kd = 0.1;
    // ... dst initialization ...
    
    Serial.println("Firmware Ready");
}

// ============ MAIN LOOP ============
void loop() {
    long current_time = millis();
    
    // Serial command processing
    if (Serial.available() > 0) {
        // ... parse commands ...
    }
    
    // Auto mode execution
    if (operating_mode == 1 || operating_mode == 2) {
        // ... motor control ...
    }
    
    // Manual mode
    else {
        // ... manual control ...
    }
}
```

### Keuntungan ✅
- Mudah, 1 file
- Langsung bisa di Arduino IDE
- Tidak perlu setup kompleks

### Kelemahan ❌
- Sangat panjang (1000+ lines)
- Susah di-organize
- Sulit untuk maintain

---

## 📝 Cara 2: Arduino IDE dengan Tabs

Arduino IDE support multiple "tabs" dalam 1 project!

### Step 1: Buat Folder Project

```bash
mkdir -p ~/Arduino/rehabilitation
cd ~/Arduino/rehabilitation
```

### Step 2: Buat File-File Terpisah

Arduino IDE akan menganggap setiap file sebagai "tab"

```
rehabilitation/
├── rehabilitation.ino        (Main file - HARUS ada)
├── config.h
├── MotorController.h
├── MotorController.cpp
├── LoadCell.h
├── LoadCell.cpp
└── ...
```

### Step 3: Main File - `rehabilitation.ino`

```cpp
//==================================================================
// rehabilitation.ino - Main File
// Arduino akan auto-include semua .h/.cpp files
//==================================================================

#include <stdint.h>

// Forward declarations
void setup();
void loop();
void update_encoders();
void calculate_ctc_control();
void apply_motor_control();
// ... dst

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("========== REHABILITATION SYSTEM ==========");
    Serial.println("Initializing...");
    
    // Setup pins
    pinMode(RPWM1, OUTPUT);
    pinMode(LPWM1, OUTPUT);
    // ... dst
    
    // Initialize
    calibrate_system();
    
    Serial.println("System Ready!");
}

void loop() {
    long current_time = millis();
    
    // Handle serial commands
    handle_serial_input();
    
    // Execute state machine
    execute_state_machine(current_time);
    
    // Update sensors
    update_sensors(current_time);
    
    // Control motors
    update_motor_control(current_time);
    
    delay(1);
}
```

### Step 4: Separate Files

**File: `config.h`**
```cpp
#ifndef CONFIG_H
#define CONFIG_H

// ============ PINS ============
#define RPWM1 3
#define LPWM1 5
// ... dst pins ...

// ============ CONSTANTS ============
const float GEAR_RATIO = 0.2786;
const float MOTOR_KT = 0.0663;
// ... dst constants ...

// ============ STRUCTURES ============
struct MotorState {
    float ref_pos, ref_velo;
    // ...
};

// ============ GLOBALS ============
extern MotorState m1, m2, m3;
extern long load_cell_offset;
// ... declare extern variables ...

#endif
```

**File: `MotorController.h`**
```cpp
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "config.h"

class MotorController {
public:
    void begin();
    void updateEncoders();
    void calculateCTCControl();
    void applyMotorControl();
    
private:
    // ...
};

// Global instance
extern MotorController motor;

#endif
```

**File: `MotorController.cpp`**
```cpp
#include "MotorController.h"

// Implement methods
void MotorController::begin() {
    pinMode(RPWM1, OUTPUT);
    // ...
}

void MotorController::updateEncoders() {
    // ...
}
```

### Step 5: How to Use in Arduino IDE

1. **Buka Arduino IDE**
2. **File → Open → rehabilitation folder**
3. Tabs akan muncul:
   - `rehabilitation.ino` (Main)
   - `config.h`
   - `MotorController.h`
   - `MotorController.cpp`
   - dll

4. **Edit tab yang mau**
5. **Verify (Ctrl+R)** - Compile
6. **Upload (Ctrl+U)** - Upload

### Keuntungan ✅
- Organized dengan tabs
- Mudah navigate antar files
- Arduino IDE native

### Kelemahan ❌
- Arduino IDE agak slow
- Path resolution ribet
- Debugging terbatas

---

## 📝 Cara 3: Arduino CLI (Command Line)

Jika prefer command line, bisa pakai **Arduino CLI**

### Install Arduino CLI

```bash
# macOS
brew install arduino-cli

# Linux
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Windows
choco install arduino-cli
```

### Setup Project

```bash
# Create sketch
arduino-cli sketch new rehabilitation

# Navigate
cd rehabilitation

# Copy files
cp ../firmware/src/*.* .
```

### Compile

```bash
# Verify (compile)
arduino-cli compile --fqbn arduino:avr:arduino rehabilitation.ino

# Upload
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:arduino rehabilitation.ino

# Monitor
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

---

## 🔄 Perbandingan 3 Metode

| Metode | Setup | Ease | Performance | Recommended |
|--------|-------|------|-------------|------------|
| **1 File .ino** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐ | Beginner |
| **IDE Tabs** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | Intermediate |
| **Arduino CLI** | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | Advanced |
| **PlatformIO** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | **BEST** ⭐ |

---

## 📝 Template: Convert Modular ke Arduino IDE Tabs

### Original Structure
```
firmware/src/
├── config/pins.h
├── config/constants.h
├── control/MotorController.h
├── control/MotorController.cpp
├── io/LoadCell.h
├── io/LoadCell.cpp
└── main.cpp
```

### Convert ke Arduino IDE Tabs

```
rehabilitation/
├── rehabilitation.ino          ← Copy isi dari main.cpp
├── config.h                    ← Merge pins.h + constants.h
├── MotorController.h           ← Copy as-is
├── MotorController.cpp         ← Copy as-is
├── LoadCell.h                  ← Copy as-is
├── LoadCell.cpp                ← Copy as-is
└── ...
```

### Step-by-Step

**1. Merge pins.h + constants.h → config.h**

```cpp
// config.h
#ifndef CONFIG_H
#define CONFIG_H

// ===== PINS =====
const int RPWM1 = 3;
// ... from pins.h ...

// ===== CONSTANTS =====
const float GEAR_RATIO = 0.2786;
// ... from constants.h ...

#endif
```

**2. Copy headers as-is**
```cpp
// MotorController.h (no changes needed)
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "config.h"

class MotorController {
    // ...
};

#endif
```

**3. Copy implementations as-is**
```cpp
// MotorController.cpp (no changes needed)
#include "MotorController.h"

void MotorController::begin() {
    // ...
}
```

**4. Main file dari main.cpp**
```cpp
// rehabilitation.ino
#include "config.h"
#include "MotorController.h"
// ... dst includes ...

void setup() {
    Serial.begin(115200);
    // ... from main.cpp setup ...
}

void loop() {
    // ... from main.cpp loop ...
}
```

---

## ✅ Best Practice untuk Arduino IDE

### DO ✅
```cpp
// DO: Use header guards
#ifndef CONFIG_H
#define CONFIG_H
// ...
#endif

// DO: Forward declare if needed
extern MotorController motor;

// DO: Include what you need
#include "MotorController.h"

// DO: Keep functions small
void updateEncoders() { /* ... */ }
```

### DON'T ❌
```cpp
// DON'T: Circular includes
#include "A.h"
#include "B.h"  // if B includes A → error!

// DON'T: Global variable soup
int x, y, z, a, b, c, ...  // Confusing!

// DON'T: One giant function
void loop() {
    // 500 lines of code
}

// DON'T: Magic numbers
pwm_value = pwm_value * 125;  // What is 125?
```

---

## 🎯 Recommended Path

### For Beginners
```
1. Copy everything to 1 file (.ino)
2. Test & verify it works
3. After comfortable → refactor to tabs
4. After experienced → switch to PlatformIO
```

### For Intermediate
```
1. Use Arduino IDE with Tabs
2. Keep organized with folders/structure
3. Use git for version control
4. Eventually move to PlatformIO
```

### For Advanced
```
Use PlatformIO from the start!
- Better tools
- Faster development
- Professional workflow
```

---

## 🔄 Migration Path: Arduino IDE → PlatformIO

**If you start with Arduino IDE, can easily migrate:**

```bash
# 1. Copy Arduino IDE tabs to folder
cp ~/Arduino/rehabilitation/* firmware/src/

# 2. Initialize PlatformIO
cd firmware
pio project init -b arduino

# 3. Rename .ino to main.cpp
mv rehabilitation.ino src/main.cpp

# 4. Update includes if needed
# (Usually no changes needed!)

# 5. Build & test
pio run
```

**That's it! All your code works in PlatformIO now.**

---

## 📞 Troubleshooting Arduino IDE

### Error: "No such file"
```cpp
// WRONG
#include "MotorController.cpp"

// CORRECT
#include "MotorController.h"
```

### Error: "Multiple definitions"
- Make sure header guards present
- Check no duplicate #include

### Slow compilation
- Split to multiple files
- Remove unnecessary #includes
- Consider PlatformIO (much faster!)

---

## 🎉 Summary

✅ **Recommend: PlatformIO** - Best tooling  
⚠️ **Alternative: Arduino IDE Tabs** - Native IDE support  
⚠️ **Last Resort: 1 File** - Simplest but messy  

**My advice:**
- Start simple (1 file) if beginner
- Use tabs when organized
- **Switch to PlatformIO ASAP** - Worth the setup time!

**Happy Coding! 🚀**