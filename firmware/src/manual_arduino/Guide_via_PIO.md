# Panduan Edit & Upload Arduino - Struktur Modular

## 📋 Masalah & Solusi

**Masalah:**
- Arduino IDE biasanya hanya support 1 file `.ino`
- File `.h` dan `.cpp` terpisah susah di-manage
- Include paths bisa error

**Solusi:**
Gunakan **PlatformIO** - lebih powerful & flexible dari Arduino IDE

---

## 🚀 Option 1: PlatformIO (Recommended ⭐)

### Step 1: Install PlatformIO

#### **Via VS Code (Easiest):**
1. Install **Visual Studio Code** dari https://code.visualstudio.com/
2. Buka VS Code
3. Klik icon Extensions (Ctrl+Shift+X)
4. Search "PlatformIO"
5. Klik "Install" pada "PlatformIO IDE"
6. Tunggu install selesai (~5 menit)
7. Restart VS Code

#### **Via Command Line:**
```bash
pip install platformio
pio --version
```

### Step 2: Setup Project Structure

```bash
# Navigate ke firmware folder
cd firmware

# Initialize PlatformIO project
pio project init -d . -b arduino

# Atau jika sudah ada folder:
pio init --board arduino --project-dir .
```

### Step 3: Configure `platformio.ini`

Edit file `firmware/platformio.ini`:

```ini
[env:arduino]
platform = atmelavr
board = arduino
framework = arduino
upload_port = /dev/ttyACM0        ; Linux/Mac
; upload_port = COM3              ; Windows (ganti sesuai port)
monitor_speed = 115200
build_flags = -Wall

[env:debug]
platform = atmelavr
board = arduino
framework = arduino
build_type = debug
```

### Step 4: Project Folder Structure

```
firmware/
├── platformio.ini              ← Build config
├── src/
│   ├── main.cpp                ← Main program (rename dari .ino)
│   ├── config/
│   │   ├── pins.h
│   │   └── constants.h
│   ├── control/
│   │   ├── MotorController.h
│   │   └── MotorController.cpp
│   ├── io/
│   │   ├── LoadCell.h
│   │   ├── LoadCell.cpp
│   │   ├── CurrentSensor.h
│   │   ├── CurrentSensor.cpp
│   │   ├── SerialComm.h
│   │   ├── SerialComm.cpp
│   │   ├── CommandParser.h
│   │   └── CommandParser.cpp
│   └── ...
└── lib/                        ← External libraries (if any)
```

### Step 5: Edit Code

**Example: Edit MotorController.cpp**

1. Open VS Code
2. Open folder: `File → Open Folder → firmware`
3. Navigate ke `src/control/MotorController.cpp`
4. Edit code
5. Save (Ctrl+S)

**PlatformIO automatically detects changes!**

### Step 6: Compile

**Option A: Via VS Code**
- Press `Ctrl+Shift+B` (Build)
- Atau klik tombol ✔️ di bawah

**Option B: Via Terminal**
```bash
cd firmware
pio run
```

**Output:**
```
Building in release mode
Linking .pio/build/arduino/firmware.elf
Checking size .pio/build/arduino/firmware.elf
  RAM:   [====      ]  37.5% (used 768 bytes from 2048 bytes)
  Flash: [========  ]  82.3% (used 26648 bytes from 32384 bytes)
========================= [ SUCCESS ] ==========================
```

### Step 7: Upload ke Arduino

**Option A: Via VS Code**
1. Colok Arduino via USB
2. Klik tombol ➡️ (Upload) di bawah
3. Tunggu "Success!"

**Option B: Via Terminal**
```bash
pio run -t upload
```

**Option C: Via VS Code Command**
- Press `Ctrl+Shift+P`
- Type "PlatformIO: Upload"
- Press Enter

### Step 8: Monitor Serial

**Option A: Via VS Code**
1. Klik tombol 🔌 (Monitor) di bawah
2. Serial monitor akan terbuka
3. Lihat output Arduino

**Option B: Via Terminal**
```bash
pio device monitor -p /dev/ttyACM0 -b 115200
```

**Output:**
```
--- Miniterm on /dev/ttyACM0  115200,8,N,1 ---
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
[MotorController] Initialized successfully
[LoadCell] Calibrated - Offset: 12345
[SerialComm] Initialized
```

---

## 📝 Workflow: Edit → Compile → Upload

### Scenario 1: Ubah Motor Gains

**File:** `firmware/src/config/constants.h`

```cpp
// BEFORE
const float MOTOR1_KP = 110.0;
const float MOTOR1_KD = 0.1;

// AFTER - Naikkan Kp
const float MOTOR1_KP = 120.0;  // ← EDIT INI
const float MOTOR1_KD = 0.1;
```

**Upload:**
```bash
cd firmware
pio run -t upload
pio device monitor
```

### Scenario 2: Edit MotorController Logic

**File:** `firmware/src/control/MotorController.cpp`

```cpp
// BEFORE
void MotorController::calculateCTCControl() {
    float kp1_scaled = m1.kp;
    // ...
}

// AFTER - Add load scaling
void MotorController::calculateCTCControl() {
    float load_scale = getLoadScaling();
    float kp1_scaled = m1.kp * load_scale;  // ← EDIT INI
    // ...
}
```

**Compile & Upload:**
```bash
pio run
pio run -t upload
```

### Scenario 3: Add New Function

**File:** `firmware/src/control/MotorController.h` (Declaration)

```cpp
class MotorController {
    // ... existing functions ...
    
public:
    // ✅ NEW: Add reset position function
    void resetMotorPosition(int motor_id);
};
```

**File:** `firmware/src/control/MotorController.cpp` (Implementation)

```cpp
// ✅ NEW: Implement function
void MotorController::resetMotorPosition(int motor_id) {
    MotorState* motor = getMotorState(motor_id);
    if (motor) {
        motor->encoder_count = 0;
        motor->actual_pos = 0;
        motor->prev_pos = 0;
    }
}
```

**File:** `firmware/src/main.cpp` (Use function)

```cpp
// In setup or wherever needed
motor.resetMotorPosition(1);
motor.resetMotorPosition(2);
motor.resetMotorPosition(3);
```

**Upload:**
```bash
pio run -t upload
```

---

## 🔧 Common Edit Scenarios

### ✏️ Edit 1: Ubah Load Thresholds

**File:** `firmware/src/config/constants.h`

```cpp
// BEFORE
const int THRESHOLD1_DEFAULT = 20;
const int THRESHOLD2_DEFAULT = 40;

// AFTER
const int THRESHOLD1_DEFAULT = 25;  // ← Ubah ini
const int THRESHOLD2_DEFAULT = 50;  // ← Dan ini
```

**Upload:**
```bash
pio run -t upload
```

### ✏️ Edit 2: Ubah PWM Speed

**File:** `firmware/src/config/constants.h`

```cpp
// BEFORE
const int MANUAL_SPEED = 125;

// AFTER
const int MANUAL_SPEED = 150;  // ← Lebih cepat
```

### ✏️ Edit 3: Ubah Timing Interval

**File:** `firmware/src/config/constants.h`

```cpp
// BEFORE
const int LOAD_CELL_INTERVAL = 100;  // Read load setiap 100ms

// AFTER
const int LOAD_CELL_INTERVAL = 50;   // Read load lebih sering (50ms)
```

### ✏️ Edit 4: Ubah Control Gains

**File:** `firmware/src/config/constants.h`

```cpp
// BEFORE - Motor 1
const float MOTOR1_KP = 110.0;
const float MOTOR1_KD = 0.1;
const float MOTOR1_KPC = 30.0;
const float MOTOR1_KDC = 0.1;

// AFTER - Increase responsiveness
const float MOTOR1_KP = 130.0;    // ← Naikkan Kp
const float MOTOR1_KD = 0.15;     // ← Naikkan Kd
const float MOTOR1_KPC = 40.0;    // ← Naikkan Kpc
const float MOTOR1_KDC = 0.15;    // ← Naikkan Kdc
```

### ✏️ Edit 5: Ubah Pin Assignment

**File:** `firmware/src/config/pins.h`

```cpp
// BEFORE
const int RPWM1 = 3;
const int LPWM1 = 5;

// AFTER - Ganti ke pin berbeda
const int RPWM1 = 6;   // ← Ubah ini
const int LPWM1 = 9;   // ← Ubah ini
```

---

## 🚨 Troubleshooting Edit & Upload

### Problem 1: "Compilation Error"

**Error Message:**
```
error: no matching function for call to 'MotorController::someFunction(int)'
```

**Solusi:**
1. Check `.h` file sudah punya declaration
2. Check parameter types match
3. Check include guards (`#ifndef`, `#define`, `#endif`)

**Contoh Fix:**
```cpp
// File: MotorController.h
class MotorController {
    void someFunction(int value);  // ← Add declaration
};

// File: MotorController.cpp
void MotorController::someFunction(int value) {  // ← Add implementation
    // ...
}
```

### Problem 2: "Include Not Found"

**Error Message:**
```
fatal error: control/MotorController.h: No such file or directory
```

**Solusi:**
Check include path di `main.cpp`:
```cpp
// WRONG
#include "MotorController.h"

// CORRECT
#include "control/MotorController.h"
```

### Problem 3: "Upload Failed"

**Error:**
```
avrdude: stk500_recv(): programmer is not responding
```

**Solusi:**
1. Check Arduino plugged in & port correct di `platformio.ini`
2. Try: `pio device list` untuk detect port
3. Update `platformio.ini`:
   ```ini
   upload_port = /dev/ttyACM0  ; ubah sesuai output
   ```

### Problem 4: "Function Redefined"

**Error:**
```
multiple definition of 'MotorController::someFunction()'
```

**Solusi:**
Function implemented di 2 tempat. Delete duplicate.

---

## 💡 Best Practices

### 1️⃣ Edit Hanya Perlu Changes

❌ **JANGAN:**
```cpp
// Buat file baru setiap kali
firmware_v1.cpp
firmware_v2.cpp
firmware_v3.cpp
```

✅ **LAKUKAN:**
```cpp
// Edit file yang sama, simpan dengan version control
git add firmware/src/control/MotorController.cpp
git commit -m "Increase Kp from 110 to 130"
```

### 2️⃣ Test Changes Incrementally

❌ **JANGAN:**
```cpp
// Ubah banyak hal sekaligus
Ubah Kp, Kd, Pin, Gains, Timing, dll
// Tidak tahu yang mana yang error
```

✅ **LAKUKAN:**
```cpp
// Ubah 1 hal, test, ulangi
1. Ubah Kp → Upload → Test → OK?
2. Ubah Kd → Upload → Test → OK?
3. Ubah Pin → Upload → Test → OK?
```

### 3️⃣ Use Comments untuk Track Changes

```cpp
// MOTOR GAINS - Updated 2024-01-15
// Previous: Kp=110, Kd=0.1
// Current: Kp=120, Kd=0.15 (more responsive)
const float MOTOR1_KP = 120.0;
const float MOTOR1_KD = 0.15;
```

### 4️⃣ Keep Backup of Working Code

```bash
# Sebelum edit besar
git branch -b feature/new-control-algorithm
# Edit code
# Test
# Jika berhasil: git merge
# Jika gagal: git checkout (revert)
```

---

## 📋 Quick Reference - Commands

### Build & Upload
```bash
pio run                          # Compile
pio run -t upload                # Compile + Upload
pio run -t clean                 # Clean build files
pio run -e debug                 # Build debug version
```

### Monitor & Debug
```bash
pio device monitor               # Serial monitor
pio device list                  # List serial ports
pio run -t monitor               # Upload + monitor
```

### Project Management
```bash
pio init                         # Initialize new project
pio lib install "library name"   # Install library
pio lib list                     # List installed libraries
```

---

## 🎯 Workflow Example: Complete Edit Cycle

### Scenario: Naikkan Motor 1 Responsiveness

```bash
# 1. Navigate to project
cd firmware

# 2. Edit constants
# File: src/config/constants.h
# Change:
#   MOTOR1_KP: 110.0 → 130.0
#   MOTOR1_KD: 0.1 → 0.2

# 3. Compile
pio run
# Output: SUCCESS

# 4. Upload to Arduino
pio run -t upload
# Output: Upload successful

# 5. Monitor result
pio device monitor -p /dev/ttyACM0 -b 115200
# Observe motor response

# 6. If good, commit
git add src/config/constants.h
git commit -m "Increase Motor1 responsiveness: Kp 110→130, Kd 0.1→0.2"

# 7. If bad, revert
git checkout src/config/constants.h
pio run -t upload
```

---

## 🔄 Switching Between Different Versions

### Save Current Working Version
```bash
# Create branch
git checkout -b production/v1.0

# Or save file copy
cp src/config/constants.h constants_v1_0.h
```

### Try New Experimental Version
```bash
# Create new branch
git checkout -b experimental/new-algorithm

# Edit & test
# If works: git merge experimental/new-algorithm master
# If not: git checkout master (revert)
```

---

## ✅ Checklist Sebelum Upload

- [ ] All `.h` files ada di folder yang benar
- [ ] All `.cpp` files punya `#include` header nya
- [ ] Tidak ada circular includes
- [ ] `platformio.ini` ada & correct port
- [ ] Arduino plugged in via USB
- [ ] `pio run` compile SUCCESS (no errors)
- [ ] `pio device list` detect Arduino port
- [ ] Backup working code (git commit)

---

## 🎓 Next Level: Advanced Editing

### Conditional Compilation
```cpp
// In constants.h atau platformio.ini
#define DEBUG_MODE 1

// In code
#ifdef DEBUG_MODE
    Serial.println("[DEBUG] Motor position: " + String(pos));
#endif
```

### Multiple Configurations
```ini
; platformio.ini
[env:production]
build_flags = -DPRODUCTION_MODE

[env:debug]
build_flags = -DDEBUG_MODE -Wall -g
```

### Remote Monitoring
```bash
# Upload & monitor in one command
pio run -t upload && pio device monitor
```

---

## 📞 Getting Help

- **PlatformIO Docs:** https://docs.platformio.org/
- **Arduino Docs:** https://www.arduino.cc/reference/en/
- **GitHub Issues:** Buka issue di repository

---

## 🎉 Summary

✅ **Gunakan PlatformIO** - Most user-friendly  
✅ **Edit file directly** - No copy-paste needed  
✅ **Auto compile** - One command: `pio run -t upload`  
✅ **Easy monitoring** - `pio device monitor`  
✅ **Version control** - Git integration  

**Happy Coding! 🚀**