# Arduino Edit & Upload - Quick Cheatsheet

## 🚀 Fastest Way (PlatformIO)

### Install (5 minutes)
```bash
# Install VS Code + PlatformIO extension
# Done! Ready to code
```

### Edit & Upload (1 command)
```bash
cd firmware
pio run -t upload              # Compile & Upload
pio device monitor             # See Arduino output
```

---

## 📋 3 Ways to Edit & Upload

### Method 1: PlatformIO (⭐ RECOMMENDED)

```bash
# Setup (one time)
cd firmware
pio init -b arduino

# Edit file
vim src/config/constants.h     # Edit gains, pins, etc

# Upload
pio run -t upload              # Done! Simple!

# Monitor
pio device monitor
```

**Best for:** Professional development

---

### Method 2: Arduino IDE with Tabs

```bash
# Setup (one time)
1. Open Arduino IDE
2. File → New
3. Create folder: ~/Arduino/rehabilitation
4. Copy all .h & .cpp files there

# Edit
1. Double-click file in tabs
2. Make changes
3. Ctrl+S save

# Upload
1. Ctrl+U (Upload)
2. Wait for success

# Monitor
1. Tools → Serial Monitor
2. Set 115200 baud
```

**Best for:** Beginners, Arduino IDE users

---

### Method 3: Single File (.ino)

```bash
# Setup (one time)
1. Concatenate ALL files to 1 file.ino
2. Copy to ~/Arduino/rehabilitation/

# Edit
1. Arduino IDE
2. Edit the big file
3. Ctrl+S

# Upload
1. Ctrl+U
2. Wait...

# Monitor
Tools → Serial Monitor
```

**Best for:** Total beginners

---

## 🔧 Common Edit Tasks

### Task 1: Change Motor Gain

```bash
# Edit file (any method)
firmware/src/config/constants.h

# Find & change:
const float MOTOR1_KP = 110.0;  ← Change to 130.0

# Save

# Upload: pio run -t upload
```

### Task 2: Change Load Threshold

```bash
# Edit:
firmware/src/config/constants.h

const int THRESHOLD1_DEFAULT = 20;  ← Change to 25
const int THRESHOLD2_DEFAULT = 40;  ← Change to 45

# Upload: pio run -t upload
```

### Task 3: Edit Motor Control Logic

```bash
# Edit:
firmware/src/control/MotorController.cpp

# Find function & edit
void MotorController::calculateCTCControl() {
    // Make your changes here
}

# Save

# Upload: pio run -t upload
```

### Task 4: Add New Function

```bash
# 1. Declare in .h file
firmware/src/control/MotorController.h:

void newFunction(int param);  ← Add this

# 2. Implement in .cpp file
firmware/src/control/MotorController.cpp:

void MotorController::newFunction(int param) {
    // Your code here
}

# 3. Use in main.cpp
firmware/src/main.cpp:

motor.newFunction(123);  ← Call it

# 4. Upload: pio run -t upload
```

---

## ⚡ Super Quick Upload Commands

### PlatformIO
```bash
cd firmware
pio run -t upload    # Upload
pio device monitor   # Monitor
pio run              # Just compile (no upload)
```

### Arduino IDE
```
Ctrl+U               # Upload
Ctrl+R               # Compile only
Ctrl+Shift+M         # Serial Monitor
```

### Arduino CLI
```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:arduino .
```

---

## 🐛 Fix Common Errors

### "Board not found"
```bash
# PlatformIO: Update platformio.ini
upload_port = /dev/ttyACM0   # or COM3 on Windows

# Arduino IDE: Select board
Tools → Board → Arduino Uno
Tools → Port → /dev/ttyACM0
```

### "Compile error: no matching function"
```cpp
// Check .h file has declaration
class MotorController {
public:
    void myFunction();  // ← Must be here
};

// And .cpp has implementation
void MotorController::myFunction() {
    // Code here
}
```

### "Include not found"
```cpp
// Check include path is correct
#include "config/pins.h"        // ✅ Correct
#include "pins.h"               // ❌ Wrong (path missing)
```

### Slow compilation
```bash
# Use PlatformIO (10x faster!)
pio run

# Or clean build
pio run -t clean
pio run
```

---

## 📊 Before/After Checklist

### Before Edit
- [ ] Arduino plugged in
- [ ] Know which file to edit
- [ ] Backup current version (git commit)

### After Edit
- [ ] Compile successful (no red errors)
- [ ] Arduino still plugged in
- [ ] Click upload button / run command
- [ ] Wait for "Upload successful"
- [ ] Open serial monitor to verify

---

## 🎯 Decision Tree

```
Mau edit firmware?
│
├─ Pengen yang paling mudah?
│  └─ Use: Arduino IDE dengan tabs
│     Edit file → Ctrl+U → Done
│
├─ Pengen yang paling professional?
│  └─ Use: PlatformIO
│     Edit file → pio run -t upload → Done
│
└─ Pengen yang super simple (tapi ribet)?
   └─ Use: 1 file .ino
      Edit 1 giant file → Ctrl+U → Done
```

---

## 📝 File Locations Quick Reference

```
firmware/
├── platformio.ini              ← Build config (if using PlatformIO)
│
├── src/
│   ├── main.cpp                ← Main program - EDIT THIS
│   │
│   ├── config/
│   │   ├── pins.h              ← Pin definitions - EDIT THIS
│   │   └── constants.h         ← Gains, thresholds - EDIT THIS OFTEN
│   │
│   ├── control/
│   │   ├── MotorController.h
│   │   └── MotorController.cpp ← Motor logic - EDIT THIS
│   │
│   └── io/
│       ├── LoadCell.h
│       ├── LoadCell.cpp        ← Load cell logic - EDIT THIS
│       ├── SerialComm.h
│       └── CommandParser.h
```

---

## 🚀 Super Quick Start (Impatient?)

### Using PlatformIO (recommended)

```bash
# 1. Install (1 time)
# Install VS Code → Install PlatformIO extension

# 2. Open project
# File → Open Folder → firmware/

# 3. Edit
# Open any file in src/

# 4. Upload
# Press Ctrl+Shift+B (Build)
# Then press Upload button or...
# Terminal → pio run -t upload

# 5. Monitor
# Terminal → pio device monitor

# That's it! 🎉
```

### Using Arduino IDE (simple)

```bash
# 1. Copy files
# ~/Arduino/rehabilitation/rehabilitation.ino
# + all .h/.cpp files

# 2. Open Arduino IDE
# File → Open → rehabilitation.ino

# 3. Edit (click tabs to switch files)

# 4. Ctrl+U (Upload)

# 5. Tools → Serial Monitor

# Done! 🎉
```

---

## 💾 Save/Backup Workflow

```bash
# Before making big changes
git add -A
git commit -m "Backup before editing motor gains"

# Edit files...

# If broken, revert
git checkout firmware/src/config/constants.h

# If working, commit
git add -A
git commit -m "Increase Kp from 110 to 130 - more responsive"
```

---

## 🎓 Learning Path

```
Day 1: Edit constants (easiest)
  └─ Change MOTOR1_KP in constants.h

Day 2: Edit functions (medium)
  └─ Change code in MotorController.cpp

Day 3: Add functions (hard)
  └─ Add declaration in .h, implementation in .cpp

Day 4: Refactor (advanced)
  └─ Reorganize code, split functions, optimize
```

---

## 📞 Need Help?

| Problem | Check |
|---------|-------|
| Upload fails | Arduino plugged in? Port correct? |
| Compile error | Syntax? Include path? |
| Motor not responsive | Gains too low? Pin wrong? |
| Serial garbage | Baud rate 115200? |

**Quick Debug:**
```bash
pio run -v              # Verbose output (more details)
pio device list         # List available ports
pio run -t clean && pio run    # Clean rebuild
```

---

## ✅ Pro Tips

1. **Edit one thing at a time**
   - Change Kp → test → OK?
   - Change Kd → test → OK?
   - Not all at once!

2. **Use git frequently**
   ```bash
   git add .
   git commit -m "description"
   ```

3. **Keep comments**
   ```cpp
   // MOTOR GAINS - Updated 2024
   // Old: Kp=110 (slow)
   // New: Kp=130 (responsive)
   const float MOTOR1_KP = 130.0;
   ```

4. **Test incrementally**
   - Edit → Compile → Upload → Test
   - Not: Edit all → compile → get errors

---

## 🎉 TL;DR (Too Long; Didn't Read)

**Fastest way:**
```bash
cd firmware
pio run -t upload
pio device monitor
```

**Simplest way:**
- Open Arduino IDE
- Edit tab
- Ctrl+U

**Pick your tool & start coding! 🚀**