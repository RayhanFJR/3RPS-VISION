# Scripts Quick Reference Guide

## 📁 Scripts Location

```
scripts/
├── install_dependencies.sh    # Install all dependencies (RUN FIRST!)
├── build_firmware.sh          # Build & upload Arduino firmware
├── build_server.sh            # Build C++ server
├── run_vision.sh              # Start vision system
├── test_system.sh             # Run all tests
├── start_all.sh               # Start complete system
├── clean.sh                   # Clean build artifacts
├── format_code.sh             # Format code (optional)
└── README.md                  # This file
```

---

## 🚀 Quick Start (3 commands)

```bash
# 1. Setup (one time)
bash scripts/install_dependencies.sh

# 2. Build everything
bash scripts/build_firmware.sh
bash scripts/build_server.sh

# 3. Run (or use start_all.sh)
bash scripts/start_all.sh
```

---

## 📋 Script Reference

### 1. `install_dependencies.sh` - Setup All Dependencies

**What it does:**
- Detects OS (Linux, macOS, Windows)
- Installs system packages (cmake, build tools, libraries)
- Installs Python packages (MediaPipe, OpenCV, NumPy)
- Installs PlatformIO for Arduino
- Initializes Git repository

**Usage:**
```bash
bash scripts/install_dependencies.sh
```

**Output:**
```
✓ Python dependencies installed
✓ Linux dependencies installed
✓ PlatformIO installed
✓ Git repository initialized
```

**One-time only** - Run this first!

---

### 2. `build_firmware.sh` - Build & Upload Arduino

**What it does:**
- Compiles Arduino firmware
- Detects Arduino board on USB
- Uploads compiled code
- Opens serial monitor

**Usage:**
```bash
bash scripts/build_firmware.sh
```

**Interactive:**
```
Available ports:
/dev/ttyACM0
Proceed with upload? (y/n) y

[Build] Uploading firmware...
✓ Upload successful
```

**When to run:**
- After firmware changes
- After first setup
- Before testing

---

### 3. `build_server.sh` - Build C++ Server

**What it does:**
- Creates build directory
- Runs CMake configuration
- Compiles server with make
- Verifies executable

**Usage:**
```bash
bash scripts/build_server.sh
```

**Output:**
```
✓ CMake configured
✓ Build successful
✓ Executable ready: .../build/rehab_server
```

**When to run:**
- After server code changes
- After first setup

---

### 4. `run_vision.sh` - Start Vision System

**What it does:**
- Checks Python dependencies
- Verifies camera detection
- Creates directories
- Starts vision application

**Usage:**
```bash
bash scripts/run_vision.sh
```

**Controls:**
```
l = LEFT foot
r = RIGHT foot
c = CALIBRATE
q = QUIT
```

**When to run:**
- To test vision system
- Real-time pose tracking
- Foot angle calibration

---

### 5. `test_system.sh` - Run All Tests

**What it does:**
- Runs Python unit tests
- Runs Python integration tests
- Reports test results
- Shows summary

**Usage:**
```bash
bash scripts/test_system.sh
```

**Output:**
```
[Test] Python tests...
✓ Python tests passed

[Test] Integration tests...
✓ Integration tests passed

✓ All tests passed!
```

**When to run:**
- After code changes
- Before deployment
- Quality assurance

---

### 6. `start_all.sh` - Start Complete System

**What it does:**
- Starts firmware monitor
- Starts server
- Starts vision system
- Optional: Background mode with logging

**Usage (Interactive):**
```bash
bash scripts/start_all.sh
```
Instructions to run in separate terminals.

**Usage (Background):**
```bash
bash scripts/start_all.sh --background
```

**Background mode output:**
```
Process IDs:
  Firmware: 1234
  Server:   1235
  Vision:   1236

Log files in: logs/

To stop:
  kill 1234 1235 1236
```

**When to run:**
- Normal operation
- Testing complete system

---

### 7. `clean.sh` - Clean Build Artifacts

**What it does:**
- Cleans firmware build
- Cleans server build
- Removes Python cache
- Frees disk space

**Usage:**
```bash
bash scripts/clean.sh
```

**When to run:**
- Before fresh build
- To free space
- Troubleshooting build issues

---

### 8. `format_code.sh` - Format Code

**What it does:**
- Formats C++ files (clang-format)
- Formats Python files (black)
- Standardizes code style

**Usage:**
```bash
bash scripts/format_code.sh
```

**Requirements:**
- `clang-format` (C++)
- `black` (Python)

**When to run:**
- Before code review
- Before commit
- Optional

---

## 🎯 Common Workflows

### Workflow 1: First Time Setup (30 min)

```bash
# 1. Install all dependencies
bash scripts/install_dependencies.sh

# 2. Build firmware
bash scripts/build_firmware.sh
# Follow prompts, upload to Arduino

# 3. Build server
bash scripts/build_server.sh

# 4. Verify tests pass
bash scripts/test_system.sh

# 5. Start system
bash scripts/start_all.sh
```

---

### Workflow 2: Daily Development

```bash
# 1. Make code changes
vim firmware/src/control/MotorController.cpp

# 2. Rebuild firmware
bash scripts/build_firmware.sh

# 3. Run tests
bash scripts/test_system.sh

# 4. Start system
bash scripts/start_all.sh
```

---

### Workflow 3: Before Deployment

```bash
# 1. Clean old builds
bash scripts/clean.sh

# 2. Fresh build everything
bash scripts/build_firmware.sh
bash scripts/build_server.sh

# 3. Run comprehensive tests
bash scripts/test_system.sh

# 4. Check code quality
bash scripts/format_code.sh

# 5. Ready to deploy!
```

---

### Workflow 4: Testing Specific Component

```bash
# Test firmware only
cd firmware && pio run -t upload

# Test server only
cd server/build && ./rehab_server

# Test vision only
bash scripts/run_vision.sh

# All tests
bash scripts/test_system.sh
```

---

## 📊 Script Execution Order

```
Recommended sequence:

1st Run (Setup):
  install_dependencies.sh
  ↓
  build_firmware.sh
  ↓
  build_server.sh
  ↓
  test_system.sh

Daily Development:
  build_firmware.sh (if firmware changed)
  ↓
  build_server.sh (if server changed)
  ↓
  test_system.sh
  ↓
  start_all.sh

Before Deployment:
  clean.sh
  ↓
  build_firmware.sh
  ↓
  build_server.sh
  ↓
  test_system.sh
  ↓
  format_code.sh
```

---

## 🐛 Troubleshooting Scripts

### Problem: "Permission denied"

```bash
# Make scripts executable
chmod +x scripts/*.sh

# Then run
bash scripts/install_dependencies.sh
```

### Problem: "Command not found: pio"

```bash
# Reinstall PlatformIO
pip install platformio
pio platform install atmelavr
```

### Problem: "CMake not found"

```bash
# Install CMake
# Linux:
sudo apt-get install cmake

# macOS:
brew install cmake

# Windows:
# Download from https://cmake.org/download/
```

### Problem: "Arduino not detected"

```bash
# Check USB connection
lsusb | grep Arduino

# Check port
pio device list

# Try different port in build_firmware.sh
```

---

## 📝 Customization

### Change Arduino Port

Edit `build_firmware.sh`:
```bash
# Add specific port
upload_port = /dev/ttyACM0   # Linux
upload_port = COM3           # Windows
upload_port = /dev/tty.usbmodem*  # Mac
```

### Change Python Interpreter

Edit any script:
```bash
python3 src/main.py  # Use specific version
```

### Disable Background Mode

In `start_all.sh`, remove `--background` flag usage.

---

## 🔗 Integration with CI/CD

### GitHub Actions Example

```yaml
name: Build & Test

on: [push]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - run: bash scripts/install_dependencies.sh
      - run: bash scripts/build_firmware.sh
      - run: bash scripts/build_server.sh
      - run: bash scripts/test_system.sh
```

---

## 📊 Logs & Output

### Log Locations (with `--background`)

```
logs/
├── firmware.log   # Arduino output
├── server.log     # Server output
└── vision.log     # Vision output
```

**View logs:**
```bash
tail -f logs/server.log
tail -f logs/vision.log
```

---

## ✅ Pre-Deployment Checklist

- [ ] Run `install_dependencies.sh`
- [ ] Run `build_firmware.sh` - ✓ Upload successful
- [ ] Run `build_server.sh` - ✓ Executable ready
- [ ] Run `test_system.sh` - ✓ All tests passed
- [ ] Run `start_all.sh` - ✓ All components running
- [ ] Check logs in `logs/` directory
- [ ] Verify HMI connection to Modbus
- [ ] Test manual control
- [ ] Test trajectory execution
- [ ] Ready to deploy!

---

## 🎓 Learning Path

**Day 1: Setup**
- Run `install_dependencies.sh`
- Run `build_firmware.sh`
- Understand what each script does

**Day 2: Development**
- Make small code changes
- Run `build_firmware.sh` & `build_server.sh`
- Run `test_system.sh`

**Day 3: Testing**
- Run `test_system.sh` frequently
- Run `start_all.sh`
- Monitor logs

**Day 4: Deployment**
- Run `clean.sh`
- Run all build scripts
- Run `test_system.sh`
- Deploy!

---

## 🚀 Tips & Tricks

### Quick Rebuild
```bash
cd firmware && pio run -t upload && pio device monitor
```

### Background Server
```bash
cd server/build && nohup ./rehab_server > server.log 2>&1 &
```

### Watch Changes
```bash
watch -n 1 'tail -10 logs/server.log'
```

### Kill All Processes
```bash
pkill -f rehab_server
pkill -f "pio device monitor"
pkill -f "python.*main.py"
```

---

## 📞 Help & Documentation

- Check individual script comments for details
- Read ARCHITECTURE.md for system overview
- Read SETUP.md for detailed instructions
- Run `bash script_name.sh --help` (if implemented)

---

## 🎉 You're Ready!

All scripts are ready to use. Start with:

```bash
bash scripts/install_dependencies.sh
bash scripts/build_firmware.sh
bash scripts/build_server.sh
bash scripts/start_all.sh
```

**Happy coding!** 🚀