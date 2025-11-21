===================================
===========Prerequisites===========
===================================
Arduino IDE v1.8.19+ atau PlatformIO
C++ compiler (g++ atau MSVC untuk server)
Python 3.8+ untuk vision system
Git untuk version control


===========================================
===========Structure Setup===========
===========================================

rehabilitation-system/
├── firmware/
│   ├── src/
│   │   ├── main.cpp
│   │   ├── config/
│   │   │   ├── pins.h
│   │   │   └── constants.h
│   │   ├── control/
│   │   ├── io/
│   │   └── ...
│   └── platformio.ini (optional)
│
├── server/
│   ├── src/
│   │   ├── main.cpp
│   │   ├── modbus/
│   │   ├── trajectory/
│   │   ├── serial/
│   │   ├── state_machine/
│   │   └── config/
│   ├── CMakeLists.txt
│   └── build/
│
├── vision/
│   ├── src/
│   │   ├── main.py
│   │   ├── vision/
│   │   ├── control/
│   │   ├── utils/
│   │   └── config/
│   ├── requirements.txt
│   └── ...
│
├── data/
│   ├── trajectory_1/
│   ├── trajectory_2/
│   └── trajectory_3/
│
├── docs/
│   ├── ARCHITECTURE.md
│   ├── SETUP.md (ini)
│   ├── API.md
│   └── TROUBLESHOOTING.md
│
└── README.md

==================================================
==========Step 1: Setup Arduino Firmware==========
==================================================

Option A: Arduino IDE

Buka Arduino IDE
File → Preferences

Tambahkan library path ke project firmware folder


Buat folder struktur:

   firmware/src/config/
   firmware/src/control/

Copy file:

firmware/src/main.cpp → controll_2.ino (rename)
Atau gunakan .cpp langsung di Arduino IDE (v1.6.6+)



Option B: PlatformIO (Recommended)
Install PlatformIO:
bashpip install platformio
Buat project:
bashpio project init -d firmware -b arduino
Update firmware/platformio.ini:
ini[env:arduino]
platform = atmelavr
board = arduino
framework = arduino
upload_port = /dev/ttyACM0  ; atau COM3 di Windows
monitor_speed = 115200
build_flags = -Wall

[env:test]
platform = atmelavr
board = arduino
framework = arduino
build_type = debug
Compile:
bashpio run -d firmware -e arduino
Upload:
bashpio run -d firmware -t upload
Verify Upload:
bashpio device monitor -p /dev/ttyACM0 -b 115200
Pastikan output:
===========================================
  3-RPS Parallel Robot Control System
  Adaptive CTC + Load-based Control
===========================================

============================================================
=============Step 2: Setup Control Server (C++)=============
============================================================

Install Dependencies
Linux (Ubuntu/Debian):
bashsudo apt-get install -y \
    build-essential \
    cmake \
    libmodbus-dev \
    libboost-all-dev \
    libc-dev
Windows:

Install MinGW atau Visual Studio
Download modbus library dari http://libmodbus.org/download/
Download Boost library

Build Server
Navigate to server folder:
bashcd server
Create build directory:
bashmkdir build
cd build
Configure & Build:
bashcmake ..
make -j4
Expected output:
Built target rehab_server
Test Server
Run the server:
bash./rehab_server
Expected output:
===========================================
  SISTEM KONTROL REHABILITASI
  Multi-Trajectory + Cycle Counter
===========================================
Server Modbus berjalan. Menunggu koneksi HMI...

==========================================================
===========Step 3: Setup Vision System (Python)===========
==========================================================

Create Virtual Environment
bashcd vision
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# atau
venv\Scripts\activate  # Windows
Install Dependencies
bashpip install -r requirements.txt
File vision/requirements.txt:
opencv-python==4.8.0.76
mediapipe==0.10.0
numpy==1.24.0
Test Vision System
bashpython src/main.py
Expected output:
[Vision] Initializing MediaPipe...
[Vision] Camera opened successfully
[Vision] Starting frame capture...
Press 'l' untuk LEFT foot atau 'r' untuk RIGHT foot.
Press 'q' untuk quit.

=====================================================
============Step 4: Load Trajectory Data=============
=====================================================

Data Format
Setiap trajectory membutuhkan files:
data_1/
├── grafik.txt          # Format: x,y (1 line per point)
├── pos1.txt            # Reference positions for motor 1
├── pos2.txt
├── pos3.txt
├── velo1.txt           # Reference velocities
├── velo2.txt
├── velo3.txt
├── fc1.txt             # Feedforward forces
├── fc2.txt
└── fc3.txt
File Format Example
grafik.txt:
0.0,0.0
10.5,5.2
20.3,10.1
...
pos1.txt:
0.0
5.2
10.4
15.6
...
Load Data
Server akan otomatis load data saat startup dari folder data/:
data/
├── trajectory_1/  (816 points)
├── trajectory_2/  (1370 points)
└── trajectory_3/  (1370 points)

===================================================
============Step 5: Run Complete System============
===================================================

Terminal 1 - Arduino Monitor
bashpio device monitor -p /dev/ttyACM0 -b 115200
Tunggu sampai Arduino ready.
Terminal 2 - Control Server
bashcd server/build
./rehab_server
Tunggu sampai: Menunggu koneksi HMI...
Terminal 3 - Vision System (Optional)
bashcd vision
source venv/bin/activate
python src/main.py
Terminal 4 - HMI Connection
Gunakan software Modbus TCP client untuk connect ke server:

Host: localhost (127.0.0.1)
Port: 5020
Slave ID: 1


🔌 Hardware Connection
Arduino Pinout
Motor Control (PWM):
Motor 1: RPWM=D3, LPWM=D5 → H-Bridge Motor 1
Motor 2: RPWM=D6, LPWM=D9 → H-Bridge Motor 2
Motor 3: RPWM=D10, LPWM=D11 → H-Bridge Motor 3
Feedback:
ENC1=D4, ENC2=D2, ENC3=D8 → Encoder inputs
CurrSen1=A0, CurrSen2=A1, CurrSen3=A2 → Current sensors
Load Cell (HX711):
DOUT=D12, CLK=D13
Serial Connection
Arduino ↔ PC via USB (Auto detect COM port)

Baud rate: 115200
Data bits: 8
Stop bits: 1
Parity: None

 Verification Checklist

 Arduino firmware upload successful
 Serial monitor shows startup message
 Control server builds without errors
 Server connects successfully
 Trajectory data loaded (check console output)
 Vision system captures video frame
 All 3 motors respond to manual commands
 Load cell reads value
 HMI can communicate via Modbus


Troubleshooting
Arduino tidak terdeteksi
bash# List available ports
pio device list

# Update upload_port in platformio.ini
Modbus connection failed
bash# Check port availability
lsof -i :5020  # Linux
netstat -ano | findstr :5020  # Windows
Trajectory data not loading

Verify file paths di server/src/config/trajectory_paths.h
Check file format (CSV with comma separator)
Ensure correct number of lines

Vision system slow

Reduce camera resolution
Lower FPS target (30 → 15)
Check CPU usage


📝 Configuration Files
Firmware Configuration
firmware/src/config/constants.h:
cppconst float GEAR_RATIO = 0.2786;
const float MOTOR_KT = 0.0663;
const int MANUAL_SPEED = 125;
const int threshold1 = 20;  // Load threshold (N)
const int threshold2 = 40;  // Retreat threshold (N)
Server Configuration
server/src/config/config.h:
cppconst int MODBUS_PORT = 5020;
const int SERIAL_BAUDRATE = 115200;
const int TRAJECTORY_UPDATE_RATE = 100;  // ms
const int POST_REHAB_DELAY = 5;  // seconds
Vision Configuration
vision/src/config/settings.py:
pythonPID_PARAMS = {
    'kp': 0.5,
    'ki': 0.05,
    'kd': 0.1
}

ANGLE_OFFSET = -35
TARGET_ANGLE = 90
CAMERA_ID = 0
FPS = 30

🎓 Testing Protocol
Test 1: Manual Motor Control
1. Serial console: Send "1" (forward)
   → Motors harus bergerak maju
2. Serial console: Send "2" (backward)
   → Motors harus bergerak mundur
3. Serial console: Send "0" (stop)
   → Motors harus berhenti

   
Test 2: Load Cell Calibration
1. Serial console: Send "X" (calibrate)
   → "System Reset: Tare & Zero OK"
2. Place 10 kg on load cell
   → Read value harus ~100 N


Test 3: Trajectory Execution
1. Server: Trajectory 1 loaded
2. HMI: Click START button
   → Animation harus berjalan smooth
3. Monitor motor positions: harus smooth tracking

📞 Support
Jika ada masalah:

Check docs/TROUBLESHOOTING.md
Check serial output untuk error messages
Verify hardware connections
Review configuration files