# Server Module - Comprehensive Guide

## 📁 Folder Structure

```
server/
├── src/
│   ├── main_server.cpp          # Main program (entry point)
│   │
│   ├── trajectory/
│   │   ├── TrajectoryManager.h/.cpp
│   │   └── DataLoader.h/.cpp (optional - can merge with manager)
│   │
│   ├── modbus/
│   │   ├── ModbusServer.h/.cpp
│   │   ├── DataHandler.h/.cpp
│   │   └── RegisterMap.h
│   │
│   ├── serial/
│   │   ├── SerialPort.h/.cpp
│   │   └── ArduinoFeedback.h/.cpp (optional - can integrate)
│   │
│   ├── state_machine/
│   │   ├── StateMachine.h/.cpp
│   │   └── StateHandlers.h/.cpp (optional)
│   │
│   └── config/
│       ├── config.h
│       └── trajectory_paths.h
│
├── CMakeLists.txt               # Build configuration
├── build/                       # Build output
│   └── rehab_server            # Compiled executable
│
└── data/                        # Trajectory data
    ├── trajectory_1/
    ├── trajectory_2/
    └── trajectory_3/
```

---

## 🔧 Module Descriptions

### **1. TrajectoryManager** - Trajectory Data Management
**Files:** `trajectory/TrajectoryManager.h/.cpp`

**Fungsi:**
- Load trajectory data dari file (3 trajectories)
- Manage trajectory points (positions, velocities, forces)
- Provide access ke specific trajectory points
- Support multiple trajectory configurations
- Provide graph data untuk HMI display

**Data Structure:**
```cpp
struct TrajectoryPoint {
    float pos1, pos2, pos3;      // Position reference for 3 motors
    float velo1, velo2, velo3;   // Velocity reference
    float fc1, fc2, fc3;         // Feedforward force
};

struct TrajectoryData {
    int trajectory_id;           // 1, 2, or 3
    int total_points;            // Total points in trajectory
    int gait_start_index;        // Start of main gait cycle
    int gait_end_index;          // End of main gait cycle
    int graph_start_index;       // HMI display start
    int graph_end_index;         // HMI display end
    
    std::vector<TrajectoryPoint> points;
    std::vector<float> graph_x;
    std::vector<float> graph_y;
};
```

**Usage Example:**
```cpp
TrajectoryManager traj_mgr;

// Load all trajectories from data folder
if (!traj_mgr.loadAllTrajectories()) {
    std::cerr << "Error loading trajectories" << std::endl;
}

// Switch to trajectory 1
traj_mgr.switchTrajectory(1);

// Get trajectory data
int total_pts = traj_mgr.getTotalPoints();
int gait_start = traj_mgr.getGaitStartIndex();
int gait_end = traj_mgr.getGaitEndIndex();

// Get specific point
for (int i = gait_start; i < gait_end; i++) {
    TrajectoryPoint pt = traj_mgr.getPoint(i);
    std::cout << "Motor 1: " << pt.pos1 << ", " << pt.velo1 << std::endl;
}
```

**Key Functions:**
```cpp
bool loadAllTrajectories();                           // Load all 3 trajectories
void switchTrajectory(int trajectory_id);            // Switch active trajectory
TrajectoryPoint getPoint(int index);                 // Get single point
int getTotalPoints();                                // Get total point count
int getGaitStartIndex(), getGaitEndIndex();          // Get gait cycle range
float* getGraphDataX(), getGraphDataY();             // Get graph coordinates
void printTrajectoryInfo(int trajectory_id);        // Debug
```

**Data File Format:**
```
data/trajectory_1/
├── grafik.txt        # x,y coordinates (for HMI display)
├── pos1.txt          # Position reference motor 1 (1 per line)
├── pos2.txt
├── pos3.txt
├── velo1.txt         # Velocity reference
├── velo2.txt
├── velo3.txt
├── fc1.txt           # Feedforward force
├── fc2.txt
└── fc3.txt

Example grafik.txt:
0.0,0.0
10.5,5.2
20.3,10.1
```

---

### **2. ModbusServer** - Modbus TCP Communication
**Files:** `modbus/ModbusServer.h/.cpp`

**Fungsi:**
- Modbus TCP server (port 5020)
- Manage register mapping (8000 registers)
- Accept client connections
- Send/receive Modbus messages
- Floating-point data support

**Register Map:**
```cpp
// Control Registers (99-108)
MANUAL_MAJU = 99        // Manual forward button
MANUAL_STOP = 100       // Manual stop button
MANUAL_MUNDUR = 101     // Manual backward button
CALIBRATE = 102         // Calibration button
START = 103             // Start rehab button
EMERGENCY = 104         // Emergency stop button
RESET = 105             // Reset button
TRAJEKTORI_1/2/3 = 106/107/108  // Trajectory selection

// Threshold Registers (130-131)
THRESHOLD_1 = 130       // Load warning threshold
THRESHOLD_2 = 131       // Load retreat threshold

// Cycle Counter (132)
JUMLAH_CYCLE = 132      // Number of cycles

// Graph Registers (120-6000)
COMMAND_REG = 120       // Graph command (1=start, 2=clear, 3=animate)
NUM_OF_DATA_CH0 = 121   // Trajectory point count
NUM_OF_DATA_CH1 = 122   // Animation counter
REALTIME_LOAD_CELL = 126 // Current load value (float)

// Data Registers
X_DATA_CH0_START = 200   // Trajectory X data starts
Y_DATA_CH0_START = 2000  // Trajectory Y data starts
X_DATA_CH1_START = 4000  // Animation X data starts
Y_DATA_CH1_START = 6000  // Animation Y data starts
```

**Usage Example:**
```cpp
ModbusServer modbus;

// Initialize Modbus server
if (!modbus.initialize("0.0.0.0", 5020)) {
    std::cerr << "Failed to init Modbus" << std::endl;
}

// Accept HMI connection
modbus.acceptConnection();

// In main loop
uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
int rc = modbus.receiveQuery(query, sizeof(query));

if (rc > 0) {
    // Process and send reply
    modbus.sendReply(query, rc);
}

// Write/read registers
modbus.writeRegister(ModbusAddr::MANUAL_MAJU, 1);
uint16_t value = modbus.readRegister(ModbusAddr::MANUAL_STOP);

// Write float (load cell)
modbus.writeFloat(ModbusAddr::REALTIME_LOAD_CELL, 25.5);
float load = modbus.readFloat(ModbusAddr::REALTIME_LOAD_CELL);
```

**Key Functions:**
```cpp
bool initialize(const char* ip, int port);         // Start server
void acceptConnection();                           // Accept HMI connection
int receiveQuery(uint8_t* query, int max_size);   // Receive Modbus query
void sendReply(uint8_t* query, int query_size);   // Send reply

uint16_t readRegister(int address);               // Read 16-bit register
void writeRegister(int address, uint16_t value);  // Write 16-bit register
float readFloat(int address);                     // Read 32-bit float
void writeFloat(int address, float value);        // Write 32-bit float

void resetRegisters();                            // Clear all registers
bool isConnected();                               // Check connection
void printStatus();                               // Debug
```

---

### **3. DataHandler** - High-Level Data Operations
**Files:** `modbus/DataHandler.h/.cpp`

**Fungsi:**
- Abstraction layer untuk Modbus & Trajectory operations
- Load trajectory data ke HMI registers
- Update animation data secara real-time
- Manage thresholds & load cell values
- Check button presses (simplified)

**Usage Example:**
```cpp
ModbusServer modbus;
TrajectoryManager traj_mgr;
DataHandler data_handler(&modbus, &traj_mgr);

// Load trajectory ke HMI
data_handler.loadTrajectoryToHMI(1);  // Load trajectory 1

// Update animation point
for (int i = 0; i < num_points; i++) {
    float x = graph_x[i];
    float y = graph_y[i];
    data_handler.updateAnimationPoint(i, x, y);
}

// Set load cell value
data_handler.setRealTimeLoadCell(25.5);  // 25.5 Newton

// Check button press
if (data_handler.isButtonPressed(ModbusAddr::START)) {
    data_handler.clearButton(ModbusAddr::START);
    // Start rehab...
}

// Check trajectory selection
int selected = data_handler.checkTrajectorySelection();  // Returns 1, 2, 3, or 0
```

**Key Functions:**
```cpp
void loadTrajectoryToHMI(int trajectory_id);      // Load trajectory to HMI
void clearAnimationData();                        // Clear animation buffer
void updateAnimationPoint(int idx, float x, float y);  // Update animation

void setRealTimeLoadCell(float load_value);       // Update load cell display
void setCycleCounter(int current, int target);    // Update cycle display

bool isButtonPressed(int button_address);         // Check if button pressed
void clearButton(int button_address);             // Clear button register
int checkTrajectorySelection();                   // Returns selected trajectory

int getThreshold1(), getThreshold2();             // Get current thresholds
void updateThresholds(int t1, int t2);           // Update thresholds
```

---

### **4. SerialPort** - Arduino Communication
**Files:** `serial/SerialPort.h/.cpp`

**Fungsi:**
- Manage serial connection ke Arduino (115200 baud)
- Send trajectory points ke Arduino
- Receive status messages dari Arduino
- Parse Arduino feedback (load, positions, status)

**Usage Example:**
```cpp
io_context io_ctx;
SerialPort serial_port(io_ctx);

// Connect to Arduino
if (!serial_port.open("/dev/ttyACM0", 115200)) {
    std::cerr << "Failed to connect to Arduino" << std::endl;
}

// Send trajectory data
serial_port.sendControlData(
    100.0, 105.0, 98.0,      // positions
    50.0, 50.0, 50.0,        // velocities
    0.5, 0.5, 0.5            // forces
);

// Send thresholds
serial_port.sendThresholds(20, 40);

// Send manual commands
serial_port.sendManualCommand('1');  // Forward
serial_port.sendManualCommand('2');  // Backward
serial_port.sendManualCommand('0');  // Stop

// Receive feedback
if (serial_port.hasData()) {
    std::string msg = serial_port.readLine();
    
    if (msg.find("RETREAT") != std::string::npos) {
        // Handle retreat trigger
    }
    
    // Parse status
    auto status = serial_port.parseStatusMessage(msg);
    std::cout << "Load: " << status.load << " N" << std::endl;
    std::cout << "Pos: " << status.pos1 << ", " << status.pos2 << ", " << status.pos3 << std::endl;
}
```

**Key Functions:**
```cpp
bool open(const std::string& port, unsigned int baud_rate);  // Open port
void close();                                     // Close port
bool isOpen();                                    // Check if open

void sendCommand(const std::string& cmd);        // Send raw command
void sendControlData(float p1, float p2, float p3, ...);  // Send trajectory
void sendThresholds(int t1, int t2);             // Send thresholds
void sendCalibrate();                            // Send calibration
void sendManualCommand(char direction);          // '1', '2', '0'

bool hasData();                                  // Check if data available
std::string readLine();                          // Read until newline
ArduinoStatus parseStatusMessage(const std::string& msg);  // Parse feedback
```

---

### **5. StateMachine** - System State Management
**Files:** `state_machine/StateMachine.h/.cpp`

**Fungsi:**
- Manage system states (IDLE, AUTO_REHAB, etc)
- Track cycles & trajectory index
- Handle state transitions
- Manage timers

**State Diagram:**
```
        ┌─────────────────────────────────┐
        │          IDLE                   │
        │  Wait for user input            │
        └──────────┬──────────────────────┘
                   │ START button
                   ↓
        ┌─────────────────────────────────┐
        │       AUTO_REHAB                │
        │  Execute trajectory points      │
        │  Update animation               │
        └──────────┬──────────────────────┘
                   │ Trajectory complete
                   ↓
        ┌─────────────────────────────────┐
        │    POST_REHAB_DELAY             │
        │  Wait 5 seconds                 │
        └──────────┬──────────────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
        │ More cycles?        │
        ↓                     ↓
     NO: IDLE             YES: AUTO_REHAB
                          (cycle++)

Special States:
- AUTO_RETREAT: Backward motion on high load
- EMERGENCY_STOP: System stopped
- RESETTING: Waiting for reset
```

**Usage Example:**
```cpp
StateMachine state_machine;

// Start rehab with 3 cycles
state_machine.startRehabCycle(3);  // Starts AUTO_REHAB state

// In main loop
switch (state_machine.getState()) {
    case SystemState::IDLE:
        // Handle user input
        break;
        
    case SystemState::AUTO_REHAB:
        // Execute trajectory
        int idx = state_machine.getTrajectoryIndex();
        // Send point to Arduino
        state_machine.incrementTrajectoryIndex();
        break;
        
    case SystemState::POST_REHAB_DELAY:
        if (state_machine.isDelayTimerExpired(5)) {
            if (state_machine.hasCycleRemaining()) {
                state_machine.incrementCycle();
                state_machine.setState(SystemState::AUTO_REHAB);
            } else {
                state_machine.setState(SystemState::IDLE);
            }
        }
        break;
}
```

**Key Functions:**
```cpp
void setState(SystemState new_state);             // Change state
SystemState getState();                           // Get current state
std::string getStateString();                     // Get state name

void startRehabCycle(int target_cycles);          // Start rehab
int getCurrentCycle(), getTargetCycle();          // Cycle tracking
bool hasCycleRemaining();                         // More cycles?
void incrementCycle();                            // Move to next cycle

void setTrajectoryIndex(int index);               // Set trajectory point
void incrementTrajectoryIndex();                  // Move to next point
int getTrajectoryIndex();                         // Current point

void startDelayTimer();                           // Start delay timer
bool isDelayTimerExpired(int seconds);            // Check if expired

void setRetreatTriggered(bool value);             // Retreat flag
bool isRetreatTriggered();                        // Retreat flag check

void printState();                                // Debug output
```

---

## 📝 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(RehabControlServer)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(modbus REQUIRED)
find_package(Boost 1.66.0 REQUIRED COMPONENTS system)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Source files
set(SOURCES
    src/main_server.cpp
    
    src/trajectory/TrajectoryManager.cpp
    
    src/modbus/ModbusServer.cpp
    src/modbus/DataHandler.cpp
    
    src/serial/SerialPort.cpp
    
    src/state_machine/StateMachine.cpp
)

# Create executable
add_executable(rehab_server ${SOURCES})

# Link libraries
target_link_libraries(rehab_server PRIVATE
    modbus
    Boost::system
)

# Compiler flags
if(MSVC)
    target_compile_options(rehab_server PRIVATE /W4)
else()
    target_compile_options(rehab_server PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Install
install(TARGETS rehab_server DESTINATION bin)
```

---

## 🚀 Build & Run

### Build:
```bash
cd server
mkdir -p build
cd build
cmake ..
make -j4
```

### Run:
```bash
./rehab_server
```

### Expected Output:
```
==========================================
  SISTEM KONTROL REHABILITASI
  Multi-Trajectory + Cycle Counter
==========================================

Initializing Modbus Server...
[ModbusServer] Listening on 0.0.0.0:5020

Loading trajectory data...
=== Loading All Trajectory Data ===

[Trajectory 1 - 816 points]
Loaded Trajectory 1: 816 points

[Trajectory 2 - 1370 points]
Loaded Trajectory 2: 1370 points

[Trajectory 3 - 1370 points]
Loaded Trajectory 3: 1370 points

=== All trajectory data loaded successfully ===

*** TRAJECTORY 1 ACTIVE ***
Total points: 816
Gait range: 101 to 715

[SerialPort] Opened /dev/ttyACM0 @ 115200 baud

[DataHandler] Loaded trajectory 1 to HMI: 615 points

=== SYSTEM READY ===
Waiting for HMI commands...
```

---

## 🔄 Data Flow

```
HMI (Modbus TCP)
    │
    ├─ Button Press (START, TRAJECTORY, etc)
    │
    ↓
Modbus Server
    │
    ├─ Parse Button
    ├─ Update Display
    ├─ Read Thresholds
    │
    ↓
DataHandler / TrajectoryManager
    │
    ├─ Load Trajectory
    ├─ Get Point Data
    ├─ Update Animation
    │
    ↓
SerialPort (Arduino)
    │
    ├─ Send Trajectory Point
    ├─ Send Control Data
    │
    ↓
Arduino (Motor Control)
    │
    ├─ Execute Motor Movement
    ├─ Read Sensors
    │
    ↓
SerialPort (Receive Feedback)
    │
    ├─ Parse Status
    ├─ Detect RETREAT
    │
    ↓
Modbus Server (Update Registers)
    │
    └─ Send to HMI Display
```

---

## 📊 Control Loop Timing

```
Main Loop (runs every 1ms):

1. Receive Modbus query          (variable)
2. Process state machine         (varies by state)
   - IDLE: Check buttons
   - AUTO_REHAB: Every 100ms → send trajectory point
   - POST_REHAB_DELAY: Check timer
3. Parse Arduino feedback        (if available)
4. Sleep 1ms
```

---

## ⚙️ Dependencies

### Linux:
```bash
sudo apt-get install -y \
    libmodbus-dev \
    libboost-all-dev \
    cmake \
    build-essential
```

### macOS:
```bash
brew install libmodbus boost cmake
```

### Windows:
- MinGW or Visual Studio
- Download libmodbus from http://libmodbus.org/
- Download Boost from https://www.boost.org/

---

## 🔍 Debugging

### Enable verbose output:
Add to main_server.cpp:
```cpp
std::cout << "[DEBUG] State: " << state_machine.getStateString() << std::endl;
std::cout << "[DEBUG] Index: " << trajectory_index << std::endl;
traj_manager.printTrajectoryInfo(1);
```

### Monitor serial communication:
```bash
cat /dev/ttyACM0 &
```

### Check Modbus connection:
```bash
# Using modbus-cli or similar tool
modbus-cli -h localhost -p 5020
```

---

## ✅ Next Steps

1. Copy all files ke `server/src/`
2. Update `CMakeLists.txt` dengan paths
3. Build dengan `cmake && make`
4. Run dengan `./rehab_server`
5. Connect HMI via Modbus TCP

**Good luck! 🚀**