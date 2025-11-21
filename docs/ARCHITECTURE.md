==========================================
=========File: vision/src/main.py=========
==========================================
PoseEstimator (MediaPipe)
    ├─ Detect 33 body landmarks
    ├─ Extract knee, ankle, foot positions
    └─ Output: (x, y, z) coordinates

AngleCalculator
    ├─ Calculate angle from 3 points
    ├─ Smooth angle with buffer
    └─ Apply calibration offset

PIDController
    ├─ Setpoint: TARGET_ANGLE (90°)
    ├─ Measurement: compensated_angle
    └─ Output: pid_output (gap correction)

Main Loop (30 FPS):
    1. Capture frame from camera
    2. Process pose estimation
    3. Calculate foot angle
    4. Apply PID control
    5. Display on screen
    6. Log data

ANGLE_OFFSET = -35      # Calibration offset
TARGET_ANGLE = 90       # Target angle for PID
PID Gains: Kp=0.5, Ki=0.05, Kd=0.1



===========================================
=======File: firmware/src/main.cpp=========
===========================================

Control Architecture:
Serial Command Input
    ├─ "S..." = Trajectory command (forward)
    ├─ "R..." = Retreat command (backward with scaling)
    ├─ "X" = Calibrate/Reset
    ├─ "E" = Emergency stop
    ├─ "1" = Manual forward
    ├─ "2" = Manual backward
    ├─ "0" = Manual stop
    ├─ "K..." = Set outer loop gains
    ├─ "P..." = Set inner loop gains
    └─ "T..." = Set thresholds

Command Parser
    └─ Extract position, velocity, force references

State Manager
    ├─ operatingMode: 0=Manual, 1=Forward, 2=Retreat
    ├─ manipulatorState: 0=Running, 1=Paused
    └─ retreatHasBeenTriggered: Load-based retreat

Load Cell Monitoring (100ms interval)
    ├─ Read load from HX711
    ├─ Compare with threshold1 & threshold2
    ├─ If load > threshold2: Trigger RETREAT command
    └─ If load > threshold1: Pause (manipulatorState=1)

Encoder Reading (1ms interval)
    └─ Update position for each motor

Velocity Calculation (10s interval)
    └─ velocity = (pos_now - pos_prev) / dt

Control Loop (100ms interval):

    ├─ OUTER LOOP (Trajectory Level):
    │  ├─ Calculate position error: e_pos = ref_pos - actual_pos
    │  ├─ Apply PD: ref_current = Kp*e_pos + Kd*e_velo
    │  └─ Add feedforward: ref_current += ref_force
    │
    ├─ INNER LOOP (Current Level):
    │  ├─ Calculate current error: e_current = ref_current - actual_current
    │  ├─ Apply PD: pwm_value = Kpc*e_current + Kdc*de_current/dt
    │  ├─ Apply load-based scaling: pwm_value *= load_scale
    │  └─ Clamp: pwm_value = [-255, 255]
    │
    └─ MOTOR OUTPUT:
       ├─ Determine direction (forward/backward/stop)
       ├─ Write PWM to RPWM/LPWM pins
       └─ H-bridge drives DC motor

ADAPTIVE CONTROL (Optional):
    ├─ Estimate system parameters (mass, friction, inertia)
    ├─ Update control law in real-time
    ├─ Improve tracking performance
    └─ Reduce overshoot
Load-Based Adaptive Scaling:
Load Monitoring:
    if load < threshold1 (20 N):
        → Normal operation, load_scale = 1.0
    
    if threshold1 <= load <= threshold2 (20-40 N):
        → Ramping, load_scale decreases linearly
        → Reduce Kp damping
    
    if load > threshold2 (40 N):
        → High load, load_scale = 0.15
        → Reduced Kp (damping applied)
        → Motor speed reduced to 15% of max
Motor Parameters:
Each Motor (1, 2, 3):

OUTER LOOP GAINS (Position Control):
    Motor 1: Kp=110.0, Kd=0.1
    Motor 2: Kp=142.0, Kd=0.6
    Motor 3: Kp=150.0, Kd=0.3

INNER LOOP GAINS (Current Control):
    Motor 1: Kpc=30.0, Kdc=0.1
    Motor 2: Kpc=33.0, Kdc=0.1
    Motor 3: Kpc=38.0, Kdc=0.1

HARDWARE:
    Gear Ratio: 0.2786
    Motor Kt: 0.0663
    Position Scale: 0.245 (encoder to mm)
    Manual Speed: 125 PWM
    Retreat Speed: 150 PWM (scaled by 1.5x)
Retreat Mechanism:
Forward Motion → Load Cell detects high load
    │
    ├─ if load > threshold2:
    │  ├─ Set retreatHasBeenTriggered = true
    │  ├─ Send "RETREAT" command to server
    │  └─ Motor stops (manipulatorState = 1)
    │
    ├─ Server receives "RETREAT"
    │  ├─ Switch to AUTO_RETREAT state
    │  └─ Start sending retreat trajectory (R commands)
    │
    └─ Arduino receives "R..." commands:
       ├─ Set operatingMode = 2 (Retreat)
       ├─ Scale velocity by RETREAT_VELOCITY_SCALE (1.5x)
       ├─ Execute backward motion
       └─ When done, send "ACK_RETREAT_COMPLETE"




===========================================
=========File: server/src/main.cpp=========
===========================================

Server Architecture:
Modbus TCP Server (Port 5020)
    ├─ Listen for HMI connections
    ├─ Receive button presses from HMI
    ├─ Send sensor data to HMI
    └─ Update graphs in real-time

State Machine:

    IDLE:
        ├─ Wait for user input
        ├─ Handle manual controls (forward/backward/stop)
        ├─ Allow trajectory selection
        └─ Accept calibration command

    AUTO_REHAB:
        ├─ Execute trajectory points sequentially
        ├─ Update HMI animation in real-time
        ├─ Monitor Arduino feedback
        ├─ When complete → POST_REHAB_DELAY

    POST_REHAB_DELAY (5 seconds):
        ├─ Pause between cycles
        ├─ Check if more cycles remain
        ├─ If yes → back to AUTO_REHAB (cycle++)
        ├─ If no → back to IDLE

    AUTO_RETREAT:
        ├─ Execute retreat sequence backward
        ├─ Decrement trajectory index
        ├─ When complete → back to IDLE

    EMERGENCY_STOP:
        ├─ Stop all motors immediately
        ├─ Disable automatic control
        ├─ Require manual RESET

Trajectory Management:

    3 Trajectories available:
    
    Trajectory 1 (816 points):
        ├─ Data files: data_1/pos1.txt, pos2.txt, pos3.txt
        │             data_1/velo1.txt, velo2.txt, velo3.txt
        │             data_1/fc1.txt, fc2.txt, fc3.txt
        ├─ HMI display: points 101-715 (615 points)
        ├─ Main gait cycle: points 101-715 (615 points)
        └─ Used for rehabilitation therapy
    
    Trajectory 2 (1370 points):
        ├─ Data files: data_2/...
        ├─ HMI display: points 1-1370 (full trajectory)
        ├─ Main gait cycle: points 165-1177 (1012 points)
        └─ Alternative motion pattern
    
    Trajectory 3 (1370 points):
        ├─ Data files: data_3/...
        ├─ HMI display: points 1-1370 (full trajectory)
        ├─ Main gait cycle: points 165-1177 (1012 points)
        └─ Another alternative pattern

Multi-Cycle Support:
    ├─ User selects number of cycles (1-10)
    ├─ After 1 cycle completes → 5 sec delay
    ├─ Automatically start next cycle
    ├─ Counter displays current/total cycle
    └─ After last cycle → back to IDLE

HMI Communication (Modbus Registers):

    CONTROL REGISTERS:
    ├─ 99:  MANUAL_MAJU (forward)
    ├─ 100: MANUAL_STOP
    ├─ 101: MANUAL_MUNDUR (backward)
    ├─ 102: CALIBRATE
    ├─ 103: START (begin rehab)
    ├─ 104: EMERGENCY
    ├─ 105: RESET
    ├─ 106: TRAJEKTORI_1
    ├─ 107: TRAJEKTORI_2
    └─ 108: TRAJEKTORI_3

    THRESHOLD REGISTERS:
    ├─ 130: THRESHOLD_1 (default 20 N)
    └─ 131: THRESHOLD_2 (default 40 N)

    CYCLE COUNTER:
    └─ 132: JUMLAH_CYCLE (number of cycles)

    GRAPH REGISTERS:
    ├─ 120: COMMAND_REG (1=start, 2=clear, 3=animate)
    ├─ 121: NUM_OF_DATA_CH0 (trajectory point count)
    ├─ 122: NUM_OF_DATA_CH1 (animation counter)
    ├─ 126: REALTIME_LOAD_CELL (current load value)
    ├─ 200-2000: X_DATA_CH0 (trajectory X coordinates)
    ├─ 2000-4000: Y_DATA_CH0 (trajectory Y coordinates)
    ├─ 4000-6000: X_DATA_CH1 (animation X points)
    └─ 6000-8000: Y_DATA_CH1 (animation Y points)

Serial Communication with Arduino:

    Send (100ms interval):
        ├─ "S..." = Position, Velocity, Force for each motor
        ├─ "R..." = Retreat trajectory data
        ├─ "K..." = Outer loop gains
        ├─ "P..." = Inner loop gains
        ├─ "T..." = Load thresholds
        └─ "X/E/0/1/2" = Control commands

    Receive:
        ├─ "RETREAT" = High load detected
        ├─ "status:..." = Current state
        ├─ "load:..." = Load cell value
        ├─ "pos:..." = Motor positions
        └─ Frequency: 10 Hz (every 100ms)
Data Flow - HMI Animation:
Trajectory Data Loaded:
├─ Copy trajectory points to HMI registers
├─ HMI displays full trajectory as blue line
└─ NUM_OF_DATA_CH0 = total points

During Execution:
├─ For each trajectory point:
│  ├─ Send point data to Arduino
│  ├─ Copy same point to X_DATA_CH1, Y_DATA_CH1
│  ├─ Increment NUM_OF_DATA_CH1 counter
│  └─ HMI shows green dot following blue line
└─ Real-time animation feedback




==========================================
==========Hardware Integration============
==========================================

Arduino Pin Mapping:
Motor Control:
    Motor 1: RPWM=3, LPWM=5 (PWM pins for H-bridge)
    Motor 2: RPWM=6, LPWM=9
    Motor 3: RPWM=10, LPWM=11

Encoder Feedback:
    ENC1=4, ENC2=2, ENC3=8 (digital pins for quadrature)

Current Sensing:
    CurrSen1=A0, CurrSen2=A1, CurrSen3=A2 (analog pins)

Load Cell:
    DOUT=12, CLK=13 (SPI-like communication)
Sensor Data Processing:
Load Cell (HX711):
    ├─ Read 24-bit ADC value
    ├─ Subtract calibration offset
    ├─ Convert to Newton: load = raw_value / 10000
    ├─ Apply exponential smoothing
    └─ Compare with thresholds

Encoders:
    ├─ Rising edge detection
    ├─ Increment/decrement counter based on motor direction
    ├─ Convert count to position: pos = count * 0.245
    └─ Update every 1ms

Current Sensors:
    ├─ Read analog voltage
    ├─ Average 3 samples
    ├─ Normalize: current = analog_val / 1023
    └─ Used for current loop feedback




==========================================
==========Control Flow Diagram============
==========================================

START
  │
  ├─→ Load All Trajectory Data (3 trajectories)
  │
  └─→ Initialize Modbus Server
       │
       └─→ Main Control Loop:
            │
            ├─ Receive Modbus command from HMI
            │
            ├─ State Machine:
            │  ├─ IDLE → Handle manual input / start rehab
            │  ├─ AUTO_REHAB → Execute trajectory
            │  ├─ AUTO_RETREAT → Backward motion
            │  ├─ POST_REHAB_DELAY → Wait 5 sec
            │  ├─ EMERGENCY_STOP → Stop & wait reset
            │  └─ RESETTING → Prepare for next cycle
            │
            ├─ Every 100ms: Send next trajectory point to Arduino
            │
            ├─ Every 100ms: Update HMI with load cell value
            │
            ├─ Real-time: Receive Arduino feedback
            │   ├─ Parse "RETREAT" command
            │   ├─ Switch to AUTO_RETREAT state
            │   └─ Execute retreat sequence
            │
            └─ Repeat until shutdown