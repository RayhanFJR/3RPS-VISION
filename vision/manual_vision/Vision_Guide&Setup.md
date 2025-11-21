# Vision System - Complete Guide

## 📁 Folder Structure

```
vision/
├── requirements.txt              # Python dependencies
├── src/
│   ├── main.py                   # Main program (entry point)
│   │
│   ├── vision/                   # Vision modules
│   │   ├── __init__.py
│   │   ├── PoseEstimator.py      # MediaPipe wrapper
│   │   ├── AngleCalculator.py    # Geometric calculations
│   │   └── FrameProcessor.py     # Camera & display
│   │
│   ├── control/                  # Control modules
│   │   ├── __init__.py
│   │   └── PIDController.py      # PID controller
│   │
│   └── utils/                    # Utility modules
│       ├── __init__.py
│       ├── Calibration.py        # Calibration management
│       ├── DataBuffer.py         # Data smoothing
│       ├── Logger.py             # CSV logging
│       └── ConfigLoader.py       # Config management
│
├── config/                       # Configuration files
│   ├── settings.json             # Main settings
│   └── calibration.json          # Calibration data (auto-generated)
│
├── logs/                         # Output logs (auto-generated)
│   └── foot_angle_YYYYMMDD_HHMMSS.csv
│
└── README.md
```

---

## 🚀 Quick Start (5 minutes)

### 1. Install Dependencies

```bash
cd vision
pip install -r requirements.txt
```

**File: `requirements.txt`**
```
opencv-python==4.8.0.76
mediapipe==0.10.0
numpy==1.24.0
```

### 2. Run Program

```bash
python src/main.py
```

### 3. Controls

```
l = Switch to LEFT foot
r = Switch to RIGHT foot
c = Start calibration (collect 30 samples)
q = Quit
```

---

## 🔧 Module Descriptions

### **1. PoseEstimator** - MediaPipe Integration
**File:** `vision/PoseEstimator.py`

**Fungsi:**
- Detect body landmarks using MediaPipe
- Extract foot/ankle/knee positions
- Draw landmarks on frame
- Provide detection statistics

**Usage:**
```python
from vision.PoseEstimator import PoseEstimator

estimator = PoseEstimator(model_complexity=1)

# Process frame
landmarks = estimator.process_frame(cv2_frame)

if landmarks and landmarks.is_valid():
    print(f"Left ankle: {landmarks.left_ankle}")
    
    # Draw on frame
    frame = estimator.draw_landmarks(frame, landmarks)

# Get statistics
stats = estimator.get_statistics()
print(f"Detection rate: {stats['detection_rate']:.1f}%")
```

**Key Classes:**
```python
Point3D:
    - x, y, z: Normalized coordinates
    - visibility: Confidence (0-1)
    - to_2d(width, height): Convert to pixel coordinates

BodyLandmarks:
    - left_knee, left_ankle, left_foot
    - right_knee, right_ankle, right_foot
    - is_valid(min_visibility): Check if all visible

PoseEstimator:
    - process_frame(frame): Detect landmarks
    - draw_landmarks(frame, landmarks): Draw on frame
    - get_statistics(): Get detection rate
```

---

### **2. AngleCalculator** - Geometric Calculations
**File:** `vision/AngleCalculator.py`

**Fungsi:**
- Calculate angle from 3 points (knee → ankle → foot)
- Validate angle calculations
- Apply calibration offset
- Constrain to valid range

**Usage:**
```python
from vision.AngleCalculator import AngleCalculator

# From 3 pixel coordinates
result = AngleCalculator.calculate_angle(
    point_a=(100, 200),   # Knee
    point_b=(150, 300),   # Ankle (vertex)
    point_c=(200, 250)    # Foot
)

if result.is_valid:
    print(f"Angle: {result.angle_degrees:.1f}°")
else:
    print(f"Error: {result.error_message}")

# Or from landmarks directly
result = AngleCalculator.calculate_angle_from_landmarks(landmarks)

# Apply calibration
calibrated = AngleCalculator.apply_calibration_offset(
    angle=angle_raw,
    offset=-35.0  # Calibration offset
)

# Constrain to valid range
constrained = AngleCalculator.constrain_angle(
    angle=angle,
    min_angle=0.0,
    max_angle=180.0
)
```

---

### **3. FrameProcessor** - Camera & Display
**File:** `vision/FrameProcessor.py`

**Fungsi:**
- Capture frames from camera
- Handle frame properties
- Display with OpenCV
- Add text overlays

**Usage:**
```python
from vision.FrameProcessor import FrameProcessor

proc = FrameProcessor(
    camera_id=0,
    frame_width=1280,
    frame_height=720,
    fps=30
)

# Read frame
ret, frame = proc.read_frame()

if ret:
    # Add text
    frame = proc.add_text(
        frame, "Angle: 90.0°",
        position=(10, 50),
        font_scale=1.5,
        color=(0, 255, 0)
    )
    
    # Display
    key = proc.display_frame(frame, "Window", wait_ms=1)
    
    if key == ord('q'):
        break

# Cleanup
proc.close()
```

---

### **4. PIDController** - Control Logic
**File:** `control/PIDController.py`

**Fungsi:**
- Implement PID controller
- Compute control output
- Handle gains

**Usage:**
```python
from control.PIDController import PIDController

pid = PIDController(kp=0.5, ki=0.05, kd=0.1)

# In control loop
setpoint = 90.0  # Target angle
measurement = 85.5  # Current angle

output = pid.compute(setpoint, measurement, dt=0.033)

print(f"Control signal: {output:.2f}")

# Update gains
pid.set_gains(kp=0.6, ki=0.1, kd=0.15)

# Reset state
pid.reset()
```

---

### **5. CalibrationManager** - Calibration
**File:** `utils/Calibration.py`

**Fungsi:**
- Manage calibration data
- Calculate offset from reference
- Save/load calibration
- Apply calibration to measurements

**Usage:**
```python
from utils.Calibration import CalibrationManager

calib = CalibrationManager()

# Method 1: Single reference
offset = calib.calibrate_from_reference(
    measured_angle=87.5,      # What we measure
    reference_angle=90.0      # What it should be
)

# Method 2: From multiple samples
offset = calib.calibrate_from_samples(
    samples=[87.2, 87.4, 87.6, 87.5],
    reference_angle=90.0
)

# Apply calibration
compensated = calib.apply_calibration(raw_angle)

# Get info
info = calib.get_calibration_info()
print(f"Offset: {info['offset']}")
print(f"Target: {info['target']}")

# Save
calib.save_calibration()
```

---

### **6. DataBuffer** - Smoothing
**File:** `utils/DataBuffer.py`

**Fungsi:**
- Circular buffer for smoothing
- Calculate average/median
- Detect outliers
- Apply exponential smoothing

**Usage:**
```python
from utils.DataBuffer import DataBuffer

buffer = DataBuffer(max_size=5)

# Add samples
for angle in [89.5, 90.1, 89.8, 90.2, 89.9]:
    buffer.add(angle)

# Get smoothed value
avg = buffer.get_average()          # 89.95
median = buffer.get_median()        # 90.1
std = buffer.get_std_dev()          # ~0.26

# Exponential smoothing
smoothed = buffer.exponential_smooth(alpha=0.3)

# Remove outliers
cleaned_avg = buffer.remove_outliers(threshold=2.0)

# Range
min_val, max_val = buffer.get_range()

# Check status
if buffer.is_full():
    print("Buffer full")
```

---

### **7. DataLogger** - CSV Logging
**File:** `utils/Logger.py`

**Fungsi:**
- Log data to CSV
- Auto-create log files
- Track timestamps

**Usage:**
```python
from utils.Logger import DataLogger

logger = DataLogger(log_dir="./logs", enabled=True)

# Log data
logger.log_data(
    frame_num=150,
    raw_angle=87.5,
    smoothed_angle=90.1,
    compensated_angle=90.0,
    pid_output=0.05,
    side="LEFT",
    confidence=0.95
)

# Get log file path
log_file = logger.get_log_file()
print(f"Logging to: {log_file}")

# Disable logging
logger.disable()
```

**Output CSV Format:**
```
timestamp,frame_num,raw_angle,smoothed_angle,compensated_angle,pid_output,side,detection_confidence
2024-01-15T10:30:45.123456,1,87.50,90.10,90.00,0.05,LEFT,0.950
2024-01-15T10:30:45.156789,2,87.52,90.15,90.05,0.06,LEFT,0.951
```

---

### **8. ConfigLoader** - Configuration
**File:** `utils/ConfigLoader.py`

**Fungsi:**
- Load configuration from JSON
- Get/set config values
- Provide defaults

**Usage:**
```python
from utils.ConfigLoader import ConfigLoader

config = ConfigLoader("./config/settings.json")

# Get values (dot notation)
fps = config.get("camera.fps")          # 30
kp = config.get("pid.kp")               # 0.5

# Get whole sections
camera_config = config.get_camera_config()
pid_gains = config.get_pid_gains()

# Set values
config.set_value("pid.kp", 0.6)
config.set_value("camera.fps", 60)

# Save
config.save()

# Print
config.print_config()
```

**Config File Format (`settings.json`):**
```json
{
  "camera": {
    "id": 0,
    "width": 1280,
    "height": 720,
    "fps": 30
  },
  "mediapipe": {
    "model_complexity": 1,
    "min_detection_confidence": 0.5
  },
  "angle": {
    "offset": -35.0,
    "target": 90.0
  },
  "pid": {
    "kp": 0.5,
    "ki": 0.05,
    "kd": 0.1
  }
}
```

---

## 📊 Data Flow

```
Camera Input (webcam)
    ↓
FrameProcessor (capture & resize)
    ↓
PoseEstimator (MediaPipe detection)
    ↓
BodyLandmarks (knee, ankle, foot points)
    ↓
AngleCalculator (compute foot angle)
    ↓
DataBuffer (smooth angle)
    ↓
CalibrationManager (apply offset)
    ↓
PIDController (compute control signal)
    ↓
DataLogger (save to CSV)
    ↓
Display (draw on frame & show)
```

---

## 🎯 Calibration Procedure

### Manual Calibration

```bash
1. Run program:
   python src/main.py

2. Position foot at 90° (standing naturally)

3. Press 'c' to start calibration

4. Hold position for ~30 frames (~1 second)

5. System auto-calculates offset

6. Calibration data saved to config/calibration.json
```

### Programmatic Calibration

```python
calib = CalibrationManager()

# Get 30 samples while foot at 90°
samples = []
for _ in range(30):
    # Get angle from vision system
    angle = angle_buffer.get_average()
    samples.append(angle)

# Calibrate
calib.calibrate_from_samples(samples, reference_angle=90.0)

# Result saved automatically
```

---

## 🔄 Typical Workflow

```python
import sys
from vision.PoseEstimator import PoseEstimator
from vision.AngleCalculator import AngleCalculator
from vision.FrameProcessor import FrameProcessor
from control.PIDController import PIDController
from utils.Calibration import CalibrationManager
from utils.DataBuffer import DataBuffer

# Initialize
pose = PoseEstimator()
frame_proc = FrameProcessor()
angle_calc = AngleCalculator()
pid = PIDController(kp=0.5, ki=0.05, kd=0.1)
calib = CalibrationManager()
buffer = DataBuffer(max_size=5)

# Main loop
while True:
    # 1. Read frame
    ret, frame = frame_proc.read_frame()
    if not ret:
        break
    
    # 2. Detect pose
    landmarks = pose.process_frame(frame)
    if not landmarks:
        continue
    
    # 3. Calculate angle
    angle_result = angle_calc.calculate_angle_from_landmarks(landmarks)
    if not angle_result.is_valid:
        continue
    
    # 4. Smooth angle
    buffer.add(angle_result.angle_degrees)
    smoothed = buffer.get_average()
    
    # 5. Apply calibration
    compensated = calib.apply_calibration(smoothed)
    
    # 6. PID control
    pid_out = pid.compute(setpoint=90.0, measurement=compensated)
    
    # 7. Display
    frame = frame_proc.add_text(frame, f"Angle: {compensated:.1f}°", (10, 50))
    frame = pose.draw_landmarks(frame, landmarks)
    key = frame_proc.display_frame(frame)
    
    if key == ord('q'):
        break

# Cleanup
frame_proc.close()
```

---

## ⚙️ Configuration Examples

### High Performance (Fast Laptop)
```json
{
  "camera": {"width": 1920, "height": 1080, "fps": 60},
  "mediapipe": {"model_complexity": 1},
  "processing": {"angle_buffer_size": 3}
}
```

### Low Latency (Real-time Control)
```json
{
  "camera": {"width": 640, "height": 480, "fps": 30},
  "mediapipe": {"model_complexity": 0},
  "processing": {"angle_buffer_size": 2}
}
```

### Accurate (Lab/Testing)
```json
{
  "camera": {"width": 2560, "height": 1440, "fps": 30},
  "mediapipe": {"model_complexity": 1},
  "processing": {"angle_buffer_size": 10}
}
```

---

## 🐛 Troubleshooting

### Camera Not Working
```python
# Check available cameras
import cv2
for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera {i} available")
        cap.release()

# Update config
config.set_value("camera.id", <working_id>)
```

### Low Detection Rate
- Better lighting
- Adjust `model_complexity` (1 = more accurate)
- Adjust `min_detection_confidence` lower
- Wear contrasting clothing

### Angle Jumping
- Increase `angle_buffer_size`
- Use exponential smoothing instead
- Check lighting stability

### Slow Performance
- Reduce resolution
- Reduce FPS
- Use `model_complexity: 0`
- Disable logging

---

## ✅ Checklist

- [ ] Python 3.8+
- [ ] Dependencies installed (`pip install -r requirements.txt`)
- [ ] Camera working (`camera.id` correct)
- [ ] Pose detected (press 'p' for stats)
- [ ] Angle calculated (watch foot angle in console)
- [ ] Calibration done (press 'c')
- [ ] Angle smooth (wait ~5 frames for buffer to fill)
- [ ] PID working (check output values)

---

## 🎓 Next Steps

1. **Integrate with Server:**
   ```python
   # Connect to server via network
   # Send calculated angle
   # Receive PID setpoint
   ```

2. **Advanced Filtering:**
   ```python
   # Kalman filter for smoother results
   # Multi-point smoothing
   # Outlier rejection
   ```

3. **Multiple People:**
   ```python
   # Track multiple poses
   # Multiple angle calculations
   # Selective display
   ```

---

## 📞 Getting Help

Check logs: `./logs/foot_angle_*.csv`
Check config: `./config/calibration.json`
Enable debug: Set `DEBUG=1` in code

**Happy Tracking! 🎬**