#==================================================================
# FILE : vision/src/config/settings.py
# Python configuration file
#==================================================================

"""
Vision System Configuration
Settings untuk foot angle tracking dengan MediaPipe
"""

# ============== CAMERA SETTINGS ==============
CAMERA_ID = 0                    # Camera index (0 = default webcam)
CAMERA_WIDTH = 1280              # Frame width (pixels)
CAMERA_HEIGHT = 720              # Frame height (pixels)
CAMERA_FPS = 30                  # Target FPS

# ============== MEDIAPIPE SETTINGS ==============
MEDIAPIPE_MODEL_COMPLEXITY = 1   # 0=lite, 1=full (slow but accurate)
MIN_DETECTION_CONFIDENCE = 0.5   # Confidence threshold (0-1)
MIN_TRACKING_CONFIDENCE = 0.5    # Tracking confidence threshold

# ============== FOOT ANGLE CALCULATION ==============
# Kalkulasi sudut dari 3 points: knee → ankle → foot_index

# Landmarks yang digunakan:
# LEFT_KNEE = 25, LEFT_ANKLE = 27, LEFT_FOOT_INDEX = 31
# RIGHT_KNEE = 26, RIGHT_ANKLE = 28, RIGHT_FOOT_INDEX = 32

# Default active side (LEFT or RIGHT)
DEFAULT_ACTIVE_SIDE = "LEFT"

# ============== ANGLE COMPENSATION ==============
# Offset untuk kalibrasi (ditentukan saat setup)
ANGLE_OFFSET = -35               # Sudut offset (degrees)
TARGET_ANGLE = 90                # Target angle untuk PID (degrees)

# ============== PID CONTROLLER SETTINGS ==============
PID_PARAMS = {
    'kp': 0.5,      # Proportional gain
    'ki': 0.05,     # Integral gain
    'kd': 0.1       # Derivative gain
}

# PID Setpoint
PID_SETPOINT = 90.0              # Target angle (degrees)

# ============== ANGLE SMOOTHING ==============
# Moving average buffer untuk smooth angle
ANGLE_BUFFER_SIZE = 5            # Number of samples to average
ANGLE_BUFFER_DTYPE = float       # Data type

# ============== DISPLAY SETTINGS ==============
WINDOW_NAME = "Foot Angle Tracker"
DISPLAY_WIDTH = 960              # Display window width
DISPLAY_HEIGHT = 540             # Display window height

# Text overlay settings
TEXT_FONT = "cv2.FONT_HERSHEY_SIMPLEX"
TEXT_FONT_SCALE = 1.0
TEXT_THICKNESS = 2
TEXT_COLOR_ANGLE = (0, 255, 0)   # Green for angle display
TEXT_COLOR_INFO = (0, 255, 255)  # Cyan for info text

# ============== LOGGING SETTINGS ==============
ENABLE_LOGGING = True            # Save data to file
LOG_FILENAME = "foot_angle_log.csv"
LOG_INTERVAL = 100               # Log every 100ms

LOG_DATA_POINTS = [
    'timestamp',
    'side',
    'raw_angle',
    'smoothed_angle',
    'compensated_angle',
    'pid_output',
    'gap'
]

# ============== DEBUG SETTINGS ==============
ENABLE_DEBUG = True              # Print debug messages
DEBUG_LEVEL = 2                  # 0=minimal, 1=normal, 2=verbose
PRINT_INTERVAL = 1000            # Print every 1000ms (1 second)

# ============== KEYBOARD CONTROLS ==============
KEY_LEFT_SIDE = ord('l')         # 'l' = switch to LEFT foot
KEY_RIGHT_SIDE = ord('r')        # 'r' = switch to RIGHT foot
KEY_QUIT = ord('q')              # 'q' = quit application
KEY_PAUSE = ord('p')             # 'p' = pause/resume
KEY_RESET = ord('x')             # 'x' = reset calibration
KEY_SAVE = ord('s')              # 's' = save screenshot

# ============== CALIBRATION SETTINGS ==============
# Run calibration untuk menentukan ANGLE_OFFSET
ENABLE_CALIBRATION_MODE = False  # Set True untuk calibration
CALIBRATION_TARGET = 0.0         # Expected angle at rest (degrees)

# ============== PERFORMANCE SETTINGS ==============
ENABLE_THREADING = True          # Use separate thread untuk video capture
FRAME_BUFFER_SIZE = 2            # Number of frames to buffer
ENABLE_GPU = False               # Use GPU if available (Experimental)

# ============== CONNECTIVITY ==============
# Optional: Connect ke server untuk send data
ENABLE_SERVER_CONNECTION = False
SERVER_HOST = "localhost"
SERVER_PORT = 5000

# ============== FILE PATHS ==============
DATA_DIR = "./data"              # Data output directory
LOG_DIR = "./logs"               # Logs output directory
CONFIG_DIR = "./config"          # Config files directory

# ============== ANKLE VISIBILITY THRESHOLD ==============
# Skip angle calculation jika body part tidak visible enough
MIN_VISIBILITY = 0.5             # Minimum visibility confidence (0-1)

# ============== ANGLE BOUNDS ==============
# Limit angle ke range yang masuk akal
MIN_ANGLE = 0.0                  # Minimum angle (degrees)
MAX_ANGLE = 180.0                # Maximum angle (degrees)

# ============== STATISTICS ==============
# Track performa sistem
TRACK_FPS = True                 # Monitor actual FPS
TRACK_INFERENCE_TIME = True      # Monitor pose detection time
TRACK_ERRORS = True              # Count detection errors

print("Vision Configuration Loaded Successfully")
print(f"Camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS} FPS")
print(f"PID Gains: Kp={PID_PARAMS['kp']}, Ki={PID_PARAMS['ki']}, Kd={PID_PARAMS['kd']}")
print(f"Target Angle: {TARGET_ANGLE}°")
print(f"Angle Offset: {ANGLE_OFFSET}°")