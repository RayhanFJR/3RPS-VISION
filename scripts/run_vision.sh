#==================================================================
# FILE 4: scripts/run_vision.sh
# Run Vision system
#==================================================================

#!/bin/bash

echo "=========================================="
echo "  Starting Vision System"
echo "=========================================="
echo ""

cd vision || exit 1

# Check dependencies
echo "[Vision] Checking dependencies..."

python3 -c "import cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "✗ Missing: OpenCV"
    echo "  Install with: pip install opencv-python"
    exit 1
fi

python3 -c "import mediapipe" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "✗ Missing: MediaPipe"
    echo "  Install with: pip install mediapipe"
    exit 1
fi

python3 -c "import numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "✗ Missing: NumPy"
    echo "  Install with: pip install numpy"
    exit 1
fi

echo "✓ All dependencies found"
echo ""

# Create directories if not exist
mkdir -p logs config

# Check camera
echo "[Vision] Checking camera..."
python3 << 'PYTHON_EOF'
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    print("✓ Camera detected")
    cap.release()
else:
    print("✗ Camera not found")
    exit(1)
PYTHON_EOF

echo ""
echo "[Vision] Starting vision system..."
echo "Controls: l=LEFT, r=RIGHT, c=CALIBRATE, q=QUIT"
echo ""

python3 src/main.py

cd ..