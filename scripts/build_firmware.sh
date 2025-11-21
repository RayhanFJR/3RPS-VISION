#==================================================================
# FILE 2: scripts/build_firmware.sh
# Build and upload Arduino firmware
#==================================================================

#!/bin/bash

echo "=========================================="
echo "  Building Firmware"
echo "=========================================="
echo ""

cd firmware || exit 1

# Clean
echo "[Build] Cleaning previous builds..."
pio run -t clean > /dev/null 2>&1

# Verify (compile)
echo "[Build] Compiling firmware..."
if pio run; then
    echo "✓ Compilation successful"
else
    echo "✗ Compilation failed"
    exit 1
fi

echo ""

# Upload
echo "[Build] Detecting Arduino board..."
PORTS=$(pio device list 2>/dev/null | grep -i "COM\|ttyACM\|ttyUSB")

if [ -z "$PORTS" ]; then
    echo "✗ No Arduino detected. Please connect via USB."
    exit 1
fi

echo "Available ports:"
echo "$PORTS"
echo ""

read -p "Proceed with upload? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "[Build] Uploading firmware..."
    if pio run -t upload; then
        echo "✓ Upload successful"
        echo ""
        echo "[Build] Starting serial monitor..."
        echo "Press Ctrl+C to exit"
        sleep 2
        pio device monitor
    else
        echo "✗ Upload failed"
        exit 1
    fi
fi

cd ..