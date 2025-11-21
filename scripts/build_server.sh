#==================================================================
# FILE 3: scripts/build_server.sh
# Build C++ server
#==================================================================

#!/bin/bash

echo "=========================================="
echo "  Building Server"
echo "=========================================="
echo ""

cd server || exit 1

# Create build directory
if [ ! -d "build" ]; then
    echo "[Build] Creating build directory..."
    mkdir -p build
fi

cd build || exit 1

# Configure
echo "[Build] Running CMake..."
if ! cmake ..; then
    echo "✗ CMake configuration failed"
    exit 1
fi

echo "[Build] ✓ CMake configured"
echo ""

# Build
echo "[Build] Compiling server..."
if ! make -j4; then
    echo "✗ Build failed"
    exit 1
fi

echo "✓ Build successful"
echo ""

# Check executable
if [ -f "rehab_server" ]; then
    echo "✓ Executable ready: $(pwd)/rehab_server"
    echo ""
    echo "To run the server:"
    echo "  cd build"
    echo "  ./rehab_server"
else
    echo "✗ Executable not found"
    exit 1
fi

cd ../..
