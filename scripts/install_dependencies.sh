#!/bin/bash
#==================================================================
# FILE 1: scripts/install_dependencies.sh
# Install all dependencies for the project
#==================================================================

#!/bin/bash

echo "=========================================="
echo "  Installing All Dependencies"
echo "=========================================="
echo ""

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    OS="windows"
else
    OS="unknown"
fi

echo "[Install] Detected OS: $OS"
echo ""

# ============ PYTHON DEPENDENCIES ============
echo "[Install] Installing Python dependencies..."

if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version)
    echo "  Found: $PYTHON_VERSION"
    
    # Create virtual environment (optional but recommended)
    if [ ! -d "venv" ]; then
        echo "  Creating Python virtual environment..."
        python3 -m venv venv
        source venv/bin/activate 2>/dev/null || . venv/Scripts/activate 2>/dev/null
    fi
    
    # Install Python packages
    echo "  Installing Python packages..."
    pip install --upgrade pip
    
    if [ -f "vision/requirements.txt" ]; then
        pip install -r vision/requirements.txt
        echo "  ✓ Vision dependencies installed"
    else
        echo "  Installing MediaPipe, OpenCV, NumPy..."
        pip install opencv-python mediapipe numpy
    fi
    
    # Testing tools
    pip install pytest pytest-cov
    echo "  ✓ Python dependencies installed"
else
    echo "  ✗ Python 3 not found. Please install Python 3.8+"
    exit 1
fi

echo ""

# ============ C++ DEPENDENCIES ============
echo "[Install] Installing C++ dependencies..."

if [ "$OS" == "linux" ]; then
    echo "  Installing for Linux..."
    
    # Update package manager
    sudo apt-get update -qq
    
    # Development tools
    echo "  Installing build tools..."
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        gcc \
        g++ \
        gdb \
        make
    
    # Libraries
    echo "  Installing libraries..."
    sudo apt-get install -y \
        libmodbus-dev \
        libboost-all-dev \
        libssl-dev
    
    echo "  ✓ Linux dependencies installed"

elif [ "$OS" == "macos" ]; then
    echo "  Installing for macOS..."
    
    if ! command -v brew &> /dev/null; then
        echo "  Installing Homebrew..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    fi
    
    echo "  Installing development tools..."
    brew install cmake boost libmodbus openssl
    
    echo "  ✓ macOS dependencies installed"

elif [ "$OS" == "windows" ]; then
    echo "  Installing for Windows..."
    echo "  Please install manually:"
    echo "    1. MinGW or Visual Studio"
    echo "    2. CMake (https://cmake.org/download/)"
    echo "    3. Boost (https://www.boost.org/)"
    echo "    4. libmodbus (http://libmodbus.org/)"
    
else
    echo "  ✗ Unknown OS. Manual installation required."
fi

echo ""

# ============ PLATFORMIO ============
echo "[Install] Installing PlatformIO for Arduino..."

pip install platformio

if command -v pio &> /dev/null; then
    echo "  ✓ PlatformIO installed"
    echo "  Installing Arduino platform..."
    pio platform install atmelavr
else
    echo "  ✗ PlatformIO installation failed"
fi

echo ""

# ============ GIT ============
echo "[Install] Setting up Git..."

if [ ! -d ".git" ]; then
    echo "  Initializing Git repository..."
    git init
    git config user.name "Rehabilitation System"
    git config user.email "rehab@system.local"
    
    echo "  Creating .gitignore..."
    cat > .gitignore << 'EOF'
# Build
build/
bin/
obj/
*.o
*.a
*.so

# Python
__pycache__/
*.py[cod]
*.egg-info/
venv/
.pytest_cache/
.coverage
htmlcov/

# IDE
.vscode/
.idea/
*.swp
*~

# OS
.DS_Store
Thumbs.db

# Data & Logs
logs/
*.log
data/*.csv

# Environment
.env
*.local
EOF
    
    echo "  ✓ Git repository initialized"
fi

echo ""
echo "=========================================="
echo "  ✓ All dependencies installed!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. cd firmware && pio run"
echo "  2. cd server && mkdir build && cd build && cmake .. && make"
echo "  3. cd vision && python src/main.py"
echo ""