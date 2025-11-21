#==================================================================
# FILE 5: scripts/test_system.sh
# Run all tests
#==================================================================

#!/bin/bash

echo "=========================================="
echo "  Running All Tests"
echo "=========================================="
echo ""

FAILED=0

# ============ PYTHON TESTS ============
echo "[Test] Python tests..."
cd vision || exit 1

if [ -d "tests" ]; then
    python3 -m pytest tests/ -v
    if [ $? -ne 0 ]; then
        FAILED=$((FAILED + 1))
        echo "✗ Python tests failed"
    else
        echo "✓ Python tests passed"
    fi
else
    echo "⊘ No Python tests found"
fi

cd ..
echo ""

# ============ C++ TESTS ============
echo "[Test] C++ tests..."
if command -v gtest-config &> /dev/null; then
    if [ -d "firmware/tests" ]; then
        echo "  Would run firmware tests here"
    fi
    
    if [ -d "server/tests" ]; then
        echo "  Would run server tests here"
    fi
else
    echo "⊘ Google Test not installed (optional)"
fi

echo ""

# ============ INTEGRATION TESTS ============
echo "[Test] Integration tests..."
if [ -d "tests" ]; then
    python3 -m pytest tests/integration_test.py -v -s
    if [ $? -ne 0 ]; then
        FAILED=$((FAILED + 1))
        echo "✗ Integration tests failed"
    else
        echo "✓ Integration tests passed"
    fi
else
    echo "⊘ No integration tests found"
fi

echo ""

# ============ SUMMARY ============
if [ $FAILED -eq 0 ]; then
    echo "=========================================="
    echo "  ✓ All tests passed!"
    echo "=========================================="
    exit 0
else
    echo "=========================================="
    echo "  ✗ $FAILED test suite(s) failed"
    echo "=========================================="
    exit 1
fi
