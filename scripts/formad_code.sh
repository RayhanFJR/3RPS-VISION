#==================================================================
# FILE 8: scripts/format_code.sh
# Format code (optional - requires clang-format)
#==================================================================

#!/bin/bash

echo "Formatting code..."
echo ""

# C++ files
echo "Formatting C++ files..."
find firmware/src -name "*.cpp" -o -name "*.h" | while read file; do
    if command -v clang-format &> /dev/null; then
        clang-format -i "$file"
        echo "  ✓ $file"
    fi
done

find server/src -name "*.cpp" -o -name "*.h" | while read file; do
    if command -v clang-format &> /dev/null; then
        clang-format -i "$file"
        echo "  ✓ $file"
    fi
done

# Python files
echo "Formatting Python files..."
if command -v black &> /dev/null; then
    black vision/src --quiet
    black tests --quiet
    echo "  ✓ Python files formatted"
else
    echo "  (black not installed, skipping)"
fi

echo ""
echo "✓ Code formatting complete"