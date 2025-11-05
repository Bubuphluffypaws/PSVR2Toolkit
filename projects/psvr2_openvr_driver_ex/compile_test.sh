#!/bin/bash

# Compile the eyelid calibration test
# This is a standalone test that doesn't require the full driver build

echo "Compiling eyelid calibration test..."

# Compiler flags
CXX="clang++"
CXXFLAGS="-std=c++17 -Wall -Wextra -O2"
INCLUDES="-I."
OUTPUT="test_eyelid_calibration"

# Source files
SOURCES="test_eyelid_calibration.cpp modern_eyelid_estimator.cpp headset_calibrator.cpp"

# Check if source files exist
for src in $SOURCES; do
    if [ ! -f "$src" ]; then
        echo "Error: Source file $src not found!"
        exit 1
    fi
done

# Compile
$CXX $CXXFLAGS $INCLUDES $SOURCES -o $OUTPUT

if [ $? -eq 0 ]; then
    echo "✓ Compilation successful!"
    echo "Run the test with: ./$OUTPUT"
else
    echo "✗ Compilation failed!"
    exit 1
fi
