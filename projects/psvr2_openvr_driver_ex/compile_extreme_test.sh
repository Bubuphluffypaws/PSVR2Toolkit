#!/bin/bash

# Compile the extreme scenarios test
echo "Compiling extreme scenarios test..."

# Compiler flags
CXX="clang++"
CXXFLAGS="-std=c++17 -Wall -Wextra -O2"
INCLUDES="-I."
OUTPUT="test_extreme_scenarios"

# Source files
SOURCES="test_extreme_scenarios.cpp modern_eyelid_estimator.cpp headset_calibrator.cpp"

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
