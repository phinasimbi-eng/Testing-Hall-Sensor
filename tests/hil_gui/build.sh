#!/bin/bash
# Build script for Motor Control Virtual Test Simulator

cd "$(dirname "$0")"

echo "🔧 Building Motor Control Virtual Test Simulator..."
echo ""

# Set Qt path
export PATH="/opt/homebrew/opt/qt@6/bin:$PATH"

# Create build directory
mkdir -p build
cd build

# Run cmake
echo "📝 Configuring with CMake..."
cmake .. || {
    echo "❌ CMake configuration failed!"
    exit 1
}

# Build
echo "🔨 Compiling..."
make -j$(sysctl -n hw.ncpu) || {
    echo "❌ Build failed!"
    exit 1
}

echo ""
echo "✅ Build successful!"
echo "📦 Executable: $(pwd)/MotorTestGUI"
echo ""
echo "To run the simulator:"
echo "  cd build"
echo "  ./MotorTestGUI"
echo ""
