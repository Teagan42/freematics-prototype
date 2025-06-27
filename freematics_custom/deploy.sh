#!/bin/bash

# Freematics Custom Sketch Deployment Script
# Verifies Freematics ONE+ connection and deploys the sketch

set -e

echo "🚀 Freematics Custom Sketch Deployment Script"
echo "=============================================="

# Configuration
SKETCH_DIR="."
SKETCH_NAME="freematics_custom"
BOARD_TYPE="esp32:esp32:esp32"
EXPECTED_DEVICE_NAME="Freematics"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Check if Arduino CLI is installed
check_arduino_cli() {
    if ! command -v arduino-cli &> /dev/null; then
        print_error "Arduino CLI not found. Please install it first:"
        echo "  brew install arduino-cli  # macOS"
        echo "  Or download from: https://arduino.github.io/arduino-cli/"
        exit 1
    fi
    print_status "Arduino CLI found"
}

# Check if ESP32 board package is installed
check_esp32_board() {
    if ! arduino-cli board listall | grep -q "esp32:esp32"; then
        print_warning "ESP32 board package not found. Installing..."
        arduino-cli core update-index
        arduino-cli core install esp32:esp32
    fi
    print_status "ESP32 board package available"
}

# Detect connected Freematics device
detect_freematics() {
    echo "🔍 Scanning for connected devices..."
    
    # Get list of connected boards
    BOARDS=$(arduino-cli board list)
    
    if [ -z "$BOARDS" ]; then
        print_error "No devices detected. Please check:"
        echo "  1. Freematics ONE+ is connected via USB"
        echo "  2. USB cable supports data transfer"
        echo "  3. Device drivers are installed"
        exit 1
    fi
    
    echo "Connected devices:"
    echo "$BOARDS"
    
    # Look for USB serial devices first, then ESP32 or Freematics device
    PORT=$(echo "$BOARDS" | grep -i -E "(usbserial|cp210|ch340)" | head -1 | awk '{print $1}')
    
    # If no USB serial found, look for ESP32 devices (but not Bluetooth)
    if [ -z "$PORT" ]; then
        PORT=$(echo "$BOARDS" | grep -i "esp32" | grep -v -i "bluetooth" | head -1 | awk '{print $1}')
    fi
    
    if [ -z "$PORT" ]; then
        print_warning "Freematics device not automatically detected"
        echo "Available ports:"
        echo "$BOARDS" | tail -n +2
        read -p "Enter the port manually (e.g., /dev/cu.usbserial-*): " PORT
    fi
    
    if [ -z "$PORT" ]; then
        print_error "No port specified. Exiting."
        exit 1
    fi
    
    print_status "Using port: $PORT"
    export ARDUINO_PORT="$PORT"
}

# Verify sketch files exist
check_sketch_files() {
    echo "📁 Checking sketch files..."
    
    REQUIRED_FILES=(
        "${SKETCH_NAME}.ino"
        "config.h"
        "telestore.h"
    )
    
    # Check if we're in the freematics_custom directory or parent directory
    if [ ! -f "${SKETCH_NAME}.ino" ] && [ -f "../${SKETCH_NAME}.ino" ]; then
        cd ..
        echo "Changed to parent directory to find sketch files"
    fi
    
    for file in "${REQUIRED_FILES[@]}"; do
        if [ ! -f "$file" ]; then
            print_error "Required file missing: $file"
            exit 1
        fi
        print_status "Found: $file"
    done
}

# Check for required libraries
check_libraries() {
    echo "📚 Checking libraries..."
    
    # Check if FreematicsPlus library exists
    if ! arduino-cli lib list | grep -q "FreematicsPlus"; then
        print_warning "FreematicsPlus library not found in Arduino libraries"
        print_warning "Make sure you've copied the Freematics libraries as instructed"
        echo "Expected location: ~/Documents/Arduino/libraries/"
    else
        print_status "FreematicsPlus library found"
    fi
}

# Compile the sketch
compile_sketch() {
    echo "🔨 Compiling sketch..."
    
    if arduino-cli compile --fqbn "$BOARD_TYPE" "${SKETCH_NAME}.ino"; then
        print_status "Compilation successful"
    else
        print_error "Compilation failed. Check the error messages above."
        exit 1
    fi
}

# Upload the sketch
upload_sketch() {
    echo "📤 Uploading sketch to device..."
    
    if arduino-cli upload -p "$ARDUINO_PORT" --fqbn "$BOARD_TYPE" "${SKETCH_NAME}.ino"; then
        print_status "Upload successful"
    else
        print_error "Upload failed. Check the error messages above."
        exit 1
    fi
}

# Monitor serial output
monitor_serial() {
    echo "📺 Starting serial monitor..."
    echo "Press Ctrl+C to exit monitoring"
    echo "----------------------------------------"
    
    # Give device time to restart
    sleep 2
    
    if command -v screen &> /dev/null; then
        screen "$ARDUINO_PORT" 115200
    elif command -v arduino-cli &> /dev/null; then
        arduino-cli monitor -p "$ARDUINO_PORT" -c baudrate=115200
    else
        print_warning "No serial monitor available. Install 'screen' or use Arduino IDE"
        echo "Manual command: screen $ARDUINO_PORT 115200"
    fi
}

# Main deployment process
main() {
    echo "Starting deployment process..."
    echo
    
    check_arduino_cli
    check_esp32_board
    check_sketch_files
    check_libraries
    detect_freematics
    compile_sketch
    upload_sketch
    
    echo
    print_status "Deployment completed successfully!"
    echo
    echo "Next steps:"
    echo "1. Monitor serial output to verify initialization"
    echo "2. Connect via Freematics BLE Android app"
    echo "3. Test with vehicle OBD-II port"
    echo
    
    read -p "Start serial monitor now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        monitor_serial
    fi
}

# Run main function
main "$@"
