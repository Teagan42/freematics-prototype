# Freematics Custom Sketch - LLM Documentation

## Project Overview

This Arduino sketch creates a simplified vehicle telemetry system for ESP32 devices. It collects OBD-II data, GPS location, and hardware sensor readings, transmitting them via Bluetooth Low Energy (BLE) for real-time monitoring through a web dashboard or mobile app.

## Purpose

- **Vehicle Data Collection**: Monitor engine RPM, speed, coolant temperature via OBD-II
- **Hardware Sensors**: Read battery voltage, ambient temperature from built-in sensors
- **Location Tracking**: Capture GPS coordinates with satellite averaging
- **Simulation Mode**: Generate test data for development without vehicle connection
- **Real-time Monitoring**: Stream data via BLE to web dashboard or Android app
- **Development Support**: Comprehensive debugging and command protocol

## Hardware Requirements

- Freematics ONE+ (ESP32-based) or compatible ESP32 board
- OBD-II port access in target vehicle (optional - simulation available)
- Built-in GPS and sensors (external modules supported)

## Deployment

### Firmware Deployment
Use the provided deployment script for streamlined setup:

```bash
chmod +x deploy.sh
./deploy.sh
```

The script automatically:
- Installs Arduino CLI and ESP32 board package if needed
- Detects connected Freematics ONE+ device
- Verifies sketch files and dependencies
- Compiles and uploads the sketch
- Starts serial monitoring

### Web Dashboard Deployment
Deploy the monitoring interface to a web server:

```bash
chmod +x deploy-web.sh
./deploy-web.sh -u username server.com
```

### Manual Deployment
1. Install Arduino CLI and ESP32 board package
2. Connect Freematics ONE+ via USB
3. Compile: `arduino-cli compile --fqbn esp32:esp32:esp32 freematics_custom.ino`
4. Upload: `arduino-cli upload -p /dev/cu.usbserial-* --fqbn esp32:esp32:esp32 freematics_custom.ino`

## Testing Strategy

### 1. Bench Testing
- Run deployment script or connect manually via USB
- Monitor Serial output at 115200 baud using `screen /dev/cu.usbserial-* 115200`
- Verify initialization of OBD, GPS, storage, and BLE components
- Test client-side simulation mode without vehicle connection
- Verify hardware sensor readings (battery voltage, ambient temperature)

### 2. Web Dashboard Testing
- Open `dashboard.html` in Chrome/Edge browser (HTTPS required)
- Test BLE connection and pairing with proper error handling
- Verify real-time data display and charting with null safety
- Test client-side simulation controls (start/stop simulation)
- Check command protocol (DIAGNOSTIC, STATUS, PING) with connection validation
- Verify chart updates handle undefined data gracefully
- Test diagnostics button functionality and error reporting

### 3. Vehicle Testing
- Connect OBD-II cable to vehicle diagnostic port
- Start engine and drive to generate real data
- Monitor transition from simulation to real OBD data
- Verify data accuracy against vehicle dashboard
- Test graceful fallback when OBD connection lost

### 4. BLE Protocol Testing
- Use web dashboard or Freematics BLE Android application
- Connect to device named "FreematicsCustom"
- Test command protocol (DIAGNOSTIC, STATUS, PING) with proper error handling
- Verify API version compatibility checking
- Monitor message counter and data formatting
- Test connection state validation before sending commands

## BLE Data Protocol

### Connection Details
- **Device Name**: "FreematicsCustom"
- **Service UUID**: "12345678-1234-1234-1234-123456789abc"
- **Transmission Interval**: 1000ms (configurable in config.h)
- **API Version**: 1 (reported on connection)

### Data Format
BLE messages include message counter and follow this format:
```
messageId:timestamp,RPM:value;SPD:value;GPS:lat,lng;SAT:count;MODE:status;
```

### Example BLE Messages
```
1:0:12345,CONNECT:SUCCESS,API_VERSION:1;
2:12345,MODE:REAL;RPM:1500;SPD:60;GPS:37.774900,-122.419400;SAT:7;BATTERY:1250;
3:13345,STATUS:OBD=REAL,GPS=OK,STORAGE=OK,BLE=OK,API_VERSION=1,UPTIME=13;
4:14345,HEARTBEAT:OK;
5:15345,DIAGNOSTIC_STARTED:RUNNING;
6:16345,DIAGNOSTIC_RESULTS:ADC Test: PASS|OBD Interface: PASS|BLE Stack: PASS;
7:17345,DIAGNOSTIC_COMPLETE:SUCCESS;
```

### Message Types
- **Data Messages**: Sensor readings with MODE indicator (REAL/DISABLED)
- **Status Messages**: System component status with averaging
- **Heartbeat Messages**: Keep-alive signals every 2 seconds
- **Command Responses**: PONG, status confirmations, diagnostic results
- **Diagnostic Messages**: Multi-stage diagnostic process with detailed results
- **Error Messages**: Connection failures and command errors with proper logging

### Data Codes

#### Engine Management PIDs (0x04-0x11)
- **ENGINE_LOAD**: Calculated engine load percentage (PID 0x04)
- **COOLANT_TEMP**: Engine coolant temperature in °C (PID 0x05)
- **FUEL_TRIM_1/2**: Short/long term fuel trim percentages (PIDs 0x06-0x09)
- **FUEL_PRESSURE**: Fuel system pressure in kPa (PID 0x0A)
- **INTAKE_MAP**: Intake manifold absolute pressure (PID 0x0B)
- **RPM**: Engine revolutions per minute (PID 0x0C)
- **SPEED**: Vehicle speed in km/h (PID 0x0D)
- **TIMING_ADVANCE**: Ignition timing advance in degrees (PID 0x0E)
- **INTAKE_TEMP**: Intake air temperature in °C (PID 0x0F)
- **MAF_FLOW**: Mass air flow rate in g/s (PID 0x10)
- **THROTTLE_POS**: Throttle position percentage (PID 0x11)

#### Emissions Control PIDs (0x14-0x3F)
- **O2_S1-S8_VOLTAGE**: Oxygen sensor voltages (PIDs 0x14-0x1B)
- **OBD_STANDARDS**: OBD compliance standard (PID 0x1C)
- **O2_SENSORS_PRESENT**: Oxygen sensor configuration (PID 0x1D)
- **RUNTIME**: Engine runtime since start (PID 0x1F)
- **FUEL_RAIL_PRESSURE**: Fuel rail pressure (PID 0x22)
- **COMMANDED_EGR**: EGR valve position command (PID 0x2C)
- **EGR_ERROR**: EGR system error percentage (PID 0x2D)
- **COMMANDED_EVAP_PURGE**: Evaporative purge control (PID 0x2E)
- **FUEL_TANK_LEVEL**: Fuel tank level percentage (PID 0x2F)
- **CATALYST_TEMP_B1S1/B2S1**: Catalyst temperatures (PIDs 0x3C-0x3F)

#### Advanced Diagnostics PIDs (0x42-0x5F)
- **CONTROL_MODULE_VOLTAGE**: ECU supply voltage (PID 0x42)
- **ABSOLUTE_LOAD_VALUE**: Absolute load value (PID 0x43)
- **FUEL_AIR_EQUIV_RATIO**: Commanded equivalence ratio (PID 0x44)
- **RELATIVE_THROTTLE_POS**: Relative throttle position (PID 0x45)
- **AMBIENT_AIR_TEMP**: Ambient air temperature (PID 0x46)
- **ACCELERATOR_PEDAL_POS**: Accelerator pedal positions (PIDs 0x49-0x4B)
- **COMMANDED_THROTTLE_ACTUATOR**: Throttle actuator control (PID 0x4C)
- **FUEL_TYPE**: Fuel type identifier (PID 0x51)
- **ETHANOL_FUEL_PERCENT**: Ethanol fuel percentage (PID 0x52)
- **ENGINE_OIL_TEMP**: Engine oil temperature (PID 0x5C)
- **FUEL_INJECTION_TIMING**: Fuel injection timing (PID 0x5D)
- **ENGINE_FUEL_RATE**: Engine fuel consumption rate (PID 0x5E)

#### Turbo/Boost Control PIDs (0x70-0x7F)
- **BOOST_PRESSURE_CONTROL**: Boost pressure control (PID 0x70)
- **TURBOCHARGER_RPM**: Turbocharger speed (PID 0x74)
- **EXHAUST_GAS_TEMP**: Exhaust gas temperatures (PIDs 0x78-0x79)
- **ENGINE_RUN_TIME**: Total engine runtime (PID 0x7F)

#### Hardware Sensor Parameters
- **BATTERY**: Vehicle battery voltage in centivolt (PID 0x42)
- **AMBIENT**: Ambient temperature in °C, converted to °F (PID 0x46)
- **PRESSURE**: Engine oil pressure in psi (PID 0x43)
- **BAROMETRIC**: Atmospheric pressure in mbar (PID 0x33)

#### GPS Data
- **GPS**: Latitude,Longitude in decimal degrees (6 decimal places)
- **SAT**: Number of satellites with averaging indicator (PID 0x22)

#### System Status
- **MODE**: Data source (REAL/DISABLED - simulation moved to client-side)
- **API_VERSION**: Protocol version for compatibility
- **STATUS**: Component health (OBD/GPS/STORAGE/BLE)
- **DIAG**: Hardware diagnostic results (7 tests: ADC,OBD,BLE,VIN,HEAP,CLIENT,SERVER)

### Supported PID Count
- **Total PIDs**: 80+ EPA-compliant parameters
- **Engine Management**: 14 core engine parameters
- **Emissions Control**: 25+ emissions and fuel system parameters
- **Advanced Diagnostics**: 20+ diagnostic and performance parameters
- **Turbo/Boost**: 16 turbocharger and exhaust parameters
- **Hardware Sensors**: 4 direct hardware readings

### Data Collection Intervals (Configurable in config.h)
- **OBD**: 1000ms
- **GPS**: 2000ms
- **BLE Transmission**: 1000ms
- **Status Updates**: 5000ms
- **Heartbeat**: 2000ms

## Command Protocol

### Supported Commands
Send via BLE characteristic write:
```
CMD:STATUS     - Request immediate status update
CMD:PING       - Test connection (responds with PONG)
CMD:DIAGNOSTIC - Run comprehensive hardware diagnostics
```

### Command Responses
```
PONG:OK                           - Response to PING command
STATUS:OBD=REAL,GPS=OK...         - System status with component health
DIAGNOSTIC_STARTED:RUNNING        - Diagnostic process initiated
DIAGNOSTIC_RESULTS:detailed_info  - Comprehensive test results with pipe separators
DIAGNOSTIC_COMPLETE:SUCCESS       - Diagnostic process finished
```

### Enhanced Diagnostic System
The DIAGNOSTIC command runs comprehensive hardware tests with detailed reporting:
- **ADC Tests**: Multiple pin voltage testing (GPIO 36, 39, 34, 35, 32, 33)
- **Digital Pin Tests**: State verification (GPIO 2, 4, 5, 16, 17, 18)
- **I2C Bus Scanning**: Device detection (SDA=21, SCL=22)
- **SPI Interface**: Pin state verification (MISO=19, MOSI=23, SCK=18, SS=5)
- **OBD-II Testing**: Multiple baud rate testing and interface validation
- **ESP32 System**: Memory, voltage, and health checks
- **BLE Connectivity**: Stack verification and connection testing
- **Automatic Issue Detection**: Problem identification with recommendations

### Diagnostic Error Handling
- **Connection Validation**: Commands only sent when device properly connected
- **Error Logging**: Failed commands logged with detailed error messages
- **Graceful Degradation**: Dashboard shows connection status and error details
- **User Feedback**: Clear indication when device not available for commands

## Configuration

Key settings in `config.h`:
- Data collection intervals (OBD_INTERVAL, GPS_INTERVAL, BLE_INTERVAL)
- Storage buffer size (MAX_LOG_ENTRIES)
- BLE device name and service UUID
- Pin definitions for LED and buzzer
- OBD PID definitions for monitoring

## Expected Behavior

1. **Startup**: Serial output shows component initialization status
2. **Connection**: LED blinks until BLE client connects, then stays on
3. **Data Modes**: 
   - DISABLED: No sensor data (default)
   - REAL: Actual OBD-II and hardware sensor data
   - SIMULATED: Client-side test data generation (web dashboard only)
4. **BLE Streaming**: Message-counted data with API version validation
5. **Storage**: Circular buffer logging (100 entries max)
6. **Error Handling**: Graceful fallback when OBD connection lost
7. **Status Monitoring**: Averaged component health over 10 readings
8. **Enhanced Diagnostics**: Multi-stage hardware testing with detailed results
9. **Dashboard Stability**: Robust error handling for simulation and chart updates
10. **Command Validation**: Connection state checking before command transmission

## Architecture Notes

- **Simplified Design**: No dependency on full Freematics library
- **Hardware Abstraction**: Direct sensor reading via ESP32 ADC
- **Client-Side Simulation**: Test data generation moved to web dashboard with crash protection
- **Protocol Versioning**: API compatibility checking
- **Web Integration**: Designed for modern web dashboard interface with robust error handling
- **Enhanced Diagnostic Framework**: Comprehensive hardware testing with detailed reporting
- **Real-Time Focus**: Device optimized for actual vehicle data collection
- **Crash Prevention**: Dashboard handles undefined data gracefully with null coalescing
- **Connection Management**: Proper BLE connection state validation and error reporting
- **Chart Stability**: All chart updates include fallback values to prevent crashes

## ⚠️ CRITICAL: Legacy Hardware Compatibility Notes

**This project targets OLDER ESP32 boards (pre-ESP32-S3/C3) and uses CUSTOM IMPLEMENTATIONS.**

### Common LLM-Generated Code Issues:
1. **Newer ESP32 Functions**: Modern ESP32 functions may not exist on older boards
2. **Library Assumptions**: Standard Arduino/ESP32 libraries may behave differently
3. **Hardware Abstractions**: We implement custom sensor reading instead of using newer APIs
4. **Temperature Sensors**: Uses custom `temperatureRead()` fallback, not newer temp sensor APIs
5. **ADC Reading**: Direct `analogRead()` calls instead of newer ADC APIs
6. **BLE Implementation**: Uses basic ESP32 BLE, not newer BLE mesh or advanced features

### Debugging Strategy for "Not Working" Issues:
1. **Check if function exists**: If code fails to compile, the function likely doesn't exist on older ESP32
2. **Look for custom implementations**: Search codebase for custom versions (e.g., `readBatteryVoltage()`, `readAmbientTemperature()`)
3. **Verify hardware assumptions**: Older boards have different pin mappings and capabilities
4. **Test with simulation first**: Use simulation mode to isolate hardware vs software issues
5. **Check ESP32 core version**: This targets ESP32 Arduino Core 3.2.0+, not latest versions

### Custom Implementation Examples:
- **Battery Voltage**: Custom ADC reading with voltage divider calculation
- **Temperature**: Fallback chain from OBD → external sensor → ESP32 internal temp
- **OBD Communication**: Custom Serial2 implementation, not CAN bus libraries
- **GPS**: Simulated data generation, not real GPS module integration
- **Storage**: Custom circular buffer, not SPIFFS/LittleFS

**When adding new features, always check existing patterns and implement similar custom solutions rather than assuming modern ESP32 APIs are available.**

## ⚠️ CRITICAL: Compilation Error - Wire Library Missing

**The sketch FAILS TO COMPILE due to missing Wire library include.**

### Current Status (Latest Deployment Attempt):
- **Compilation Error**: `'Wire' was not declared in this scope` at line 241
- **Location**: `SimpleOBD::runFullDiagnostics()` function in I2C bus scanning section
- **Root Cause**: Wire library was removed to reduce flash size but diagnostic code still uses it
- **Problem**: Diagnostic function calls `Wire.begin(21, 22)` without proper include

### Immediate Fix Required:
The diagnostic system uses I2C bus scanning which requires the Wire library:
```cpp
Wire.begin(21, 22); // SDA=21, SCL=22
```

### Size vs Functionality Trade-off:
1. ❌ **Wire Library Removed**: Eliminated `#include <Wire.h>` to reduce flash size
2. ❌ **Diagnostic Code Retained**: I2C scanning code still present but non-functional
3. ⚠️ **Compilation Failure**: Code references undefined Wire object

### Resolution Options:
1. **Add Wire Include**: Restore `#include <Wire.h>` (increases flash usage)
2. **Remove I2C Diagnostics**: Comment out Wire-dependent diagnostic code
3. **Conditional Compilation**: Use `#ifdef` to make I2C diagnostics optional

**STATUS: COMPILATION BLOCKED - Wire library dependency must be resolved**

### IMMEDIATE ACTIONS REQUIRED:

#### Option 1: Fix Wire Library Dependency (RECOMMENDED)
```cpp
// Add to top of freematics_custom.ino:
#include <Wire.h>
```

#### Option 2: Remove I2C Diagnostics
```cpp
// Comment out Wire-dependent code in runFullDiagnostics():
// Wire.begin(21, 22); // SDA=21, SCL=22
// ... I2C scanning code ...
```

#### Option 3: Conditional I2C Support
```cpp
#ifdef ENABLE_I2C_DIAGNOSTICS
#include <Wire.h>
#endif
```

### Compilation Command Test:
```bash
cd freematics_custom && ./deploy.sh
```

**STATUS: COMPILATION BLOCKED - Must resolve Wire library dependency before testing**

### Next Steps:
1. **Fix Wire Include**: Add missing `#include <Wire.h>` to resolve compilation error
2. **Test Compilation**: Run `./deploy.sh` to verify build success
3. **Check Flash Usage**: Monitor if Wire library addition exceeds flash limits
4. **Optimize if Needed**: Remove I2C diagnostics if flash space becomes critical

### Recent Changes Summary:
- Moved 50KB+ simulation code to client-side JavaScript
- Enhanced diagnostics with comprehensive hardware testing
- Focused device on real vehicle data collection
- Fixed dashboard simulation crashes with proper data handling
- Fixed diagnostics button with connection state validation
- Added null safety to all chart updates and data history
- Enhanced error handling for BLE command transmission
- Improved user feedback for connection status and errors
- Maintained full OBD-II protocol support for real data

### Latest Status (Device-UI Synchronization):
- **Enhanced Diagnostics**: Comprehensive system health reporting with 12 test categories
- **Dashboard Simulation**: Fixed undefined data access in chart updates with null coalescing
- **Diagnostics Button**: Added proper connection validation and error handling
- **Chart Updates**: Added fallback values (|| 0) to prevent undefined value crashes
- **Data History**: Enhanced fallback logic for missing sensor data
- **Command Protocol**: Improved error logging and user feedback for failed commands
- **Connection Management**: Better BLE connection state checking and validation
- **PID Synchronization**: Added missing engine parameters (oil temp, exhaust temp, fuel rate, etc.)
- **Temperature Units**: Consistent Celsius to Fahrenheit conversion throughout
- **Turbo Support**: Added realistic turbo/boost simulation and device support
- **GPS Precision**: Fixed coordinate storage and transmission format
- **Diagnostic Formatting**: Improved pipe-to-newline conversion for readability

### Synchronization Fixes Applied:
- **Device Firmware**: Added 7 missing PID mappings for advanced engine parameters
- **UI Dashboard**: Enhanced temperature conversion and data parsing for new PIDs
- **Simulation**: Improved realism with conditional turbo/boost data generation
- **Data Protocol**: Synchronized GPS coordinate precision and diagnostic formatting
- **Error Handling**: Better validation for missing or malformed sensor data

### Current Status:
- **Compilation**: RESOLVED - Wire library included, I2C diagnostics functional
- **Device-UI Sync**: COMPLETE - All dashboard fields now have corresponding device data
- **Temperature Units**: CONSISTENT - All temperatures properly converted C→F
- **Advanced Features**: ENABLED - Turbo, boost, exhaust temps, fuel rates supported
- **Data Integrity**: IMPROVED - Better validation and fallback handling
