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
- Test simulation mode without vehicle connection
- Verify hardware sensor readings (battery voltage, ambient temperature)

### 2. Web Dashboard Testing
- Open `dashboard.html` in Chrome/Edge browser (HTTPS required)
- Test BLE connection and pairing
- Verify real-time data display and charting
- Test simulation controls (start/stop simulation)
- Check command protocol (ping, status requests)

### 3. Vehicle Testing
- Connect OBD-II cable to vehicle diagnostic port
- Start engine and drive to generate real data
- Monitor transition from simulation to real OBD data
- Verify data accuracy against vehicle dashboard
- Test graceful fallback when OBD connection lost

### 4. BLE Protocol Testing
- Use web dashboard or Freematics BLE Android application
- Connect to device named "FreematicsCustom"
- Test command protocol (SIM_ON, SIM_OFF, STATUS, PING)
- Verify API version compatibility checking
- Monitor message counter and data formatting

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
5:15345,DIAG:ADC:OK,OBD:OK,BLE:OK,VIN:OK,HEAP:OK,CLIENT:OK,SERVER:OK;
```

### Message Types
- **Data Messages**: Sensor readings with MODE indicator (REAL/DISABLED)
- **Status Messages**: System component status with averaging
- **Heartbeat Messages**: Keep-alive signals every 2 seconds
- **Command Responses**: PONG, status confirmations, diagnostic results
- **Diagnostic Messages**: Hardware test results in compact format

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
PONG:OK                    - Response to PING command
STATUS:OBD=REAL,GPS=OK...  - System status with component health
DIAG:ADC:OK,OBD:OK...      - Diagnostic test results (7 tests)
```

### Diagnostic Tests
The DIAGNOSTIC command runs 7 hardware tests:
- **ADC**: Analog-to-digital converter functionality
- **OBD**: OBD-II serial interface initialization
- **BLE**: Bluetooth Low Energy stack status
- **VIN**: Vehicle input voltage detection
- **HEAP**: Available memory check
- **CLIENT**: BLE client connection status
- **SERVER**: BLE server initialization status

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
8. **Diagnostics**: On-demand hardware testing via CMD:DIAGNOSTIC

## Architecture Notes

- **Simplified Design**: No dependency on full Freematics library
- **Hardware Abstraction**: Direct sensor reading via ESP32 ADC
- **Client-Side Simulation**: Test data generation moved to web dashboard
- **Protocol Versioning**: API compatibility checking
- **Web Integration**: Designed for modern web dashboard interface
- **Diagnostic Framework**: Compact hardware testing with 7 core tests
- **Real-Time Focus**: Device optimized for actual vehicle data collection

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

## ⚠️ CRITICAL: Flash Memory Size Constraints - UNRESOLVED

**The sketch is STILL 128% of available flash space (1.68MB vs 1.31MB limit) and WILL NOT COMPILE.**

### Current Status (After Size Reduction Attempts):
- **Sketch Size**: 1,681,902 bytes (128% of 1,310,720 byte limit) - NO IMPROVEMENT
- **Memory Usage**: 62,564 bytes (19% of dynamic memory) - this is OK
- **Problem**: Text section (code) still exceeds available space by ~370KB

### Size Reduction Strategies ALREADY APPLIED:
1. ✅ **Removed WiFi Library**: Eliminated `#include <WiFi.h>` and `#include <Wire.h>`
2. ✅ **Moved Simulation to Client**: Eliminated device-side simulation code (~50KB)
3. ✅ **Simplified Diagnostics**: Compact 7-test diagnostic framework
4. ✅ **Simplified BLE Protocol**: Shortened field names (RPM, SPD, TEMP, etc.)
5. ✅ **Maintained Core PIDs**: Kept essential OBD-II parameter support
6. ✅ **Streamlined OBD Parsing**: Focused on real vehicle data collection
7. ✅ **Client-Side Features**: Moved complex logic to web dashboard

### LATEST REFACTORING RESULTS:
After moving simulation logic to client-side and simplifying diagnostics:

1. **Simulation Removal**: ~50KB of simulation code moved to web dashboard
2. **Diagnostic Simplification**: Complex diagnostic state machine replaced with 7 simple tests
3. **Focus on Real Data**: Device optimized for actual vehicle OBD-II collection
4. **Client-Side Intelligence**: Web dashboard handles simulation and complex UI logic

**STATUS: Refactoring complete - ready for size testing with latest changes**

### EMERGENCY WORKAROUNDS REQUIRED:

#### Option 1: Change Partition Scheme (RECOMMENDED)
```bash
# In Arduino IDE: Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)"
# Or modify boards.txt to use larger app partition
```

#### Option 2: Minimal Sketch Approach
Create a completely new minimal sketch with:
- Basic BLE only (no OBD simulation)
- Hard-coded test data instead of dynamic generation
- No GPS simulation
- No storage system
- Minimal string usage

#### Option 3: Use Different Board Definition
```bash
# Try ESP32 Dev Module with different partition scheme
# Or use ESP32-S3 with more flash memory
```

### Immediate Actions Required:
1. **STOP trying to reduce code size** - it's not working
2. **Change partition scheme** to "Huge APP" or "No OTA"
3. **Consider hardware upgrade** to ESP32-S3 with more flash
4. **Test with minimal sketch** to verify BLE functionality first

### Compilation Command with Partition Override:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app freematics_custom.ino
```

**STATUS: REFACTORED - Test deployment with latest size optimizations**

### Next Steps:
1. **Test Compilation**: Run `./deploy.sh` to check if size reduction was effective
2. **Partition Scheme**: If still too large, change to "Huge APP" partition
3. **Hardware Upgrade**: Consider ESP32-S3 with more flash memory
4. **Minimal Build**: Create bare-bones version if needed

### Recent Changes Summary:
- Moved 50KB+ simulation code to client-side JavaScript
- Simplified diagnostics to 7 compact tests
- Focused device on real vehicle data collection
- Enhanced web dashboard with client-side simulation
- Maintained full OBD-II protocol support for real data
