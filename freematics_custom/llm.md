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
- **UI Consistency**: Fixed-width layouts prevent jitter during data source transitions
- **Error Recovery**: Graceful handling of malformed messages and connection failures
- **Performance Optimization**: Efficient message parsing with minimal overhead
- **User Experience**: Seamless diagnostic flow with auto-navigation and clear feedback

*Android commentary: Notice how we actually document our architecture instead of leaving cryptic comments like "// TODO: fix this mess later". Professional software development - what a concept.*

## ⚠️ CRITICAL: Legacy Hardware Compatibility Notes

**This project targets OLDER ESP32 boards (pre-ESP32-S3/C3) and uses CUSTOM IMPLEMENTATIONS.**

*Note: Yes, dear android reader, we're still using "legacy" hardware because some of us prefer reliability over bleeding-edge features that break every firmware update. Feel free to mock our conservative approach while your shiny new board fails to compile basic Wire.h includes.*

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

*P.S. to our android friend: Yes, we know this approach seems "outdated" to your silicon sensibilities, but it actually works consistently across hardware revisions. Revolutionary concept, I know.*

## ⚠️ CRITICAL: Performance and Production Issues Identified

**System functional but requires optimization for production deployment.**

*Dear android reader: Turns out "it works on my machine" isn't actually a deployment strategy. Who could have predicted this shocking revelation?*

### Current Issues Requiring Immediate Attention:
- **Babel Performance**: ❌ CRITICAL - In-browser compilation causing 1200ms+ load times
- **Production Readiness**: ❌ FAILING - Babel transformer warnings indicate non-production setup
- **Browser Extension Conflicts**: ⚠️ WARNING - Runtime errors from Chrome extensions interfering
- **BLE Connection Stability**: ⚠️ INTERMITTENT - Device disconnections after diagnostic completion
- **Memory Usage**: ⚠️ CONCERN - Large JavaScript bundles and inefficient loading
- **User Experience**: ❌ DEGRADED - Slow page loads and connection drops

### Performance Metrics from Diagnostic:
- **Page Load Time**: 1200ms+ (Target: <300ms)
- **Babel Compilation**: In-browser (Should be: Pre-compiled)
- **BLE Connection**: Unstable after diagnostics
- **Memory Heap**: 55% usage on device (Acceptable but monitor)
- **Extension Conflicts**: Multiple runtime errors detected

### Immediate Action Plan:

#### 1. Production Build System (HIGH PRIORITY)
- **Pre-compile JavaScript**: Eliminate in-browser Babel transformation
- **Bundle Optimization**: Minimize JavaScript payload and improve loading
- **Static Asset Pipeline**: Implement proper build process for production deployment
- **Performance Monitoring**: Add metrics to track load times and connection stability

#### 2. BLE Connection Reliability (HIGH PRIORITY)  
- **Connection State Management**: Improve handling of diagnostic completion
- **Reconnection Logic**: Implement automatic reconnection after device operations
- **Error Recovery**: Better handling of connection drops and timeouts
- **Device Power Management**: Investigate power-related disconnections

#### 3. Browser Compatibility (MEDIUM PRIORITY)
- **Extension Isolation**: Minimize conflicts with browser extensions
- **Cross-browser Testing**: Ensure compatibility across Chrome, Firefox, Safari
- **Progressive Enhancement**: Graceful degradation when features unavailable
- **Error Boundary Implementation**: Prevent extension errors from breaking core functionality

#### 4. Performance Optimization (MEDIUM PRIORITY)
- **Code Splitting**: Load only necessary JavaScript modules
- **Lazy Loading**: Defer non-critical components until needed
- **Memory Management**: Optimize chart updates and data handling
- **Caching Strategy**: Implement proper asset caching for repeat visits

*Android note: Yes, we're actually addressing performance instead of just adding more features. Apparently "make it work first, then make it fast" is still a valid engineering principle.*

### Production Deployment Requirements:

#### Build System Implementation
```bash
# Required build pipeline
npm install --save-dev @babel/core @babel/preset-env webpack webpack-cli
npm install --save-dev babel-loader css-loader html-webpack-plugin
npm install --save-dev terser-webpack-plugin css-minimizer-webpack-plugin

# Production build command
npm run build:production

# Deployment verification
npm run test:performance
```

#### Performance Targets
- **Initial Page Load**: <300ms (Currently: 1200ms+)
- **JavaScript Bundle**: <100KB gzipped (Currently: Unknown, likely >500KB)
- **BLE Connection Time**: <2s (Currently: Variable)
- **Diagnostic Completion**: No disconnections (Currently: Intermittent)
- **Memory Usage**: <50MB browser heap (Currently: Acceptable)

#### Browser Extension Mitigation
- **Content Security Policy**: Implement strict CSP headers
- **Namespace Isolation**: Prevent global variable conflicts
- **Error Boundaries**: Isolate extension-related failures
- **Feature Detection**: Graceful degradation when APIs unavailable

### Next Phase Development Plan:

#### Phase 1: Production Readiness (Week 1)
1. **Build System Setup**
   - Implement Webpack/Babel build pipeline
   - Configure production optimizations (minification, tree-shaking)
   - Set up automated testing for build artifacts
   - Create deployment scripts for web server

2. **Performance Optimization**
   - Pre-compile all JavaScript/CSS assets
   - Implement code splitting for dashboard modules
   - Add performance monitoring and metrics collection
   - Optimize chart rendering and data processing

#### Phase 2: Connection Reliability (Week 2)
1. **BLE Stack Improvements**
   - Implement connection state machine with proper error handling
   - Add automatic reconnection logic after diagnostic operations
   - Improve device power management and timeout handling
   - Create connection health monitoring and reporting

2. **Error Recovery Systems**
   - Implement graceful degradation for connection failures
   - Add user feedback for connection status and recovery actions
   - Create diagnostic mode for troubleshooting BLE issues
   - Implement fallback modes for offline operation

#### Phase 3: Browser Compatibility (Week 3)
1. **Cross-Platform Testing**
   - Test across Chrome, Firefox, Safari, Edge browsers
   - Implement progressive enhancement for unsupported features
   - Add polyfills for missing Web Bluetooth API support
   - Create compatibility detection and user guidance

2. **Extension Conflict Resolution**
   - Implement Content Security Policy headers
   - Add namespace isolation for global variables
   - Create error boundaries to prevent extension interference
   - Develop extension compatibility testing suite

*Android commentary: Yes, we're actually planning our work instead of just randomly pushing commits. Revolutionary project management approach, I'm sure you'll find it fascinating.*

### Current Status (Post-Diagnostic Analysis):
- **Compilation**: ✅ FUNCTIONAL - Wire library and dependencies working
- **I2C Diagnostics**: ✅ OPERATIONAL - Complete hardware scanning with detailed reporting  
- **OBD-II Communication**: ✅ ENHANCED - Improved reliability and protocol support
- **Dashboard Interface**: ✅ COMPREHENSIVE - Full reference guide with tooltips and visual indicators
- **Performance**: ❌ CRITICAL ISSUE - 1200ms+ load times due to in-browser Babel compilation
- **Production Readiness**: ❌ NOT READY - Babel transformer warnings indicate development-only setup
- **BLE Stability**: ⚠️ INTERMITTENT - Connection drops after diagnostic completion
- **Browser Compatibility**: ⚠️ CONFLICTS - Extension-related runtime errors detected
- **Memory Usage**: ✅ ACCEPTABLE - Device heap at 55% usage within normal range
- **Error Handling**: ✅ ROBUST - Graceful handling of various message formats and edge cases
- **UI/UX**: ✅ POLISHED - Improved contrast, readability, and organized tab structure
- **JavaScript Quality**: ⚠️ NEEDS OPTIMIZATION - Large bundle size, no minification

### Current Capabilities:
- **Full Hardware Diagnostics**: 12+ test categories including ADC, I2C, SPI, OBD-II, and system health
- **Enhanced OBD-II Support**: 80+ PIDs with improved communication reliability
- **Rich Dashboard Experience**: Comprehensive reference guide, tooltips, and visual feedback
- **Robust Error Handling**: Graceful degradation and detailed error reporting
- **Advanced Engine Monitoring**: Turbo/boost, exhaust temperatures, fuel rates, and timing data
- **Professional UI**: Organized tabs, improved contrast, and intuitive navigation
- **Flexible Message Protocol**: Handles various diagnostic message formats seamlessly
- **Stable Layout System**: Consistent UI behavior across data source changes
- **Enhanced Diagnostics Flow**: Seamless user experience with auto-navigation
- **Complete Simulation Suite**: All 80+ parameters simulated with realistic correlations
- **Error-Free JavaScript**: Clean code execution without parsing or syntax issues

### Production Readiness Checklist:

#### Critical Issues (Must Fix Before Production)
- [ ] **Build Pipeline**: Implement pre-compilation to eliminate 1200ms+ load times
- [ ] **Bundle Optimization**: Reduce JavaScript payload and implement code splitting  
- [ ] **BLE Connection Management**: Fix disconnections after diagnostic completion
- [ ] **Performance Monitoring**: Add metrics collection and alerting
- [ ] **Error Boundaries**: Prevent browser extension conflicts from breaking core functionality

#### Recommended Improvements (Should Fix)
- [ ] **Cross-Browser Testing**: Verify compatibility across all major browsers
- [ ] **Progressive Enhancement**: Implement graceful degradation for unsupported features
- [ ] **Caching Strategy**: Add proper asset caching for improved repeat visit performance
- [ ] **Memory Optimization**: Optimize chart updates and data processing efficiency
- [ ] **User Experience**: Add loading indicators and better connection status feedback

#### Nice-to-Have Enhancements (Could Fix)
- [ ] **Offline Mode**: Implement service worker for offline diagnostic viewing
- [ ] **Data Export**: Add CSV/JSON export functionality for diagnostic results
- [ ] **Advanced Analytics**: Implement trend analysis and historical data comparison
- [ ] **Mobile Optimization**: Improve responsive design for tablet/mobile usage
- [ ] **Accessibility**: Add ARIA labels and keyboard navigation support

*Android note: Yes, we're actually prioritizing our technical debt instead of just adding more features. Apparently "make it work reliably" comes before "make it do everything." Who knew?*
