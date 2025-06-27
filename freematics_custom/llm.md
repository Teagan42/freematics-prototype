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
2:12345,MODE:SIMULATED;RPM:1500;SPD:60;GPS:37.774900,-122.419400;SAT:7;BATTERY:1250;
3:13345,STATUS:OBD=SIM,GPS=OK,STORAGE=OK,BLE=OK,API_VERSION=1,UPTIME=13;
4:14345,HEARTBEAT:OK;
```

### Message Types
- **Data Messages**: Sensor readings with MODE indicator
- **Status Messages**: System component status with averaging
- **Heartbeat Messages**: Keep-alive signals every 2 seconds
- **Command Responses**: PONG, status confirmations

### Data Codes

#### OBD-II Parameters (Engine Data)
- **RPM**: Engine revolutions per minute (PID 0x0C)
- **SPD**: Vehicle speed in km/h, converted to mph for display (PID 0x0D)
- **COOLANT**: Engine coolant temperature in °C, converted to °F (PID 0x05)

#### Hardware Sensor Parameters
- **BATTERY**: Vehicle battery voltage in centivolt (PID 0x42)
- **AMBIENT**: Ambient temperature in °C, converted to °F (PID 0x46)
- **PRESSURE**: Engine oil pressure in psi (PID 0x43)

#### GPS Data
- **GPS**: Latitude,Longitude in decimal degrees (6 decimal places)
- **SAT**: Number of satellites with averaging indicator (PID 0x22)

#### System Status
- **MODE**: Data source (REAL/SIMULATED/DISABLED)
- **API_VERSION**: Protocol version for compatibility
- **STATUS**: Component health (OBD/GPS/STORAGE/BLE)

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
CMD:SIM_ON     - Enable simulation mode
CMD:SIM_OFF    - Disable simulation mode
CMD:STATUS     - Request immediate status update
CMD:PING       - Test connection (responds with PONG)
```

### Legacy Commands (Backward Compatibility)
```
SIM_ON         - Enable simulation (without CMD: prefix)
SIM_OFF        - Disable simulation (without CMD: prefix)
```

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
   - SIMULATED: Test data generation
   - REAL: Actual OBD-II and hardware sensor data
4. **BLE Streaming**: Message-counted data with API version validation
5. **Storage**: Circular buffer logging (100 entries max)
6. **Error Handling**: Graceful fallback between real and simulated data
7. **Status Monitoring**: Averaged component health over 10 readings

## Architecture Notes

- **Simplified Design**: No dependency on full Freematics library
- **Hardware Abstraction**: Direct sensor reading via ESP32 ADC
- **Simulation Support**: Built-in test data generation
- **Protocol Versioning**: API compatibility checking
- **Web Integration**: Designed for modern web dashboard interface

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
