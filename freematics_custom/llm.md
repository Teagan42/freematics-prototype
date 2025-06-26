# Freematics Custom Sketch - LLM Documentation

## Project Overview

This Arduino sketch creates a vehicle telemetry system using the Freematics platform. It collects OBD-II data, GPS location, and accelerometer readings from a vehicle and transmits them via Bluetooth Low Energy (BLE) for real-time monitoring.

## Purpose

- **Vehicle Data Collection**: Monitor engine RPM, speed, coolant temperature, and other OBD-II parameters
- **Location Tracking**: Capture GPS coordinates and satellite information
- **Motion Sensing**: Record accelerometer data for vehicle dynamics analysis
- **Real-time Debugging**: Stream data to Android devices via BLE for development and testing

## Hardware Requirements

- Freematics ONE+ (ESP32-based) or compatible ESP32 board
- OBD-II port access in target vehicle
- Integrated GPS and MEMS sensors (or external modules)

## Testing Strategy

### 1. Bench Testing
- Connect to computer via USB
- Monitor Serial output at 115200 baud
- Verify initialization of OBD, GPS, MEMS, and storage components
- Check data formatting without vehicle connection

### 2. Vehicle Testing
- Connect OBD-II cable to vehicle diagnostic port
- Start engine and drive to generate data
- Monitor BLE transmission via Freematics Android app
- Verify data accuracy against vehicle dashboard

### 3. BLE Testing
- Use Freematics BLE Android application
- Connect to device named "FreematicsCustom"
- Monitor real-time data stream
- Verify data format matches expected protocol

## BLE Data Protocol

### Connection Details
- **Device Name**: "FreematicsCustom"
- **Service UUID**: "12345678-1234-1234-1234-123456789abc"
- **Transmission Interval**: 1000ms (configurable)

### Data Format
BLE messages follow this comma-separated format:
```
timestamp,RPM:value;SPD:value;GPS:lat,lng;
```

### Example BLE Messages
```
12345,RPM:2500;SPD:65;GPS:37.7749,-122.4194;
12346,RPM:2600;SPD:67;
12347,GPS:37.7750,-122.4195;
```

### Data Codes

#### OBD-II Parameters
- **RPM**: Engine revolutions per minute (0x0C)
- **SPD**: Vehicle speed in km/h (0x0D)
- **COOLANT**: Engine coolant temperature in °C (0x05)

#### GPS Data
- **GPS**: Latitude,Longitude in decimal degrees (6 decimal places)
- **SAT**: Number of satellites (logged as PID 0x22)

#### MEMS Data (Internal Storage Only)
- **0x10**: X-axis acceleration (×100)
- **0x11**: Y-axis acceleration (×100)  
- **0x12**: Z-axis acceleration (×100)

### Data Collection Intervals
- **OBD**: 1000ms
- **GPS**: 2000ms
- **MEMS**: 500ms
- **BLE Transmission**: 1000ms

## Configuration

Key settings in `config.h`:
- Adjust collection intervals
- Modify BLE device name/UUID
- Add additional OBD PIDs
- Configure pin assignments

## Expected Behavior

1. **Startup**: Serial output shows component initialization status
2. **Data Collection**: Continuous sampling based on configured intervals
3. **BLE Streaming**: Formatted data transmitted to connected Android app
4. **Storage**: All data logged to internal buffer (100 entries max)
5. **Error Handling**: Graceful degradation if components fail to initialize
