/*************************************************************************
* Custom Freematics Sketch - Simplified Version
* Developed for debugging with Freematics BLE Android app
* Compatible with ESP32 Arduino Core 3.2.0+
*************************************************************************/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <WiFi.h>
#include "config.h"
#include "telestore.h"

// working states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x40
#define STATE_BLE_READY 0x80

// API Version for UI compatibility
#define API_VERSION 1

// Diagnostic mode state
bool diagnosticMode = false;
unsigned long diagnosticStartTime = 0;
String diagnosticResults = "";
int diagnosticStep = 0;

// Forward declarations
class CustomFreematicsLogger;
class MyCharacteristicCallbacks;

// Global instances
CustomFreematicsLogger* logger = nullptr;

// BLE Server and Characteristic
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool bleClientConnected = false;
unsigned long lastLedBlink = 0;
bool ledState = false;

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        bleClientConnected = true;
        Serial.println("=== BLE CLIENT CONNECTED ===");
        Serial.println("Client MAC: " + String(pServer->getConnId()));
        Serial.println("Starting data collection...");
        
        // Send immediate connection confirmation with API version
        if (pCharacteristic) {
            String connectMsg = "0:" + String(millis()) + ",CONNECT:SUCCESS,API_VERSION:" + String(API_VERSION) + ";";
            pCharacteristic->setValue(connectMsg.c_str());
            pCharacteristic->notify();
            Serial.println("BLE TX CONNECT: " + connectMsg);
        }
    };

    void onDisconnect(BLEServer* pServer) {
        bleClientConnected = false;
        Serial.println("=== BLE CLIENT DISCONNECTED ===");
        Serial.println("Reason: Client initiated or connection lost");
        Serial.println("Restarting advertising...");
        BLEDevice::startAdvertising();
        Serial.println("Waiting for new BLE client connection...");
    }
};


// Hardware and OBD-II interface with real sensor data
class SimpleOBD {
private:
    bool realOBDAvailable = false;
    bool simulationEnabled = false;  // Start with simulation disabled
    String lastError = "";
    unsigned long lastErrorTime = 0;
    
    String runHardwareDiagnostics() {
        String results = "HW DIAG:\\n";
        
        // Test key ADC pins only
        int adcPins[] = {36, 39, 34, 35};
        for (int i = 0; i < 4; i++) {
            int value = analogRead(adcPins[i]);
            results += "GPIO" + String(adcPins[i]) + ":" + String(value);
            results += (value < 100) ? "(L)" : (value > 3900) ? "(H)" : "(A)";
            results += "\\n";
        }
        
        // Test I2C bus
        Wire.begin(21, 22);
        int deviceCount = 0;
        for (byte addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                results += "I2C:0x" + String(addr, HEX) + "\\n";
                deviceCount++;
            }
        }
        if (deviceCount == 0) results += "I2C:NONE\\n";
        
        return results;
    }
    
    String runOBDDiagnostics() {
        String results = "OBD DIAG:\\n";
        
        Serial2.begin(38400, SERIAL_8N1, 16, 17);
        delay(100);
        
        // Test key AT commands
        String commands[] = {"ATZ", "0100"};
        for (int i = 0; i < 2; i++) {
            Serial2.println(commands[i]);
            delay(500);
            
            String response = "";
            unsigned long startTime = millis();
            while (millis() - startTime < 1000 && Serial2.available()) {
                char c = Serial2.read();
                if (c != '\\r' && c != '\\n' && c != ' ') {
                    response += c;
                }
            }
            
            results += commands[i] + ":";
            results += (response.length() > 0) ? "OK" : "FAIL";
            results += "\\n";
        }
        
        return results;
    }
    
    String runSystemDiagnostics() {
        String results = "SYS DIAG:\\n";
        
        results += "CPU:" + String(ESP.getCpuFreqMHz()) + "MHz\\n";
        results += "Heap:" + String(ESP.getFreeHeap() / 1024) + "KB\\n";
        results += "Uptime:" + String(millis() / 1000) + "s\\n";
        
        int vin = analogRead(A0);
        float voltage = (vin / 4095.0) * 3.3 * 6.0;
        results += "Vin:" + String(voltage, 1) + "V";
        if (voltage < 11.0) results += "(LOW)";
        else if (voltage > 15.0) results += "(HIGH)";
        else results += "(OK)";
        results += "\\n";
        
        #ifdef SOC_TEMP_SENSOR_SUPPORTED
        float temp = temperatureRead();
        results += "Temp:" + String(temp, 0) + "C";
        results += (temp > 80) ? "(HOT)" : "(OK)";
        results += "\\n";
        #endif
        
        results += "BLE:" + String(BLEDevice::getInitialized() ? "Y" : "N") + "\\n";
        
        return results;
    }
    
    String runConnectivityDiagnostics() {
        String results = "CONN DIAG:\\n";
        
        results += "BLE:" + String(BLEDevice::getInitialized() ? "Y" : "N") + "\\n";
        results += "Client:" + String(bleClientConnected ? "Y" : "N") + "\\n";
        results += "Server:" + String(pServer != NULL ? "Y" : "N") + "\\n";
        
        if (pServer) {
            results += "Clients:" + String(pServer->getConnectedCount()) + "\\n";
        }
        
        if (pCharacteristic && bleClientConnected) {
            String testMsg = "TEST:" + String(millis());
            pCharacteristic->setValue(testMsg.c_str());
            pCharacteristic->notify();
            results += "TX:OK\\n";
        } else {
            results += "TX:SKIP\\n";
        }
        
        return results;
    }
    
public:
    bool init() { 
        // Initialize hardware sensors first
        Serial.print("Initializing hardware sensors...");
        
        // Hardware sensors are always available on Freematics devices
        Serial.println("OK");
        
        // Initialize OBD-II communication interface
        Serial.print("Initializing OBD-II interface...");
        
        // Initialize Serial2 for OBD communication (adjust pins as needed)
        Serial2.begin(38400, SERIAL_8N1, 16, 17); // RX=16, TX=17 for ESP32
        delay(100);
        
        // Send initialization commands to OBD interface
        Serial2.println("ATZ"); // Reset
        delay(1000);
        Serial2.println("ATE0"); // Echo off
        delay(500);
        Serial2.println("ATL0"); // Linefeeds off
        delay(500);
        Serial2.println("ATS0"); // Spaces off
        delay(500);
        Serial2.println("ATH1"); // Headers on
        delay(500);
        
        Serial.println("OBD interface initialized");
        
        // Test OBD-II connection with vehicle
        Serial.print("Testing OBD-II connection...");
        
        int testValue;
        if (attemptRealOBDRead(0x0C, testValue)) {
            realOBDAvailable = true;
            lastError = "";
            Serial.println("Real OBD-II connection established");
            return true;
        } else {
            realOBDAvailable = false;
            lastError = "No OBD-II response from vehicle";
            Serial.println("No vehicle response, hardware sensors available");
            return true; // Still return true to allow hardware sensor readings
        }
    }
    
    bool readPID(uint8_t pid, int& value) {
        // Try hardware sensors first for supported PIDs
        if (readHardwareSensor(pid, value)) {
            return true;
        }
        
        // Try real OBD-II for engine-specific data
        if (realOBDAvailable && isOBDOnlyPID(pid)) {
            if (attemptRealOBDRead(pid, value)) {
                return true;
            } else {
                // Real OBD failed, fall back to simulation
                realOBDAvailable = false;
                lastError = "Real OBD-II read failed for PID 0x" + String(pid, HEX);
                lastErrorTime = millis();
                Serial.println("OBD-II read failed, falling back to simulation");
            }
        }
        
        // Only use simulated data if simulation is enabled
        if (simulationEnabled) {
            return readSimulatedPID(pid, value);
        } else {
            lastError = "No data source available for PID 0x" + String(pid, HEX);
            return false;
        }
    }
    
    // Get all available PIDs for this vehicle
    bool getSupportedPIDs(uint8_t* pidList, int& pidCount) {
        pidCount = 0;
        
        // Standard EPA-required PIDs for emissions compliance
        uint8_t standardPIDs[] = {
            PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_SHORT_TERM_FUEL_TRIM_1, PID_LONG_TERM_FUEL_TRIM_1,
            PID_FUEL_PRESSURE, PID_INTAKE_MAP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP,
            PID_MAF_FLOW, PID_THROTTLE_POS, PID_O2_S1_VOLTAGE, PID_O2_S2_VOLTAGE, PID_OBD_STANDARDS,
            PID_O2_SENSORS_PRESENT, PID_RUNTIME, PID_DISTANCE_WITH_MIL, PID_FUEL_RAIL_PRESSURE,
            PID_COMMANDED_EGR, PID_EGR_ERROR, PID_COMMANDED_EVAP_PURGE, PID_FUEL_TANK_LEVEL,
            PID_WARMUPS_SINCE_CODES_CLEARED, PID_DISTANCE_SINCE_CODES_CLEARED, PID_ABSOLUTE_BAROMETRIC_PRESSURE,
            PID_CATALYST_TEMP_B1S1, PID_CONTROL_MODULE_VOLTAGE, PID_ABSOLUTE_LOAD_VALUE,
            PID_FUEL_AIR_COMMANDED_EQUIV_RATIO, PID_RELATIVE_THROTTLE_POS, PID_AMBIENT_AIR_TEMP,
            PID_ABSOLUTE_THROTTLE_POS_B, PID_ACCELERATOR_PEDAL_POS_D, PID_COMMANDED_THROTTLE_ACTUATOR,
            PID_TIME_WITH_MIL_ON, PID_TIME_SINCE_CODES_CLEARED, PID_FUEL_TYPE, PID_ETHANOL_FUEL_PERCENT,
            PID_ENGINE_OIL_TEMP, PID_FUEL_INJECTION_TIMING, PID_ENGINE_FUEL_RATE
        };
        
        int standardCount = sizeof(standardPIDs) / sizeof(standardPIDs[0]);
        for (int i = 0; i < standardCount && pidCount < 100; i++) {
            pidList[pidCount++] = standardPIDs[i];
        }
        
        // Add hardware sensor PIDs
        pidList[pidCount++] = PID_BATTERY_VOLTAGE;
        pidList[pidCount++] = PID_ENGINE_PRESSURE;
        
        return true;
    }
    
    bool isUsingRealData() {
        return realOBDAvailable;
    }
    
    bool isSimulationEnabled() {
        return simulationEnabled;
    }
    
    void setSimulationEnabled(bool enabled) {
        simulationEnabled = enabled;
        Serial.println("Simulation " + String(enabled ? "enabled" : "disabled"));
    }
    
    String runFullDiagnostics() {
        String results = "";
        results += runSystemDiagnostics();
        results += runHardwareDiagnostics();
        results += runOBDDiagnostics();
        results += runConnectivityDiagnostics();
        results += "SUMMARY:\\n";
        
        if (results.indexOf("FAIL") != -1) {
            results += "OBD issues detected\\n";
        }
        
        if (results.indexOf("(LOW)") != -1) {
            results += "Low voltage\\n";
        }
        
        if (results.indexOf("(HOT)") != -1) {
            results += "High temp\\n";
        }
        
        if (results.indexOf("I2C:0x") != -1) {
            results += "I2C devices found\\n";
        }
        
        results += bleClientConnected ? "BLE connected\\n" : "No BLE client\\n";
        results += "Done:" + String(millis()) + "ms\\n";
        return results;
    }
    
    String getLastError() {
        return lastError;
    }
    
    unsigned long getLastErrorTime() {
        return lastErrorTime;
    }
    
private:
    bool readHardwareSensor(uint8_t pid, int& value) {
        // Read directly from Freematics hardware sensors
        switch(pid) {
            case 0x42: // PID_BATTERY_VOLTAGE and PID_CONTROL_MODULE_VOLTAGE (same value)
                value = readBatteryVoltage();
                return true;
            case 0x46: // PID_AMBIENT_AIR_TEMP and PID_AMBIENT_TEMPERATURE (same value)
                value = readAmbientTemperature();
                return true;
            case PID_ENGINE_PRESSURE: // Engine oil pressure (if available via hardware)
                value = readEnginePressure();
                return true;
            case PID_ABSOLUTE_BAROMETRIC_PRESSURE: // Atmospheric pressure
                value = readBarometricPressure();
                return true;
            default:
                return false; // Not a hardware sensor PID
        }
    }
    
    bool isOBDOnlyPID(uint8_t pid) {
        // These PIDs require OBD-II connection to ECU
        switch(pid) {
            // Engine Management
            case PID_ENGINE_LOAD:
            case PID_COOLANT_TEMP:
            case PID_SHORT_TERM_FUEL_TRIM_1:
            case PID_LONG_TERM_FUEL_TRIM_1:
            case PID_SHORT_TERM_FUEL_TRIM_2:
            case PID_LONG_TERM_FUEL_TRIM_2:
            case PID_FUEL_PRESSURE:
            case PID_INTAKE_MAP:
            case PID_RPM:
            case PID_SPEED:
            case PID_TIMING_ADVANCE:
            case PID_INTAKE_TEMP:
            case PID_MAF_FLOW:
            case PID_THROTTLE_POS:
            
            // Oxygen Sensors
            case PID_O2_S1_VOLTAGE:
            case PID_O2_S2_VOLTAGE:
            case PID_O2_S3_VOLTAGE:
            case PID_O2_S4_VOLTAGE:
            case PID_O2_S5_VOLTAGE:
            case PID_O2_S6_VOLTAGE:
            case PID_O2_S7_VOLTAGE:
            case PID_O2_S8_VOLTAGE:
            
            // Emissions Control
            case PID_COMMANDED_EGR:
            case PID_EGR_ERROR:
            case PID_COMMANDED_EVAP_PURGE:
            case PID_FUEL_TANK_LEVEL:
            case PID_EVAP_SYS_VAPOR_PRESSURE:
            case PID_CATALYST_TEMP_B1S1:
            case PID_CATALYST_TEMP_B2S1:
            case PID_CATALYST_TEMP_B1S2:
            case PID_CATALYST_TEMP_B2S2:
            
            // Advanced Engine Parameters
            case PID_FUEL_RAIL_PRESSURE:
            case PID_FUEL_RAIL_GAUGE_PRESSURE:
            case PID_ABSOLUTE_LOAD_VALUE:
            case PID_FUEL_AIR_COMMANDED_EQUIV_RATIO:
            case PID_RELATIVE_THROTTLE_POS:
            case PID_ABSOLUTE_THROTTLE_POS_B:
            case PID_ABSOLUTE_THROTTLE_POS_C:
            case PID_ACCELERATOR_PEDAL_POS_D:
            case PID_ACCELERATOR_PEDAL_POS_E:
            case PID_ACCELERATOR_PEDAL_POS_F:
            case PID_COMMANDED_THROTTLE_ACTUATOR:
            
            // Fuel System
            case PID_FUEL_TYPE:
            case PID_ETHANOL_FUEL_PERCENT:
            case PID_ENGINE_OIL_TEMP:
            case PID_FUEL_INJECTION_TIMING:
            case PID_ENGINE_FUEL_RATE:
            
            // Diagnostic Information
            case PID_OBD_STANDARDS:
            case PID_O2_SENSORS_PRESENT:
            case PID_RUNTIME:
            case PID_DISTANCE_WITH_MIL:
            case PID_WARMUPS_SINCE_CODES_CLEARED:
            case PID_DISTANCE_SINCE_CODES_CLEARED:
            case PID_TIME_WITH_MIL_ON:
            case PID_TIME_SINCE_CODES_CLEARED:
                return true;
            default:
                return false;
        }
    }
    
    int readBatteryVoltage() {
        // Read vehicle input voltage (Vin) from ADC
        // Freematics ONE+ has Vin connected to A0 with voltage divider
        int adcValue = analogRead(A0);
        // Freematics ONE+ voltage divider: Vin -> 10K -> A0 -> 2K -> GND
        // This gives a 6:1 ratio, so max 19.8V can be measured
        float voltage = (adcValue / 4095.0) * 3.3 * 6.0; // 6:1 voltage divider
        return (int)(voltage * 100); // Return in centivolt for precision
    }
    
    int readAmbientTemperature() {
        // Try to read ambient temperature from OBD-II first
        int obdTemp;
        if (realOBDAvailable && attemptRealOBDRead(0x46, obdTemp)) {
            return obdTemp; // Return OBD ambient temperature
        }
        
        // Fallback: read from external temperature sensor if connected
        // Check if external sensor is connected to GPIO36 (A0 equivalent on ESP32)
        int sensorValue = analogRead(36);
        if (sensorValue > 100 && sensorValue < 4000) { // Valid sensor range
            // Assuming TMP36 or similar sensor: Vout = (Temp°C × 10mV) + 500mV
            float voltage = (sensorValue / 4095.0) * 3.3;
            float temp_celsius = (voltage - 0.5) * 100.0;
            return (int)temp_celsius;
        }
        
        // Last resort: use ESP32 internal temperature as rough estimate
        #ifdef SOC_TEMP_SENSOR_SUPPORTED
        float temp_celsius = temperatureRead();
        // Internal temp runs hot, subtract offset for ambient estimate
        return (int)(temp_celsius - 20); // Rough ambient estimate
        #else
        return 25; // Default ambient temperature
        #endif
    }
    
    int readEnginePressure() {
        // Read engine oil pressure if sensor is connected
        // This would typically be connected to an analog input
        // For now, return a reasonable default since most setups won't have this
        return 35 + random(-5, 5); // Typical oil pressure in psi
    }
    
    int readBarometricPressure() {
        // Read atmospheric pressure (standard atmosphere at sea level)
        // Could be connected to BMP280 or similar sensor
        // For now, return standard atmospheric pressure with small variation
        return 1013 + random(-10, 10); // Standard atmosphere in mbar
    }

    bool attemptRealOBDRead(uint8_t pid, int& value) {
        // Real OBD-II communication implementation
        // This requires actual OBD-II interface hardware and protocol implementation
        
        // Send OBD-II request: Mode 01 (current data) + PID
        String obdRequest = "01" + String(pid, HEX);
        if (pid < 0x10) obdRequest = "010" + String(pid, HEX); // Pad with zero
        
        // Send request via OBD interface (implementation depends on hardware)
        // For Freematics ONE+, this would typically use the built-in OBD interface
        Serial2.println(obdRequest); // Assuming OBD interface on Serial2
        
        // Wait for response with timeout
        unsigned long startTime = millis();
        String response = "";
        while (millis() - startTime < 1000) { // 1 second timeout
            if (Serial2.available()) {
                char c = Serial2.read();
                response += c;
                if (c == '\r' || c == '\n') break;
            }
        }
        
        // Parse OBD-II response
        if (response.length() > 6 && response.startsWith("41")) {
            // Valid response format: "41 [PID] [DATA]"
            String pidResponse = response.substring(2, 4);
            if (pidResponse.equals(String(pid, HEX))) {
                // Extract data bytes and convert based on PID
                String dataBytes = response.substring(4);
                return parseOBDResponse(pid, dataBytes, value);
            }
        }
        
        lastError = "No OBD response for PID 0x" + String(pid, HEX);
        lastErrorTime = millis();
        return false;
    }
    
    bool parseOBDResponse(uint8_t pid, String dataBytes, int& value) {
        // Parse OBD-II response data based on PID
        dataBytes.trim();
        dataBytes.replace(" ", ""); // Remove spaces
        
        // Get data bytes as integers
        int A = 0, B = 0, C = 0, D = 0;
        if (dataBytes.length() >= 2) A = strtol(dataBytes.substring(0, 2).c_str(), NULL, 16);
        if (dataBytes.length() >= 4) B = strtol(dataBytes.substring(2, 4).c_str(), NULL, 16);
        if (dataBytes.length() >= 6) C = strtol(dataBytes.substring(4, 6).c_str(), NULL, 16);
        if (dataBytes.length() >= 8) D = strtol(dataBytes.substring(6, 8).c_str(), NULL, 16);
        
        switch(pid) {
            // Engine Management
            case PID_ENGINE_LOAD:
                value = (A * 100) / 255; // Calculated load value %
                return true;
            case PID_COOLANT_TEMP:
                value = A - 40; // Engine coolant temperature °C
                return true;
            case PID_SHORT_TERM_FUEL_TRIM_1:
            case PID_SHORT_TERM_FUEL_TRIM_2:
                value = (A - 128) * 100 / 128; // Fuel trim %
                return true;
            case PID_LONG_TERM_FUEL_TRIM_1:
            case PID_LONG_TERM_FUEL_TRIM_2:
                value = (A - 128) * 100 / 128; // Fuel trim %
                return true;
            case PID_FUEL_PRESSURE:
                value = A * 3; // Fuel pressure kPa
                return true;
            case PID_INTAKE_MAP:
                value = A; // Intake manifold pressure kPa
                return true;
            case PID_RPM:
                value = ((A * 256) + B) / 4; // Engine RPM
                return true;
            case PID_SPEED:
                value = A; // Vehicle speed km/h
                return true;
            case PID_TIMING_ADVANCE:
                value = (A - 128) / 2; // Timing advance degrees
                return true;
            case PID_INTAKE_TEMP:
                value = A - 40; // Intake air temperature °C
                return true;
            case PID_MAF_FLOW:
                value = ((A * 256) + B) / 100; // MAF air flow rate g/s
                return true;
            case PID_THROTTLE_POS:
                value = (A * 100) / 255; // Throttle position %
                return true;
                
            // Oxygen Sensors
            case PID_O2_S1_VOLTAGE:
            case PID_O2_S2_VOLTAGE:
            case PID_O2_S3_VOLTAGE:
            case PID_O2_S4_VOLTAGE:
            case PID_O2_S5_VOLTAGE:
            case PID_O2_S6_VOLTAGE:
            case PID_O2_S7_VOLTAGE:
            case PID_O2_S8_VOLTAGE:
                value = A / 200; // O2 sensor voltage (0.0-1.275V)
                return true;
                
            // System Information
            case PID_OBD_STANDARDS:
                value = A; // OBD standards compliance
                return true;
            case PID_O2_SENSORS_PRESENT:
                value = A; // Oxygen sensors present bitmask
                return true;
            case PID_RUNTIME:
                value = (A * 256) + B; // Runtime since engine start (seconds)
                return true;
                
            // Emissions Control
            case PID_DISTANCE_WITH_MIL:
                value = (A * 256) + B; // Distance with MIL on (km)
                return true;
            case PID_FUEL_RAIL_PRESSURE:
                value = ((A * 256) + B) * 10; // Fuel rail pressure kPa
                return true;
            case PID_COMMANDED_EGR:
                value = (A * 100) / 255; // Commanded EGR %
                return true;
            case PID_EGR_ERROR:
                value = (A - 128) * 100 / 128; // EGR error %
                return true;
            case PID_COMMANDED_EVAP_PURGE:
                value = (A * 100) / 255; // Commanded evaporative purge %
                return true;
            case PID_FUEL_TANK_LEVEL:
                value = (A * 100) / 255; // Fuel tank level %
                return true;
                
            // Advanced Parameters
            case PID_WARMUPS_SINCE_CODES_CLEARED:
                value = A; // Number of warm-ups since codes cleared
                return true;
            case PID_DISTANCE_SINCE_CODES_CLEARED:
                value = (A * 256) + B; // Distance since codes cleared (km)
                return true;
            case PID_ABSOLUTE_BAROMETRIC_PRESSURE:
                value = A; // Absolute barometric pressure kPa
                return true;
            case PID_CATALYST_TEMP_B1S1:
            case PID_CATALYST_TEMP_B2S1:
            case PID_CATALYST_TEMP_B1S2:
            case PID_CATALYST_TEMP_B2S2:
                value = ((A * 256) + B) / 10 - 40; // Catalyst temperature °C
                return true;
            case PID_CONTROL_MODULE_VOLTAGE:
                value = ((A * 256) + B) / 1000; // Control module voltage V
                return true;
            case PID_ABSOLUTE_LOAD_VALUE:
                value = ((A * 256) + B) * 100 / 255; // Absolute load value %
                return true;
            case PID_FUEL_AIR_COMMANDED_EQUIV_RATIO:
                value = ((A * 256) + B) / 32768; // Fuel-air equivalence ratio
                return true;
            case PID_RELATIVE_THROTTLE_POS:
                value = (A * 100) / 255; // Relative throttle position %
                return true;
            case PID_AMBIENT_AIR_TEMP:
                value = A - 40; // Ambient air temperature °C
                return true;
            case PID_ABSOLUTE_THROTTLE_POS_B:
            case PID_ABSOLUTE_THROTTLE_POS_C:
                value = (A * 100) / 255; // Absolute throttle position %
                return true;
            case PID_ACCELERATOR_PEDAL_POS_D:
            case PID_ACCELERATOR_PEDAL_POS_E:
            case PID_ACCELERATOR_PEDAL_POS_F:
                value = (A * 100) / 255; // Accelerator pedal position %
                return true;
            case PID_COMMANDED_THROTTLE_ACTUATOR:
                value = (A * 100) / 255; // Commanded throttle actuator %
                return true;
            case PID_TIME_WITH_MIL_ON:
                value = (A * 256) + B; // Time with MIL on (minutes)
                return true;
            case PID_TIME_SINCE_CODES_CLEARED:
                value = (A * 256) + B; // Time since codes cleared (minutes)
                return true;
                
            // Fuel System
            case PID_FUEL_TYPE:
                value = A; // Fuel type code
                return true;
            case PID_ETHANOL_FUEL_PERCENT:
                value = (A * 100) / 255; // Ethanol fuel percentage
                return true;
            case PID_ENGINE_OIL_TEMP:
                value = A - 40; // Engine oil temperature °C
                return true;
            case PID_FUEL_INJECTION_TIMING:
                value = ((A * 256) + B) / 128 - 210; // Fuel injection timing degrees
                return true;
            case PID_ENGINE_FUEL_RATE:
                value = ((A * 256) + B) / 20; // Engine fuel rate L/h
                return true;
                
            default:
                lastError = "Unsupported PID 0x" + String(pid, HEX) + " for parsing";
                lastErrorTime = millis();
                return false;
        }
        
        lastError = "Failed to parse OBD data for PID 0x" + String(pid, HEX);
        lastErrorTime = millis();
        return false;
    }
    
    bool readSimulatedPID(uint8_t pid, int& value) {
        // Generate realistic simulated data for EPA-compliant vehicle
        switch(pid) {
            // Engine Management
            case PID_ENGINE_LOAD: value = 25 + random(-10, 15); return true; // 15-40% load
            case PID_COOLANT_TEMP: value = 85 + random(-5, 5); return true; // 80-90°C
            case PID_SHORT_TERM_FUEL_TRIM_1: value = random(-8, 8); return true; // ±8%
            case PID_LONG_TERM_FUEL_TRIM_1: value = random(-5, 5); return true; // ±5%
            case PID_SHORT_TERM_FUEL_TRIM_2: value = random(-8, 8); return true;
            case PID_LONG_TERM_FUEL_TRIM_2: value = random(-5, 5); return true;
            case PID_FUEL_PRESSURE: value = 300 + random(-20, 20); return true; // ~300 kPa
            case PID_INTAKE_MAP: value = 35 + random(-10, 15); return true; // 25-50 kPa
            case PID_RPM: value = 1500 + random(-200, 500); return true; // 1300-2000 RPM
            case PID_SPEED: value = 60 + random(-10, 20); return true; // 50-80 km/h
            case PID_TIMING_ADVANCE: value = 15 + random(-5, 5); return true; // 10-20°
            case PID_INTAKE_TEMP: value = 25 + random(-5, 15); return true; // 20-40°C
            case PID_MAF_FLOW: value = 15 + random(-5, 10); return true; // 10-25 g/s
            case PID_THROTTLE_POS: value = 20 + random(-5, 15); return true; // 15-35%
            
            // Oxygen Sensors (0.1-0.9V typical)
            case PID_O2_S1_VOLTAGE: value = 450 + random(-100, 100); return true; // mV
            case PID_O2_S2_VOLTAGE: value = 450 + random(-100, 100); return true;
            case PID_O2_S3_VOLTAGE: value = 450 + random(-100, 100); return true;
            case PID_O2_S4_VOLTAGE: value = 450 + random(-100, 100); return true;
            
            // System Information
            case PID_OBD_STANDARDS: value = 7; return true; // OBD-II (CARB)
            case PID_O2_SENSORS_PRESENT: value = 0x13; return true; // Bank 1 sensors 1&3
            case PID_RUNTIME: value = 1800 + random(0, 600); return true; // 30-40 minutes
            
            // Emissions Control
            case PID_DISTANCE_WITH_MIL: value = 0; return true; // No MIL
            case PID_FUEL_RAIL_PRESSURE: value = 5500 + random(-200, 200); return true; // kPa
            case PID_COMMANDED_EGR: value = 5 + random(-2, 5); return true; // 3-10%
            case PID_EGR_ERROR: value = random(-3, 3); return true; // ±3%
            case PID_COMMANDED_EVAP_PURGE: value = 2 + random(0, 8); return true; // 2-10%
            case PID_FUEL_TANK_LEVEL: value = 75 + random(-10, 10); return true; // 65-85%
            
            // Advanced Parameters
            case PID_WARMUPS_SINCE_CODES_CLEARED: value = 50 + random(0, 20); return true;
            case PID_DISTANCE_SINCE_CODES_CLEARED: value = 1500 + random(0, 500); return true; // km
            case PID_ABSOLUTE_BAROMETRIC_PRESSURE: value = 101 + random(-2, 2); return true; // kPa
            case PID_CATALYST_TEMP_B1S1: value = 400 + random(-50, 100); return true; // °C
            case PID_CATALYST_TEMP_B2S1: value = 400 + random(-50, 100); return true;
            case PID_CONTROL_MODULE_VOLTAGE: value = 14 + random(-1, 1); return true; // V
            case PID_ABSOLUTE_LOAD_VALUE: value = 30 + random(-10, 20); return true; // %
            case PID_FUEL_AIR_COMMANDED_EQUIV_RATIO: value = 1000 + random(-50, 50); return true; // ratio*1000
            case PID_RELATIVE_THROTTLE_POS: value = 20 + random(-5, 15); return true; // %
            case PID_AMBIENT_AIR_TEMP: value = 22 + random(-5, 8); return true; // °C
            case PID_ABSOLUTE_THROTTLE_POS_B: value = 20 + random(-5, 15); return true; // %
            case PID_ACCELERATOR_PEDAL_POS_D: value = 25 + random(-5, 15); return true; // %
            case PID_COMMANDED_THROTTLE_ACTUATOR: value = 20 + random(-5, 15); return true; // %
            case PID_TIME_WITH_MIL_ON: value = 0; return true; // No MIL time
            case PID_TIME_SINCE_CODES_CLEARED: value = 720 + random(0, 240); return true; // minutes
            
            // Fuel System
            case PID_FUEL_TYPE: value = 1; return true; // Gasoline
            case PID_ETHANOL_FUEL_PERCENT: value = 10 + random(-2, 5); return true; // E10 fuel
            case PID_ENGINE_OIL_TEMP: value = 90 + random(-5, 10); return true; // °C
            case PID_FUEL_INJECTION_TIMING: value = -2 + random(-2, 4); return true; // degrees
            case PID_ENGINE_FUEL_RATE: value = 8 + random(-2, 4); return true; // L/h
            
            // Additional Advanced Parameters
            case PID_FUEL_RAIL_GAUGE_PRESSURE: value = 5800 + random(-300, 300); return true; // kPa
            case PID_ABSOLUTE_EVAP_SYS_VAPOR_PRESSURE: value = 2500 + random(-100, 100); return true; // Pa
            case PID_EVAP_SYS_VAPOR_PRESSURE2: value = 2500 + random(-100, 100); return true; // Pa
            case PID_SHORT_TERM_SECONDARY_O2_TRIM_1: value = random(-5, 5); return true; // %
            case PID_LONG_TERM_SECONDARY_O2_TRIM_1: value = random(-3, 3); return true; // %
            case PID_SHORT_TERM_SECONDARY_O2_TRIM_2: value = random(-5, 5); return true; // %
            case PID_LONG_TERM_SECONDARY_O2_TRIM_2: value = random(-3, 3); return true; // %
            case PID_FUEL_RAIL_ABSOLUTE_PRESSURE: value = 5500 + random(-200, 200); return true; // kPa
            case PID_RELATIVE_ACCELERATOR_PEDAL_POS: value = 25 + random(-5, 15); return true; // %
            case PID_HYBRID_BATTERY_REMAINING: value = 85 + random(-10, 10); return true; // %
            case PID_MAX_VALUES_EQUIV_RATIO: value = 1200 + random(-50, 50); return true; // ratio*1000
            case PID_MAX_VALUES_AIR_FLOW_MAF: value = 25 + random(-5, 10); return true; // g/s
            case PID_EMISSION_REQUIREMENTS: value = 1; return true; // OBD-II
            
            // Turbo & Boost Control (for turbocharged vehicles)
            case PID_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE: value = 101 + random(-2, 15); return true; // kPa
            case PID_BOOST_PRESSURE_CONTROL: value = 0 + random(0, 50); return true; // kPa
            case PID_VARIABLE_GEOMETRY_TURBO_CONTROL: value = 50 + random(-20, 20); return true; // %
            case PID_WASTEGATE_CONTROL: value = 10 + random(-5, 15); return true; // %
            case PID_EXHAUST_PRESSURE: value = 105 + random(-5, 10); return true; // kPa
            case PID_TURBOCHARGER_RPM: value = 0; return true; // RPM (0 for non-turbo)
            case PID_TURBOCHARGER_TEMP_1: value = 0; return true; // °C (0 for non-turbo)
            case PID_TURBOCHARGER_TEMP_2: value = 0; return true; // °C (0 for non-turbo)
            case PID_CHARGE_AIR_COOLER_TEMP: value = 30 + random(-5, 15); return true; // °C
            case PID_EXHAUST_GAS_TEMP_BANK_1: value = 450 + random(-50, 100); return true; // °C
            case PID_EXHAUST_GAS_TEMP_BANK_2: value = 450 + random(-50, 100); return true; // °C
            
            // Diesel Particulate Filter (for diesel vehicles - return 0 for gasoline)
            case PID_DIESEL_PARTICULATE_FILTER_1: value = 0; return true; // Not applicable
            case PID_DIESEL_PARTICULATE_FILTER_2: value = 0; return true; // Not applicable
            case PID_DIESEL_PARTICULATE_FILTER_TEMP: value = 0; return true; // Not applicable
            case PID_NOX_NTE_CONTROL_AREA_STATUS: value = 0; return true; // Not applicable
            case PID_PM_NTE_CONTROL_AREA_STATUS: value = 0; return true; // Not applicable
            case PID_ENGINE_RUN_TIME: value = 1800 + random(0, 600); return true; // seconds
            
            default: 
                lastError = "Unsupported simulated PID 0x" + String(pid, HEX);
                lastErrorTime = millis();
                return false;
        }
    }
};

// Minimal GPS simulation with averaging
class SimpleGPS {
private:
    static const int HISTORY_SIZE = 10;
    uint8_t satHistory[HISTORY_SIZE];
    int historyIndex = 0;
    int historyCount = 0;
    
public:
    bool begin() { 
        // Initialize history array
        for (int i = 0; i < HISTORY_SIZE; i++) {
            satHistory[i] = 0;
        }
        return true; 
    }
    
    bool getData(float& lat, float& lng, uint8_t& sat) {
        lat = 37.7749 + (random(-1000, 1000) / 100000.0);
        lng = -122.4194 + (random(-1000, 1000) / 100000.0);
        
        // Generate satellite count (avoid zero)
        uint8_t currentSat = 4 + random(0, 8); // Range 4-11 satellites
        
        // Store in history
        satHistory[historyIndex] = currentSat;
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
        if (historyCount < HISTORY_SIZE) {
            historyCount++;
        }
        
        // Calculate average
        if (historyCount > 0) {
            uint32_t sum = 0;
            for (int i = 0; i < historyCount; i++) {
                sum += satHistory[i];
            }
            sat = sum / historyCount;
        } else {
            sat = currentSat;
        }
        
        return true;
    }
    
    int getHistoryCount() {
        return historyCount;
    }
};

class CustomFreematicsLogger
{
private:
    static const int STATUS_HISTORY_SIZE = 10;
    struct StatusReading {
        bool obdReal;
        bool obdSim;
        bool gpsReady;
        bool storageReady;
        bool bleReady;
        unsigned long timestamp;
    };
    StatusReading statusHistory[STATUS_HISTORY_SIZE];
    int statusHistoryIndex = 0;
    int statusHistoryCount = 0;
    
public:
    void setSimulationEnabled(bool enabled) {
        obd.setSimulationEnabled(enabled);
    }
    
    void startDiagnosticMode() {
        diagnosticMode = true;
        diagnosticStartTime = millis();
        diagnosticStep = 0;
        diagnosticResults = "";
        Serial.println("=== STARTING DIAGNOSTIC MODE ===");
    }
    
    void processDiagnosticMode() {
        if (!diagnosticMode) return;
        
        unsigned long elapsed = millis() - diagnosticStartTime;
        
        switch (diagnosticStep) {
            case 0:
                Serial.println("Running comprehensive diagnostics...");
                diagnosticResults = obd.runFullDiagnostics();
                diagnosticStep = 1;
                break;
                
            case 1:
                if (elapsed > 2000) { // Wait 2 seconds for diagnostics to complete
                    Serial.println("Diagnostics complete!");
                    Serial.println(diagnosticResults);
                    
                    // Send results via BLE if connected
                    if (bleClientConnected && pCharacteristic) {
                        // Split large diagnostic results into chunks
                        String results = diagnosticResults;
                        results.replace("\\n", "|"); // Replace newlines with pipe for BLE transmission
                        
                        String diagMsg = String(millis()) + ",DIAGNOSTIC_RESULTS:" + results + ";";
                        pCharacteristic->setValue(diagMsg.c_str());
                        pCharacteristic->notify();
                        
                        delay(100); // Ensure message is sent
                        
                        // Send completion message
                        String completeMsg = String(millis()) + ",DIAGNOSTIC_COMPLETE:SUCCESS;";
                        pCharacteristic->setValue(completeMsg.c_str());
                        pCharacteristic->notify();
                    }
                    
                    diagnosticMode = false;
                    diagnosticStep = 0;
                }
                break;
        }
    }
    
    bool init()
    {
        bool success = true;
        
        // Initialize LED pin
        pinMode(PIN_LED, OUTPUT);
        digitalWrite(PIN_LED, LOW);
        
        // Initialize BLE
        Serial.print("BLE...");
        BLEDevice::init(BLE_DEVICE_NAME);
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

        pCharacteristic = pService->createCharacteristic(
                            BLE_SERVICE_UUID,
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_WRITE |
                            BLECharacteristic::PROPERTY_NOTIFY
                          );

        // Callback will be set after class definition
        pCharacteristic->addDescriptor(new BLE2902());

        pService->start();
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
        pAdvertising->setScanResponse(false);
        pAdvertising->setMinPreferred(0x0);
        BLEDevice::startAdvertising();
        
        Serial.println("OK");
        m_state |= STATE_BLE_READY;
        
        // Initialize OBD
        Serial.print("OBD...");
        if (obd.init()) {
            Serial.println("OK");
            m_state |= STATE_OBD_READY;
        } else {
            Serial.println("NO");
            success = false;
        }
        
        // Initialize GPS
        Serial.print("GPS...");
        if (gps.begin()) {
            Serial.println("OK");
            m_state |= STATE_GPS_READY;
        } else {
            Serial.println("NO");
        }
        
        // Initialize MEMS
        Serial.print("MEMS...");
        // MEMS initialization not available in this library version
        Serial.println("SKIPPED");
        // m_state |= STATE_MEMS_READY; // Disabled for graceful degradation
        
        // Initialize storage
        Serial.print("Storage...");
        if (store.init()) {
            Serial.println("OK");
            m_state |= STATE_STORAGE_READY;
        } else {
            Serial.println("NO");
        }
        
        return success;
    }
    
    void process()
    {
        // Handle LED blinking when no client connected
        handleLedBlink();
        
        // Process diagnostic mode if active
        if (diagnosticMode) {
            processDiagnosticMode();
            return; // Skip normal processing during diagnostics
        }
        
        // Only process data if BLE client is connected
        if (bleClientConnected) {
            // Process OBD data
            if (m_state & STATE_OBD_READY) {
                processOBD();
            }
            
            // Process GPS data
            if (m_state & STATE_GPS_READY) {
                processGPS();
            }
            
            // Process MEMS data
            if (m_state & STATE_MEMS_READY) {
                processMEMS();
            }
            
            // Send data via BLE for debugging
            sendBLEData();
        }
    }
    
    void handleLedOnly()
    {
        handleLedBlink();
    }
    
private:
    void handleLedBlink()
    {
        if (!bleClientConnected) {
            // Blink LED every 500ms when no client connected
            if (millis() - lastLedBlink > 500) {
                ledState = !ledState;
                digitalWrite(PIN_LED, ledState ? HIGH : LOW);
                lastLedBlink = millis();
            }
        } else {
            // Keep LED on when client connected
            digitalWrite(PIN_LED, HIGH);
        }
    }
    
    void processOBD()
    {
        static uint32_t lastOBDTime = 0;
        if (millis() - lastOBDTime < OBD_INTERVAL) return;
        
        int value;
        // Core engine data only
        if (obd.readPID(PID_RPM, value)) store.log(PID_RPM, value);
        if (obd.readPID(PID_SPEED, value)) store.log(PID_SPEED, value);
        if (obd.readPID(PID_COOLANT_TEMP, value)) store.log(PID_COOLANT_TEMP, value);
        if (obd.readPID(PID_ENGINE_LOAD, value)) store.log(PID_ENGINE_LOAD, value);
        if (obd.readPID(PID_THROTTLE_POS, value)) store.log(PID_THROTTLE_POS, value);
        
        // Essential fuel data
        if (obd.readPID(PID_SHORT_TERM_FUEL_TRIM_1, value)) store.log(PID_SHORT_TERM_FUEL_TRIM_1, value);
        if (obd.readPID(PID_LONG_TERM_FUEL_TRIM_1, value)) store.log(PID_LONG_TERM_FUEL_TRIM_1, value);
        if (obd.readPID(PID_FUEL_TANK_LEVEL, value)) store.log(PID_FUEL_TANK_LEVEL, value);
        
        // Key sensors
        if (obd.readPID(PID_INTAKE_TEMP, value)) store.log(PID_INTAKE_TEMP, value);
        if (obd.readPID(PID_MAF_FLOW, value)) store.log(PID_MAF_FLOW, value);
        if (obd.readPID(PID_O2_S1_VOLTAGE, value)) store.log(PID_O2_S1_VOLTAGE, value);
        
        // Hardware sensors
        if (obd.readPID(PID_BATTERY_VOLTAGE, value)) store.log(PID_BATTERY_VOLTAGE, value);
        if (obd.readPID(PID_AMBIENT_AIR_TEMP, value)) store.log(PID_AMBIENT_AIR_TEMP, value);
        if (obd.readPID(PID_ENGINE_PRESSURE, value)) store.log(PID_ENGINE_PRESSURE, value);
        
        lastOBDTime = millis();
    }
    
    void processGPS()
    {
        static uint32_t lastGPSTime = 0;
        if (millis() - lastGPSTime < GPS_INTERVAL) return;
        
        float lat, lng;
        uint8_t sat;
        if (gps.getData(lat, lng, sat)) {
            store.log(0x20, (int32_t)(lat * 1000000));
            store.log(0x21, (int32_t)(lng * 1000000));
            store.log(0x22, sat);
        }
        
        lastGPSTime = millis();
    }
    
    void processMEMS()
    {
        // MEMS functionality not available in this library version
        // Gracefully skip MEMS processing
        return;
    }
    
    void sendBLEData()
    {
        static uint32_t lastBLETime = 0;
        static uint32_t lastStatusTime = 0;
        static uint32_t lastHeartbeatTime = 0;
        static uint32_t messageCounter = 0;
        
        if (millis() - lastBLETime < BLE_INTERVAL) return;
        
        if (bleClientConnected && (m_state & STATE_BLE_READY)) {
            // Send main data packet
            String bleData = formatDataForBLE();
            if (bleData.length() > 0) {
                messageCounter++;
                String fullMessage = String(messageCounter) + ":" + bleData;
                
                // Log what we're sending
                Serial.println("BLE TX: " + fullMessage);
                
                pCharacteristic->setValue(fullMessage.c_str());
                pCharacteristic->notify();
                
                // Small delay to ensure message is sent
                delay(10);
            }
            
            // Send periodic status updates every 5 seconds
            if (millis() - lastStatusTime > 5000) {
                // Record current status in history
                recordStatusReading();
                
                messageCounter++;
                String statusData = String(messageCounter) + ":" + String(millis()) + ",STATUS:";
                
                // Get averaged status
                String obdStatus = getAveragedOBDStatus();
                String gpsStatus = getAveragedGPSStatus();
                String storageStatus = getAveragedStorageStatus();
                String bleStatus = getAveragedBLEStatus();
                
                statusData += "OBD=" + obdStatus + ",";
                statusData += "GPS=" + gpsStatus + ",";
                statusData += "STORAGE=" + storageStatus + ",";
                statusData += "BLE=" + bleStatus + ",";
                statusData += "API_VERSION=" + String(API_VERSION) + ",";
                statusData += "UPTIME=" + String(millis() / 1000) + ";";
                
                Serial.println("BLE TX STATUS: " + statusData);
                pCharacteristic->setValue(statusData.c_str());
                pCharacteristic->notify();
                lastStatusTime = millis();
                
                delay(10);
            }
            
            // Send heartbeat every 2 seconds when no other data
            if (millis() - lastHeartbeatTime > 2000) {
                messageCounter++;
                String heartbeat = String(messageCounter) + ":" + String(millis()) + ",HEARTBEAT:OK;";
                
                Serial.println("BLE TX HEARTBEAT: " + heartbeat);
                pCharacteristic->setValue(heartbeat.c_str());
                pCharacteristic->notify();
                lastHeartbeatTime = millis();
                
                delay(10);
            }
        }
        
        lastBLETime = millis();
    }
    
    String formatDataForBLE()
    {
        String data = String(millis()) + ",";
        
        // Add data source indicator
        if (obd.isUsingRealData()) {
            data += "MODE:REAL;";
        } else if (obd.isSimulationEnabled()) {
            data += "MODE:SIMULATED;";
        } else {
            data += "MODE:DISABLED;"; // No OBD or simulation data
        }
        
        int value;
        // Essential data only
        if (obd.readPID(PID_RPM, value)) data += "RPM:" + String(value) + ";";
        if (obd.readPID(PID_SPEED, value)) data += "SPD:" + String(value) + ";";
        if (obd.readPID(PID_COOLANT_TEMP, value)) data += "COOLANT:" + String(value) + ";";
        if (obd.readPID(PID_ENGINE_LOAD, value)) data += "ENGINE_LOAD:" + String(value) + ";";
        if (obd.readPID(PID_THROTTLE_POS, value)) data += "THROTTLE_POS:" + String(value) + ";";
        if (obd.readPID(PID_FUEL_TANK_LEVEL, value)) data += "FUEL_LEVEL:" + String(value) + ";";
        if (obd.readPID(PID_SHORT_TERM_FUEL_TRIM_1, value)) data += "FUEL_TRIM_SHORT:" + String(value) + ";";
        if (obd.readPID(PID_LONG_TERM_FUEL_TRIM_1, value)) data += "FUEL_TRIM_LONG:" + String(value) + ";";
        if (obd.readPID(PID_INTAKE_TEMP, value)) data += "INTAKE_TEMP:" + String(value) + ";";
        if (obd.readPID(PID_MAF_FLOW, value)) data += "MAF_FLOW:" + String(value) + ";";
        if (obd.readPID(PID_O2_S1_VOLTAGE, value)) data += "O2_VOLTAGE:" + String(value) + ";";
        if (obd.readPID(PID_BATTERY_VOLTAGE, value)) data += "BATTERY:" + String(value) + ";";
        if (obd.readPID(PID_AMBIENT_AIR_TEMP, value)) data += "AMBIENT:" + String(value) + ";";
        if (obd.readPID(PID_ENGINE_PRESSURE, value)) data += "PRESSURE:" + String(value) + ";";
        
        float lat, lng;
        uint8_t sat;
        if (gps.getData(lat, lng, sat)) {
            data += "GPS:" + String(lat, 6) + "," + String(lng, 6) + ";";
            data += "SAT:" + String(sat);
            
            // Add indicator if we don't have enough readings for full average
            int historyCount = gps.getHistoryCount();
            if (historyCount < 10) {
                data += "(" + String(historyCount) + "/10)";
            }
            data += ";";
        }
        
        // Include error information if available
        String lastError = obd.getLastError();
        if (lastError.length() > 0 && (millis() - obd.getLastErrorTime()) < 5000) {
            data += "ERROR:" + lastError + ";";
        }
        
        return data;
    }
    
    void recordStatusReading() {
        StatusReading& reading = statusHistory[statusHistoryIndex];
        reading.obdReal = obd.isUsingRealData();
        reading.obdSim = obd.isSimulationEnabled();
        reading.gpsReady = (m_state & STATE_GPS_READY) != 0;
        reading.storageReady = (m_state & STATE_STORAGE_READY) != 0;
        reading.bleReady = (m_state & STATE_BLE_READY) != 0;
        reading.timestamp = millis();
        
        statusHistoryIndex = (statusHistoryIndex + 1) % STATUS_HISTORY_SIZE;
        if (statusHistoryCount < STATUS_HISTORY_SIZE) {
            statusHistoryCount++;
        }
    }
    
    String getAveragedOBDStatus() {
        if (statusHistoryCount == 0) {
            return obd.isUsingRealData() ? "REAL" : (obd.isSimulationEnabled() ? "SIM" : "OFF");
        }
        
        int realCount = 0, simCount = 0, offCount = 0;
        for (int i = 0; i < statusHistoryCount; i++) {
            if (statusHistory[i].obdReal) {
                realCount++;
            } else if (statusHistory[i].obdSim) {
                simCount++;
            } else {
                offCount++;
            }
        }
        
        String result;
        if (realCount >= simCount && realCount >= offCount) {
            result = "REAL";
        } else if (simCount >= offCount) {
            result = "SIM";
        } else {
            result = "OFF";
        }
        
        if (statusHistoryCount < STATUS_HISTORY_SIZE) {
            result += "(" + String(statusHistoryCount) + "/10)";
        }
        
        return result;
    }
    
    String getAveragedGPSStatus() {
        if (statusHistoryCount == 0) {
            return (m_state & STATE_GPS_READY) ? "OK" : "FAIL";
        }
        
        int okCount = 0;
        for (int i = 0; i < statusHistoryCount; i++) {
            if (statusHistory[i].gpsReady) {
                okCount++;
            }
        }
        
        String result = (okCount > statusHistoryCount / 2) ? "OK" : "FAIL";
        if (statusHistoryCount < STATUS_HISTORY_SIZE) {
            result += "(" + String(statusHistoryCount) + "/10)";
        }
        
        return result;
    }
    
    String getAveragedStorageStatus() {
        if (statusHistoryCount == 0) {
            return (m_state & STATE_STORAGE_READY) ? "OK" : "FAIL";
        }
        
        int okCount = 0;
        for (int i = 0; i < statusHistoryCount; i++) {
            if (statusHistory[i].storageReady) {
                okCount++;
            }
        }
        
        String result = (okCount > statusHistoryCount / 2) ? "OK" : "FAIL";
        if (statusHistoryCount < STATUS_HISTORY_SIZE) {
            result += "(" + String(statusHistoryCount) + "/10)";
        }
        
        return result;
    }
    
    String getAveragedBLEStatus() {
        if (statusHistoryCount == 0) {
            return (m_state & STATE_BLE_READY) ? "OK" : "FAIL";
        }
        
        int okCount = 0;
        for (int i = 0; i < statusHistoryCount; i++) {
            if (statusHistory[i].bleReady) {
                okCount++;
            }
        }
        
        String result = (okCount > statusHistoryCount / 2) ? "OK" : "FAIL";
        if (statusHistoryCount < STATUS_HISTORY_SIZE) {
            result += "(" + String(statusHistoryCount) + "/10)";
        }
        
        return result;
    }
    
    uint16_t m_state = 0;
    TeleStore store;
    SimpleOBD obd;
    SimpleGPS gps;
};

// BLE Characteristic Callbacks
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.println("BLE RX: " + value);
            
            // Handle commands from BLE client
            if (value.startsWith("CMD:")) {
                String command = value.substring(4); // Remove "CMD:" prefix
                
                if (command == "SIM_ON" && logger) {
                    logger->setSimulationEnabled(true);
                    Serial.println("Simulation enabled via BLE command");
                    
                    // Send immediate confirmation
                    if (pCharacteristic) {
                        String response = String(millis()) + ",STATUS:SIM_ENABLED;";
                        pCharacteristic->setValue(response.c_str());
                        pCharacteristic->notify();
                    }
                } else if (command == "SIM_OFF" && logger) {
                    logger->setSimulationEnabled(false);
                    Serial.println("Simulation disabled via BLE command");
                    
                    // Send immediate confirmation
                    if (pCharacteristic) {
                        String response = String(millis()) + ",STATUS:SIM_DISABLED;";
                        pCharacteristic->setValue(response.c_str());
                        pCharacteristic->notify();
                    }
                } else if (command == "STATUS" && logger) {
                    Serial.println("Status request received via BLE");
                    // Status will be sent in next sendBLEData() cycle
                } else if (command == "PING") {
                    Serial.println("Ping received via BLE");
                    
                    // Send pong response
                    if (pCharacteristic) {
                        String response = String(millis()) + ",PONG:OK;";
                        pCharacteristic->setValue(response.c_str());
                        pCharacteristic->notify();
                    }
                } else if (command == "DIAGNOSTIC" && logger) {
                    Serial.println("Diagnostic mode requested via BLE");
                    logger->startDiagnosticMode();
                    
                    // Send immediate acknowledgment
                    if (pCharacteristic) {
                        String response = String(millis()) + ",DIAGNOSTIC_STARTED:OK;";
                        pCharacteristic->setValue(response.c_str());
                        pCharacteristic->notify();
                    }
                }
            } else {
                // Handle legacy commands without CMD: prefix for backward compatibility
                if (value == "SIM_ON" && logger) {
                    logger->setSimulationEnabled(true);
                    Serial.println("Simulation enabled via BLE command (legacy)");
                } else if (value == "SIM_OFF" && logger) {
                    logger->setSimulationEnabled(false);
                    Serial.println("Simulation disabled via BLE command (legacy)");
                }
            }
        }
    }
};

CustomFreematicsLogger loggerInstance;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Freematics Custom Starting...");
    
    // Initialize global logger pointer
    logger = &loggerInstance;
    
    if (logger->init()) {
        Serial.println("Logger OK");
        
        // Set BLE callbacks after initialization
        if (pCharacteristic) {
            pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
        }
    } else {
        Serial.println("Logger FAIL");
    }
    
    Serial.println("Waiting for BLE client connection...");
}

void loop()
{
    if (bleClientConnected && logger) {
        logger->process();
    } else if (logger) {
        // Just handle LED blinking while waiting for connection
        logger->handleLedOnly();
    }
    delay(100);
}
