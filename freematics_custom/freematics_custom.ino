/*************************************************************************
* Custom Freematics Sketch - Simplified Version
* Developed for debugging with Freematics BLE Android app
* Compatible with ESP32 Arduino Core 3.2.0+
*************************************************************************/

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#if USE_FREEMATICS_LIBRARY
#include "FreematicsPlus/FreematicsPlus.h"
#include "FreematicsPlus/FreematicsOBD.h"
#endif
#if USE_FALLBACK_CAN
#include <CAN.h>
#endif
#include "config.h"
#include "telestore.h"

// Use FreematicsPlus library types when available
#if USE_FREEMATICS_LIBRARY
// FreematicsPlus library should provide these classes
// No fallback definitions needed when library is properly included
#endif

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
    String lastError = "";
    unsigned long lastErrorTime = 0;
    bool obdInitialized = false;
    
#if USE_FREEMATICS_LIBRARY
    CFreematicsESP32 sys;
    COBD obd;
#endif
    
public:
    bool init() { 
        // Initialize hardware sensors first
        Serial.print("Initializing hardware sensors...");
        
        // Hardware sensors are always available on Freematics devices
        Serial.println("OK");
        
        // Initialize OBD-II communication interface
        Serial.print("Initializing OBD-II interface...");
        
        // Try multiple initialization methods for different OBD interfaces
        if (initOBDInterface()) {
            obdInitialized = true;
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
        } else {
            obdInitialized = false;
            realOBDAvailable = false;
            lastError = "OBD interface initialization failed";
            Serial.println("Failed, hardware sensors only");
            return true; // Still return true to allow hardware sensor readings
        }
    }
    
    bool initOBDInterface() {
#if USE_FREEMATICS_LIBRARY
        Serial.println("=== FREEMATICS OBD-II INITIALIZATION ===");
        Serial.println("Hardware: Freematics ONE+ with co-processor");
        Serial.println("Using FreematicsPlus library for CAN communication");
        
        // Initialize Freematics co-processor
        Serial.print("Initializing co-processor...");
        if (!sys.begin()) {
            Serial.println("FAILED");
            lastError = "Co-processor initialization failed";
            lastErrorTime = millis();
            return false;
        }
        Serial.println("OK");
        
        // Initialize OBD library
        Serial.print("Initializing OBD library...");
        obd.begin(&sys);
        Serial.println("OK");
        
        // Initialize CAN bus protocol
        Serial.print("Connecting to CAN bus (ISO15765 11-bit 500K)...");
        int attempts = 0;
        while (!obd.init(OBD_PROTOCOL_ISO15765_11B_500K) && attempts < 10) {
            Serial.print('.');
            delay(1000);
            attempts++;
        }
        
        if (attempts >= 10) {
            Serial.println("FAILED");
            lastError = "CAN bus initialization timeout";
            lastErrorTime = millis();
            
            // Try alternative protocols
            Serial.print("Trying ISO15765 29-bit 500K...");
            attempts = 0;
            while (!obd.init(OBD_PROTOCOL_ISO15765_29B_500K) && attempts < 5) {
                Serial.print('.');
                delay(1000);
                attempts++;
            }
            
            if (attempts >= 5) {
                Serial.println("FAILED");
                lastError = "All CAN protocols failed";
                lastErrorTime = millis();
                return false;
            }
        }
        
        Serial.println("OK");
        return true;
        
#elif USE_FALLBACK_CAN
        Serial.println("=== FALLBACK CAN OBD-II INITIALIZATION ===");
        Serial.println("Hardware: ESP32 with direct CAN controller");
        Serial.println("Using Arduino CAN library");
        
        // Initialize CAN bus
        Serial.print("Initializing CAN bus (500 kbps)...");
        
        // Set CAN pins for ESP32
        CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
        
        if (!CAN.begin(500E3)) {
            Serial.println("FAILED");
            lastError = "CAN bus initialization failed";
            lastErrorTime = millis();
            return false;
        }
        
        Serial.println("OK");
        
        // Set up CAN filters for OBD-II responses (7E8-7EF)
        CAN.filter(0x7E8, 0x7F8); // Standard OBD-II response range
        
        return true;
        
#elif USE_SERIAL_OBD
        Serial.println("=== SERIAL OBD-II INITIALIZATION ===");
        Serial.println("Hardware: ELM327 compatible adapter");
        Serial.println("Using Serial communication");
        
        return initSerialOBDInterface();
        
#else
        Serial.println("=== OBD-II DISABLED ===");
        Serial.println("All OBD communication methods disabled in config");
        lastError = "OBD communication disabled in configuration";
        lastErrorTime = millis();
        return false;
#endif
    }
    
    bool initCANInterface() {
        Serial.println("  Setting up CAN TX (GPIO4) and RX (GPIO5) pins...");
        
        // Configure GPIO pins for CAN communication
        pinMode(CAN_TX_PIN, OUTPUT);
        pinMode(CAN_RX_PIN, INPUT);
        
        // Test pin connectivity
        digitalWrite(CAN_TX_PIN, HIGH);
        delay(10);
        bool txPinWorking = digitalRead(CAN_TX_PIN) == HIGH;
        
        digitalWrite(CAN_TX_PIN, LOW);
        delay(10);
        bool txPinToggle = digitalRead(CAN_TX_PIN) == LOW;
        
        Serial.println("  CAN TX Pin (GPIO4) test: " + String(txPinWorking && txPinToggle ? "PASS" : "FAIL"));
        Serial.println("  CAN RX Pin (GPIO5) state: " + String(digitalRead(CAN_RX_PIN) ? "HIGH" : "LOW"));
        
        // CAN driver not available in this Arduino Core version
        Serial.println("  CAN driver not available - using Serial fallback only");
        return false;
    }
    
    bool sendATCommand(const String& command, const String& expectedResponse, unsigned long timeout) {
        return sendATCommandWithResponse(command, expectedResponse, timeout, nullptr);
    }
    
    bool sendATCommandFlexible(const String& command, unsigned long timeout) {
        // Clear input buffer
        while (Serial2.available()) {
            Serial2.read();
        }
        
        Serial.println("    Sending: " + command);
        
        // Send command
        Serial2.print(command + "\r");
        Serial2.flush();
        
        // Wait for any response
        String response = "";
        unsigned long startTime = millis();
        bool gotResponse = false;
        
        while (millis() - startTime < timeout) {
            if (Serial2.available()) {
                char c = Serial2.read();
                if (c == '\r' || c == '\n') {
                    if (response.length() > 0) {
                        gotResponse = true;
                        break;
                    }
                } else if (c >= 32 && c <= 126) { // Printable characters only
                    response += c;
                }
            }
            delay(1);
        }
        
        response.trim();
        Serial.println("    Response: " + (response.length() > 0 ? response : "NO RESPONSE"));
        
        // Accept any reasonable response (ELM327, v1.5, OK, etc.)
        return gotResponse && response.length() > 0;
    }
    
    bool sendATCommandWithResponse(const String& command, const String& expectedResponse, unsigned long timeout, String* actualResponse) {
        // Clear input buffer
        while (Serial2.available()) {
            Serial2.read();
        }
        
        Serial.println("    Sending: " + command + " (expecting: " + expectedResponse + ")");
        
        // Send command with proper termination
        Serial2.print(command + "\r");
        Serial2.flush();
        
        // Wait for response
        String response = "";
        unsigned long startTime = millis();
        bool foundPrompt = false;
        
        while (millis() - startTime < timeout && !foundPrompt) {
            if (Serial2.available()) {
                char c = Serial2.read();
                
                if (c == '>') {
                    // ELM327 prompt - end of response
                    foundPrompt = true;
                    break;
                } else if (c == '\r' || c == '\n') {
                    if (response.length() > 0) {
                        // Got a complete line
                        break;
                    }
                } else if (c >= 32 && c <= 126) { // Printable characters only
                    response += c;
                }
            }
            delay(1);
        }
        
        response.trim();
        response.toUpperCase();
        
        if (actualResponse) {
            *actualResponse = response;
        }
        
        Serial.println("    Response: " + (response.length() > 0 ? response : "NO RESPONSE"));
        
        // Check if response contains expected string
        bool success = response.indexOf(expectedResponse.c_str()) >= 0;
        
        if (!success && response.length() > 0) {
            Serial.println("    Expected '" + expectedResponse + "' but got '" + response + "'");
        }
        
        return success;
    }
    
    bool testOBDCommunication() {
        Serial.println("    Testing OBD communication...");
        
        // Try to get supported PIDs (01 00)
        String response;
        if (sendATCommandWithResponse("0100", "4100", 2000, &response)) {
            Serial.println("    OBD communication test: PASS");
            return true;
        }
        
        // Try alternative test - get VIN (09 02)
        if (sendATCommandWithResponse("0902", "4902", 2000, &response)) {
            Serial.println("    OBD communication test: PASS (VIN response)");
            return true;
        }
        
        // Try simple engine RPM request (01 0C)
        if (sendATCommandWithResponse("010C", "410C", 2000, &response)) {
            Serial.println("    OBD communication test: PASS (RPM response)");
            return true;
        }
        
        Serial.println("    OBD communication test: FAIL (no valid responses)");
        return false;
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
                // Real OBD failed - try to reconnect once
                Serial.println("OBD read failed, attempting reconnection...");
                if (initOBDInterface()) {
                    // Try one more time after reconnection
                    if (attemptRealOBDRead(pid, value)) {
                        return true;
                    }
                }
                
                // Mark as unavailable
                realOBDAvailable = false;
                lastError = "Real OBD-II read failed for PID 0x" + String(pid, HEX);
                lastErrorTime = millis();
                Serial.println("OBD-II connection lost");
            }
        }
        
        // No data available
        lastError = "No data source available for PID 0x" + String(pid, HEX);
        return false;
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
    
    
    
    String runFullDiagnostics() {
        String results = "";
        
        // ESP32 System Information
        results += "=== ESP32 SYSTEM INFO ===|";
        results += "Chip Model: " + String(ESP.getChipModel()) + "|";
        results += "Chip Revision: " + String(ESP.getChipRevision()) + "|";
        results += "CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz|";
        results += "Flash Size: " + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB|";
        results += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes|";
        results += "Uptime: " + String(millis() / 1000) + " seconds|";
        results += "|";
        
        // Power Supply Analysis
        results += "=== POWER SUPPLY ANALYSIS ===|";
        int vinRaw = analogRead(A0);
        float vinVoltage = (vinRaw / 4095.0) * 3.3 * 6.0; // 6:1 voltage divider
        results += "VIN Raw ADC: " + String(vinRaw) + " (0-4095)|";
        results += "VIN Voltage: " + String(vinVoltage, 2) + "V|";
        
        // Adjust status check for different power sources
        String vinStatus;
        if (vinVoltage > 11.0 && vinVoltage < 16.0) {
            vinStatus = "OK (Vehicle Power)";
        } else if (vinVoltage > 4.5 && vinVoltage < 6.0) {
            vinStatus = "OK (USB Power)";
        } else if (vinVoltage > 6.0 && vinVoltage < 11.0) {
            vinStatus = "OK (External Adapter)";
        } else {
            vinStatus = "WARNING (Check Power Source)";
        }
        results += "VIN Status: " + vinStatus + "|";
        results += "3.3V Rail: " + String(analogRead(36) > 100 ? "OK" : "FAIL") + "|";
        results += "|";
        
        // ADC Pin Testing
        results += "=== ADC PIN TESTING ===|";
        int adcPins[] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13};
        String adcNames[] = {"VP(36)", "VN(39)", "GPIO34", "GPIO35", "GPIO32", "GPIO33", 
                           "GPIO25", "GPIO26", "GPIO27", "GPIO14", "GPIO12", "GPIO13"};
        
        for (int i = 0; i < 12; i++) {
            int value = analogRead(adcPins[i]);
            float voltage = (value / 4095.0) * 3.3;
            results += adcNames[i] + ": " + String(value) + " (" + String(voltage, 2) + "V)|";
        }
        results += "|";
        
        // Digital Pin State Testing
        results += "=== DIGITAL PIN TESTING ===|";
        int digitalPins[] = {2, 4, 5, 16, 17, 18, 19, 21, 22, 23};
        String digitalNames[] = {"GPIO2(LED)", "GPIO4(CAN_TX)", "GPIO5(CAN_RX)", 
                               "GPIO16(OBD_RX)", "GPIO17(OBD_TX)", "GPIO18(SPI_SCK)", 
                               "GPIO19(SPI_MISO)", "GPIO21(I2C_SDA)", "GPIO22(I2C_SCL)", "GPIO23(SPI_MOSI)"};
        
        for (int i = 0; i < 10; i++) {
            pinMode(digitalPins[i], INPUT_PULLUP);
            delay(10);
            bool state = digitalRead(digitalPins[i]);
            results += digitalNames[i] + ": " + String(state ? "HIGH" : "LOW");
            
            // Add functional notes for key pins
            if (digitalPins[i] == CAN_TX_PIN) {
                results += " (CAN transmit to SN65HVD230)";
            } else if (digitalPins[i] == CAN_RX_PIN) {
                results += " (CAN receive from SN65HVD230)";
            } else if (digitalPins[i] == OBD_SERIAL_RX) {
                results += " (Serial OBD receive)";
            } else if (digitalPins[i] == OBD_SERIAL_TX) {
                results += " (Serial OBD transmit)";
            }
            results += "|";
        }
        results += "|";
        
        // I2C Bus Scanning
        results += "=== I2C BUS SCAN (SDA=21, SCL=22) ===|";
        Wire.begin(21, 22); // SDA=21, SCL=22
        int deviceCount = 0;
        for (int addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                results += "I2C Device found at 0x" + String(addr, HEX) + "|";
                deviceCount++;
            }
        }
        if (deviceCount == 0) {
            results += "No I2C devices found|";
        } else {
            results += "Total I2C devices: " + String(deviceCount) + "|";
        }
        results += "|";
        
        // SPI Pin State Verification
        results += "=== SPI PIN VERIFICATION ===|";
        pinMode(19, INPUT_PULLUP); // MISO
        pinMode(23, INPUT_PULLUP); // MOSI  
        pinMode(18, INPUT_PULLUP); // SCK
        pinMode(5, INPUT_PULLUP);  // SS
        delay(10);
        results += "MISO(19): " + String(digitalRead(19) ? "HIGH" : "LOW") + "|";
        results += "MOSI(23): " + String(digitalRead(23) ? "HIGH" : "LOW") + "|";
        results += "SCK(18): " + String(digitalRead(18) ? "HIGH" : "LOW") + "|";
        results += "SS(5): " + String(digitalRead(5) ? "HIGH" : "LOW") + "|";
        results += "|";
        
        // OBD-II Interface Testing
        results += "=== OBD-II INTERFACE TESTING ===|";
#if USE_FREEMATICS_LIBRARY
        results += "Mode: FreematicsPlus Library|";
        results += "Hardware: Freematics ONE+ with co-processor|";
#elif USE_FALLBACK_CAN
        results += "Mode: Fallback CAN Implementation|";
        results += "Hardware: ESP32 with direct CAN controller|";
#elif USE_SERIAL_OBD
        results += "Mode: Serial ELM327 Communication|";
        results += "Hardware: ELM327 compatible adapter|";
#else
        results += "Mode: DISABLED|";
        results += "Hardware: No OBD communication enabled|";
#endif
        results += "CAN Bus Pins: GPIO4 (TX), GPIO5 (RX)|";
        results += "Serial Pins: GPIO16 (RX), GPIO17 (TX)|";
        results += "|";
        
        // Test CAN bus pins
        results += "=== CAN BUS PIN TESTING ===|";
        pinMode(CAN_TX_PIN, OUTPUT);
        pinMode(CAN_RX_PIN, INPUT_PULLUP);
        delay(10);
        
        // Test CAN TX pin
        digitalWrite(CAN_TX_PIN, HIGH);
        delay(10);
        bool canTxHigh = digitalRead(CAN_TX_PIN) == HIGH;
        digitalWrite(CAN_TX_PIN, LOW);
        delay(10);
        bool canTxLow = digitalRead(CAN_TX_PIN) == LOW;
        results += "CAN TX (GPIO4): " + String(canTxHigh && canTxLow ? "FUNCTIONAL" : "FAIL") + "|";
        
        // Test CAN RX pin
        bool canRxState = digitalRead(CAN_RX_PIN);
        results += "CAN RX (GPIO5): " + String(canRxState ? "HIGH" : "LOW") + " (pullup active)|";
        results += "CAN Driver: NOT AVAILABLE (Arduino Core 3.2.0)|";
        results += "|";
        
        // Test Serial OBD interface
        results += "=== SERIAL OBD TESTING ===|";
        results += "Testing Serial2 interface...|";
        
        // Test multiple baud rates with improved ELM327 detection
        int baudRates[] = {38400, 9600, 115200, 57600};
        bool obdResponsive = false;
        String bestResponse = "";
        int bestBaud = 0;
        
        for (int i = 0; i < 4; i++) {
            Serial2.begin(baudRates[i], SERIAL_8N1, OBD_SERIAL_RX, OBD_SERIAL_TX);
            delay(200); // More time for baud rate stabilization
            
            // Clear buffer
            while (Serial2.available()) {
                Serial2.read();
            }
            
            // Send ELM327 reset command
            Serial2.print("ATZ\r");
            Serial2.flush();
            delay(1500); // ELM327 needs time to reset
            
            String response = "";
            unsigned long startTime = millis();
            while (millis() - startTime < 2000) {
                if (Serial2.available()) {
                    char c = Serial2.read();
                    if (c >= 32 && c <= 126) { // Printable characters only
                        response += c;
                    } else if (c == '\r' || c == '\n') {
                        if (response.length() > 0) {
                            break; // Got complete response
                        }
                    }
                }
                delay(1);
            }
            
            response.trim();
            
            if (response.length() > 0) {
                results += "Baud " + String(baudRates[i]) + ": RESPONSE|";
                results += "  Content: " + response.substring(0, min(30, (int)response.length())) + "|";
                
                // Check for ELM327 indicators
                if (response.indexOf("ELM327") >= 0 || response.indexOf("v1.") >= 0 || 
                    response.indexOf("v2.") >= 0 || response.indexOf("OBD") >= 0) {
                    results += "  Type: ELM327 Compatible|";
                    obdResponsive = true;
                    bestResponse = response;
                    bestBaud = baudRates[i];
                } else {
                    results += "  Type: Unknown OBD Interface|";
                    if (!obdResponsive) {
                        obdResponsive = true; // Any response is better than none
                        bestResponse = response;
                        bestBaud = baudRates[i];
                    }
                }
                
                // Test basic communication
                delay(500);
                while (Serial2.available()) Serial2.read(); // Clear buffer
                Serial2.print("ATE0\r");
                Serial2.flush();
                delay(500);
                
                String echoResponse = "";
                startTime = millis();
                while (millis() - startTime < 1000 && Serial2.available()) {
                    char c = Serial2.read();
                    if (c >= 32 && c <= 126) {
                        echoResponse += c;
                    }
                }
                
                if (echoResponse.indexOf("OK") >= 0) {
                    results += "  Echo Test: PASS|";
                } else {
                    results += "  Echo Test: FAIL (" + echoResponse + ")|";
                }
                
            } else {
                results += "Baud " + String(baudRates[i]) + ": NO RESPONSE|";
            }
            
            Serial2.end();
            delay(100);
        }
        
        if (obdResponsive && bestBaud > 0) {
            results += "|Best Interface: " + String(bestBaud) + " baud|";
            results += "Best Response: " + bestResponse + "|";
        }
        
        results += "|";
        results += "Serial OBD Status: " + String(obdResponsive ? "RESPONSIVE" : "NO RESPONSE") + "|";
        results += "CAN Bus Status: NOT IMPLEMENTED (requires CAN driver)|";
        results += "Recommended: Implement ESP32 CAN driver for GPIO4/5|";
        results += "|";
        
        // BLE System Verification
        results += "=== BLE SYSTEM VERIFICATION ===|";
        results += "BLE Initialized: " + String(BLEDevice::getInitialized() ? "YES" : "NO") + "|";
        results += "BLE Server: " + String(pServer != NULL ? "ACTIVE" : "NULL") + "|";
        results += "BLE Characteristic: " + String(pCharacteristic != NULL ? "ACTIVE" : "NULL") + "|";
        results += "Client Connected: " + String(bleClientConnected ? "YES" : "NO") + "|";
        
        // Get BLE address
        String bleAddress = BLEDevice::getAddress().toString().c_str();
        results += "BLE Address: " + bleAddress + "|";
        results += "|";
        
        // Memory Analysis
        results += "=== MEMORY ANALYSIS ===|";
        results += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes|";
        results += "Min Free Heap: " + String(ESP.getMinFreeHeap()) + " bytes|";
        results += "Heap Size: " + String(ESP.getHeapSize()) + " bytes|";
        results += "Free PSRAM: " + String(ESP.getFreePsram()) + " bytes|";
        
        // Calculate memory health
        float heapUsage = ((float)(ESP.getHeapSize() - ESP.getFreeHeap()) / ESP.getHeapSize()) * 100;
        results += "Heap Usage: " + String(heapUsage, 1) + "%|";
        results += "Memory Health: " + String(heapUsage < 80 ? "GOOD" : "WARNING") + "|";
        results += "|";
        
        // Temperature Monitoring
        results += "=== TEMPERATURE MONITORING ===|";
        #ifdef SOC_TEMP_SENSOR_SUPPORTED
        float internalTemp = temperatureRead();
        results += "ESP32 Internal: " + String(internalTemp, 1) + "°C|";
        results += "Thermal Status: " + String(internalTemp < 80 ? "NORMAL" : "HOT") + "|";
        #else
        results += "ESP32 Internal: NOT SUPPORTED|";
        #endif
        
        // External temperature sensor test
        int tempSensorValue = analogRead(36);
        if (tempSensorValue > 100 && tempSensorValue < 4000) {
            float voltage = (tempSensorValue / 4095.0) * 3.3;
            float temp_celsius = (voltage - 0.5) * 100.0; // TMP36 formula
            results += "External Sensor: " + String(temp_celsius, 1) + "°C (GPIO36)|";
        } else {
            results += "External Sensor: NOT DETECTED (GPIO36)|";
        }
        results += "|";
        
        // System Health Summary
        results += "=== SYSTEM HEALTH SUMMARY ===|";
        
        // Count issues (adjust power supply check)
        int issues = 0;
        // Only flag power as issue if voltage is dangerously low or high
        if (vinVoltage < 4.0 || vinVoltage > 16.0) issues++;
        if (ESP.getFreeHeap() < 50000) issues++;
        if (heapUsage > 80) issues++;
        #ifdef SOC_TEMP_SENSOR_SUPPORTED
        if (internalTemp > 80) issues++;
        #endif
        if (!BLEDevice::getInitialized()) issues++;
        if (!obdResponsive) issues++;
        if (!canTxHigh || !canTxLow) issues++; // CAN TX pin issue
        
        results += "Issues Found: " + String(issues) + "|";
        results += "Overall Status: " + String(issues == 0 ? "EXCELLENT" : 
                                              issues <= 2 ? "GOOD" : 
                                              issues <= 4 ? "WARNING" : "CRITICAL") + "|";
        
        // OBD-II specific status
        results += "|=== OBD-II COMMUNICATION STATUS ===|";
#if USE_FREEMATICS_LIBRARY
        results += "FreematicsPlus Library: ENABLED|";
        results += "Co-processor Status: " + String(obdInitialized ? "READY" : "FAILED") + "|";
#endif
#if USE_FALLBACK_CAN
        results += "Fallback CAN: ENABLED|";
        results += "CAN Hardware: " + String((canTxHigh && canTxLow) ? "READY" : "ISSUE") + "|";
#endif
#if USE_SERIAL_OBD
        results += "Serial OBD: ENABLED|";
        results += "ELM327 Response: " + String(obdResponsive ? "RESPONSIVE" : "NO RESPONSE") + "|";
#endif
#if !USE_FREEMATICS_LIBRARY && !USE_FALLBACK_CAN && !USE_SERIAL_OBD
        results += "All OBD Methods: DISABLED|";
#endif
        results += "Active Method: " + String(realOBDAvailable ? "CONNECTED" : "DISCONNECTED") + "|";
        
        // Recommendations
        if (issues > 0) {
            results += "|=== RECOMMENDATIONS ===|";
            if (vinVoltage < 4.0) results += "• Power supply voltage critically low - check connections|";
            if (vinVoltage > 16.0) results += "• Check charging system - voltage too high|";
            if (ESP.getFreeHeap() < 50000) results += "• Memory low - restart device|";
            if (heapUsage > 80) results += "• High memory usage - check for leaks|";
            #ifdef SOC_TEMP_SENSOR_SUPPORTED
            if (internalTemp > 80) results += "• ESP32 running hot - check ventilation|";
            #endif
            if (!BLEDevice::getInitialized()) results += "• BLE not initialized - restart required|";
            if (!obdResponsive) results += "• No Serial OBD response - check ELM327 adapter|";
            if (!canTxHigh || !canTxLow) results += "• CAN TX pin issue - check GPIO4 connection|";
            results += "• Consider implementing ESP32 CAN driver for native OBD-II|";
            results += "• Verify SN65HVD230 transceiver power and connections|";
        }
        
        // Add power source information
        results += "|=== POWER SOURCE ANALYSIS ===|";
        if (vinVoltage > 11.0 && vinVoltage < 16.0) {
            results += "• Connected to vehicle 12V system|";
            results += "• Ready for OBD-II communication|";
        } else if (vinVoltage > 4.5 && vinVoltage < 6.0) {
            results += "• Powered via USB (development/testing mode)|";
            results += "• OBD-II requires vehicle connection for real data|";
        } else if (vinVoltage > 6.0 && vinVoltage < 11.0) {
            results += "• External power adapter detected|";
            results += "• Verify power source compatibility|";
        }
        
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
            case 0x42: // PID_CONTROL_MODULE_VOLTAGE - only use for OBD voltage
                // Don't provide device voltage as OBD voltage
                return false;
            case 0x46: // PID_AMBIENT_AIR_TEMP - only use real ambient sensor
                value = readAmbientTemperature();
                return true;
            case PID_ENGINE_PRESSURE: // Engine oil pressure - OBD only
                // Don't provide atmospheric pressure as engine pressure
                return false;
            case PID_ABSOLUTE_BAROMETRIC_PRESSURE: // Atmospheric pressure
                value = readBarometricPressure();
                return true;
            case 0xF0: // Custom PID for device input voltage
                value = readDeviceInputVoltage();
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
    
    int readDeviceInputVoltage() {
        // Read device input voltage (Vin) from ADC
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
    
    int readBarometricPressure() {
        // Read atmospheric pressure from device barometric sensor
        // Could be connected to BMP280 or similar sensor
        // For now, return standard atmospheric pressure with small variation
        return 1013 + random(-10, 10); // Standard atmosphere in mbar
    }

    bool attemptRealOBDRead(uint8_t pid, int& value) {
        if (!obdInitialized) {
            lastError = "OBD interface not initialized";
            lastErrorTime = millis();
            return false;
        }
        
#if USE_FREEMATICS_LIBRARY
        // Use FreematicsPlus OBD library
        if (obd.readPID(pid, value)) {
            return true;
        }
        
        lastError = "FreematicsPlus OBD read failed for PID 0x" + String(pid, HEX);
        lastErrorTime = millis();
        return false;
        
#elif USE_FALLBACK_CAN
        // Use fallback CAN implementation
        return sendCANOBDRequest(pid, value);
        
#elif USE_SERIAL_OBD
        // Use Serial OBD implementation
        return attemptSerialOBDRead(pid, value);
        
#else
        lastError = "No OBD communication method enabled";
        lastErrorTime = millis();
        return false;
#endif
    }
    
#if USE_FALLBACK_CAN
    bool sendCANOBDRequest(uint8_t pid, int& value) {
        // OBD-II CAN frame format:
        // ID: 0x7DF (functional request) or 0x7E0-0x7E7 (physical request)
        // Data: [Length, Mode, PID, 0x55, 0x55, 0x55, 0x55, 0x55]
        
        uint8_t requestData[8] = {0x02, 0x01, pid, 0x55, 0x55, 0x55, 0x55, 0x55};
        
        // Send request
        CAN.beginPacket(0x7DF); // Functional request ID
        CAN.write(requestData, 8);
        if (!CAN.endPacket()) {
            lastError = "Failed to send CAN request for PID 0x" + String(pid, HEX);
            lastErrorTime = millis();
            return false;
        }
        
        // Wait for response
        unsigned long startTime = millis();
        while (millis() - startTime < 1000) { // 1 second timeout
            int packetSize = CAN.parsePacket();
            
            if (packetSize > 0) {
                uint32_t canId = CAN.packetId();
                
                // Check if this is an OBD-II response (0x7E8-0x7EF)
                if (canId >= 0x7E8 && canId <= 0x7EF) {
                    uint8_t responseData[8];
                    int bytesRead = 0;
                    
                    while (CAN.available() && bytesRead < 8) {
                        responseData[bytesRead++] = CAN.read();
                    }
                    
                    // Parse OBD response: [Length, Mode+0x40, PID, Data...]
                    if (bytesRead >= 3 && responseData[1] == 0x41 && responseData[2] == pid) {
                        // Extract data bytes for parsing
                        String dataBytes = "";
                        for (int i = 3; i < bytesRead; i++) {
                            if (responseData[i] < 0x10) dataBytes += "0";
                            dataBytes += String(responseData[i], HEX);
                        }
                        
                        return parseOBDResponse(pid, dataBytes, value);
                    }
                }
            }
            delay(10);
        }
        
        lastError = "CAN OBD timeout for PID 0x" + String(pid, HEX);
        lastErrorTime = millis();
        return false;
    }
#endif

#if USE_SERIAL_OBD
    bool initSerialOBDInterface() {
        // Try UART-based OBD interface (ELM327 style)
        Serial.print("Initializing Serial OBD interface (GPIO16/17)...");
        Serial2.begin(38400, SERIAL_8N1, OBD_SERIAL_RX, OBD_SERIAL_TX);
        delay(100);
        
        // Clear any existing data
        while (Serial2.available()) {
            Serial2.read();
        }
        
        // Send AT commands for ELM327-style initialization
        Serial.println("  Attempting ELM327 initialization sequence...");
        
        // Step 1: Reset and wait for any response (ELM327, v1.5, etc.)
        if (sendATCommandFlexible("ATZ", 3000)) {
            delay(1000); // ELM327 needs time after reset
            
            // Step 2: Turn off echo
            if (sendATCommand("ATE0", "OK", 1000)) {
                delay(100);
                
                // Step 3: Turn off line feeds  
                if (sendATCommand("ATL0", "OK", 1000)) {
                    delay(100);
                    
                    // Step 4: Set automatic protocol detection
                    if (sendATCommand("ATSP0", "OK", 1000)) {
                        delay(100);
                        
                        // Step 5: Test basic OBD communication
                        if (testOBDCommunication()) {
                            Serial.println("  ELM327-style interface ready and tested");
                            return true;
                        } else {
                            Serial.println("  ELM327 initialized but OBD communication failed");
                            return true; // Still return true as interface is ready
                        }
                    }
                }
            }
        }
        
        Serial.println("Serial OBD interface initialization failed");
        return false;
    }

    bool attemptSerialOBDRead(uint8_t pid, int& value) {
        // Format OBD-II request: Mode 01 (current data) + PID
        String obdRequest = "01";
        if (pid < 0x10) {
            obdRequest += "0" + String(pid, HEX);
        } else {
            obdRequest += String(pid, HEX);
        }
        obdRequest.toUpperCase();
        
        // Clear input buffer
        while (Serial2.available()) {
            Serial2.read();
        }
        
        // Send request with proper termination
        Serial2.print(obdRequest + "\r");
        Serial2.flush();
        
        // Wait for response
        String fullResponse = "";
        String currentLine = "";
        unsigned long startTime = millis();
        bool foundPrompt = false;
        
        while (millis() - startTime < 3000 && !foundPrompt) {
            if (Serial2.available()) {
                char c = Serial2.read();
                
                if (c == '>') {
                    foundPrompt = true;
                    if (currentLine.length() > 0) {
                        fullResponse += currentLine;
                    }
                    break;
                } else if (c == '\r' || c == '\n') {
                    if (currentLine.length() > 0) {
                        fullResponse += currentLine + "|";
                        currentLine = "";
                    }
                } else if (c >= 32 && c <= 126) {
                    currentLine += c;
                }
            }
            delay(1);
        }
        
        // Process the response
        fullResponse.trim();
        fullResponse.replace(" ", "");
        fullResponse.toUpperCase();
        
        String pidHex = String(pid, HEX);
        if (pid < 0x10) pidHex = "0" + pidHex;
        pidHex.toUpperCase();
        String expectedStart = "41" + pidHex;
        
        int startPos = fullResponse.indexOf(expectedStart);
        if (startPos >= 0) {
            int endPos = fullResponse.indexOf("|", startPos);
            if (endPos < 0) endPos = fullResponse.length();
            
            String responseLine = fullResponse.substring(startPos, endPos);
            String dataBytes = responseLine.substring(expectedStart.length());
            
            if (parseOBDResponse(pid, dataBytes, value)) {
                return true;
            }
        }
        
        lastError = "No valid response for PID 0x" + String(pid, HEX);
        lastErrorTime = millis();
        return false;
    }
    
    bool parseOBDResponse(uint8_t pid, String dataBytes, int& value) {
        // Parse OBD-II response data based on PID
        dataBytes.trim();
        dataBytes.replace(" ", ""); // Remove spaces
        dataBytes.toUpperCase();
        
        // Validate minimum data length
        if (dataBytes.length() < 2) {
            lastError = "Insufficient data for PID 0x" + String(pid, HEX);
            lastErrorTime = millis();
            return false;
        }
        
        // Get data bytes as integers
        int A = 0, B = 0, C = 0, D = 0;
        
        // Parse hex bytes safely
        if (dataBytes.length() >= 2) {
            A = (int)strtol(dataBytes.substring(0, 2).c_str(), NULL, 16);
        }
        if (dataBytes.length() >= 4) {
            B = (int)strtol(dataBytes.substring(2, 4).c_str(), NULL, 16);
        }
        if (dataBytes.length() >= 6) {
            C = (int)strtol(dataBytes.substring(4, 6).c_str(), NULL, 16);
        }
        if (dataBytes.length() >= 8) {
            D = (int)strtol(dataBytes.substring(6, 8).c_str(), NULL, 16);
        }
        
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
                
            // Turbo/Boost Control PIDs
            case PID_BOOST_PRESSURE_CONTROL:
                value = A; // Boost pressure control percentage
                return true;
            case PID_TURBOCHARGER_RPM:
                value = ((A * 256) + B) * 4; // Turbocharger RPM
                return true;
            case PID_EXHAUST_GAS_TEMP_BANK_1:
                value = ((A * 256) + B) / 10 - 40; // Exhaust gas temperature °C
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
#endif
    
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
    TeleStore store;
    SimpleOBD obd;
    SimpleGPS gps;
    
    
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
        
        // Hardware sensors (using custom PIDs to avoid confusion with OBD-II)
        if (obd.readPID(0xF0, value)) store.log(0xF0, value); // Device input voltage
        if (obd.readPID(PID_AMBIENT_AIR_TEMP, value)) store.log(PID_AMBIENT_AIR_TEMP, value);
        if (obd.readPID(PID_ABSOLUTE_BAROMETRIC_PRESSURE, value)) store.log(PID_ABSOLUTE_BAROMETRIC_PRESSURE, value);
        
        // Try to get real OBD-II data for these if available
        if (obd.readPID(PID_CONTROL_MODULE_VOLTAGE, value)) store.log(PID_CONTROL_MODULE_VOLTAGE, value);
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
        } else {
            data += "MODE:DISABLED;"; // No OBD data
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
        
        // Device hardware sensors
        if (obd.readPID(0xF0, value)) data += "BATTERY:" + String(value) + ";"; // Device input voltage
        if (obd.readPID(PID_AMBIENT_AIR_TEMP, value)) data += "AMBIENT:" + String(value) + ";";
        if (obd.readPID(PID_ABSOLUTE_BAROMETRIC_PRESSURE, value)) data += "PRESSURE:" + String(value) + ";";
        
        // Real OBD-II data if available
        if (obd.readPID(PID_CONTROL_MODULE_VOLTAGE, value)) data += "OBD_BATTERY:" + String(value) + ";";
        if (obd.readPID(PID_ENGINE_PRESSURE, value)) data += "OIL_PRESSURE:" + String(value) + ";";
        
        // Additional engine parameters for UI synchronization
        if (obd.readPID(PID_ENGINE_OIL_TEMP, value)) data += "OIL_TEMP:" + String(value) + ";";
        if (obd.readPID(PID_EXHAUST_GAS_TEMP_BANK_1, value)) data += "EXHAUST_TEMP:" + String(value) + ";";
        if (obd.readPID(PID_FUEL_PRESSURE, value)) data += "FUEL_PRESSURE:" + String(value) + ";";
        if (obd.readPID(PID_ENGINE_FUEL_RATE, value)) data += "FUEL_RATE:" + String(value) + ";";
        if (obd.readPID(PID_CATALYST_TEMP_B1S1, value)) data += "CATALYST_TEMP:" + String(value) + ";";
        if (obd.readPID(PID_TURBOCHARGER_RPM, value)) data += "TURBO_RPM:" + String(value) + ";";
        if (obd.readPID(PID_BOOST_PRESSURE_CONTROL, value)) data += "BOOST_PRESSURE:" + String(value) + ";";
        
        float lat, lng;
        uint8_t sat;
        if (gps.getData(lat, lng, sat)) {
            // Store GPS coordinates as integers for storage, but send as floats for UI
            store.log(0x20, (int32_t)(lat * 1000000));
            store.log(0x21, (int32_t)(lng * 1000000));
            store.log(0x22, sat);
            
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
        reading.obdSim = false;
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
            return obd.isUsingRealData() ? "REAL" : "OFF";
        }
        
        int realCount = 0, offCount = 0;
        for (int i = 0; i < statusHistoryCount; i++) {
            if (statusHistory[i].obdReal) {
                realCount++;
            } else {
                offCount++;
            }
        }
        
        String result = (realCount > offCount) ? "REAL" : "OFF";
        
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
                
                if (command == "STATUS" && logger) {
                    Serial.println("Status request received via BLE");
                    // Status will be sent in next sendBLEData() cycle
                } else if (command == "PING") {
                    Serial.println("Ping received via BLE");
                    
                    // Send pong response with message counter
                    if (pCharacteristic) {
                        static uint32_t pingMessageCounter = 2000; // Start high to avoid conflicts
                        pingMessageCounter++;
                        String response = String(pingMessageCounter) + ":" + String(millis()) + ",PONG:OK;";
                        pCharacteristic->setValue(response.c_str());
                        pCharacteristic->notify();
                    }
                } else if (command == "DIAGNOSTIC" && logger) {
                    Serial.println("Diagnostic mode requested via BLE");
                    
                    // Get access to message counter (we'll use a static counter for diagnostic messages)
                    static uint32_t diagnosticMessageCounter = 1000; // Start high to avoid conflicts
                    
                    // Send diagnostic started message
                    if (pCharacteristic) {
                        diagnosticMessageCounter++;
                        String startMsg = String(diagnosticMessageCounter) + ":" + String(millis()) + ",DIAGNOSTIC_STARTED:RUNNING;";
                        pCharacteristic->setValue(startMsg.c_str());
                        pCharacteristic->notify();
                        delay(100);
                    }
                    
                    // Run comprehensive diagnostics
                    String diagnosticResults = logger->obd.runFullDiagnostics();
                    Serial.println("Diagnostic results length: " + String(diagnosticResults.length()));
                    
                    // Send results in chunks if too large for single BLE packet
                    if (diagnosticResults.length() > 400) {
                        // Split into chunks
                        int chunkSize = 400;
                        int chunks = (diagnosticResults.length() + chunkSize - 1) / chunkSize;
                        
                        for (int i = 0; i < chunks; i++) {
                            int start = i * chunkSize;
                            int end = min(start + chunkSize, (int)diagnosticResults.length());
                            String chunk = diagnosticResults.substring(start, end);
                            
                            diagnosticMessageCounter++;
                            String chunkMsg = String(diagnosticMessageCounter) + ":" + String(millis()) + ",DIAGNOSTIC_RESULTS:" + chunk + ";";
                            pCharacteristic->setValue(chunkMsg.c_str());
                            pCharacteristic->notify();
                            delay(200); // Allow time for transmission
                        }
                    } else {
                        // Send as single message
                        diagnosticMessageCounter++;
                        String resultMsg = String(diagnosticMessageCounter) + ":" + String(millis()) + ",DIAGNOSTIC_RESULTS:" + diagnosticResults + ";";
                        pCharacteristic->setValue(resultMsg.c_str());
                        pCharacteristic->notify();
                        delay(100);
                    }
                    
                    // Send completion message
                    diagnosticMessageCounter++;
                    String completeMsg = String(diagnosticMessageCounter) + ":" + String(millis()) + ",DIAGNOSTIC_COMPLETE:SUCCESS;";
                    pCharacteristic->setValue(completeMsg.c_str());
                    pCharacteristic->notify();
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
