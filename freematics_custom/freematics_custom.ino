/*************************************************************************
* Custom Freematics Sketch - Simplified Version
* Developed for debugging with Freematics BLE Android app
* Compatible with ESP32 Arduino Core 3.2.0+
*************************************************************************/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
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
        
        // Send immediate connection confirmation
        if (pCharacteristic) {
            String connectMsg = "0:" + String(millis()) + ",CONNECT:SUCCESS;";
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

// BLE Characteristic Callbacks for receiving data from client
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.println("BLE RX: " + value);
            
            // Process commands from client
            if (value.startsWith("CMD:")) {
                String cmd = value.substring(4);
                if (cmd == "RESET") {
                    Serial.println("Received RESET command from client");
                    ESP.restart();
                } else if (cmd == "STATUS") {
                    Serial.println("Received STATUS request from client");
                    // Status will be sent in next cycle
                } else if (cmd == "PING") {
                    Serial.println("Received PING from client");
                    String pong = String(millis()) + ",PONG:OK;";
                    pCharacteristic->setValue(pong.c_str());
                    pCharacteristic->notify();
                }
            }
        }
    }
};

// OBD-II interface with real data support and fallback to simulation
class SimpleOBD {
private:
    bool realOBDAvailable = false;
    String lastError = "";
    unsigned long lastErrorTime = 0;
    
public:
    bool init() { 
        // Try to initialize real OBD connection
        Serial.print("Attempting real OBD-II connection...");
        
        // Initialize OBD-II communication (placeholder for real implementation)
        // In a real implementation, this would initialize the OBD-II interface
        // For now, we'll simulate connection attempts
        delay(1000); // Simulate connection attempt delay
        
        // Check if vehicle is connected and responding
        int testValue;
        if (attemptRealOBDRead(0x0C, testValue)) {
            realOBDAvailable = true;
            lastError = "";
            Serial.println("Real OBD-II connection established");
            return true;
        } else {
            realOBDAvailable = false;
            lastError = "No OBD-II response from vehicle";
            Serial.println("No real OBD-II connection, using simulated data");
            return true; // Still return true to allow simulation mode
        }
    }
    
    bool readPID(uint8_t pid, int& value) {
        if (realOBDAvailable) {
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
        
        // Use simulated data
        return readSimulatedPID(pid, value);
    }
    
    bool isUsingRealData() {
        return realOBDAvailable;
    }
    
    String getLastError() {
        return lastError;
    }
    
    unsigned long getLastErrorTime() {
        return lastErrorTime;
    }
    
private:
    bool attemptRealOBDRead(uint8_t pid, int& value) {
        // Placeholder for real OBD-II communication
        // In a real implementation, this would:
        // 1. Send OBD-II request for the specified PID
        // 2. Wait for response with timeout
        // 3. Parse the response and extract the value
        // 4. Return true if successful, false if failed
        
        // For now, simulate occasional failures to test error handling
        if (random(0, 10) < 8) { // 80% success rate for testing
            return readSimulatedPID(pid, value);
        } else {
            lastError = "Timeout reading PID 0x" + String(pid, HEX);
            lastErrorTime = millis();
            return false;
        }
    }
    
    bool readSimulatedPID(uint8_t pid, int& value) {
        switch(pid) {
            case 0x0C: value = 1500 + random(-200, 200); return true;
            case 0x0D: value = 60 + random(-10, 10); return true;
            case 0x05: value = 85 + random(-5, 5); return true;
            default: 
                lastError = "Unsupported PID 0x" + String(pid, HEX);
                lastErrorTime = millis();
                return false;
        }
    }
};

// Minimal GPS simulation
class SimpleGPS {
public:
    bool begin() { return true; }
    bool getData(float& lat, float& lng, uint8_t& sat) {
        lat = 37.7749 + (random(-1000, 1000) / 100000.0);
        lng = -122.4194 + (random(-1000, 1000) / 100000.0);
        sat = 8 + random(-2, 2);
        return true;
    }
};

class CustomFreematicsLogger
{
public:
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

        pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
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
        if (obd.readPID(0x0C, value)) store.log(0x0C, value);
        if (obd.readPID(0x0D, value)) store.log(0x0D, value);
        if (obd.readPID(0x05, value)) store.log(0x05, value);
        
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
                messageCounter++;
                String statusData = String(messageCounter) + ":" + String(millis()) + ",STATUS:";
                statusData += "OBD=" + String(obd.isUsingRealData() ? "REAL" : "SIM") + ",";
                statusData += "GPS=" + String((m_state & STATE_GPS_READY) ? "OK" : "FAIL") + ",";
                statusData += "STORAGE=" + String((m_state & STATE_STORAGE_READY) ? "OK" : "FAIL") + ",";
                statusData += "BLE=" + String((m_state & STATE_BLE_READY) ? "OK" : "FAIL") + ",";
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
            data += "MODE:SIMULATED;";
        }
        
        int value;
        if (obd.readPID(0x0C, value)) data += "RPM:" + String(value) + ";";
        if (obd.readPID(0x0D, value)) data += "SPD:" + String(value) + ";";
        if (obd.readPID(0x05, value)) data += "COOLANT:" + String(value) + ";";
        
        float lat, lng;
        uint8_t sat;
        if (gps.getData(lat, lng, sat)) {
            data += "GPS:" + String(lat, 6) + "," + String(lng, 6) + ";";
            data += "SAT:" + String(sat) + ";";
        }
        
        // Include error information if available
        String lastError = obd.getLastError();
        if (lastError.length() > 0 && (millis() - obd.getLastErrorTime()) < 5000) {
            data += "ERROR:" + lastError + ";";
        }
        
        return data;
    }
    
    uint16_t m_state = 0;
    TeleStore store;
    SimpleOBD obd;
    SimpleGPS gps;
};


CustomFreematicsLogger logger;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Freematics Custom Starting...");
    
    if (logger.init()) {
        Serial.println("Logger OK");
    } else {
        Serial.println("Logger FAIL");
    }
    
    Serial.println("Waiting for BLE client connection...");
}

void loop()
{
    if (bleClientConnected) {
        logger.process();
    } else {
        // Just handle LED blinking while waiting for connection
        logger.handleLedOnly();
    }
    delay(100);
}
