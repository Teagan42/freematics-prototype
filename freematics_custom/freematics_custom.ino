/*************************************************************************
* Custom Freematics Sketch - Simplified Version
* Developed for debugging with Freematics BLE Android app
* Compatible with ESP32 Arduino Core 3.2.0+
*************************************************************************/

#include <WiFi.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
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

// Expected Bluetooth client addresses
const String EXPECTED_BT_ADDRESSES[] = {
    "A4:83:E7:CC:52:C3",
    "24:95:2F:D5:59:1B"
};
const int NUM_EXPECTED_ADDRESSES = 2;

// Bluetooth Serial instance
BluetoothSerial SerialBT;
bool bluetoothClientConnected = false;
unsigned long lastLedBlink = 0;
bool ledState = false;

// Simple OBD-II implementation
class SimpleOBD {
public:
    bool init() {
        // Initialize OBD communication (simplified)
        Serial.println("OBD init - simplified mode");
        return true; // Always return true for testing
    }
    
    bool readPID(uint8_t pid, int& value) {
        // Simulate OBD data for testing
        switch(pid) {
            case 0x0C: // RPM
                value = 1500 + random(-200, 200);
                return true;
            case 0x0D: // Speed
                value = 60 + random(-10, 10);
                return true;
            case 0x05: // Coolant temp
                value = 85 + random(-5, 5);
                return true;
            default:
                return false;
        }
    }
};

// Simple GPS implementation
class SimpleGPS {
public:
    bool begin() {
        Serial.println("GPS begin - simplified mode");
        return true; // Always return true for testing
    }
    
    bool getData(float& lat, float& lng, uint8_t& sat) {
        // Simulate GPS data for testing
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
        
        // Initialize Bluetooth
        Serial.print("Bluetooth...");
        if (SerialBT.begin(BLE_DEVICE_NAME)) {
            Serial.println("OK");
            m_state |= STATE_BLE_READY;
            SerialBT.register_callback(bluetoothCallback);
        } else {
            Serial.println("NO");
        }
        
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
    
private:
    void handleLedBlink()
    {
        if (!bluetoothClientConnected) {
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
        // Read engine RPM
        if (obd.readPID(0x0C, value)) {
            store.log(0x0C, value);
            Serial.print("RPM: ");
            Serial.println(value);
        }
        
        // Read vehicle speed
        if (obd.readPID(0x0D, value)) {
            store.log(0x0D, value);
            Serial.print("Speed: ");
            Serial.println(value);
        }
        
        // Read engine coolant temperature
        if (obd.readPID(0x05, value)) {
            store.log(0x05, value);
            Serial.print("Coolant Temp: ");
            Serial.println(value);
        }
        
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
            
            Serial.print("GPS: ");
            Serial.print(lat, 6);
            Serial.print(",");
            Serial.print(lng, 6);
            Serial.print(" SAT:");
            Serial.println(sat);
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
        if (millis() - lastBLETime < BLE_INTERVAL) return;
        
        // Send data via Bluetooth Serial if client connected
        if (bluetoothClientConnected && (m_state & STATE_BLE_READY)) {
            String bleData = formatDataForBLE();
            if (bleData.length() > 0) {
                SerialBT.println(bleData);
                Serial.print("BT Data sent: ");
                Serial.println(bleData);
            }
        }
        
        lastBLETime = millis();
    }
    
    String formatDataForBLE()
    {
        // Format the latest data for BLE transmission
        // This should match the format expected by the Freematics BLE Android app
        String data = "";
        
        // Add timestamp
        data += String(millis());
        data += ",";
        
        // Add any available OBD data
        int value;
        if (obd.readPID(0x0C, value)) {
            data += "RPM:" + String(value) + ";";
        }
        if (obd.readPID(0x0D, value)) {
            data += "SPD:" + String(value) + ";";
        }
        
        // Add GPS data if available
        float lat, lng;
        uint8_t sat;
        if (gps.getData(lat, lng, sat)) {
            data += "GPS:" + String(lat, 6) + "," + String(lng, 6) + ";";
        }
        
        return data;
    }
    
    uint16_t m_state = 0;
    TeleStore store;
    SimpleOBD obd;
    SimpleGPS gps;
};

// Bluetooth callback function
void bluetoothCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    if (event == ESP_SPP_SRV_OPEN_EVT) {
        // Get client address
        char clientAddress[18];
        sprintf(clientAddress, "%02X:%02X:%02X:%02X:%02X:%02X",
                param->srv_open.rem_bda[0], param->srv_open.rem_bda[1],
                param->srv_open.rem_bda[2], param->srv_open.rem_bda[3],
                param->srv_open.rem_bda[4], param->srv_open.rem_bda[5]);
        
        Serial.print("Bluetooth client connected: ");
        Serial.println(clientAddress);
        
        // Check if client address is in expected list
        bool isExpectedClient = false;
        for (int i = 0; i < NUM_EXPECTED_ADDRESSES; i++) {
            if (EXPECTED_BT_ADDRESSES[i].equals(clientAddress)) {
                isExpectedClient = true;
                break;
            }
        }
        
        if (isExpectedClient) {
            bluetoothClientConnected = true;
            Serial.println("Expected Freematics client connected - LED will stay on");
        } else {
            Serial.println("Warning: Unexpected client connected");
            Serial.println("Expected addresses:");
            for (int i = 0; i < NUM_EXPECTED_ADDRESSES; i++) {
                Serial.println("  " + EXPECTED_BT_ADDRESSES[i]);
            }
            bluetoothClientConnected = true; // Still allow connection
        }
    } else if (event == ESP_SPP_CLOSE_EVT) {
        Serial.println("Bluetooth client disconnected - LED will blink");
        bluetoothClientConnected = false;
    }
}

CustomFreematicsLogger logger;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Freematics Custom Sketch Starting (Simplified Mode)...");
    Serial.println("Expected Bluetooth clients:");
    for (int i = 0; i < NUM_EXPECTED_ADDRESSES; i++) {
        Serial.println("  " + EXPECTED_BT_ADDRESSES[i]);
    }
    Serial.println("LED will blink until a client connects...");
    
    // Initialize the logger
    if (logger.init()) {
        Serial.println("Logger initialized successfully");
    } else {
        Serial.println("Logger initialization failed");
    }
    
    Serial.println("Setup complete. Starting main loop...");
    Serial.println("Waiting for Bluetooth connection...");
}

void loop()
{
    logger.process();
    delay(100);
}
