/*************************************************************************
* Custom Freematics Sketch - Simplified Version
* Developed for debugging with Freematics BLE Android app
* Compatible with ESP32 Arduino Core 3.2.0+
*************************************************************************/

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

// Forward declaration of bluetooth callback
void bluetoothCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

// Minimal OBD simulation
class SimpleOBD {
public:
    bool init() { return true; }
    bool readPID(uint8_t pid, int& value) {
        switch(pid) {
            case 0x0C: value = 1500 + random(-200, 200); return true;
            case 0x0D: value = 60 + random(-10, 10); return true;
            case 0x05: value = 85 + random(-5, 5); return true;
            default: return false;
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
        
        // Only process data if BLE client is connected
        if (bluetoothClientConnected) {
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
        if (millis() - lastBLETime < BLE_INTERVAL) return;
        
        if (bluetoothClientConnected && (m_state & STATE_BLE_READY)) {
            String bleData = formatDataForBLE();
            if (bleData.length() > 0) {
                SerialBT.println(bleData);
            }
        }
        
        lastBLETime = millis();
    }
    
    String formatDataForBLE()
    {
        String data = String(millis()) + ",";
        
        int value;
        if (obd.readPID(0x0C, value)) data += "RPM:" + String(value) + ";";
        if (obd.readPID(0x0D, value)) data += "SPD:" + String(value) + ";";
        
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
        char clientAddress[18];
        sprintf(clientAddress, "%02X:%02X:%02X:%02X:%02X:%02X",
                param->srv_open.rem_bda[0], param->srv_open.rem_bda[1],
                param->srv_open.rem_bda[2], param->srv_open.rem_bda[3],
                param->srv_open.rem_bda[4], param->srv_open.rem_bda[5]);
        
        // Check if client address is in expected list
        bool isExpectedClient = false;
        for (int i = 0; i < NUM_EXPECTED_ADDRESSES; i++) {
            if (EXPECTED_BT_ADDRESSES[i].equals(String(clientAddress))) {
                isExpectedClient = true;
                break;
            }
        }
        
        if (isExpectedClient) {
            bluetoothClientConnected = true;
            Serial.println("BT connected (authorized): " + String(clientAddress));
            Serial.println("Starting data collection...");
        } else {
            Serial.println("BT connection rejected (unauthorized): " + String(clientAddress));
            // Disconnect unauthorized client
            SerialBT.disconnect();
        }
    } else if (event == ESP_SPP_CLOSE_EVT) {
        bluetoothClientConnected = false;
        Serial.println("BT disconnected");
        Serial.println("Stopping data collection, waiting for BLE client...");
    }
}

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
    if (bluetoothClientConnected) {
        logger.process();
    } else {
        // Just handle LED blinking while waiting for connection
        logger.handleLedOnly();
    }
    delay(100);
}
