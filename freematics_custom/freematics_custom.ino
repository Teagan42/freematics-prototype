/*************************************************************************
* Custom Freematics Sketch
* Developed for debugging with Freematics BLE Android app
* Based on Freematics ONE+ ESP32 platform
*************************************************************************/

#include <FreematicsPlus.h>
#include "config.h"
#include "telestore.h"

// working states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x40

typedef struct {
  float lat;
  float lng;
  uint8_t sat;
  uint16_t date;
  uint32_t time;
} CUSTOM_GPS_DATA;

class CustomFreematicsLogger : public COBD, public CGPS, public CFreematicsESP32
{
public:
    bool init()
    {
        bool success = true;
        
        // Initialize OBD
        Serial.print("OBD...");
        if (COBD::init()) {
            Serial.println("OK");
            m_state |= STATE_OBD_READY;
        } else {
            Serial.println("NO");
            success = false;
        }
        
        // Initialize GPS
        Serial.print("GPS...");
        if (CGPS::init()) {
            Serial.println("OK");
            m_state |= STATE_GPS_READY;
        } else {
            Serial.println("NO");
        }
        
        // Initialize MEMS
        Serial.print("MEMS...");
        if (memsInit()) {
            Serial.println("OK");
            m_state |= STATE_MEMS_READY;
        } else {
            Serial.println("NO");
        }
        
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
    void processOBD()
    {
        static uint32_t lastOBDTime = 0;
        if (millis() - lastOBDTime < OBD_INTERVAL) return;
        
        int value;
        // Read engine RPM
        if (readPID(PID_RPM, value)) {
            store.log(PID_RPM, value);
            Serial.print("RPM: ");
            Serial.println(value);
        }
        
        // Read vehicle speed
        if (readPID(PID_SPEED, value)) {
            store.log(PID_SPEED, value);
            Serial.print("Speed: ");
            Serial.println(value);
        }
        
        // Read engine coolant temperature
        if (readPID(PID_COOLANT_TEMP, value)) {
            store.log(PID_COOLANT_TEMP, value);
            Serial.print("Coolant Temp: ");
            Serial.println(value);
        }
        
        lastOBDTime = millis();
    }
    
    void processGPS()
    {
        static uint32_t lastGPSTime = 0;
        if (millis() - lastGPSTime < GPS_INTERVAL) return;
        
        CUSTOM_GPS_DATA gd = {0};
        if (getLocation(&gd.lat, &gd.lng)) {
            gd.sat = satellites();
            gd.date = date();
            gd.time = time();
            
            store.log(0x20, (int32_t)(gd.lat * 1000000));
            store.log(0x21, (int32_t)(gd.lng * 1000000));
            store.log(0x22, gd.sat);
            
            Serial.print("GPS: ");
            Serial.print(gd.lat, 6);
            Serial.print(",");
            Serial.print(gd.lng, 6);
            Serial.print(" SAT:");
            Serial.println(gd.sat);
        }
        
        lastGPSTime = millis();
    }
    
    void processMEMS()
    {
        static uint32_t lastMEMSTime = 0;
        if (millis() - lastMEMSTime < MEMS_INTERVAL) return;
        
        MEMS_DATA md;
        if (memsRead(&md)) {
            store.log(0x10, (int16_t)(md.acc[0] * 100));
            store.log(0x11, (int16_t)(md.acc[1] * 100));
            store.log(0x12, (int16_t)(md.acc[2] * 100));
            
            Serial.print("ACC: ");
            Serial.print(md.acc[0]);
            Serial.print(",");
            Serial.print(md.acc[1]);
            Serial.print(",");
            Serial.println(md.acc[2]);
        }
        
        lastMEMSTime = millis();
    }
    
    void sendBLEData()
    {
        static uint32_t lastBLETime = 0;
        if (millis() - lastBLETime < BLE_INTERVAL) return;
        
        // Send data via BLE for debugging with Android app
        if (store.getDataCount() > 0) {
            // Format data for BLE transmission
            String bleData = formatDataForBLE();
            if (bleData.length() > 0) {
                // Send via BLE (implementation depends on specific BLE library)
                Serial.print("BLE Data: ");
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
        if (readPID(PID_RPM, value)) {
            data += "RPM:" + String(value) + ";";
        }
        if (readPID(PID_SPEED, value)) {
            data += "SPD:" + String(value) + ";";
        }
        
        // Add GPS data if available
        float lat, lng;
        if (getLocation(&lat, &lng)) {
            data += "GPS:" + String(lat, 6) + "," + String(lng, 6) + ";";
        }
        
        return data;
    }
    
    uint16_t m_state = 0;
    TeleStore store;
};

CustomFreematicsLogger logger;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Freematics Custom Sketch Starting...");
    
    // Initialize the logger
    if (logger.init()) {
        Serial.println("Logger initialized successfully");
    } else {
        Serial.println("Logger initialization failed");
    }
    
    Serial.println("Setup complete. Starting main loop...");
}

void loop()
{
    logger.process();
    delay(100);
}
