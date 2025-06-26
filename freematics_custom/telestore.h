#ifndef TELESTORE_H
#define TELESTORE_H

#include <Arduino.h>
#include "config.h"

typedef struct {
    uint32_t timestamp;
    uint16_t pid;
    int32_t value;
} LOG_DATA;

class TeleStore {
public:
    bool init() {
        m_dataCount = 0;
        m_writeIndex = 0;
        Serial.println("TeleStore initialized");
        return true;
    }
    
    void log(uint16_t pid, int32_t value) {
        if (m_dataCount < MAX_LOG_ENTRIES) {
            m_data[m_writeIndex].timestamp = millis();
            m_data[m_writeIndex].pid = pid;
            m_data[m_writeIndex].value = value;
            
            m_writeIndex = (m_writeIndex + 1) % MAX_LOG_ENTRIES;
            if (m_dataCount < MAX_LOG_ENTRIES) {
                m_dataCount++;
            }
        }
    }
    
    uint16_t getDataCount() {
        return m_dataCount;
    }
    
    LOG_DATA* getData(uint16_t index) {
        if (index < m_dataCount) {
            return &m_data[index];
        }
        return nullptr;
    }
    
    void clear() {
        m_dataCount = 0;
        m_writeIndex = 0;
    }
    
private:
    LOG_DATA m_data[MAX_LOG_ENTRIES];
    uint16_t m_dataCount;
    uint16_t m_writeIndex;
};

#endif // TELESTORE_H
