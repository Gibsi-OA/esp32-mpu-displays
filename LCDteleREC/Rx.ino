#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ================= SETTINGS =================
#define BAUD_RATE           115200

// REPLACE WITH TX UNIT'S ACTUAL MAC ADDRESS
uint8_t tx_peer_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// ================= GLOBAL VARIABLES =================
char serial_buffer[64]; 
int buffer_index = 0;
volatile int64_t pc_time_us = 0; 
portMUX_TYPE timeSyncMux = portMUX_INITIALIZER_UNLOCKED;

// ================= DATA STRUCTURES =================
// MUST MATCH TX STRUCT EXACTLY OR DATA WILL NOT BE RECEIVED
struct __attribute__((packed)) TimeSyncPacket {
    int64_t unix_us;
};

struct __attribute__((packed)) dataTx {
    int64_t esp_elapsed_us;
    int64_t pc_us;         
    uint32_t utc_date;
    uint32_t utc_time;
    float acc[3];
    float gyro[3];
    float mag[3];       // Now included in output
    float temp;
    float roll, pitch, yaw; // Yaw now included in output
    double lat;
    double lng;
    float speed_kmph;
    float alt_meters;   // Now included in output
    uint32_t satellites;
    uint32_t hdop_value; // Now included in output
    float heading_deg;
    bool REC;
    char filename[14];
};

// ================= ESP-NOW CALLBACKS =================
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t * incomingData, int len) {
    
    if (len == sizeof(TimeSyncPacket)) {
        TimeSyncPacket sync;
        memcpy(&sync, incomingData, sizeof(sync));
        if(sync.unix_us > 1000000000000000LL) { 
            portENTER_CRITICAL(&timeSyncMux);
            pc_time_us = sync.unix_us;
            portEXIT_CRITICAL(&timeSyncMux);
            Serial.printf("\nTIME SYNC CONFIRMED: %lld us\n", pc_time_us); 
        }
    }
    
    else if(len == sizeof(dataTx)) {
        dataTx data;
        memcpy(&data, incomingData, sizeof(data));
        
        // --- FULL SERIAL OUTPUT INCLUDING MAG, YAW, ALT, HDOP ---
        Serial.printf(
            "%lld,%lld,%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%u,%u,%.2f,%d,%s\n",
            data.esp_elapsed_us,
            data.pc_us,
            data.utc_date,
            data.utc_time,
            data.acc[0], data.acc[1], data.acc[2],
            data.gyro[0], data.gyro[1], data.gyro[2],
            data.mag[0], data.mag[1], data.mag[2],  // Added magnetometer
            data.temp,
            data.roll, data.pitch, data.yaw,  // Added yaw
            data.lat, data.lng,
            data.speed_kmph,
            data.alt_meters,  // Added altitude
            data.satellites,
            data.hdop_value,  // Added HDOP
            data.heading_deg,
            data.REC,
            data.filename
        );
    }
}

// ================= SERIAL & SYNC =================
void check_serial_sync() {
    while(Serial.available()) {
        char in_char = Serial.read();
        if(in_char == '\n') {
            serial_buffer[buffer_index] = '\0';
            if(serial_buffer[0] == 'T') {
                int64_t unix_us = atoll(&serial_buffer[1]);
                TimeSyncPacket syncPacket;
                syncPacket.unix_us = unix_us;
                esp_now_send(tx_peer_mac, (uint8_t *)&syncPacket, sizeof(syncPacket));
                Serial.printf("LOG: Sync Command Sent to TX: %lld\n", unix_us);
            }
            buffer_index = 0;
        } else if(buffer_index < 63) {
            serial_buffer[buffer_index++] = in_char;
        }
    }
}

void setup() {
    Serial.begin(BAUD_RATE);
    WiFi.mode(WIFI_STA);
    
    if(esp_now_init() != ESP_OK) return;
    esp_now_register_recv_cb(OnDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, tx_peer_mac, 6);
    peerInfo.channel = 1;
    esp_now_add_peer(&peerInfo);
    
    Serial.println("RX Ready. Send 'T[timestamp]' to begin.");
    
    // Updated CSV Header with ALL fields
    Serial.println("ESP_us,PC_us,Date,Time,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX,MagY,MagZ,Temp,Roll,Pitch,Yaw,Lat,Lng,Speed,Alt,Sats,HDOP,Heading,REC,FileName");
}

void loop() {
    check_serial_sync();
    delay(1);
}