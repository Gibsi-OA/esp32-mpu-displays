#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h" // Required for portMUX_TYPE
#include "freertos/task.h"     // Required for portMUX_TYPE

// ================= SETTINGS =================
#define BAUD_RATE           115200     // Serial monitor baud rate
// NOTE: This MAC address MUST be the actual MAC of the TX Unit 
// so that the RX unit can forward the time sync command back to it.
// *** REPLACE WITH TX UNIT'S ACTUAL MAC ADDRESS ***
uint8_t tx_peer_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// ================= GLOBAL VARIABLES (Defined early to prevent compilation errors) =================
char serial_buffer[64]; 
int buffer_index = 0;

// PC time received via sync
volatile int64_t pc_time_us = 0; // The last absolute PC time received by the RX unit

// Start time for ESP timer (for RX unit's internal reference)
int64_t start_esp_us = 0;
bool start_time_initialized = false;

// Mutex for critical sections
portMUX_TYPE timeSyncMux = portMUX_INITIALIZER_UNLOCKED;

// ================= DATA STRUCTURES (Defined early to prevent compilation errors) =================
struct __attribute__((packed)) TimeSyncPacket {
    int64_t unix_us; // PC Unix Time in Microseconds
};

struct __attribute__((packed)) dataTx {
    int64_t esp_elapsed_us; // microseconds since start (TX time)
    int64_t pc_us;          // timestamp from PC (TX time)
    uint32_t utc_date;
    uint32_t utc_time;
    float acc[3];
    float gyro[3];
    float mag[3];
    float temp;
    float roll, pitch, yaw;
    double lat;
    double lng;
    float speed_kmph;
    float alt_meters;
    uint32_t satellites;
    uint32_t hdop_value;
    float heading_deg;
};


// ================= ESP-NOW CALLBACKS =================

void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {}

// CONSOLIDATED RECEIVE CALLBACK: Handles both TimeSyncPacket and dataTx packets
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t * incomingData, int len) {
    
    // --- CASE 1: TIME SYNC PACKET (len == sizeof(TimeSyncPacket)) ---
    if (len == sizeof(TimeSyncPacket)) {
        TimeSyncPacket sync;
        memcpy(&sync, incomingData, sizeof(sync));

        if(sync.unix_us > 1000000000000000LL) { 
            portENTER_CRITICAL(&timeSyncMux);
            pc_time_us = sync.unix_us; // Store the absolute PC time
            if(!start_time_initialized) {
                start_esp_us = esp_timer_get_time();
                start_time_initialized = true;
            }
            portEXIT_CRITICAL(&timeSyncMux);
            Serial.printf("TIME SYNC RECEIVED: %lld us from TX\n", pc_time_us); 
        }
    }
    
    // --- CASE 2: SENSOR DATA PACKET (len == sizeof(dataTx)) ---
    // This case only runs after pc_time_us is initialized (i.e., after sync)
    else if(len == sizeof(dataTx)) {
        dataTx receivedData;
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        // Print CSV line
        // NOTE: We use the time stamps (esp_elapsed_us and pc_us) CALCULATED BY THE TX UNIT.
        Serial.printf(
            "%lld,%lld,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%u,%.2f,%.2f\n",
            receivedData.esp_elapsed_us,  // The TX unit's elapsed time
            receivedData.pc_us,           // The CORRECT high-resolution PC time from the TX unit
            receivedData.utc_date,
            receivedData.utc_time,
            receivedData.acc[0], receivedData.acc[1], receivedData.acc[2],
            receivedData.gyro[0], receivedData.gyro[1], receivedData.gyro[2],
            receivedData.mag[0], receivedData.mag[1], receivedData.mag[2],
            receivedData.temp,
            receivedData.roll, receivedData.pitch, receivedData.yaw,
            receivedData.lat, receivedData.lng, receivedData.speed_kmph, receivedData.alt_meters,
            receivedData.satellites, (float)receivedData.hdop_value / 100.0,
            receivedData.heading_deg
        );
    }
}


// ================= TIME SYNC FROM PC =================
void forward_time_sync(int64_t unix_us) {
    TimeSyncPacket syncPacket;
    syncPacket.unix_us = unix_us;

    // Send the Time Sync packet to the TX peer (which must be correctly set)
    if(unix_us > 1000000000000000LL) {
        esp_now_send(tx_peer_mac, (uint8_t *)&syncPacket, sizeof(syncPacket));
        Serial.printf("LOG: Forwarded Time Sync: %lld us to TX.\n", unix_us);
    } else {
        Serial.println("LOG: Received invalid sync time from PC.");
    }
}

void check_serial_sync() {
    while(Serial.available()) {
        char in_char = Serial.read();
        if(in_char == '\n') {
            serial_buffer[buffer_index] = '\0';
            if(serial_buffer[0] == 'T') {
                int64_t unix_us = atoll(&serial_buffer[1]);
                forward_time_sync(unix_us);
            }
            buffer_index = 0;
        } else if(buffer_index < sizeof(serial_buffer)-1) {
            serial_buffer[buffer_index++] = in_char;
        } else {
            buffer_index = 0;
        }
    }
}

// ================= BLOCKING WAIT (New Function) =================
void waitForInitialTimeSync() {
    Serial.println("\n--- WAITING FOR INITIAL PC TIME SYNC ---");
    Serial.println("Send 'T' followed by the current Unix microsecond time from the PC.");
    
    // Block until the global pc_time_us is initialized with a valid time
    while (pc_time_us == 0) {
        
        // This processes the incoming 'T' command from the PC over Serial
        check_serial_sync(); 

        // The ESP-NOW sync response (from TX) is handled by OnDataRecv, 
        // which updates pc_time_us.

        // Allow ESP-NOW and Serial events to process
        delay(10); 
    }
    
    Serial.printf("\n--- INITIAL SYNC RECEIVED: %lld us ---\n", pc_time_us);
    Serial.println("RX Unit is now starting to receive and process data.");
    Serial.println("TX_ESP_elapsed_us,TX_PC_time_us,UTC_Date,UTC_Time,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Temp,Roll,Pitch,Yaw,Lat,Lng,Speed_kmph,Alt_meters,Satellites,HDOP,Heading_deg");
}


// ================= SETUP (Revised to include blocking wait) =================
void setup() {
    Serial.begin(BAUD_RATE);
    delay(500);
    Serial.println("\nStarting RX Unit - Waiting for PC Sync via Serial.");
    Serial.printf("Note: Replace tx_peer_mac (currently %02X:%02X:%02X:%02X:%02X:%02X) with the TX Unit's actual MAC.\n",
                  tx_peer_mac[0], tx_peer_mac[1], tx_peer_mac[2],
                  tx_peer_mac[3], tx_peer_mac[4], tx_peer_mac[5]);

    WiFi.mode(WIFI_STA);
    if(esp_now_init() != ESP_OK){
        Serial.println("ESP-NOW init failed!");
        while(1);
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Peer setup - required for sending the Time Sync back to the TX Unit
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, tx_peer_mac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if(esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add TX peer.");
    } else {
        Serial.printf("Registered TX peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                     tx_peer_mac[0], tx_peer_mac[1], tx_peer_mac[2],
                     tx_peer_mac[3], tx_peer_mac[4], tx_peer_mac[5]);
    }
    
    // *** NEW: BLOCKING WAIT FOR CRITICAL INITIAL SYNC ***
    waitForInitialTimeSync(); 
}

// ================= LOOP (Simplified) =================
void loop() {
    // Keep checking for subsequent time sync commands from PC
    check_serial_sync();
    delay(1);
}