#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <time.h>
#include <sys/time.h>

// ================= SETTINGS =================
#define BAUD_RATE           115200      // Serial monitor baud rate
uint8_t tx_peer_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Replace with the TX unit's MAC address

// ================= GLOBAL VARIABLES =================
// Buffer to hold the incoming serial command like "T1672531200000000\n"
char serial_buffer[64]; 
int buffer_index = 0;

// ================= DATA STRUCTURES =================
// 1. Time Sync Structure (Sent from RX to TX)
struct __attribute__((packed)) TimeSyncPacket {
    int64_t unix_us; // Unix Time in Microseconds
};

// 2. Main Data Structure (Received from TX) - MUST match TX unit
struct __attribute__((packed)) dataTx {
    int64_t t_us;
    uint32_t utc_date; // YYYYMMDD
    uint32_t utc_time; // HHMMSScc
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

void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        // Serial.println("Time Sync Packet sent successfully to TX.");
    } 
}

/**
 * @brief Callback function executed when data is received (from the TX unit).
 * The received data is formatted into a single CSV line and sent to the PC via Serial.
 */
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t * incomingData, int len) {
    if (len == sizeof(dataTx)) { 
        dataTx receivedData;
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        // Format the received data as a single CSV line for the Python script
        // 23 fields total
        Serial.printf(
            "%lld,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%u,%.2f,%.2f\n",
            receivedData.t_us,
            receivedData.utc_date,
            receivedData.utc_time,
            receivedData.acc[0], receivedData.acc[1], receivedData.acc[2],
            receivedData.gyro[0], receivedData.gyro[1], receivedData.gyro[2],
            receivedData.mag[0], receivedData.mag[1], receivedData.mag[2],
            receivedData.temp,
            receivedData.roll, receivedData.pitch, receivedData.yaw,
            receivedData.lat, receivedData.lng, receivedData.speed_kmph, receivedData.alt_meters,
            receivedData.satellites, (float)receivedData.hdop_value / 100.0, // HDOP
            receivedData.heading_deg // HEADING
        );
        
    } 
}

// ================= TIME SYNC FROM PC =================

/**
 * @brief Sends the Unix timestamp received from the PC to the TX unit via ESP-NOW.
 */
void forward_time_sync(int64_t unix_us) {
    TimeSyncPacket syncPacket;
    syncPacket.unix_us = unix_us;

    if (unix_us > 1000000000000000LL) {
        esp_now_send(tx_peer_mac, (uint8_t *)&syncPacket, sizeof(syncPacket));
        Serial.printf("LOG: Forwarded Time Sync: %lld us to TX.\n", unix_us);
    } else {
        Serial.println("LOG: Received invalid sync time from PC.");
    }
}

/**
 * @brief Checks the serial buffer for the 'T<timestamp>\n' command sent by Python.
 * This is the critical change to make the RX unit listen for the sync command.
 */
void check_serial_sync() {
    while (Serial.available()) {
        char in_char = Serial.read();
        
        if (in_char == '\n') {
            serial_buffer[buffer_index] = '\0'; // Null-terminate the string
            
            // Command format: T<Unix_Time_us>
            if (serial_buffer[0] == 'T') {
                char* timestamp_str = &serial_buffer[1];
                // Use atoll to convert the string to int64_t (long long)
                int64_t unix_us = atoll(timestamp_str); 
                forward_time_sync(unix_us);
            }
            
            // Reset buffer
            buffer_index = 0;
            serial_buffer[0] = '\0';
            
        } else if (buffer_index < sizeof(serial_buffer) - 1) {
            serial_buffer[buffer_index++] = in_char;
        } else {
            // Buffer overflow, reset
            buffer_index = 0;
        }
    }
}


// ================= SETUP =================
void setup() {
    Serial.begin(BAUD_RATE);
    delay(500);
    Serial.println("\nStarting RX Unit - Waiting for PC Sync via Serial.");

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        while(1);
    }
    
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, tx_peer_mac, 6); 
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add TX peer.");
    } else {
        Serial.printf("Registered TX peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      tx_peer_mac[0], tx_peer_mac[1], tx_peer_mac[2], tx_peer_mac[3], tx_peer_mac[4], tx_peer_mac[5]);
    }

    Serial.println("RX READY. Use Python script to send sync command.");
}

// ================= LOOP =================
void loop() {
    // Check the serial port for the new command from the Python script
    check_serial_sync();
    
    // Data reception is handled by the OnDataRecv callback
    delay(1); 
}
