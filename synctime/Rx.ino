#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 

// ================= CONSTANTS & CONFIG =================
#define RECEIVE_CHANNEL 1 
#define SERIAL_BAUD     115200

// *** IMPORTANT: REPLACE WITH YOUR TX UNIT'S MAC ADDRESS ***
uint8_t tx_peer_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// ================= DATA STRUCTURES =================

// 1. Time Sync Structure (Used to send time from RX back to TX)
struct __attribute__((packed)) TimeSyncPacket {
    int64_t unix_us; // Unix Time in Microseconds
};

// 2. Data Structure (Used to receive MPU/GPS data from TX)
struct __attribute__((packed)) dataRx {
    int64_t t_us;             
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
};

// ================= ESP-NOW CALLBACKS =================

// *** FIX 2: Updated OnDataSent signature for newer IDF (v5.5) ***
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // We can still extract the MAC if needed, but the original logic didn't use it.
  if (status == ESP_NOW_SEND_SUCCESS) {
    // Serial.println("Time Sync packet relayed successfully to TX unit.");
  } else {
    // Serial.println("Warning: Time Sync packet failed to relay to TX unit.");
  }
}

// Callback executed when an ESP-NOW packet is received (from the TX unit)
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
    dataRx receivedData;
    
    if (len != sizeof(dataRx)) {
        return;
    }
    
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    float hdop = (float)receivedData.hdop_value / 100.0;
    dataRx& pkt = receivedData; 

    // Print all 20 data fields
    Serial.printf(
        "%lld," 
        "%0.6f,%0.6f,%0.6f," 
        "%0.6f,%0.6f,%0.6f," 
        "%0.6f,%0.6f,%0.6f," 
        "%0.2f," 
        "%0.2f,%0.2f,%0.2f," 
        "%0.6f,%0.6f,"
        "%0.2f,%0.2f," 
        "%u,%0.2f\n",
        
        pkt.t_us,
        pkt.acc[0], pkt.acc[1], pkt.acc[2],
        pkt.gyro[0], pkt.gyro[1], pkt.gyro[2],
        pkt.mag[0], pkt.mag[1], pkt.mag[2],
        pkt.temp,
        pkt.roll, pkt.pitch, pkt.yaw,
        pkt.lat, pkt.lng,
        pkt.speed_kmph, pkt.alt_meters,
        pkt.satellites, hdop 
    );
}


// ================= SERIAL HANDLER (Blocking Wait) =================
void waitForTimeSync() {
    Serial.println("\n--- WAITING FOR PC TIME SYNC COMMAND (Press SYNC button in Python app) ---");
    
    bool synced = false;
    while (!synced) {
        if (Serial.available() > 0) {
            if (Serial.read() == 'T') {
                String timeStr = Serial.readStringUntil('\n');
                timeStr.trim();
                
                if (timeStr.length() > 5) { 
                    
                    // *** FIX 1: Use strtoll() for 64-bit integer conversion ***
                    // Convert String to a null-terminated char array (const char*)
                    const char* time_cstr = timeStr.c_str(); 
                    // Use strtoll to convert the string to a long long (int64_t)
                    int64_t pc_unix_us = strtoll(time_cstr, NULL, 10); 
                    
                    if (pc_unix_us > 1000000) { // Simple check to ensure a non-zero time was received
                        TimeSyncPacket syncPacket;
                        syncPacket.unix_us = pc_unix_us;
                        
                        // Send the time sync packet via ESP-NOW to the TX unit
                        esp_now_send(tx_peer_mac, (uint8_t *) &syncPacket, sizeof(TimeSyncPacket));
                        
                        Serial.printf("✅ Time Sync Received: Unix %lld us. Relayed to TX unit.\n", pc_unix_us);
                        synced = true;
                    } else {
                         Serial.println("Warning: Received invalid or zero timestamp. Waiting again.");
                    }
                }
            }
        }
        // Small delay to prevent watchdog timeout during the wait
        delay(10); 
    }
}


// ================= SETUP =================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);
    Serial.println("\n--- Starting ESP-NOW Receiver ---");

    // 1. WiFi & Channel Setup
    WiFi.mode(WIFI_STA);
    if (esp_wifi_set_channel(RECEIVE_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        Serial.println("Warning: Failed to set Wi-Fi channel.");
    }
    Serial.printf("RX MAC Address: %s\n", WiFi.macAddress().c_str());

    // 2. ESP-NOW Initialization
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // 3. Register callbacks
    // The OnDataSent signature now matches the required esp_now_send_cb_t type.
    esp_now_register_send_cb(OnDataSent);      
    esp_now_register_recv_cb(OnDataRecv);      

    // 4. Register the TX unit as a peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, tx_peer_mac, 6);
    peerInfo.channel = RECEIVE_CHANNEL;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add TX peer. Check MAC address.");
      return;
    }

    // 5. Blocking Wait for Time Sync
    waitForTimeSync();

    Serial.println("Setup Complete. Ready to receive data and log.");
    Serial.println("\nCSV Header:");
    Serial.println("t_us,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Temp,Roll,Pitch,Yaw,Lat,Lng,Speed_kmph,Alt_meters,Satellites,HDOP");
}

// ================= LOOP =================
void loop() {
    // Empty loop, reception is handled by the OnDataRecv callback
    delay(1); 
}
