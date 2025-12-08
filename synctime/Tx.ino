#include <SPI.h>
#include <SD.h>
#include <MPU9250_WE.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include "MadgwickAHRS.h"
#include <TinyGPS++.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ================= SETTINGS =================
#define SAMPLE_RATE_HZ      4000        // MPU sample rate
#define ESPNOW_RATE_HZ      250         // ESP-NOW send rate
#define GPS_BAUD            9600        // GPS module baud rate
#define BUFFER_SIZE         512
#define SD_WRITE_BATCH      16
#define HSPI_MISO           26
#define HSPI_MOSI           27
#define HSPI_SCLK           25
#define HSPI_CS             14
#define SD_CS_PIN           5
#define RXD2                16          // GPS RX pin
#define TXD2                17          // GPS TX pin

// *** IMPORTANT: REPLACE WITH YOUR RECEIVER (RX) UNIT'S MAC ADDRESS ***
// The RX unit sends the TimeSyncPacket back to this TX unit.
uint8_t rx_peer_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// ================= OBJECTS & SHARED DATA =================
SPIClass hspi(HSPI);
SPIClass sdSPI(VSPI);
MPU9250_WE mpu(&hspi, HSPI_CS, true);
Madgwick filter;
File dataFile;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Use a structure and a mutex to safely share GPS data between the tasks
struct gpsData {
    double lat;
    double lng;
    float speed_kmph;
    float alt_meters;
    uint32_t satellites;
    uint32_t hdop_value; // HDOP * 100
};
gpsData currentGpsData = {0, 0, 0, 0, 0, 0};
portMUX_TYPE gpsDataMux = portMUX_INITIALIZER_UNLOCKED;

// ================= TIME SYNCHRONIZATION VARIABLES =================
volatile int64_t last_sync_unix_us = 0; // Unix time at the moment of sync
volatile int64_t last_sync_esp_us = 0;  // ESP timer value at the moment of sync
volatile bool time_synced = false;      // Flag to track if sync has occurred
portMUX_TYPE timeSyncMux = portMUX_INITIALIZER_UNLOCKED; // Mutex for sync variables

// ================= DATA STRUCTURES =================
// 1. Time Sync Structure (Matches the packet sent by the RX unit)
struct __attribute__((packed)) TimeSyncPacket {
    int64_t unix_us; // Unix Time in Microseconds
};

// 2. Main Data Structure
struct __attribute__((packed)) dataTx {
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

volatile dataTx ring[BUFFER_SIZE];
volatile uint16_t writeIndex = 0;
volatile uint16_t sdReadIndex = 0;
volatile uint16_t espNowReadIndex = 0;

// ================= SYNCHRONIZED TIME FUNCTION =================
/**
 * Calculates the current time by adding the elapsed ESP timer ticks
 * since the last sync event to the base Unix time received from the PC.
 */
int64_t getAccurateTimeUS() {
    int64_t current_time;
    portENTER_CRITICAL_ISR(&timeSyncMux);
    
    if (!time_synced) {
        // If not synced yet, return the raw ESP timer
        current_time = esp_timer_get_time(); 
    } else {
        // Calculate the time elapsed since the last sync event
        int64_t current_esp_us = esp_timer_get_time();
        int64_t elapsed_us = current_esp_us - last_sync_esp_us;
        
        // Return Unix time = Base Unix Time + Elapsed Time
        current_time = last_sync_unix_us + elapsed_us;
    }
    
    portEXIT_CRITICAL_ISR(&timeSyncMux);
    return current_time;
}


// ================= ESP-NOW CALLBACKS =================
// The TX unit needs to receive the TimeSyncPacket from the RX unit

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Standard send callback
}

/**
 * @brief Callback function executed when data is received (from the RX unit).
 * * We only use this to listen for the TimeSyncPacket.
 */
void OnTimeSyncRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
    // Only process the packet if we haven't synced yet AND the size matches the TimeSyncPacket
    if (!time_synced && len == sizeof(TimeSyncPacket)) {
        TimeSyncPacket receivedSync;
        memcpy(&receivedSync, incomingData, sizeof(TimeSyncPacket));
        
        // Use a critical section since this modifies shared volatile variables
        portENTER_CRITICAL(&timeSyncMux);
        if (receivedSync.unix_us > 1000000) { 
            last_sync_unix_us = receivedSync.unix_us;
            last_sync_esp_us = esp_timer_get_time();
            time_synced = true; // Set the flag
            
            Serial.printf("TIME SYNC SUCCESSFUL: Base Unix Time set to %lld us.\n", last_sync_unix_us);
            
            // Log sync event to SD card
            if (dataFile) {
                // Log sync event as a special entry
                dataFile.printf("SYNC_EVENT_US,%lld\n", last_sync_unix_us);
                dataFile.flush(); 
            }
        } else {
            Serial.println("Warning: Received invalid sync time.");
        }
        portEXIT_CRITICAL(&timeSyncMux);
    } 
}

// ================= SD FILE =================
char DATA_FILE_PATH[32];
bool createSequentialFile() {
    int fileIndex = 1;
    while (fileIndex <= 9999) {
        snprintf(DATA_FILE_PATH, sizeof(DATA_FILE_PATH), "/data_%04d.csv", fileIndex);
        if (!SD.exists(DATA_FILE_PATH)) {
            dataFile = SD.open(DATA_FILE_PATH, FILE_WRITE);
            if (!dataFile) return false;
            // Updated CSV header to include GPS data
            dataFile.println("t_us,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Temp,Roll,Pitch,Yaw,Lat,Lng,Speed_kmph,Alt_meters,Satellites,HDOP");
            dataFile.flush();
            Serial.printf("SD initialized, writing to file: %s\n", DATA_FILE_PATH);
            return true;
        }
        fileIndex++;
    }
    return false;
}

// ================= TIMER CALLBACK (Core 0 ISR) =================
void IRAM_ATTR samplingCallback(void* arg) {
    int idx = writeIndex;

    // 💥 CRITICAL CHANGE: Use the synchronized time function
    ring[idx].t_us = getAccurateTimeUS();

    // Read MPU9250 (UNCHANGED)
    xyzFloat acc = mpu.getGValues();
    xyzFloat gyr = mpu.getGyrValues();
    xyzFloat mag = mpu.getMagValues();
    float temp = mpu.getTemperature();

    ring[idx].acc[0] = acc.x; ring[idx].acc[1] = acc.y; ring[idx].acc[2] = acc.z;
    ring[idx].gyro[0] = gyr.x; ring[idx].gyro[1] = gyr.y; ring[idx].gyro[2] = gyr.z;
    ring[idx].mag[0] = mag.x; ring[idx].mag[1] = mag.y; ring[idx].mag[2] = mag.z;
    ring[idx].temp = temp;

    // Madgwick filter update (UNCHANGED)
    filter.update(gyr.x * DEG_TO_RAD, gyr.y * DEG_TO_RAD, gyr.z * DEG_TO_RAD,
                    acc.x, acc.y, acc.z,
                    mag.x, mag.y, mag.z);

    ring[idx].roll  = filter.getRoll();
    ring[idx].pitch = filter.getPitch();
    ring[idx].yaw   = filter.getYaw();

    // Safely read and store the latest GPS data (UNCHANGED)
    portENTER_CRITICAL_ISR(&gpsDataMux);
    ring[idx].lat = currentGpsData.lat;
    ring[idx].lng = currentGpsData.lng;
    ring[idx].speed_kmph = currentGpsData.speed_kmph;
    ring[idx].alt_meters = currentGpsData.alt_meters;
    ring[idx].satellites = currentGpsData.satellites;
    ring[idx].hdop_value = currentGpsData.hdop_value;
    portEXIT_CRITICAL_ISR(&gpsDataMux);

    writeIndex = (writeIndex + 1) % BUFFER_SIZE;
}

// 🚦 GPS Task (Core 1) (UNCHANGED) 🚦
void gpsTask(void* parameter) {
    for (;;) {
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isUpdated()) {
                    portENTER_CRITICAL(&gpsDataMux);
                    currentGpsData.lat = gps.location.lat();
                    currentGpsData.lng = gps.location.lng();
                    currentGpsData.speed_kmph = gps.speed.kmph();
                    currentGpsData.alt_meters = gps.altitude.meters();
                    currentGpsData.satellites = gps.satellites.value();
                    currentGpsData.hdop_value = gps.hdop.value();
                    portEXIT_CRITICAL(&gpsDataMux);

                    Serial.print("GPS Update | LAT: ");
                    Serial.print(currentGpsData.lat, 6);
                    Serial.print(" | Sat: ");
                    Serial.println(currentGpsData.satellites);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ================= SD WRITER (UNCHANGED) =================
void writeBatchToSD() {
    if (!dataFile) return;

    uint16_t localRead = sdReadIndex;
    uint16_t localWrite = writeIndex;
    char csvBuffer[SD_WRITE_BATCH * 300]; 
    char* ptr = csvBuffer;
    int lenTotal = 0;
    int batchCount = 0;

    while (localRead != localWrite && batchCount < SD_WRITE_BATCH) {
        dataTx s = *(dataTx*)&ring[localRead];

        int len = snprintf(ptr, sizeof(csvBuffer) - lenTotal,
                         // Format string must match the 20 fields
                         "%lld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%u,%.2f\n", 
                         s.t_us,
                         s.acc[0], s.acc[1], s.acc[2],
                         s.gyro[0], s.gyro[1], s.gyro[2],
                         s.mag[0], s.mag[1], s.mag[2],
                         s.temp,
                         s.roll, s.pitch, s.yaw,
                         s.lat, s.lng, s.speed_kmph, s.alt_meters, s.satellites, (float)s.hdop_value / 100.0);
        if (len <= 0 || lenTotal + len >= sizeof(csvBuffer)) break;
        ptr += len;
        lenTotal += len;

        localRead = (localRead + 1) % BUFFER_SIZE;
        batchCount++;
    }

    if (lenTotal > 0) {
        dataFile.write((const uint8_t*)csvBuffer, lenTotal);
        dataFile.flush();
        sdReadIndex = localRead;
    }
}

// ================= ESP-NOW SEND (UNCHANGED) =================
void sendBatch() {
    uint16_t localRead = espNowReadIndex;
    uint16_t localWrite = writeIndex;

    while (localRead != localWrite) {
        dataTx s = *(dataTx*)&ring[localRead];

        // send one struct at a time
        esp_now_send(rx_peer_mac, (uint8_t*)&s, sizeof(s)); 

        localRead = (localRead + 1) % BUFFER_SIZE;
    }

    espNowReadIndex = localRead;
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Starting TX with MPU and GPS...");

    // MPU init 
    hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    if (!mpu.init()) while (1);
    mpu.initMagnetometer();
    mpu.setSampleRateDivider(0);
    mpu.enableGyrDLPF();
    mpu.setGyrDLPF(MPU9250_DLPF_6);
    mpu.setGyrRange(MPU9250_GYRO_RANGE_2000);
    mpu.setAccRange(MPU9250_ACC_RANGE_16G);
    mpu.enableAccDLPF(false);
    Serial.println("Calibrating gyro...");
    mpu.autoOffsets();
    Serial.println("Calibration done.");

    // SD init 
    sdSPI.begin();
    if (!SD.begin(SD_CS_PIN, sdSPI) || !createSequentialFile()) {
        Serial.println("SD init failed! Continuing without SD.");
    }
    
    // GPS Serial 2 init & Task 
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Serial 2 for GPS started.");
    xTaskCreatePinnedToCore(gpsTask, "GpsTask", 4096, NULL, 1, NULL, 1);
    Serial.println("GPS Task started on Core 1.");

    // ESP-NOW init 
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) Serial.println("ESP-NOW init failed");
    
    // 💥 Register RECEIVE callback for Time Sync
    esp_now_register_recv_cb(OnTimeSyncRecv);
    
    // Register the RX peer 
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, rx_peer_mac, 6); 
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add RX peer.");
    } else {
        Serial.printf("Registered RX peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    rx_peer_mac[0], rx_peer_mac[1], rx_peer_mac[2], rx_peer_mac[3], rx_peer_mac[4], rx_peer_mac[5]);
    }

    // Timer setup for 4 kHz sampling 
    const esp_timer_create_args_t timer_args = {
        .callback = &samplingCallback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "mpu_sampling"
    };
    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 1000000 / SAMPLE_RATE_HZ);
    Serial.printf("MPU Sampling Timer started at %d Hz on Core 0 ISR.\n", SAMPLE_RATE_HZ);
    
    Serial.println("Waiting for Time Sync command from RX unit...");
}

// ================= LOOP (Core 0 default task) =================
void loop() {
    // 💥 CRITICAL CHANGE: Pause sending and writing until synced
    if (!time_synced) {
        Serial.print(".");
        delay(500); 
        return; // Do not execute sendBatch or writeBatchToSD
    }

    // --- Execution only continues once time_synced is true ---
    static unsigned long lastSend = 0;
    unsigned long now = millis();

    // ESP-NOW send at 250 Hz
    if (now - lastSend >= 1000 / ESPNOW_RATE_HZ) {
        lastSend = now;
        sendBatch();
    }

    // SD write
    writeBatchToSD();

    delay(1); // small yield
}
