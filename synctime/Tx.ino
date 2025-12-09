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
#include <time.h>

// ================= SETTINGS =================
#define SAMPLE_RATE_HZ      4000
#define ESPNOW_RATE_HZ      250
#define GPS_BAUD            9600
#define BUFFER_SIZE         512
#define SD_WRITE_BATCH      16
#define HSPI_MISO           26
#define HSPI_MOSI           27
#define HSPI_SCLK           25
#define HSPI_CS             14
#define SD_CS_PIN           5
#define RXD2                16
#define TXD2                17

// Replace with your RX MAC if you want unicast; leave as broadcast if needed
uint8_t rx_peer_mac[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; 

// ================= OBJECTS & SHARED DATA =================
SPIClass hspi(HSPI);
SPIClass sdSPI(VSPI);
MPU9250_WE mpu(&hspi, HSPI_CS, true);
Madgwick filter;
File dataFile;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

struct gpsData {
    double lat;
    double lng;
    float speed_kmph;
    float alt_meters;
    uint32_t satellites;
    uint32_t hdop_value;
    float heading_deg;
};
gpsData currentGpsData = {0,0,0,0,0,0,0};
portMUX_TYPE gpsDataMux = portMUX_INITIALIZER_UNLOCKED;

// ================= TIME SYNCH VARIABLES =================
volatile int64_t last_sync_unix_us = 0;
volatile int64_t last_sync_esp_us = 0;
volatile bool time_synced = false;
portMUX_TYPE timeSyncMux = portMUX_INITIALIZER_UNLOCKED;
char sync_datetime_str[20] = "NOSYNC";

// ================= DATA STRUCTURES =================
struct __attribute__((packed)) TimeSyncPacket {
    int64_t unix_us;
};

struct __attribute__((packed)) dataTx {
    int64_t t_us;
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

volatile dataTx ring[BUFFER_SIZE];
volatile uint16_t writeIndex = 0;
volatile uint16_t sdReadIndex = 0;
volatile uint16_t espNowReadIndex = 0;

// *** FIX FOR STACK OVERFLOW: Move csvBuffer to global scope ***
char csvBuffer[SD_WRITE_BATCH*360];

// ================= SYNCHRONIZED TIME FUNCTIONS =================
int64_t getAccurateTimeUS() {
    int64_t current_time;
    portENTER_CRITICAL_ISR(&timeSyncMux);
    if (!time_synced) current_time = esp_timer_get_time();
    else current_time = last_sync_unix_us + (esp_timer_get_time() - last_sync_esp_us);
    portEXIT_CRITICAL_ISR(&timeSyncMux);
    return current_time;
}

void unixUSToDateTime(int64_t unix_us, uint32_t &date, uint32_t &time_val) {
    time_t unix_s = unix_us / 1000000;
    uint16_t hundredths = (unix_us % 1000000) / 10000;
    struct tm* tm_struct = gmtime(&unix_s);
    if (tm_struct != NULL) {
        date = (tm_struct->tm_year+1900)*10000 + (tm_struct->tm_mon+1)*100 + tm_struct->tm_mday;
        time_val = tm_struct->tm_hour*1000000 + tm_struct->tm_min*10000 + tm_struct->tm_sec*100 + hundredths;
    } else {
        date = 0;
        time_val = 0;
    }
}

// ================= ESP-NOW CALLBACKS =================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void OnTimeSyncRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
    if (len != sizeof(TimeSyncPacket)) return;

    TimeSyncPacket receivedSync;
    memcpy(&receivedSync, incomingData, sizeof(TimeSyncPacket));

    Serial.printf("TIME SYNC RECEIVED from MAC %02X:%02X:%02X:%02X:%02X:%02X | Unix us: %lld\n",
                  info->src_addr[0], info->src_addr[1], info->src_addr[2],
                  info->src_addr[3], info->src_addr[4], info->src_addr[5],
                  receivedSync.unix_us);

    portENTER_CRITICAL(&timeSyncMux);
    if (!time_synced && receivedSync.unix_us > 1000000) {
        last_sync_unix_us = receivedSync.unix_us;
        last_sync_esp_us = esp_timer_get_time();
        time_synced = true;

        time_t sync_s = last_sync_unix_us / 1000000;
    }

    portEXIT_CRITICAL(&timeSyncMux);
}

// ================= SD FILE =================
char DATA_FILE_PATH[64];
bool createSequentialFile() {
    int fileIndex = 1;
    while (fileIndex <= 9999) {
        snprintf(DATA_FILE_PATH, sizeof(DATA_FILE_PATH), "/data_%s_%04d.csv", sync_datetime_str, fileIndex);
        if (!SD.exists(DATA_FILE_PATH)) {
            dataFile = SD.open(DATA_FILE_PATH, FILE_WRITE);
            if (!dataFile) return false;
            dataFile.println("t_us,UTC_Date,UTC_Time,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,Temp,Roll,Pitch,Yaw,Lat,Lng,Speed_kmph,Alt_meters,Satellites,HDOP,Heading_deg");
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
    int64_t current_t_us = getAccurateTimeUS();
    ring[idx].t_us = current_t_us;

    uint32_t utc_date_val, utc_time_val;
    unixUSToDateTime(current_t_us, utc_date_val, utc_time_val);
    ring[idx].utc_date = utc_date_val;
    ring[idx].utc_time = utc_time_val;

    xyzFloat acc = mpu.getGValues();
    xyzFloat gyr = mpu.getGyrValues();
    xyzFloat mag = mpu.getMagValues();
    float temp = mpu.getTemperature();

    ring[idx].acc[0] = acc.x; ring[idx].acc[1] = acc.y; ring[idx].acc[2] = acc.z;
    ring[idx].gyro[0] = gyr.x; ring[idx].gyro[1] = gyr.y; ring[idx].gyro[2] = gyr.z;
    ring[idx].mag[0] = mag.x; ring[idx].mag[1] = mag.y; ring[idx].mag[2] = mag.z;
    ring[idx].temp = temp;

    filter.update(gyr.x*DEG_TO_RAD,gyr.y*DEG_TO_RAD,gyr.z*DEG_TO_RAD,
                  acc.x,acc.y,acc.z,
                  mag.x,mag.y,mag.z);
    ring[idx].roll = filter.getRoll();
    ring[idx].pitch = filter.getPitch();
    ring[idx].yaw = filter.getYaw();

    portENTER_CRITICAL_ISR(&gpsDataMux);
    ring[idx].lat = currentGpsData.lat;
    ring[idx].lng = currentGpsData.lng;
    ring[idx].speed_kmph = currentGpsData.speed_kmph;
    ring[idx].alt_meters = currentGpsData.alt_meters;
    ring[idx].satellites = currentGpsData.satellites;
    ring[idx].hdop_value = currentGpsData.hdop_value;
    ring[idx].heading_deg = currentGpsData.heading_deg;
    portEXIT_CRITICAL_ISR(&gpsDataMux);

    writeIndex = (writeIndex+1)%BUFFER_SIZE;
}

// ================= GPS TASK =================
void gpsTask(void* parameter) {
    for (;;) {
        while(gpsSerial.available()>0){
            if(gps.encode(gpsSerial.read())){
                if(gps.location.isUpdated()){
                    portENTER_CRITICAL(&gpsDataMux);
                    currentGpsData.lat = gps.location.lat();
                    currentGpsData.lng = gps.location.lng();
                    currentGpsData.speed_kmph = gps.speed.kmph();
                    currentGpsData.alt_meters = gps.altitude.meters();
                    currentGpsData.satellites = gps.satellites.value();
                    currentGpsData.hdop_value = gps.hdop.value();
                    currentGpsData.heading_deg = gps.course.deg();
                    portEXIT_CRITICAL(&gpsDataMux);
                    Serial.printf("GPS Update LAT: %.6f | Heading: %.2f | Sat: %u\n", currentGpsData.lat, currentGpsData.heading_deg, currentGpsData.satellites);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ================= SD WRITER =================
void writeBatchToSD() {
    if(!dataFile) return;
    uint16_t localRead = sdReadIndex;
    uint16_t localWrite = writeIndex;
    // char csvBuffer[SD_WRITE_BATCH*360]; // REMOVED (now global)
    char* ptr = csvBuffer;
    int lenTotal = 0;
    int batchCount = 0;

    while(localRead != localWrite && batchCount<SD_WRITE_BATCH){
        dataTx s = *(dataTx*)&ring[localRead];
        float hdop_f = (float)s.hdop_value/100.0;
        int len = snprintf(ptr, sizeof(csvBuffer)-lenTotal,
            "%lld,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%u,%.2f,%.2f\n",
            s.t_us, s.utc_date, s.utc_time,
            s.acc[0],s.acc[1],s.acc[2],
            s.gyro[0],s.gyro[1],s.gyro[2],
            s.mag[0],s.mag[1],s.mag[2],
            s.temp,
            s.roll,s.pitch,s.yaw,
            s.lat,s.lng,s.speed_kmph,s.alt_meters,
            s.satellites,hdop_f,s.heading_deg);
        if(len<=0 || lenTotal+len>=sizeof(csvBuffer)) break;
        ptr += len; lenTotal += len;
        localRead = (localRead+1)%BUFFER_SIZE;
        batchCount++;
    }
    if(lenTotal>0){
        dataFile.write((const uint8_t*)csvBuffer,lenTotal);
        dataFile.flush();
        sdReadIndex = localRead;
    }
}

// ================= ESP-NOW SEND (One Sample) =================
void sendCurrentSample() {
    // Calculate the index of the most recently written sample (one before writeIndex)
    uint16_t currentSampleIndex = (writeIndex + BUFFER_SIZE - 1) % BUFFER_SIZE;
    
    // Only send if the index has advanced since the last send operation
    if (currentSampleIndex == espNowReadIndex) {
        return; // No new data to send
    }

    // Retrieve the sample from the volatile ring buffer
    // Note: The sample from the high-speed ISR is being sent.
    dataTx s = *(dataTx*)&ring[currentSampleIndex];
    
    // Send the single data packet
    esp_now_send(rx_peer_mac, (uint8_t*)&s, sizeof(s));

    // Update the read index to the sample that was just sent
    espNowReadIndex = currentSampleIndex;
}

// ================= SETUP =================
void setup(){
    Serial.begin(115200);
    delay(500);
    Serial.println("Starting TX with MPU and GPS...");

    hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    if(!mpu.init()) while(1);
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

    sdSPI.begin();
    gpsSerial.begin(GPS_BAUD,SERIAL_8N1,RXD2,TXD2);
    xTaskCreatePinnedToCore(gpsTask,"GpsTask",4096,NULL,1,NULL,1);
    Serial.println("GPS Task started on Core 1.");

    WiFi.mode(WIFI_STA);
    if(esp_now_init()!=ESP_OK) Serial.println("ESP-NOW init failed");

    esp_now_register_recv_cb(OnTimeSyncRecv);

    esp_now_peer_info_t peerInfo={};
    memcpy(peerInfo.peer_addr,rx_peer_mac,6);
    peerInfo.channel=1;
    peerInfo.encrypt=false;
    if(esp_now_add_peer(&peerInfo)!=ESP_OK) Serial.println("Failed to add RX peer.");
    else Serial.printf("Registered RX peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                         rx_peer_mac[0],rx_peer_mac[1],rx_peer_mac[2],
                         rx_peer_mac[3],rx_peer_mac[4],rx_peer_mac[5]);

    const esp_timer_create_args_t timer_args={
        .callback=&samplingCallback,
        .arg=NULL,
        .dispatch_method=ESP_TIMER_TASK,
        .name="mpu_sampling"
    };
    esp_timer_handle_t timer;
    esp_timer_create(&timer_args,&timer);
    esp_timer_start_periodic(timer,1000000/SAMPLE_RATE_HZ);
    Serial.printf("MPU Sampling Timer started at %d Hz on Core 0 ISR.\n",SAMPLE_RATE_HZ);

    Serial.println("Waiting for Time Sync command from RX unit...");
}

// ================= LOOP =================
void loop(){
    if(!time_synced){
        Serial.print(".");
        delay(500);
        return;
    }

    static bool sd_file_created=false;
    if(time_synced && !sd_file_created){
        if(!SD.begin(SD_CS_PIN,sdSPI) || !createSequentialFile()){
            Serial.println("SD init failed on sync! Continuing without SD.");
        }
        sd_file_created=true;
    }

    // Rate limit the ESP-NOW transmission to ESPNOW_RATE_HZ
    static unsigned long lastSend=0;
    unsigned long now=millis();
    if(now-lastSend>=1000/ESPNOW_RATE_HZ){
        lastSend=now;
        sendCurrentSample(); // Send only the single most recent sample
    }

    // Write a batch of 16 samples to the SD card when needed
    if(sd_file_created) writeBatchToSD();

    delay(1);
}
