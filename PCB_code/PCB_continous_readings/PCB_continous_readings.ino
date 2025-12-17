#include <Arduino.h>
#include <Wire.h>
#include <vl53l8ch.h>

// -------------------------------------------------------------------
// Pin Definitions
// -------------------------------------------------------------------
#define MUX_A0 2
#define MUX_A1 3
#define MUX_A2 4
#define MUX_A3 5

#define SENSOR_RX 6
#define SENSOR_TX 7

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

#define LPN_PIN 25
#define PWREN_PIN 14
#define GPIO1_PIN 26

#define COM 0x02

HardwareSerial SensorSerial(1);
int muxPins[4] = {MUX_A0, MUX_A1, MUX_A2, MUX_A3};

// -------------------------------------------------------------------
// ToF Sensor
// -------------------------------------------------------------------
VL53L8CH sensor(&Wire, LPN_PIN);
uint8_t res = VL53LMZ_RESOLUTION_8X8;

// -------------------------------------------------------------------
// Shared Data (Mutex Protected)
// -------------------------------------------------------------------
struct SensorData {
    VL53LMZ_ResultsData tof;
    int us[8];
    float frequency;

    uint8_t status[64];   // target_status
    int rawToF[64];       // raw distances
};

SensorData globalData;
SemaphoreHandle_t dataMutex;

// Flags to synchronize printing
volatile bool tofReady = false;
volatile bool usReady  = false;



int statusThreshold = 5;

// -------------------------------------------------------------------
// Function Prototypes
// -------------------------------------------------------------------
void taskToF(void *pvParameters);
void taskUltrasonic(void *pvParameters);
void taskPrinter(void *pvParameters);
void multiplexerSelect(uint8_t channel);

// -------------------------------------------------------------------
// Setup
// -------------------------------------------------------------------
void setup() {
    Serial.begin(921600);
    delay(1000);

    for (int i = 0; i < 4; i++) pinMode(muxPins[i], OUTPUT);

    SensorSerial.begin(115200, SERIAL_8N1, SENSOR_RX, SENSOR_TX);

    pinMode(LPN_PIN, OUTPUT);
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    digitalWrite(LPN_PIN, HIGH);
    delay(10);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    sensor.begin();
    sensor.init();
    sensor.set_resolution(res);
    sensor.set_ranging_frequency_hz(15);
    sensor.set_ranging_mode(VL53LMZ_RANGING_MODE_AUTONOMOUS);
    sensor.set_target_order(VL53LMZ_TARGET_ORDER_CLOSEST);
    sensor.start_ranging();

    dataMutex = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreatePinnedToCore(taskToF,       "ToF",       6000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(taskUltrasonic,"Ultrasonic",6000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskPrinter,  "Printer",   6000, NULL, 1, NULL, 1);
}

void loop() {}

// -------------------------------------------------------------------
// ToF Task (Non-blocking)
// -------------------------------------------------------------------
void taskToF(void *pvParameters) {
    VL53LMZ_ResultsData localToF;

    while (1) {
        uint8_t ready = 0;
        sensor.check_data_ready(&ready);

        if (ready) {
            sensor.get_ranging_data(&localToF);

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            globalData.tof = localToF;

            uint8_t zones = (res == VL53LMZ_RESOLUTION_4X4) ? 16 : 64;
            for (int i = 0; i < zones; i++) {
                uint8_t st = localToF.target_status[i];
                int raw = (localToF.nb_target_detected[i] > 0) ?
                          localToF.distance_mm[i] : -1;

                globalData.status[i] = st;
                globalData.rawToF[i] = raw;

                // Filter
                if (st == 0 || st == 255) globalData.tof.distance_mm[i] = -1;
                else if (st == statusThreshold) globalData.tof.distance_mm[i] = raw;
                else globalData.tof.distance_mm[i] = -1;
            }
            xSemaphoreGive(dataMutex);

            tofReady = true; // mark ToF as ready
        }

        vTaskDelay(2);
    }
}

// -------------------------------------------------------------------
// Ultrasonic Task (Non-blocking, loops through all 8 sensors)
// -------------------------------------------------------------------
void taskUltrasonic(void *pvParameters) {
    uint8_t ch = 0;
    uint8_t buf[4];
    uint32_t t_start = 0;
    bool waitingForEcho = false;

    while (1) {
        uint32_t now = millis();

        if (!waitingForEcho) {
            multiplexerSelect(ch);
            SensorSerial.flush();
            SensorSerial.write(COM);
            t_start = now;
            waitingForEcho = true;
        }

        if (SensorSerial.available() >= 1) {
            if (SensorSerial.read() == 0xFF && SensorSerial.available() >= 3) {
                buf[0] = 0xFF;
                buf[1] = SensorSerial.read();
                buf[2] = SensorSerial.read();
                buf[3] = SensorSerial.read();

                uint8_t cs = buf[0] + buf[1] + buf[2];
                int dist = (buf[3] == cs) ? ((buf[1] << 8) | buf[2]) : -1;

                xSemaphoreTake(dataMutex, portMAX_DELAY);
                globalData.us[ch] = dist;
                xSemaphoreGive(dataMutex);

                ch = (ch + 1) % 8;
                waitingForEcho = false;

                if (ch == 0) usReady = true; // full cycle complete
            }
        }

        if (waitingForEcho && (now - t_start >= 20)) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            globalData.us[ch] = -1;
            xSemaphoreGive(dataMutex);

            ch = (ch + 1) % 8;
            waitingForEcho = false;

            if (ch == 0) usReady = true;
        }

        vTaskDelay(1);
    }
}

// -------------------------------------------------------------------
// Printer Task (Prints ONE frame after both ToF and US readings are ready)
// -------------------------------------------------------------------
void taskPrinter(void *pvParameters) {
    uint32_t lastPrint = millis();

    while (1) {
        if (!tofReady || !usReady) {
            vTaskDelay(1);
            continue;
        }

        tofReady = false;
        usReady  = false;

        uint32_t now = millis();
        float freq = 1000.0 / (now - lastPrint);
        lastPrint = now;

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        globalData.frequency = freq;
        uint8_t zones = (res == VL53LMZ_RESOLUTION_4X4) ? 16 : 64;

        // 1) Filtered ToF
        for (int i = 0; i < zones; i++) {
            int v = globalData.tof.distance_mm[i];
            Serial.print(v >= 0 ? v : -1);
            Serial.print(",");
        }

        // 2) Ultrasonic
        for (int i = 0; i < 8; i++) {
            Serial.print(globalData.us[i]);
            Serial.print(",");
        }

        // 3) Frequency
        Serial.print(globalData.frequency, 2);
        Serial.print(",");

        // 4) Target status
        for (int i = 0; i < zones; i++) {
            Serial.print(globalData.status[i]);
            Serial.print(",");
        }

        // 5) Raw ToF
        for (int i = 0; i < zones; i++) {
            Serial.print(globalData.rawToF[i]);
            Serial.print(",");
        }

        Serial.println();
        xSemaphoreGive(dataMutex);
    }
}

// -------------------------------------------------------------------
// Multiplexer Selection
// -------------------------------------------------------------------
void multiplexerSelect(uint8_t ch) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(muxPins[i], (ch >> i) & 1);
    }
}
