/**
 ******************************************************************************
 * @file    VL53L8CX_HelloWorld_I2C_ESP32.cpp
 * @brief   Adapted for ESP32-WROVER-E using I2C on custom pins
 ******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h>  // Make sure the ST library is installed in lib/

// ------------------ Pin configuration ------------------
// I2C pins
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// Control pins
#define LPN_PIN 25       // Low power / reset (drive HIGH to enable sensor)
#define PWREN_PIN 14     // Power enable (drive HIGH to power sensor)
#define GPIO1_PIN 26     // Optional interrupt pin (unused in polling mode)

#define SerialPort Serial

// --------------------------------------------------------

VL53L8CX sensor(&Wire, LPN_PIN);
bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_4X4;
char report[256];
uint8_t status;

// ------------------ Function declarations ------------------
void print_result(VL53L8CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);
void toggle_resolution(void);
void toggle_signal_and_ambient(void);

// ------------------ Setup ------------------
void setup() {

  SerialPort.begin(115200);
  delay(500);
  SerialPort.println("\n[VL53L8CX] ESP32-WROVER-E HelloWorld starting...");

  // Configure control pins
  pinMode(LPN_PIN, OUTPUT);
  pinMode(PWREN_PIN, OUTPUT);
  digitalWrite(PWREN_PIN, HIGH);  // Power ON
  digitalWrite(LPN_PIN, HIGH);    // Exit reset mode
  delay(10);

  // Initialize I2C bus on custom pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // Fast I2C (400kHz)
  SerialPort.println("I2C initialized.");

  // Initialize the sensor
  sensor.begin();
  status = sensor.init();

  if (status != 0) {
    SerialPort.printf("Sensor initialization failed! (status=%d)\n", status);
    while (1) delay(1000);
  } else {
    SerialPort.println("Sensor initialized successfully.");
  }

  // Start measurements
  status = sensor.start_ranging();
  if (status != 0) {
    SerialPort.printf("Start ranging failed! (status=%d)\n", status);
    while (1) delay(1000);
  }
  SerialPort.println("Ranging started.");
}

// ------------------ Loop ------------------
void loop() {
  
  VL53L8CX_ResultsData Results;
  uint8_t NewDataReady = 0;

  do {
    status = sensor.check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor.get_ranging_data(&Results);
    print_result(&Results);
  }

  if (Serial.available() > 0) {
    handle_cmd(Serial.read());
  }
  delay(100);
}

// ------------------ Helper functions ------------------

void print_result(VL53L8CX_ResultsData *Result) {
  int8_t i, j, k;
  uint8_t l, zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;
  display_commands_banner();

  SerialPort.print("Cell Format:\n\n");

  for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
    snprintf(report, sizeof(report), " %20s : %20s\n", "Distance [mm]", "Status");
    SerialPort.print(report);

    if (EnableAmbient || EnableSignal) {
      snprintf(report, sizeof(report), " %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
      SerialPort.print(report);
    }
  }

  SerialPort.print("\n\n");

  for (j = 0; j < number_of_zones; j += zones_per_line) {
    for (i = 0; i < zones_per_line; i++) SerialPort.print(" -----------------");
    SerialPort.print("\n");

    for (i = 0; i < zones_per_line; i++) SerialPort.print("|                 ");
    SerialPort.print("|\n");

    for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
      for (k = (zones_per_line - 1); k >= 0; k--) {
        if (Result->nb_target_detected[j + k] > 0) {
          snprintf(report, sizeof(report), "| %5ld : %5ld ",
                   (long)Result->distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l],
                   (long)Result->target_status[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
          SerialPort.print(report);
        } else {
          snprintf(report, sizeof(report), "| %5s : %5s ", "X", "X");
          SerialPort.print(report);
        }
      }
      SerialPort.print("|\n");
    }
  }
  for (i = 0; i < zones_per_line; i++) SerialPort.print(" -----------------");
  SerialPort.print("\n");
}

void toggle_resolution(void) {
  status = sensor.stop_ranging();
  res = (res == VL53L8CX_RESOLUTION_4X4) ? VL53L8CX_RESOLUTION_8X8 : VL53L8CX_RESOLUTION_4X4;
  status = sensor.set_resolution(res);
  status = sensor.start_ranging();
}

void toggle_signal_and_ambient(void) {
  EnableAmbient = !EnableAmbient;
  EnableSignal = !EnableSignal;
}

void clear_screen(void) {
  snprintf(report, sizeof(report), "%c[2J", 27);
  SerialPort.print(report);
}

void display_commands_banner(void) {
  snprintf(report, sizeof(report), "%c[2H", 27);
  SerialPort.print(report);
  Serial.print("VL53L8CX Simple Ranging Demo (ESP32)\n");
  Serial.print("-------------------------------------\n\n");
  Serial.print("Commands:\n");
  Serial.print(" 'r' : change resolution\n");
  Serial.print(" 's' : toggle signal/ambient\n");
  Serial.print(" 'c' : clear screen\n\n");
}

void handle_cmd(uint8_t cmd) {
  switch (cmd) {
    case 'r': toggle_resolution(); clear_screen(); break;
    case 's': toggle_signal_and_ambient(); clear_screen(); break;
    case 'c': clear_screen(); break;
    default: break;
  }
}
