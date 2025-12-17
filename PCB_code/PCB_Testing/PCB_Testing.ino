#include <Arduino.h>
#include <Wire.h>
#include <vl53l8ch.h>

// -------------------------------
// Pin Definitions
// -------------------------------
#define MUX_A0 2
#define MUX_A1 3
#define MUX_A2 4
#define MUX_A3 5

#define SENSOR_RX 6   // ESP32-S3 RX (input)
#define SENSOR_TX 7   // ESP32-S3 TX (output)

// ToF

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// Control pins
#define LPN_PIN 25       // Low power / reset (drive HIGH to enable sensor)
#define PWREN_PIN 14     // Power enable (drive HIGH to power sensor)
#define GPIO1_PIN 26     // Optional interrupt pin (unused in polling mode)

unsigned char buffer_RTT[4] = {0};
uint8_t CS;
#define COM 0x02  // Command to request distance
int Distance = 0;
HardwareSerial SensorSerial(1);   // Use UART1
// Array to make setting mux pins easy
int muxPins[4] = {MUX_A0, MUX_A1, MUX_A2, MUX_A3};

// --------------------------------------------------------

VL53L8CH sensor(&Wire, LPN_PIN);
bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53LMZ_RESOLUTION_8X8;

char report[256];
uint8_t status;

// ------------------ Function declarations ------------------
void print_result(VL53LMZ_ResultsData  *Result);
void clear_screen(void); 
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);
void toggle_signal_and_ambient(void);

// --------------------------------------------
// Multiplexer select function
// channel = 0–15
// --------------------------------------------
void multiplexerSelect(int *pins, uint8_t channel) {
  // Set each mux pin according to the binary value of channel
  for (int i = 0; i < 4; i++) {
    int bitValue = (channel >> i) & 0x01;
    digitalWrite(pins[i], bitValue);
  }
}



// --------------------------------------------
// Setup
// --------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure multiplexer pins as outputs
  pinMode(MUX_A0, OUTPUT);
  pinMode(MUX_A1, OUTPUT);
  pinMode(MUX_A2, OUTPUT);
  pinMode(MUX_A3, OUTPUT);

  // Configure sensor UART pins
  SensorSerial.begin(115200, SERIAL_8N1, SENSOR_RX, SENSOR_TX); 

  // TOF
  // Configure control pins
  pinMode(LPN_PIN, OUTPUT);
  pinMode(PWREN_PIN, OUTPUT);
  digitalWrite(PWREN_PIN, HIGH);  // Power ON
  digitalWrite(LPN_PIN, HIGH);    // Exit reset mode
  delay(10);

  // Initialize I2C bus on custom pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // Fast I2C (400kHz)
  Serial.println("I2C initialized.");

  // Initialize the sensor
  sensor.begin();
  status = sensor.init();
  status = sensor.set_resolution(res);

  if (status != 0) {
    Serial.printf("Sensor initialization failed! (status=%d)\n", status);
    while (1) delay(1000);
  } else {
    Serial.println("Sensor initialized successfully.");
  }
  
  // Start measurements
  status = sensor.start_ranging();
  if (status != 0) {
    Serial.printf("Start ranging failed! (status=%d)\n", status);
    while (1) delay(1000);
  }
  Serial.println("Ranging started.");


  Serial.println("Multiplexer control ready. Enter a channel (0–15).");
}



// --------------------------------------------
// Skeleton for reading function (do not fill)
// --------------------------------------------
void getReading() {
  SensorSerial.write(COM);
    delay(100);
    if(SensorSerial.available() > 0){
      delay(4);
      if(SensorSerial.read() == 0xff){    
        buffer_RTT[0] = 0xff;
        for (int i=1; i<4; i++){
          buffer_RTT[i] = SensorSerial.read();   
        }
        CS = buffer_RTT[0] + buffer_RTT[1]+ buffer_RTT[2];  
        if(buffer_RTT[3] == CS) {
          Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
          Serial.print("Distance:");
          Serial.print(Distance);
          Serial.println("mm");
       }
     }
   }
}


void getTofReading(){
  VL53LMZ_ResultsData Results;
  uint8_t NewDataReady = 0;

  do {
    status = sensor.check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor.get_ranging_data(&Results);
    print_result(&Results);
  }
  delay(100);
}


// --------------------------------------------
// Main Loop
// --------------------------------------------
void loop() {

  // Check if user typed something in Serial Monitor
  if (Serial.available() > 0) {
    int channel = Serial.parseInt();   // Reads numbers like 0–15

    if (channel >= 0 && channel <= 15) {
      Serial.print("Setting multiplexer channel to: ");
      Serial.println(channel);

      multiplexerSelect(muxPins, channel);

      // Call the reading function (empty for now)
      getReading();
    }
    else if (channel == 16) {
      getTofReading();
    }
    else {
      Serial.println("Invalid channel! Enter a number between 0 and 15.");
    }
  }
}


void print_result(VL53LMZ_ResultsData  *Result) {
  int8_t i, j, k;
  uint8_t l, zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  Serial.print("Cell Format:\n\n");

  for (l = 0; l < VL53LMZ_NB_TARGET_PER_ZONE; l++) {
    snprintf(report, sizeof(report), " %20s : %20s\n", "Distance [mm]", "Status");
    Serial.print(report);

    if (EnableAmbient || EnableSignal) {
      snprintf(report, sizeof(report), " %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
      Serial.print(report);
    }
  }

  Serial.print("\n\n");

  for (j = 0; j < number_of_zones; j += zones_per_line) {
    for (i = 0; i < zones_per_line; i++) Serial.print(" -----------------");
    Serial.print("\n");

    for (i = 0; i < zones_per_line; i++) Serial.print("|                 ");
    Serial.print("|\n");

    for (l = 0; l < VL53LMZ_NB_TARGET_PER_ZONE; l++) {
      for (k = (zones_per_line - 1); k >= 0; k--) {
        if (Result->nb_target_detected[j + k] > 0) {
          snprintf(report, sizeof(report), "| %5ld : %5ld ",
                   (long)Result->distance_mm[(VL53LMZ_NB_TARGET_PER_ZONE * (j + k)) + l],
                   (long)Result->target_status[(VL53LMZ_NB_TARGET_PER_ZONE * (j + k)) + l]);
          Serial.print(report);
        } else {
          snprintf(report, sizeof(report), "| %5s : %5s ", "X", "X");
          Serial.print(report);
        }
      }
      Serial.print("|\n");
    }
  }
  for (i = 0; i < zones_per_line; i++) Serial.print(" -----------------");
  Serial.print("\n");
}