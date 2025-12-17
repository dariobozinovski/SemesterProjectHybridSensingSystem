#include <Wire.h>
#include <DFRobot_IICSerial.h>

#define COM 0x55  // Command to request distance
unsigned char buffer_RTT[4] = {0};
uint8_t control_sum;
int Distance1 = 0;
int Distance2 = 0;

// Multiplexer channels
#define SUBUART_CHANNEL_1 0
#define SUBUART_CHANNEL_2 1

// Initialize IICSerial objects for UART1 and UART2
DFRobot_IICSerial iicSerial1(Wire, SUBUART_CHANNEL_1, 0, 0);
DFRobot_IICSerial iicSerial2(Wire, SUBUART_CHANNEL_2, 0, 0);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);

  // Initialize UART communication
  while (iicSerial1.begin(115200) != 0 || iicSerial2.begin(115200) != 0) {
    Serial.println("UART init failed");
    delay(1000);
  }
  Serial.println("UART1 and UART2 initialized successfully.");
}

void loop() {
  // --- Read Distance from Sensor 1 ---
  iicSerial1.write(COM);
  delay(300);

  if (iicSerial1.available() > 0) {
    delay(4);  // Wait for complete data
    if (iicSerial1.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = iicSerial1.read();
      }
      control_sum = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == control_sum) {
        Distance1 = (buffer_RTT[1] << 8) + buffer_RTT[2];
      }
    }
  }

  // Flush buffer
  while (iicSerial1.read() > -1) {}

  // --- Read Distance from Sensor 2 ---
  iicSerial2.write(COM);
  delay(300);

  if (iicSerial2.available() > 0) {
    delay(4);
    if (iicSerial2.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = iicSerial2.read();
      }
      control_sum = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == control_sum) {
        Distance2 = (buffer_RTT[1] << 8) + buffer_RTT[2];
      }
    }
  }

  // Flush buffer
  while (iicSerial2.read() > -1) {}

  // --- Print both distances on one line ---
  Serial.print("Distance1: ");
  Serial.print(Distance1);
  Serial.print("   Distance2: ");
  Serial.println(Distance2);

  delay(100);
}
