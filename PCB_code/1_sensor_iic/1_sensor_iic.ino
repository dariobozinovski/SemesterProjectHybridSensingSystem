#include <Wire.h>
#include <DFRobot_IICSerial.h>

#define COM 0x55  // Command to request distance
unsigned char buffer_RTT[4] = {0};
uint8_t control_sum;
int Distance = 0;

// Initialize I2C Multiplexer at address 0x70
// Initialize IICSerial object for UART1
// DFRobot_IICSerial iicSerial1(Wire, SUBUART_CHANNEL_1, 0, 0);
DFRobot_IICSerial iicSerial1(Wire, SUBUART_CHANNEL_1, 0, 0);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);
  
  // Select correct I2C multiplexer port
  delay(1000);

  // Initialize UART communication
  while (iicSerial1.begin(115200) != 0){ //|| iicSerial2.begin(115200) != 0) {
    Serial.println("UART init failed");
    delay(1000);
  }
  Serial.println("UART1 initialized successfully.");
  
}

void loop() {
  iicSerial1.write(COM);
  delay(300);
  int availableBytes1 = iicSerial1.available();
  // Serial.print("sonar 2: ");
  //Serial.println(availableBytes1);


  if (availableBytes1 > 0) {
    delay(4);  // Wait for complete data
    if (iicSerial1.read() == 0xff) {  // Start byte check
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = iicSerial1.read();
      }
      control_sum = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == control_sum) {  // Verify checksum
        Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        Serial.print("Distance2: ");
        Serial.print(Distance);
        Serial.println();
      }
    }
  }
  else{
    Serial.println("no data");
  }
      while (iicSerial1.read() > -1) {
  }
  delay(50);

    

}