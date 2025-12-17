#include <Wire.h>
#include <DFRobot_IICSerial.h>

#define COM 0x02  // Command to request distance
unsigned char buffer_RTT[4] = {0};
uint8_t control_sum;
int Distance1 = 0, Distance2 = 0;
int interSensorDelay = 0;  // Delay between sending commands (ms)
bool continuousMode = false;

DFRobot_IICSerial iicSerial1(Wire, SUBUART_CHANNEL_1, 0, 0);
DFRobot_IICSerial iicSerial2(Wire, SUBUART_CHANNEL_2, 0, 0);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);

  while (iicSerial1.begin(115200) != 0 || iicSerial2.begin(115200) != 0) {
    Serial.println("UART init failed");
    delay(1000);
  }

  Serial.println("UART initialized successfully.");
  Serial.println("Commands:");
  Serial.println("  '1'  → Trigger one measurement");
  Serial.println("  'm'  → Set delay between sensors");
  Serial.println("  ' '  → Toggle continuous mode");
  Serial.println("  's'  → Sweep mode (vary delay between sensors)");
}

void loop() {
  // --- Handle keyboard input ---
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == '1') {
      measureOnce();
    }

    else if (cmd == 'm') {
      Serial.println("\nEnter delay between commands (ms): ");
      while (!Serial.available()) {}
      interSensorDelay = Serial.parseInt();
      Serial.print("New inter-sensor delay set to: ");
      Serial.print(interSensorDelay);
      Serial.println(" ms");
    }

    else if (cmd == ' ') {  // Space bar toggles continuous mode
      continuousMode = !continuousMode;
      if (continuousMode)
        Serial.println("Continuous mode ON");
      else
        Serial.println("Continuous mode OFF");
    }

    else if (cmd == 's') {  // Sweep mode
      startSweep();
    }
  }

  // --- Continuous mode ---
  if (continuousMode) {
    measureOnce();
  }
}

void measureOnce() {
  // Send command to both sensors (optionally with delay)
  iicSerial2.write(COM);
  if (interSensorDelay > 0) delay(interSensorDelay);
  iicSerial1.write(COM);

  delay(300);  // Wait for measurement to complete

  // --- Read Sensor 1 ---
  Distance1 = readDistance(iicSerial1);
  // --- Read Sensor 2 ---
  Distance2 = readDistance(iicSerial2);

  // --- Print results ---
  Serial.print("Distance1: ");
  if (Distance1 > 0)
    Serial.print(Distance1);
  else
    Serial.print("N/A");
  Serial.print("   Distance2: ");
  if (Distance2 > 0)
    Serial.print(Distance2);
  else
    Serial.print("N/A");
  Serial.print("   (Delay: ");
  Serial.print(interSensorDelay);
  Serial.println(" ms)");
}

int readDistance(DFRobot_IICSerial &sensor) {
  int distance = -1;
  if (sensor.available() > 0) {
    delay(4);
    if (sensor.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = sensor.read();
      }
      control_sum = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == control_sum) {
        distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
      }
    }
  }
  while (sensor.read() > -1) {}  // Flush
  return distance;
}

void startSweep() {
  int startDelay, step, steps;

  Serial.println("\nEnter starting delay (ms): ");
  while (!Serial.available()) {}
  startDelay = Serial.parseInt();
  Serial.println(startDelay);

  Serial.println("Enter step size (ms): ");
  while (!Serial.available()) {}
  step = Serial.parseInt();
  Serial.println(step);

  Serial.println("Enter number of steps: ");
  while (!Serial.available()) {}
  steps = Serial.parseInt();
  Serial.println(steps);

  Serial.println("\n--- Starting Sweep ---");
  Serial.println("Delay(ms)\tDistance1(mm)\tDistance2(mm)");

  for (int i = 0; i < steps; i++) {
    interSensorDelay = startDelay + i * step;

    long sum1 = 0, sum2 = 0;
    int validCount = 0;

    for (int j = 0; j < 10; j++) {  // 10 readings per delay
      measureOnce();
      if (Distance1 > 0 && Distance2 > 0) {
        sum1 += Distance1;
        sum2 += Distance2;
        validCount++;
      }
      delay(50);
    }

    if (validCount > 0) {
      Serial.print(interSensorDelay);
      Serial.print("\t");
      Serial.print(sum1 / validCount);
      Serial.print("\t\t");
      Serial.println(sum2 / validCount);
    } else {
      Serial.print(interSensorDelay);
      Serial.println("\tN/A\t\tN/A");
    }
  }

  Serial.println("--- Sweep Complete ---\n");
}
