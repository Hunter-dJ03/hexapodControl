#include <Arduino.h>
#include <Servo.h>


#define DEBUG_MODE false  // Set to true to enable Serial output, false to disable

#if DEBUG_MODE
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)  // Variadic macro for println
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)      // Variadic macro for print
#define DEBUG_WRITE(x) Serial.write(x)

unsigned long debugStartTime = 0;
#define DEBUG_TIME_START() debugStartTime = micros()
#define DEBUG_TIME_END(message) \
  Serial.print(message); \
  Serial.print(": "); \
  Serial.println(micros() - debugStartTime);
#else
#define DEBUG_PRINT(...)
#define DEBUG_WRITE(x)
#define DEBUG_PRINTLN(...)
#define DEBUG_TIME_START()
#define DEBUG_TIME_END(message)
#endif

const byte PACKET_SIZE = 38;  // 1 start byte + 36 bytes of data (18 x 16-bit) + 2 checksum
byte buffer[PACKET_SIZE];

// Min and max pulse widths for servos
const int MIN_PULSE = 500;   // Minimum pulse width in microseconds
const int MAX_PULSE = 2500;  // Maximum pulse width in microseconds (maps  48928 to 2500)

Servo coxa1;
Servo femur1;
Servo tibia1;
Servo coxa2;
Servo femur2;
Servo tibia2;
Servo coxa3;
Servo femur3;
Servo tibia3;
Servo coxa4;
Servo femur4;
Servo tibia4;
Servo coxa5;
Servo femur5;
Servo tibia5;
Servo coxa6;
Servo femur6;
Servo tibia6;

void initLegs() {
  // Attach Servos
  coxa1.attach(24);   // validated
  femur1.attach(26);  // validated
  tibia1.attach(28);  // validated

  coxa2.attach(31);   // validated
  femur2.attach(33);  // validated
  tibia2.attach(35);  // validated

  coxa3.attach(25);   // validated
  femur3.attach(27);  // validated
  tibia3.attach(29);  // validated

  coxa4.attach(37);   // validated
  femur4.attach(39);  // validated
  tibia4.attach(41);  // validated

  coxa5.attach(50);   // validated
  femur5.attach(48);  // validated
  tibia5.attach(46);  // validated

  coxa6.attach(40);   // validated
  femur6.attach(42);  // validated
  tibia6.attach(44);  // validated


  // Now set the writeMicroseconds values
  coxa1.writeMicroseconds(1500);
  femur1.writeMicroseconds(MAX_PULSE);
  tibia1.writeMicroseconds(MAX_PULSE);

  coxa2.writeMicroseconds(1500);
  femur2.writeMicroseconds(MAX_PULSE);
  tibia2.writeMicroseconds(MAX_PULSE);

  coxa3.writeMicroseconds(1500);
  femur3.writeMicroseconds(MAX_PULSE);
  tibia3.writeMicroseconds(MAX_PULSE);

  coxa4.writeMicroseconds(1500);
  femur4.writeMicroseconds(MIN_PULSE);
  tibia4.writeMicroseconds(MIN_PULSE);

  coxa5.writeMicroseconds(1500);
  femur5.writeMicroseconds(MIN_PULSE);
  tibia5.writeMicroseconds(MIN_PULSE);

  coxa6.writeMicroseconds(1500);
  femur6.writeMicroseconds(MIN_PULSE);
  tibia6.writeMicroseconds(MIN_PULSE);
};

void updateServos(uint16_t values[18]) {

  // Map and assign each value directly to the corresponding servo
  coxa1.writeMicroseconds(map(values[0], 0, 18000, MIN_PULSE, MAX_PULSE));
  femur1.writeMicroseconds(map(values[1], 0, 18000, MIN_PULSE, MAX_PULSE));
  tibia1.writeMicroseconds(map(values[2], 0, 18000, MIN_PULSE, MAX_PULSE));

  coxa2.writeMicroseconds(map(values[3], 0, 18000, MIN_PULSE, MAX_PULSE));
  femur2.writeMicroseconds(map(values[4], 0, 18000, MIN_PULSE, MAX_PULSE));
  tibia2.writeMicroseconds(map(values[5], 0, 18000, MIN_PULSE, MAX_PULSE));

  coxa3.writeMicroseconds(map(values[6], 0, 18000, MIN_PULSE, MAX_PULSE));
  femur3.writeMicroseconds(map(values[7], 0, 18000, MIN_PULSE, MAX_PULSE));
  tibia3.writeMicroseconds(map(values[8], 0, 18000, MIN_PULSE, MAX_PULSE));

  coxa4.writeMicroseconds(map(values[9], 0, 18000, MIN_PULSE, MAX_PULSE));
  femur4.writeMicroseconds(map(values[10], 0, 18000, MIN_PULSE, MAX_PULSE));
  tibia4.writeMicroseconds(map(values[11], 0, 18000, MIN_PULSE, MAX_PULSE));

  coxa5.writeMicroseconds(map(values[12], 0, 18000, MIN_PULSE, MAX_PULSE));
  femur5.writeMicroseconds(map(values[13], 0, 18000, MIN_PULSE, MAX_PULSE));
  tibia5.writeMicroseconds(map(values[14], 0, 18000, MIN_PULSE, MAX_PULSE));

  coxa6.writeMicroseconds(map(values[15], 0, 18000, MIN_PULSE, MAX_PULSE));
  femur6.writeMicroseconds(map(values[16], 0, 18000, MIN_PULSE, MAX_PULSE));
  tibia6.writeMicroseconds(map(values[17], 0, 18000, MIN_PULSE, MAX_PULSE));
}

// int deg2ms(double angle) {
//   double min = 500.0;
//   double max = 2500.0;
//   double ms = min + (((max - min) / 180) * angle);
//   return (int)ms;
// }

void setup() {
  // Set the baud rate to something more stable like 115200 or 921600
  Serial.begin(921600);  // Lower the baud rate to avoid overrunning the Arduino UART

  initLegs();  // Initialize servos or other peripherals

  delay(800);  // Short delay for any initialization tasks
}
void loop() {
  // Wait until the start byte (0xA5) is received
  if (Serial.available()) {
    // Look for the start byte
    if (Serial.read() == 0xA5) {
      DEBUG_TIME_START();
      // Now read the rest of the packet
      if (Serial.readBytes(buffer + 1, PACKET_SIZE - 1) == (PACKET_SIZE - 1)) {
        // Extract the 18 16-bit values
        uint16_t values[18];
        for (int i = 0; i < 18; i++) {
          values[i] = (buffer[2 * i + 1] << 8) | buffer[2 * i + 2];

          // DEBUG_PRINT("DOF: ");
          // DEBUG_PRINT(i+1);
          // DEBUG_PRINT(": ");
          // DEBUG_PRINT(values[i]);
          // DEBUG_PRINT(", ");
          // DEBUG_PRINTLN(values[i]/ 100.0);
        }

        // Compute checksum of received data
        byte received_checksum = buffer[PACKET_SIZE - 1];
        byte computed_checksum = 0;
        for (int i = 1; i < PACKET_SIZE - 1; i++) {
          computed_checksum += buffer[i];
        }
        computed_checksum &= 0xFF;

        // Check if checksum is valid
        if (computed_checksum == received_checksum) {
          // Data received successfully
          DEBUG_PRINTLN("SUCCESS");
          updateServos(values);
          DEBUG_WRITE(0x06);  // Send ACK (0x06 is ASCII ACK)
        } else {
          DEBUG_PRINT("ERROR: ");
          DEBUG_PRINT("Computed Checksum: ");
          DEBUG_PRINT(computed_checksum);
          DEBUG_PRINT(", Received Checksum: ");
          DEBUG_PRINTLN(received_checksum);
          DEBUG_WRITE(0x15);  // Send NACK (0x15 is ASCII NAK)
        }
      } else {
        DEBUG_PRINTLN("Incomplete packet");
      }
      DEBUG_TIME_END("Time taken");
    }
  }
}
