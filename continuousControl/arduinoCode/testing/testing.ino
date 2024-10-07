#include <Arduino.h>

const byte PACKET_SIZE = 37;  // 1 start byte + 36 bytes of data (18 x 16-bit) + 1 checksum
byte buffer[PACKET_SIZE];

void setup() {
  Serial.begin(921600);  // Use stable baud rate
  delay(800);  // Short delay for initialization
}

void loop() {
  if (Serial.available()) {
    // Wait for the start byte (0xA5)
    if (Serial.read() == 0xA5) {
      // Read the rest of the packet
      if (Serial.readBytes(buffer + 1, PACKET_SIZE - 1) == (PACKET_SIZE - 1)) {
        uint16_t values[18];
        for (int i = 0; i < 18; i++) {
          values[i] = (buffer[2 * i + 1] << 8) | buffer[2 * i + 2];
        }

        // Compute checksum of received data
        byte received_checksum = buffer[PACKET_SIZE - 1];
        byte computed_checksum = 0;
        for (int i = 1; i < PACKET_SIZE - 1; i++) {
          computed_checksum += buffer[i];
        }
        computed_checksum &= 0xFF;

        // Debugging: Print the received bytes
        Serial.println("Received Bytes:");
        for (int i = 1; i < PACKET_SIZE - 1; i++) {
          Serial.print(buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // Check if checksum is valid
        if (computed_checksum == received_checksum) {
          Serial.println("SUCCESS");
          processValues(values);
          Serial.write(0x06);  // Send ACK (0x06 is ASCII ACK)
        } else {
          Serial.print("ERROR: ");
          Serial.print("Computed Checksum: ");
          Serial.print(computed_checksum);
          Serial.print(", Received Checksum: ");
          Serial.println(received_checksum);
          Serial.write(0x15);  // Send NACK (0x15 is ASCII NAK)
        }
      } else {
        Serial.println("Incomplete packet");
      }
    }
  }
}

void processValues(uint16_t values[]) {
  // Placeholder function to process the values
  for (int i = 0; i < 18; i++) {
    // Process or use values as needed
    Serial.print("Value ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(values[i]);
  }
}
