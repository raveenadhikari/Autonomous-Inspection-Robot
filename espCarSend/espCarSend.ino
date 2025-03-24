#include <SPI.h>
#include <LoRa.h>

// Define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Sender Initializing...");

  // Setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  // Replace the LoRa.begin argument with your location's frequency
  // 433E6 for Asia, 868E6 for Europe, 915E6 for North America
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }

  // Set sync word (0xF3) to match the receiver
  LoRa.setSyncWord(0xFF);

  Serial.println("LoRa Sender Initialized!");
}

void loop() {
  // Check if there's any command received from Serial (from Python GUI)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read the command
    command.trim(); // Remove any extra whitespace or newline characters

    // Send the command over LoRa
    sendCommand(command);
    Serial.println("Command");
  }
}

void sendCommand(String command) {
  // Begin a LoRa packet
  LoRa.beginPacket();
  LoRa.print(command); // Send the command
  LoRa.endPacket();

  Serial.println("Sent command: " + command);
}
