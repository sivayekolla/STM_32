#include <Wire.h>

#define I2C_SLAVE_ADDR 0x42  // Same as STM32 master
#define LED_PIN 2            // On-board LED
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  Serial.println("ESP32 I2C Slave Ready");

  Wire.begin(I2C_SLAVE_ADDR);   // Start as I2C slave
  Wire.onReceive(receiveEvent); // Event handler for receiving data
}

void loop() {
  // Nothing here, just wait for I2C events
  delay(100);
}

// This function is called whenever STM32 master sends data
void receiveEvent(int howMany) {
  Serial.print("Data Received: ");
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
void num();

    // Example action: toggle LED based on received character
    if (c == '1') digitalWrite(LED_PIN, HIGH);
    else if (c == '0') digitalWrite(LED_PIN, LOW);
  }
  Serial.println();
}


