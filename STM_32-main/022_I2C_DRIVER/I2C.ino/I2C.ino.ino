#include <Wire.h>
#define SLAVE_ADDR 0x68


void receiveEvent(int len);

void setup() {
  Serial.begin(115200);
  Wire.begin(SLAVE_ADDR);  
  Wire.onReceive(receiveEvent);
  //Serial.println("ESP32 I2C Slave Ready...");
  rec();
}

void loop() {
  delay(100);
}

void receiveEvent(int len) {
  Serial.println("Received: ");
  while (Wire.available()) {
    char c = Wire.read();
    Serial.println(c);
  }
  Serial.println();
}

