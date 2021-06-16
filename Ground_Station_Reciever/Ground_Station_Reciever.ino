#include <SPI.h>
#include <LoRa.h>


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.write(0x44);             // Byte to say ground station is starting
  while (!LoRa.begin(915E6)) {
    Serial.write(0xFA);
    Serial.write(0xFB);
    Serial.write(0xFC);
  }
}


void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // read packet
    while (LoRa.available()) {
      Serial.print(LoRa.read());
    }
    Serial.println("");
  }
}
