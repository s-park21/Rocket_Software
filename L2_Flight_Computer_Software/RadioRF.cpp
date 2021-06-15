#include "RadioRF.h"
#include <SPI.h>
#include <LoRa.h>


bool RF::setup(const int csPin, const int resetPin, const int irqPin, const long frequency) {
  //LoRa.setSPIFrequency(48000000);   // Set SPI frequency to 48MHz
  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(frequency)) {
    return 1;
  }
  return 0;
}

void RF::sendData(dataPacket* rfPacket, size_t packetSize) {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&rfPacket, packetSize);
  LoRa.endPacket(true); // true = async / non-blocking mode
}
