#include "transmitter.h"

/*
   LAB WEEK 5: Transmitter
   CE4951-012
   @author Dennis Droese
   @filename transmitter.c
   @date 10/2/2018
*/



/// Public Methods + Constructor

Transmitter::Transmitter(byte pin, uint8_t bp) {
  txPin = pin;
  bitPeriod = bp;
  isTransmitting = false;
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);
}

void Transmitter::transmit(byte* data) {
  isTransmitting = true;
  int len = sizeof(data);
  Serial.println("\tlen = " + len); 
  Serial.flush();

  // For every piece of data...
  for (int i = 0; i < len && isTransmitting; i++) {
    // For every bit in a byte...
    for (byte mask = 0b10000000; mask > 0 && isTransmitting; mask>>=1) {
      // Start at the MSB. binary AND with 1.  This gives us one bit.
      // Shift the data starting from MSB down to bit 0.
      byte decision = (data[i] & mask);
      switch (decision) {
        case 0:
          Serial.print("0");
          sendZero();
          break;
        case 1:
          Serial.print("1");
          sendOne();
          break;
        default:
          Serial.println("error: " + decision);
      }
    }
    Serial.print("\n");
  }

  isTransmitting = false;
}

void Transmitter::cancel() {
  if (isTransmitting) {
    digitalWrite(txPin, HIGH);
  }
  isTransmitting = false;
}

/// Private Methods

void Transmitter::sendOne() {
  digitalWrite(txPin, LOW);
  delayMicroseconds(bitPeriod / 2);
  digitalWrite(txPin, HIGH);
  delayMicroseconds(bitPeriod / 2);
}

void Transmitter::sendZero() {
  digitalWrite(txPin, HIGH);
  delayMicroseconds(bitPeriod / 2);
  digitalWrite(txPin, LOW);
  delayMicroseconds(bitPeriod / 2);
}

