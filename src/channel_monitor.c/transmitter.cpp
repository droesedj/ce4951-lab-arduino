#include "transmitter.h"

/*
   LAB WEEK 5: Transmitter
   CE4951-012
   @author Dennis Droese
   @filename transmitter.c
   @date 10/2/2018
*/

/// Public Methods + Constructor

Transmitter::Transmitter(byte pin, int bp) {
  txPin = pin;
  bitPeriod = bp;
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);
}

void Transmitter::transmit(byte* data, int len) {
  // If the data length is 0, don't do anything.
  if(len <= 0){
    return;
  }
  
  isTransmitting = true;
  Serial.print("len = ");
  Serial.println(len); 

  // For every piece of data...
  for (int i = 0; i < len && isTransmitting; i++) {
    // For every bit in a byte...
    Serial.print(i);
    Serial.print('\t');
    for (int j = 0; j < 8 && isTransmitting; j++) {
      // Start at the MSB. binary AND with 1.  This gives us one bit.
      // Shift the data starting from MSB down to bit 0.
      byte decision = (data[i] >> j) & 0b00000001 ;
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
          Serial.print("error: ");
          Serial.println(decision);
          break;
      }
    }
    Serial.print(" ");
    Serial.print((char)data[i]);
    Serial.print('\n');
  }
  isTransmitting = false;
  digitalWrite(txPin, HIGH);
  //Serial.flush();
}

void Transmitter::cancel() {
  if (isTransmitting) {
    digitalWrite(txPin, HIGH);
  }
  isTransmitting = false;
  digitalWrite(txPin, HIGH);
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

