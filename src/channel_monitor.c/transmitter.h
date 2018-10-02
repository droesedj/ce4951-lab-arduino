#ifndef transmitter_h
#define transmitter_h

#include "Arduino.h"

class Transmitter {
private:
  uint8_t bitPeriod;
  byte txPin;
  volatile bool isTransmitting;

  void sendOne();
  void sendZero();

public:
  Transmitter(byte pin, uint8_t bp);
  void transmit(byte* data);
  void cancel();
};

#endif
