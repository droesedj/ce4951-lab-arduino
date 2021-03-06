/*
   LAB WEEK 5: Transmitter
   CE4951-012
   @author Dennis Droese
   @filename transmitter.h
   @date 10/2/2018
*/

#ifndef transmitter_h
#define transmitter_h

#include "Arduino.h"

class Transmitter {
private:
  // Bit-period in microseconds.
  int bitPeriod;
  // Pin to transmit data on.
  byte txPin;
  // Whether or not we are transmitting.
  volatile bool isTransmitting = false;

  // Sends a manchester-encoded value of '1'.
  void sendOne();
  // Sends a manchester-encoded value of '0'.
  void sendZero();

public:
  // Constructor.
  // pin = pin to use as transmitter.
  // bp = bit period in microseconds.
  Transmitter(byte pin, int bp);

  // Transmits specified data of a given length to the network.
  // data = pointer to (or array of) data to be sent.
  // len = size of the data in bytes.
  bool transmit(byte* data, int len);

  // Cancels the current transmission, ceasing all outgoing packets.
  void cancel();
};

#endif
