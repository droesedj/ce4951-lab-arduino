/*
   LAB WEEK 3: CHANNEL MONITOR
   CE4951-012
   @author Dennis Droese
   @filename channel_monitor.c
   @date 9/19/2018
*/
//#include "Arduino.h"
#include "TimerOne.h"
#include "transmitter.h"

#define DEVICE_ADDRESS 23
#define DEBUG_PRINT_ENABLE true

void edgeDetect();
void timeOut();
void transmitSerial();
void debugPrint(char* s);

enum State_enum {s_IDLE, s_BUSY, s_COLLISION};

const byte rxPin = 3;
const byte txPin = 5;

// Bit period
const int bp = 1000; //1ms, 1000us

// Maximum allowed time before timing out.
const int t_DELAY = bp; //1.11ms, 1110us

volatile uint8_t state = s_IDLE;
volatile bool edge = false;

Transmitter trans(txPin, bp);

void setup()
{
  // Init the Rx Pin
  pinMode(rxPin, INPUT_PULLUP);

  // Init the LED pin.
  pinMode(LED_BUILTIN, OUTPUT);

  // Init the serial port.
  Serial.begin(9600);

  // Initialize the timer.
  Timer1.initialize(t_DELAY);

  // Attach the interrupt. Start the timer.
  Timer1.attachInterrupt(timeOut);

  // Attach edge interrupt.
  attachInterrupt(digitalPinToInterrupt(rxPin), edgeDetect, CHANGE);
  // Start the timer.
  Timer1.start();
}

void loop()
{
  // On a rising/falling edge, wait one half bit-period before doing anything
  if (edge) {
    Timer1.restart();
    noInterrupts();
    edge = false;
    delayMicroseconds((bp / 2));
    interrupts();
  }
  else
  {
    switch (state)
    {
      case s_IDLE:
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(7, HIGH);
        digitalWrite(6, LOW);
        //debugPrint("I\n");
        if (Serial.available() > 0) {
          noInterrupts();
          Serial.flush();
          transmitSerial();
          interrupts();
        }
        break;

      case s_BUSY:
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(7, LOW);
        digitalWrite(6, LOW);
        //debugPrint("B\n");
        break;

      case s_COLLISION:
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(6, HIGH);
        digitalWrite(7, LOW);
        noInterrupts();
        trans.cancel();
        interrupts();
        //debugPrint("C\n");
        break;
      default:
        debugPrint("NOT GOOD!\n");
    }
  }
}


/// Rising/Falling edge ISR.
void edgeDetect()
{
  if (!edge) {
    edge = true;
    /*  switch(state)
      {
        case s_IDLE:
          state = s_BUSY;
          break;

        default:
          state = s_BUSY;
          break;
      }*/
    state = s_BUSY;
  }
}


/// Timer ISR.
void timeOut()
{
  if (!edge && state == s_BUSY) {
    int rxVal = digitalRead(rxPin);
    if (rxVal == HIGH)
    {
      state = s_IDLE;
    }
    else
    {
      state = s_COLLISION;
    }
  }
}

/// Debug printlines.
void debugPrint(char* s)
{
  if (DEBUG_PRINT_ENABLE)
  {
    Serial.print(s);
  }
}

// Transmits incoming serial data to the 2-wire network.
void transmitSerial() {
  noInterrupts();
  // Serial.println("TRANSMIT!");

  String data = "";
  // While we are not in a collision...
  while (state != s_COLLISION && Serial.peek() != -1) {
    // If the next piece of data is not invalid...
      char c = Serial.read();
      data += c;
      // If the char is a newline, transmit the data and clear our string.
      if (c == '\n') {
        trans.transmit(data.c_str(), data.length());
    }
  }
  interrupts();
  debugPrint("END OF transmitSerial()\n");
  Serial.flush();
}

