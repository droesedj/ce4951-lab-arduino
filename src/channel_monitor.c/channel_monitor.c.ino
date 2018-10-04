/*
   LAB WEEK 3: CHANNEL MONITOR
   CE4951-012
   @author Dennis Droese
   @filename channel_monitor.c
   @date 9/19/2018
*/
#include "TimerOne.h"
#include "transmitter.h"

#define DEVICE_ADDRESS 23
#define DEBUG_PRINT_ENABLE false

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
const int t_DELAY = bp;

volatile uint8_t state = s_IDLE;
volatile bool edge = false;

// Serial data to be sent is stored here.
String txData = "";

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
        while (Serial.available() > 0) {
          char c = Serial.read();
          txData.concat(c);
          if (c == '\n') {
            transmitSerial();
            // "clear" the string buffer
            txData = "";
          }
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
  // While we are not in a collision...
  while (state != s_COLLISION) {
    trans.transmit(txData.c_str(), txData.length());
    break;
  }
  interrupts();
  debugPrint("END OF transmitSerial()\n");
}

