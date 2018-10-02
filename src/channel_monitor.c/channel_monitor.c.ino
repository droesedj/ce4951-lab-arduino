/*
   LAB WEEK 3: CHANNEL MONITOR
   CE4951-012
   @author Dennis Droese
   @filename channel_monitor.c
   @date 9/19/2018
*/
#include "Arduino.h"
#include <TimerOne.h>
#include "transmitter.h"

#define DEVICE_ADDRESS 23
#define DEBUG_PRINT_ENABLE false

void edgeDetect();
void timeOut();
void transmitSerial();
void debugPrint(char* s);

enum State_enum {s_IDLE, s_BUSY, s_COLLISION};

const byte ledPin = 13;
const byte rxPin = 3;
const byte txPin = 4;

// Bit period
const uint8_t bp = 1000; //1ms, 1000us

// Maximum allowed time before timing out.
const uint8_t t_DELAY = 1110; //1.11ms, 1110us

volatile uint8_t state = s_IDLE;

Transmitter trans(txPin, bp);

void setup()
{
  // Init the Rx Pin
  pinMode(rxPin, INPUT_PULLUP);

  // Init the LED pin.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize the timer.
  Timer1.initialize(t_DELAY);

  //if (DEBUG_PRINT_ENABLE)
  //{
    // Init the serial port for debugging purposes.
    Serial.begin(9600);
  //}

  // Attach the interrupt. Start the timer.
  Timer1.attachInterrupt(timeOut);
  Timer1.start();

  // Attach edge interrupt.
  attachInterrupt(digitalPinToInterrupt(rxPin), edgeDetect, CHANGE);
}

void loop()
{
  // Restart timer.
  Timer1.restart();
  switch (state)
  {
    case s_IDLE:
      digitalWrite(LED_BUILTIN, LOW);
      debugPrint("I\n");
      if (Serial.available() > 0) {
        transmitSerial();
      }
      break;

    case s_BUSY:
      digitalWrite(LED_BUILTIN, HIGH);
      debugPrint("B\n");
      break;

    case s_COLLISION:
      digitalWrite(LED_BUILTIN, LOW);
      trans.cancel();
      debugPrint("C\n");
      break;
  }
}


/// Rising/Falling edge ISR.
void edgeDetect()
{
  // Disable timer.
  Timer1.stop();
  switch (state)
  {
    case s_IDLE:
      state = s_BUSY;
      break;

    default:
      state = s_BUSY;
      break;
  }
}


/// Timer ISR.
void timeOut()
{
  // Disable timer.
  Timer1.stop();
  // Read the RX pin.
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

/// Debug printlines.
void debugPrint(char* s)
{
  if (DEBUG_PRINT_ENABLE)
  {
    Serial.print(s);
  }
}

void transmitSerial() {
  Timer1.stop();
  Serial.println("TRANSMIT!");
  Serial.flush();

  String data = "";
  while (state != s_COLLISION) {
    while (Serial.available()) {
      char c = Serial.read();
      data += c;
    }
    trans.transmit(data.c_str());
    break;
  }
}

