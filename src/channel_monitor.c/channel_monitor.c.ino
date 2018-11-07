/*
   LAB WEEK 3: CHANNEL MONITOR
   CE4951-012
   @author Dennis Droese
   @filename channel_monitor.c
   @date 9/19/2018
*/
#include "TimerOne.h"
#include "transmitter.h"
#include "packet.h"

#define DEVICE_ADDRESS 23
#define TARGET_ADDRESS 0
#define DEBUG_PRINT_ENABLE false
#define DEBUG_COMMANDS_ENABLE true
#define READ_ALL_MESSAGES true

void edgeDetect();
void timeOut();
void transmitSerial();
void debugPrint(char* s);
void wipeRxData();

enum State_enum {s_IDLE, s_BUSY, s_COLLISION};

const byte rxPin = 3;
const byte txPin = 5;

// Bit period
const int bp = 1000; //1ms, 1000us

// Maximum allowed time before timing out.
const int t_DELAY = bp;

volatile uint8_t state = s_IDLE;
volatile bool edge = false;


/// RECEIVER VARIABLES ///

volatile int bit_index = 0;
volatile int rx_index = 0;
volatile byte rxData[512];
volatile byte rxDataBuffer = 0;

////////////////////////

// Serial data to be sent is stored here.
String txData = "";

Transmitter trans(txPin, bp);

void setup()
{
  // Init the Rx Pin
  pinMode(rxPin, INPUT);

  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);

  // Init the LED pin.
  pinMode(LED_BUILTIN, OUTPUT);

  //pinMode(6, OUTPUT);

  // Init the serial port.
  Serial.begin(115200);

  // Initialize the timer.
  Timer1.initialize(bp);

  // Attach the interrupt. Start the timer.
  Timer1.attachInterrupt(timeOut);

  // Attach edge interrupt.
  attachInterrupt(digitalPinToInterrupt(rxPin), edgeDetect, CHANGE);
  // Start the timer.
  Timer1.start();
}

void loop()
{
  switch (state)
  {
    case s_IDLE:
      digitalWrite(LED_BUILTIN, LOW);
      if (Serial.available() > 0) {
        //state = s_BUSY;
        char c = Serial.read();
        txData.concat(c);
        if (c == '\n') {
          transmitSerial();
          // "clear" the string buffer
          txData = "";
        }
      } else if (rx_index > 0 && bit_index == 0) {
        //TODO: print the recieved data, wipe the buffer.
        noInterrupts();
        // If we don't care where the message is *supposed* to go.
        if (READ_ALL_MESSAGES) {
          //Serial.println("RECEIVED:");
          Packet pkt = Packet(rxData, rx_index);
          if (!pkt.CRC8_valid) {
            Serial.println("WARNING! THE FOLLOWING PACKET'S CRC DOES NOT MATCH THE HEADER!!!");
          }
          //Serial.println(pkt.GetSummary().c_str());
          WipeRxData();
        } else {
          // Only read messages with a destination that matches the device's address.
          Packet pkt = Packet(rxData, rx_index);
          if (pkt.GetDestination() == DEVICE_ADDRESS || pkt.GetDestination() == 0x00) {
            //Serial.println("RECEIVED:");
            if (!pkt.CRC8_valid) {
              Serial.println("WARNING! THE FOLLOWING PACKET'S CRC DOES NOT MATCH THE HEADER!!!");
            }
            //Serial.println(pkt.GetSummary().c_str());
            WipeRxData();
          }
        }
        interrupts();
      }
      break;

    case s_BUSY:
      if (bit_index > 7) {
        //char c = rxData;
        //Serial.print(' ');
        //Serial.println(rxData);
        //Serial.println("@");
        bit_index = 0;
        rxData[rx_index] = rxDataBuffer;
        rxDataBuffer = 0;
        rx_index++;
      }
      break;

    case s_COLLISION:
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println();
      trans.cancel();
      txData = "";
      WipeRxData();
      break;
    default:
      debugPrint("NOT GOOD!\n");
  }
}


/// Rising/Falling edge ISR.
void edgeDetect()
{
  Timer1.stop();
  detachInterrupt(digitalPinToInterrupt(rxPin));
  //Serial.println("EDGE");
  state = s_BUSY;

  int rxVal;
  if (state == s_BUSY) {
    if (bit_index < 8) {
      //digitalWrite(6, LOW);
      delayMicroseconds((bp / 2) + (bp * 0.0132));
      //digitalWrite(6, HIGH);
      rxVal = digitalRead(rxPin);
      rxDataBuffer = (rxDataBuffer << 1) | rxVal;
      bit_index++;
      //Serial.print(rxVal);
    }
    if (rxVal == 0) {
      attachInterrupt(digitalPinToInterrupt(rxPin), edgeDetect, RISING);
    } else {
      attachInterrupt(digitalPinToInterrupt(rxPin), edgeDetect, FALLING);
    }
  }

  //state = s_BUSY;
  Timer1.restart();
  Timer1.start();
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
  // While we are not in a collision...
  while (state != s_COLLISION) {
    if (DEBUG_COMMANDS_ENABLE) {
      if (txData.startsWith("DEV_0")) {
        // send 8 zeroes.
        byte data[] = {0b00000000};
        trans.transmit(data, 1);
        return;
      } else if (txData.startsWith("DEV_1")) {
        // send 8 ones.
        byte data[] = {0b11111111};
        trans.transmit(data, 1);
        return;
      } else if (txData.startsWith("DEV_LOOP")) {
        // send a loop of ones.
        while (state != s_COLLISION) {
          byte data = {0b11111111};
          trans.transmit(data, 1);
          //If the rx line goes low, assume collision.
          int rxVal = digitalRead(rxPin);
          if (rxVal == LOW) {
            //isTransmitting = false;
            return;
          }
        }

      } else {
        //trans.transmit(txData.c_str(), txData.length());
        Packet outPkt = Packet(txData.c_str(), txData.length(), DEVICE_ADDRESS, TARGET_ADDRESS);
        //Serial.println(outPkt.GetSummary().c_str());
        bool goodTrans = false;
        while (!goodTrans) {
          //Serial.println("OUTGOING:");
          goodTrans = outPkt.Transmit(trans);
          if (!goodTrans) {
            delay(1000);
            Serial.println("deLAY");
            Timer1.restart();
            Timer1.start();
          }
        }
        break;
      }

    } else {
      //trans.transmit(txData.c_str(), txData.length());
      Packet outPkt = Packet(txData.c_str(), txData.length(), DEVICE_ADDRESS, TARGET_ADDRESS);
      //Serial.println(outPkt.GetSummary().c_str());
      bool goodTrans = false;
      while (!goodTrans) {
        goodTrans = outPkt.Transmit(trans);
        if (!goodTrans) {
          delay(1000);
          Timer1.restart();
          Timer1.start();
          Serial.println("deLAY");
        }
      }
      break;
    }
    txData = "";
  }
  debugPrint("END OF transmitSerial()\n");
}

void WipeRxData() {
  //Serial.println("WIPED!");
  for (int i = 0; i < sizeof(rxData); i++) {
    rxData[i] = 0;
  }
  bit_index = 0;
  rx_index = 0;
  rxDataBuffer = 0;
}

