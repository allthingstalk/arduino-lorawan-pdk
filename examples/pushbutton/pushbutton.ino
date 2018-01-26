/*    _   _ _ _____ _    _              _____     _ _     ___ ___  _  __
 *   /_\ | | |_   _| |_ (_)_ _  __ _ __|_   _|_ _| | |__ / __|   \| |/ /
 *  / _ \| | | | | | ' \| | ' \/ _` (_-< | |/ _` | | / / \__ \ |) | ' <
 * /_/ \_\_|_| |_| |_||_|_|_||_\__, /__/ |_|\__,_|_|_\_\ |___/___/|_|\_\
 *                             |___/
 *
 * Copyright 2018 AllThingsTalk
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * This basic example gives you a good overview of an example sketch. It
 * has a simple push button that acts as a toggle and sends the state true
 * or false to the AllThingsTalk cloud.
 */

// Select your preferred method of sending data
//#define CONTAINERS
#define CBOR
//#define BINARY

/***************************************************************************/


 
#include <ATT_LoRaWAN.h>
#include "keys.h"
#include <MicrochipLoRaModem.h>

#define SERIAL_BAUD 57600

#define debugSerial SerialUSB
#define loraSerial Serial1

MicrochipLoRaModem modem(&loraSerial, &debugSerial);
ATTDevice device(&modem, &debugSerial, false, 7000);  // Minimum time between 2 messages set at 7000 milliseconds

#ifdef CONTAINERS
  #include <Container.h>
  Container container(device);
#endif

#ifdef CBOR
  #include <CborBuilder.h>
  CborBuilder payload(device);
#endif

#ifdef BINARY
  #include <PayloadBuilder.h>
  PayloadBuilder payload(device);
#endif

void setup() 
{
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);
  pinMode(BUTTON, INPUT_PULLUP);  // Initialize the digital pin as an input
  pinMode(LED_GREEN, OUTPUT);
  
  debugSerial.begin(SERIAL_BAUD);
  while((!debugSerial) && (millis()) < 10000){}  // Wait until the serial bus is available
  
  loraSerial.begin(modem.getDefaultBaudRate());  // Set baud rate of the serial connection to match the modem
  while((!loraSerial) && (millis()) < 10000){}   // Wait until the serial bus is available

  while(!device.initABP(DEV_ADDR, APPSKEY, NWKSKEY));
  debugSerial.println("Ready to send data");
  digitalWrite(LED_GREEN, HIGH);

  // Send initial state
  sendValue(0);
}

void process()
{
  while(device.processQueue() > 0)
  {
    debugSerial.print("QueueCount: ");
    debugSerial.println(device.queueCount());
    delay(10000);
  }
}

void sendValue(bool val)
{
  #ifdef CONTAINERS
  container.addToQueue(val, BINARY_SENSOR, false);
  process();
  #endif
  
  #ifdef CBOR
  payload.reset();
  payload.map(1);
  payload.addBoolean(val, "1");
  payload.addToQueue(false);
  process();
  #endif
  
  #ifdef BINARY
  payload.reset();
  payload.addBoolean(val);
  payload.addToQueue(false);
  process();
  #endif
}

bool sensorVal = false;
bool prevButtonState = false;

void loop() 
{
  bool sensorRead = digitalRead(BUTTON);  // Read status Digital Sensor
  if (sensorRead == 1 && prevButtonState == false)  // verify if value has changed
  {
    prevButtonState = true;
    debugSerial.println("Button pressed");
    sendValue(!sensorVal);
    digitalWrite(LED_GREEN, sensorVal);
    sensorVal = !sensorVal;
  }
  else if(sensorRead == 0)
    prevButtonState = false;
}