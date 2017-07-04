/*
  Copyright 2015-2016 AllThingsTalk

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include <ATT_IOT_LoRaWAN.h>
#include "keys.h"
#include <MicrochipLoRaModem.h>
#include <PayloadBuilder.h>

#define SERIAL_BAUD 57600

#define debugSerial SerialUSB
#define loraSerial Serial1

MicrochipLoRaModem Modem(&loraSerial, &debugSerial);
ATTDevice Device(&Modem, &debugSerial);

static uint8_t sendBuffer[51];
ATT_PB payload(51);  // buffer is set to the same size as the sendBuffer[]

bool sensorVal = false;

void setup() 
{
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);
  pinMode(BUTTON, INPUT_PULLUP);  // initialize the digital pin as an input
  pinMode(LED_GREEN, OUTPUT);
  
  debugSerial.begin(SERIAL_BAUD);                   		// set baud rate of the default serial debug connection
  loraSerial.begin(Modem.getDefaultBaudRate());   			// set baud rate of the serial connection between Mbili and LoRa modem
  while((!debugSerial) && (millis()) < 30000){}         // wait until serial bus is available, so we get the correct logging on screen

  while(!Device.InitABP(DEV_ADDR, APPSKEY, NWKSKEY));
  debugSerial.println("Ready to send data");
  digitalWrite(LED_GREEN, HIGH);
  
  SendValue(0);  // send initial state
}

bool SendValue(bool val)
{
  payload.reset();
  payload.addBoolean(val);
  payload.copy(sendBuffer);
  
  bool res = Device.AddToQueue(&sendBuffer, payload.getSize(), false);  // without ACK!
  
  while(Device.ProcessQueue() > 0) {
    debugSerial.print("QueueCount: ");
    debugSerial.println(Device.QueueCount());
    delay(10000);
  }
  return res;
}

void loop() 
{
  bool sensorRead = digitalRead(BUTTON);  // read status Digital Sensor
  if (sensorRead == 0 )  // verify if value has changed
  {
    if(SendValue(!sensorVal) == true)
    {
      digitalWrite(LED_GREEN, sensorVal);
      sensorVal = !sensorVal;
    }
    delay(1000);
  }
}