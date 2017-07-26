/*    _   _ _ _____ _    _              _____     _ _     ___ ___  _  __
 *   /_\ | | |_   _| |_ (_)_ _  __ _ __|_   _|_ _| | |__ / __|   \| |/ /
 *  / _ \| | | | | | ' \| | ' \/ _` (_-< | |/ _` | | / / \__ \ |) | ' <
 * /_/ \_\_|_| |_| |_||_|_|_||_\__, /__/ |_|\__,_|_|_\_\ |___/___/|_|\_\
 *                             |___/
 *
 * Copyright 2017 AllThingsTalk
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

#include <Wire.h>
#include "ATT_LoRaWAN_LSM303.h"
#include <ATT_LoRaWAN_RTCZero.h>

#define debugSerial SerialUSB

LSM303 compass;

void setup() 
{
	while(!debugSerial){}
	debugSerial.println("Testing Accelerometer");
  	
	Wire.begin();	
	
	compass.init(LSM303::device_D);
	compass.enableDefault();
  
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  
  //compass.writeReg(0x14, (byte) 0);
  //compass.writeReg(0x15, (byte)0);

  //compass.writeReg(0x16, 0);
  //compass.writeReg(0x17, 0);

  //compass.writeReg(0x18, 0);
  //compass.writeReg(0x19, 0);

  //compass.writeReg(0x1A, 0);
  //compass.writeReg(0x1B, 0);
  
}

void loop()
{
  compass.read();

  debugSerial.print("x: ");
  debugSerial.println(compass.m.x);
  debugSerial.print("y: ");
  debugSerial.println(compass.m.y);
  debugSerial.print("z: ");
  debugSerial.println(compass.m.z);

  //debugSerial.print("x: ");
  //debugSerial.print(compass.a.x * 0.061);
  //debugSerial.print(", y: ");
  //debugSerial.print(compass.a.y* 0.061);
  //debugSerial.print(", z: ");
  //debugSerial.println(compass.a.z* 0.061);
  
	float curHeading = compass.heading();

	debugSerial.print("Heading: ");
  debugSerial.println(curHeading);
 
  delay(1000);
}