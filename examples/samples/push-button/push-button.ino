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

 #define debugSerial SerialUSB
 
void setup()
{
  // Start serial connection
  debugSerial.begin(57600);
  
  // Configure pin2 as an input and enable the internal pull-up resistor
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_GREEN, OUTPUT);
}

void loop()
{
  // Read the pushbutton value into a variable
  int sensorVal = digitalRead(BUTTON);

  // Print out the value of the pushbutton
  debugSerial.print("new value: ");
  debugSerial.println(sensorVal);

  // Turn on the LED when the Button is pushed
  if (sensorVal == HIGH)
    digitalWrite(LED_GREEN, HIGH);
  else
    digitalWrite(LED_GREEN, LOW);
}