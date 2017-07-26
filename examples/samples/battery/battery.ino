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

#define ADC_AREF 3.3f
#define BATVOLT_R1 2.0f
#define BATVOLT_R2 2.0f
#define BATVOLT_PIN BAT_VOLT

#define debugSerial SerialUSB

void setup()
{
  debugSerial.begin(57600);
  
  while(!debugSerial)
  {
    // Wait for Serial Monitor to be opened
  }
}

void loop()
{
  uint16_t voltage = getBatteryVoltage();
  uint16_t bat = (uint16_t)((100.0/ 1200.) * (float) (3000 - voltage));

  debugSerial.print("Voltage : ");
  debugSerial.println(voltage);
  debugSerial.print("Battery : ");
  debugSerial.println(bat);
  debugSerial.println();

  delay(1000);
}

uint16_t getBatteryVoltage()
{
  uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));
  return voltage;
}
