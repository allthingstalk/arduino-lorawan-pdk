/*
   Copyright 2016 AllThingsTalk

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef _att_common_h
#define _att_common_h

#include <ATT_IOT_LoRaWAN.h>
#include <MicrochipLoRaModem.h>
#include "ATT_LoRaWAN_rtcZero.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "arduino.h"
#else
  #include "WProgram.h"
#endif

//////////////////////
//power


void initPower();
void setGPSPower(int8_t value);
void setPower(int8_t value);
//////////////////////


void informStartOfCalibration();
void informEndOfCalibration();
void signalSendResult(bool value);
void signalSendStart();

// Ledcolors

enum LedColor {
    NONE = 0,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    MAGENTA,
    CYAN,
    WHITE
};

/**
 * Turns the led on according to the given color. Makes no assumptions about the status of the pins
 * i.e. it sets them every time,
 */
void setLedColor(LedColor color);

//////////////////////
// wire communication

uint8_t readReg(uint8_t reg);
uint8_t writeReg(uint8_t reg, uint8_t val);


//////////////////////
//board

void initAcceleroInterrupts();
void attach(int pin, voidFuncPtr callback, int mode);
//returns true if the accelerometer reported a change in the magnetic field
//resets the switch so that a new change can be detected on the next run.
bool magnetoChanged();
//returns true if the accelerometer reported movement
//resets the switch so that new movement can be detected on the next run.
bool hasMoved();
//put the device is a deep sleep mode.
void sleep();
void disableUSB();
void enableUSB();

//////////////////////
//battery

//calculates the current battery level.
int batteryStatus();

#endif
