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

#include "ATT_LoRaWAN_TRACKTRACE.h"
#include <Wire.h>

#define BATTERY_REPORT_SEC 59						//determins how frequently the interrupt clock wakes up to send battery stats.
#define BATTERY_REPORT_MIN 59
#define BATTERY_REPORT_HOUR 23
#define BATTERY_REPORT_EVERY RTCZero::MATCH_MMSS 	//MATCH_HHMMSS

//These constants are used for reading the battery voltage
#define ADC_AREF 3.3
#define BATVOLT_PIN BAT_VOLT
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

#define SENDBATTERYEVERY 86400000					//send battery every 24 hours, used to check time if we missed a hartbeat from the interrupt clock.

/**
 * Turns the led on according to the given color. Makes no assumptions about the status of the pins
 * i.e. it sets them every time,
 */
void setLedColor(LedColor color)
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);

    switch (color)
    {
    case NONE:
        break;
    case RED:
        digitalWrite(LED_RED, LOW);
        break;
    case GREEN:
        digitalWrite(LED_GREEN, LOW);
        break;
    case BLUE:
        digitalWrite(LED_BLUE, LOW);
        break;
    case YELLOW:
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, LOW);
        break;
    case MAGENTA:
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, LOW);
        break;
    case CYAN:
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
        break;
    case WHITE:
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, LOW);
        break;
    default:
        break;
    }
}


//power

void initPower()
{
	pinMode(ENABLE_PIN_IO, OUTPUT);
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;		//prepare the board for deep sleep mode.
}

void setGPSPower(int8_t value)
{
	digitalWrite(GPS_ENABLE, value);
}

void setPower(int8_t value)
{
	digitalWrite(ENABLE_PIN_IO, value);
}


// wire communication

#define ACCEL_ADR 0b0011110

uint8_t writeReg(uint8_t reg, uint8_t val)
{
	Wire.beginTransmission(ACCEL_ADR);
	Wire.write(reg);
	Wire.write(val);
	Wire.endTransmission();
	delayMicroseconds(10000);
}

uint8_t readReg(uint8_t reg)
{
	Wire.beginTransmission(ACCEL_ADR);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(ACCEL_ADR, 0x01);

	uint8_t val = Wire.read();
	Wire.endTransmission();

	return val;
}

//board

volatile bool _reportMovement = false;
volatile bool _reportMagneto = false;

void onAccelInterrupt()
{
	_reportMovement = true;
}

void onMagnetoInterrupt()
{
	_reportMagneto = true;
}

void initAcceleroInterrupts()
{
	pinMode(ACCEL_INT1, INPUT_PULLUP);
	pinMode(ACCEL_INT2, INPUT_PULLUP);
	attach(ACCEL_INT1, onAccelInterrupt, FALLING);
	attach(ACCEL_INT2, onAccelInterrupt, FALLING);
}

void attach(int pin, voidFuncPtr callback, int mode)
{
	attachInterrupt(pin, callback, mode);
	//has to be set after first call to attachinterrupt
	SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;						// Set the XOSC32K to run in standby
	//possibly that we have to do |= in the next statement, to keep the settings done by the rtczero timer.
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;// Configure EIC to use GCLK1 which uses XOSC32K
}

//returns true if the accelerometer reported movement
//resets the switch so that new movement can be detected on the next run.
bool hasMoved()
{
	if (_reportMovement) {
		_reportMovement = false;
		return true;
	}
	return false;
}

//returns true if the accelerometer reported a change in the magnetic field
//resets the switch so that a new change can be detected on the next run.
bool magnetoChanged()
{
	if (_reportMagneto) {
		_reportMagneto = false;
		return true;
	}
	return false;
}

void disableUSB()
{
	USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
}

void enableUSB()
{
	USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
}

//put the device is a deep sleep mode.
void sleep()
{
	delay(180);
	//disableUSB();
	__WFI();							//Enter sleep mode
	//...Sleep forever or until interupt has arrived.
	//enableUSB();
	delay(180);							//after waking up, give the device a little bit of time to wake up, if we don't do this, the serial will block and the first call to SerialUSB will make the app fail.
}

//battery

int batteryStatus()
{
	SerialUSB.println("reporting battery status");
	uint16_t batteryVoltage = analogRead(BATVOLT_PIN);
	uint16_t battery = (ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)batteryVoltage;

	battery = (uint16_t)((100.0/ 1200.) * (float) (battery-3000));
	if (battery > 100)
		battery = 100;
	SerialUSB.print("level: ");  SerialUSB.println(battery);
  return battery;
}
