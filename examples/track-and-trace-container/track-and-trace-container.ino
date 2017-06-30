 /****
 *  Copyright (c) 2016 AllThingsTalk. All rights reserved.
 *
 *  AllThingsTalk Developer Cloud IoT demonstrator for SodaqOne
 *  Version 1.0 dd 12/6/2016
 *  Original author: Jan Bogaerts 2016
 
   Copyright 2017 AllThingsTalk

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 
 *
 *  This sketch is part of the AllThingsTalk LoRaWAN product development kit
 *  -> http://www.allthingstalk.com/lora-rapid-development-kit
 *
 *  For more information, please check our documentation
 *  -> http://allthingstalk.com/docs/tutorials/lora/setup
 * 
 **/

#include <ATT_IOT_LoRaWAN.h>
#include <MicrochipLoRaModem.h>
#include <Wire.h>
#include <ATT_LoRaWAN_LSM303.h>
#include <ATT_LoRaWAN_RTCZero.h>
#include <ATT_LoRaWAN_UBlox_GPS.h>
#include <ATT_LoRaWAN_TRACKTRACE.h>
#include "Config.h"
#include "BootMenu.h"
#include <Container.h>


#define GPS_WAKEUP_EVERY_SEC 5                 //seconds part of the clock that wakes up the device for retrieving a GPS fix.    
#define GPS_WAKEUP_EVERY_MIN 0                  //minutes part of the clock that wakes up the device for retrieving a GPS fix
#define MAX_MOVEMENT_TOLERANCE 500              //sensitivity of the accelerometer while in motion (to verify if the device is still moving), the closer to 0, the more sensitive. expressed in absolute accelero values
#define ACCELERO_NR_CALIBRATION_STEPS 20        // the nr of iterations used during calibration of the device.
#define MOVEMENT_NR_OF_SAMPLES 10               // the number of samples taken from the accelero in order to determine if the device is moving or not. (there can only be 2 measurements with diffeernt g forces)

#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'a') ? (c - 'a' + 0x0A) : ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0')))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

#define SEC_IN_A_DAY 86400                     // the number of seconds in a day -> to trigger sending over the battery level


MicrochipLoRaModem Modem(&Serial1, &SerialUSB);
ATTDevice Device(&Modem, &SerialUSB, false, 7000);  // Min Time between 2 consecutive messages set @ 7 seconds

Container container(Device);

LSM303 compass;
bool _wasMoving;                                //keeps track of the moving state. If this is true, the device was moving when last checked.
//bool _resendWasMoving = false;                  //when this flag is true, then the system needs to try and resend the fact that the device has moved -> it failed, but this is an important state to send.
bool _gpsScanning = false;                      //the gps scans for a location fix in an async way, this flag keeps track of the scan state: are we currently looking for a gps fix or not.
bool _foundGPSFix = false;						//have we found a 1st gps fix, if not, we time out longer, if we have, we can query the gps faster.

int16_t x, y = 0;                                       //the current x, y, z acceleration values (for gravity compensation)

RTCZero rtc;
volatile bool wakeFromTimer = false;


int elapsedSeconds = 0;                       // used to send over batterystatus every 24 hrs( e.g.86400 seconds)

int gpsRetryCount = 1;

bool _ledsEnabled = false;

float latPrevious = 0;
float longPrevious = 0;


#define ACCEL_ADR 0b0011110                     //ACCELERO ADDRESS, DO NOT CHANGE.
#define ACCELERO_COMPENSATION 0.061             // used to convert acceleration to g, do not change


// Payload data structure

struct gpsPayload
{
  byte msg = 0x00;
  float lat;
  float lon;
  float alt;
  float spd;
};

struct batPayload
{
  byte msg = 0x01;
  short bat;
};

gpsPayload gpsData;
batPayload batData;




void calibrate(int16_t &x, int16_t &y)
{
    x = 0;
    y = 0;
    SerialUSB.println("begin calibration");
    for (int i = 1; i <= ACCELERO_NR_CALIBRATION_STEPS; i++)
    {
        compass.read();

        SerialUSB.print("x: "); SerialUSB.print(compass.a.x * ACCELERO_COMPENSATION);
        SerialUSB.print(", y: "); SerialUSB.print(compass.a.y * ACCELERO_COMPENSATION);
        SerialUSB.print(", z: "); SerialUSB.println(compass.a.z * ACCELERO_COMPENSATION);

        if (abs(compass.a.x) > x) {
            x = abs(compass.a.x);
            SerialUSB.print("max x: "); SerialUSB.println(x * ACCELERO_COMPENSATION);
        }
        if (abs(compass.a.y) > y) {
            y = abs(compass.a.y);
            SerialUSB.print("max y: "); SerialUSB.println(y * ACCELERO_COMPENSATION);
        }
        delay(100);
    }
    SerialUSB.println("end calibration");
}



void setThresholds(int16_t x, int16_t y)
{
    uint8_t sensitivity = params.getAcceleroSensitivity();
    SerialUSB.print("base x: "); SerialUSB.println(x);
    SerialUSB.print("base y: "); SerialUSB.println(y);
    x = (round((x * ACCELERO_COMPENSATION) / 16)) + sensitivity;                 //16 mg step size at 2g accuratie
    y = (round((y * ACCELERO_COMPENSATION) / 16)) + sensitivity;
    SerialUSB.print("x: 0x"); SerialUSB.print(x, HEX); SerialUSB.print("; 0b"); SerialUSB.println(x, BIN);
    SerialUSB.print("y: 0x"); SerialUSB.print(y, HEX); SerialUSB.print("; 0b"); SerialUSB.println(y, BIN);

    writeReg(0x32, x & 0x7F); // Threshold (gen 1)
    writeReg(0x36, y & 0x7F); // Threshold (gen 2)
}

void enableAcceleroInterupts()
{
    writeReg(0x22, 0b00100000);
    writeReg(0x23, 0b00100000);
}

void disableAcceleroInterupts()
{   
    writeReg(0x22, 0b00000000);
    writeReg(0x23, 0b00000000);
}


void prepareInterrupts(int16_t x, int16_t y)
{
    initAcceleroInterrupts();
    
    writeReg(0x1F, 0b10000000); // reboot
    writeReg(0x20, 0b01010111); // ctrl1
    writeReg(0x30, 0b10000010); // Axes mask (gen 1)
    writeReg(0x34, 0b10001000); // Axes mask (gen 2)
    setThresholds(x, y);
    writeReg(0x33, 0b00010010); // Duration (gen 1)
    writeReg(0x37, 0b00010010); // Duration (gen 2)
    enableAcceleroInterupts();
}

void onSleepDone() 
{
    wakeFromTimer = true;
}

//unsigned long _nextIntervalAt = 0;								// when a wakeupclock is initialized, this value keeps track of when the next interval impuls should be given, so that we can also do	

//set up an interupt to wake up the device
void setWakeUpClock()
{
    //SerialUSB.print("get Minutes "); SerialUSB.println(rtc.getMinutes());
    //SerialUSB.print("get seconds "); SerialUSB.println(rtc.getSeconds());
    elapsedSeconds = elapsedSeconds + rtc.getMinutes()*60 + rtc.getSeconds();
    SerialUSB.print("elapsed seconds "); SerialUSB.println(elapsedSeconds);

    if(elapsedSeconds > SEC_IN_A_DAY) {                                   // if 24 hrs have been passed; sendover battery status. This payload can also be used as keepalive signal
       elapsedSeconds = elapsedSeconds -SEC_IN_A_DAY;
       trySendBatteryState();
    }
    
    if(_gpsScanning){
		   if(_foundGPSFix){										//if we already found a fix before, we can get a new one really fast, so we use a short delay. If we havent, it will take a longer time.
			    rtc.setAlarmSeconds(GPS_WAKEUP_EVERY_SEC);                      // Schedule the wakeup interrupt
			    rtc.setAlarmMinutes(GPS_WAKEUP_EVERY_MIN);
			    SerialUSB.print("set wake-up in "); SerialUSB.print(GPS_WAKEUP_EVERY_SEC);  SerialUSB.println(" seconds");
		      }
		   else{
			    rtc.setAlarmSeconds(GPS_WAKEUP_EVERY_SEC * 6);
			    rtc.setAlarmMinutes(GPS_WAKEUP_EVERY_MIN);
			    SerialUSB.print("set wake-up in "); SerialUSB.print(GPS_WAKEUP_EVERY_SEC * 6);  SerialUSB.println(" seconds");
		      }
       }
    else{
       rtc.setAlarmSeconds(params.getFixIntervalSeconds());                      // Schedule the wakeup interrupt
       rtc.setAlarmMinutes(params.getFixIntervalMinutes());
		   SerialUSB.print("set wake-up in "); SerialUSB.print(params.getFixIntervalMinutes());  SerialUSB.println(" minutes");
       }
    rtc.enableAlarm(RTCZero::MATCH_MMSS);                       // MATCH_SS
    rtc.attachInterrupt(onSleepDone);                           // Attach handler so that we can set the battery flag when the time has passed.
    rtc.setEpoch(0);                                            // This sets it to 2000-01-01
}



//starts up the gps unit and sets a flag so that the main loop knows we need to scan the gps (async scanning).
void startGPSFix()
{
    if(_gpsScanning == false){                        //only try to start it if the gps is not yet running, otherwise, we might reset the thing and never get a fix.
        gpsRetryCount = 1;
        uint32_t timeout = params.getGpsFixTimeout() * (_foundGPSFix ? 5000 : 30000);
        SerialUSB.println(String("spinning up gps ..., timeout in ") + (timeout / 1000) + String("s"));
        sodaq_gps.startScan();
        _gpsScanning = true;
    }
}

//stops the async scanning of the gps module
void stopGPSFix()
{
    SerialUSB.println("shutting down gps");
    sodaq_gps.endScan();
    _gpsScanning = false;
}

//gets the current location from the gps and sends it to the cloud.
//withSendDelay: when true, the function will pause (15 sec) before sending the message. The 15 sec is in total, so getting a gps fix is included in that time.
//returns true if the operation was terminated. Otherwise false
bool trySendGPSFix()
{
	SerialUSB.println("scanning gps");
    if (sodaq_gps.tryScan(10)) {
       SerialUSB.println(" found GPS fix! ");
       Modem.WakeUp();
       if(_ledsEnabled) setLedColor(GREEN); 
       stopGPSFix();

       
       SerialUSB.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
       SerialUSB.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
       SerialUSB.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
		   SerialUSB.println(String(" alt = ") + String(sodaq_gps.getAlt(), 7));

       SerialUSB.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));
       
       // calculate distance between previous and new coördinates
       

       if (_foundGPSFix) {
         float lat2 = sodaq_gps.getLat();
         float long2 = sodaq_gps.getLon();
         float lat1 = latPrevious;
         float long1 = longPrevious;
         SerialUSB.println(String(" distance between 2 points = ") + String(sodaq_gps.distance_between(lat1, long1, lat2,long2)));
       }

       latPrevious = sodaq_gps.getLat();
       longPrevious = sodaq_gps.getLon();

       SerialUSB.print("#bytes in payload: "); SerialUSB.println(sizeof(gpsData));
       container.AddToQueue(float(sodaq_gps.getLat()), float(sodaq_gps.getLon()), float(sodaq_gps.getAlt()), 0., GPS, false);  // No ACK
       Device.ProcessQueue();
       while(Device.ProcessQueue() > 0) {
          SerialUSB.print("QueueCount: "); SerialUSB.println(Device.QueueCount());
          delay(10000);
          }
       SerialUSB.println(String(" spd = ") + String(sodaq_gps.getSpeed(), 7)); 
       container.AddToQueue((float)sodaq_gps.getSpeed(), NUMBER_SENSOR, false);  // No ACK
       Device.ProcessQueue();
       while(Device.ProcessQueue() > 0) {
          SerialUSB.print("QueueCount: "); SerialUSB.println(Device.QueueCount());
          delay(10000);
          }
       
       Modem.Sleep();
       if(_ledsEnabled) setLedColor(NONE);
		   _foundGPSFix = true;
		   return true;
       }
    else if(gpsRetryCount >= params.getGpsFixTimeout()){
       stopGPSFix();
       SerialUSB.println("GPS module stopped: failed to find fix. new GPS fix will be attempted upon next wake-up");
		   return true;
       }
    else
        SerialUSB.println("No GPS Fix yet");
        gpsRetryCount = gpsRetryCount +1;
	      return false;
}

bool trySendBatteryState()
{
  Modem.WakeUp();
  if(_ledsEnabled) setLedColor(GREEN);
  SerialUSB.println("reading battery level");
  batData.bat = batteryStatus();
  SerialUSB.print("#bytes in payload: "); SerialUSB.println(sizeof(batData));
  container.AddToQueue(&batData, BATTERY_LEVEL, false);  // without ACK!
  Device.ProcessQueue();
  while(Device.ProcessQueue() > 0) {
     SerialUSB.print("QueueCount: "); SerialUSB.println(Device.QueueCount());
     delay(10000);
     }
  Modem.Sleep();
  if(_ledsEnabled) setLedColor(NONE);
  return true;
}

 
/**
 * Shows and handles the boot up menu if there is a serial port.
 */
void handleSerialPort()
{
    SerialUSB.begin(57600);
    if (params.keysAndAddrSpecified()){                     //main configs have been supplied, use short boot cycle.
        unsigned long start = millis();
        pinMode(BUTTON, INPUT_PULLUP);
        int sensorVal = digitalRead(BUTTON);
        while(sensorVal == HIGH && start + 5000 > millis())		//should be save from overflow: only happens at the start.
            sensorVal = digitalRead(BUTTON);
        if(sensorVal == LOW){
			while(!SerialUSB){}                                 //wait until the serial port is connected, or time out if there is none.
            do {
                showBootMenu(SerialUSB);
            } while (!params.checkConfig(SerialUSB) || !params.keysAndAddrSpecified());
            params.showConfig(&SerialUSB);
        }
    }
    else{
        while(!SerialUSB){}                                 //wait until the serial port is connected, or time out if there is none.
        do {
            showBootMenu(SerialUSB);
        } while (!params.checkConfig(SerialUSB) || !params.keysAndAddrSpecified());
        params.showConfig(&SerialUSB);
    }
}

/**
 * Converts the given hex array and returns true if it is valid hex and non-zero.
 * "hex" is assumed to be 2*resultSize bytes.
 */
bool convertAndCheckHexArray(uint8_t* result, const char* hex, size_t resultSize)
{
    bool foundNonZero = false;

    uint16_t inputIndex = 0;
    uint16_t outputIndex = 0;

    // stop at the first string termination char, or if output buffer is over
    while (outputIndex < resultSize && hex[inputIndex] != 0 && hex[inputIndex + 1] != 0) {
        if (!isxdigit(hex[inputIndex]) || !isxdigit(hex[inputIndex + 1])) {
            return false;
        }

        result[outputIndex] = HEX_PAIR_TO_BYTE(hex[inputIndex], hex[inputIndex + 1]);

        if (result[outputIndex] > 0) {
            foundNonZero = true;
        }

        inputIndex += 2;
        outputIndex++;
    }

    result[outputIndex] = 0; // terminate the string

    return foundNonZero;
}

void connect()
{
	uint8_t devAddr[5];
	uint8_t appSKey[17];
	uint8_t nwkSKey[17];

	SerialUSB.println(params.getUseAccelero());
	bool allParametersValid = convertAndCheckHexArray((uint8_t*)devAddr, params.getDevAddrOrEUI(), sizeof(devAddr) -1)
		&& convertAndCheckHexArray((uint8_t*)appSKey, params.getAppSKeyOrEUI(), sizeof(appSKey) -1)
		&& convertAndCheckHexArray((uint8_t*)nwkSKey, params.getNwSKeyOrAppKey(), sizeof(nwkSKey)-1);
		
	if(!allParametersValid){
		SerialUSB.println("Invalid parameters for the lora connection, can't start up lora modem.");
		return;
	}
	//Device.SetMinTimeBetweenSend(150000);					//we are sending out many messages after each other, make certain that they don't get blocked by base station.
	// Note: It is more power efficient to leave Serial1 running
    Serial1.begin(Modem.getDefaultBaudRate());              // init the baud rate of the serial connection so that it's ok for the modem
    Modem.Sleep();                                          //make certain taht the modem is synced and awake.
    delay(50);
    Modem.WakeUp();
    while (!Device.InitABP(devAddr, appSKey, nwkSKey));
    SerialUSB.print("Modem version: "); SerialUSB.println(Modem.getSysParam("ver"));
    SerialUSB.println("Ready to send data");
}
 


void setup()
{   
  initPower();
  sodaq_gps.init();                                       //do this as early as possible, so we have a workign gps.
  //sodaq_gps.setDiag(SerialUSB);
  params.read();                                          //get any previously saved configs, if there are any (otherwise use default).
  _ledsEnabled = params.getIsLedEnabled();
  setPower(HIGH);                                         //turn board on
  if(_ledsEnabled) setLedColor(BLUE);                                      // BLUE led indicates the device is bootingdevice
  handleSerialPort();
  SerialUSB.println("start of track and trace demo v2.0");
  delay(500);
  rtc.begin();
  connect();

  Wire.begin();
  if(_ledsEnabled) setLedColor(NONE);                                           //indicate end of init

  trySendBatteryState();
  
 
  if(params.getUseAccelero()){
     compass.init(LSM303::device_D);
     compass.enableDefault();
     calibrate(x, y);
     prepareInterrupts(x, y);
     //the default state of the accelero meter is shut-down mode.
     }
   else {
     // disable accelerometer, power-down mode
     compass.writeReg(LSM303::CTRL1, 0);
     // zero CTRL5 (including turn off TEMP sensor)
     compass.writeReg(LSM303::CTRL5, 0);
     // disable magnetometer, power-down mode
     compass.writeReg(LSM303::CTRL7, 0b00000010);
   } 

  Modem.Sleep();
 
  // Startup initial GPS Fix after bootup.
  _wasMoving = true;
  startGPSFix();
  
  setWakeUpClock();
}


void loop()
{
   if(params.getUseAccelero()){
      if (wakeFromTimer == true) {                                //we got woken up by the timer, so check if still moving.
         SerialUSB.println("wake up from timer");
         if(_ledsEnabled) setLedColor(BLUE);  
         wakeFromTimer = false;
         if(_wasMoving) {
            if(_gpsScanning){                                       //if we already started to send a gps fix, try to see if we have some data to send.  
               if (trySendGPSFix()) {
                  //calibrate(x, y);           
                  //setThresholds(x, y);
                  enableAcceleroInterupts();
                  _wasMoving = false;
                  }
               }
            else
               startGPSFix();
            }
          setWakeUpClock();  
          }  
      else if(hasMoved()){
         SerialUSB.println("Movement detected");
         if(_ledsEnabled) setLedColor(YELLOW);
         delay(1000);
         if(_ledsEnabled) setLedColor(NONE); 
         disableAcceleroInterupts();
			   if(_wasMoving == false)								                     //Mark that there was movement. When timer elapses, data will be transmitted.
				    _wasMoving = true;
         }
      }  
    else if (wakeFromTimer == true){
		   SerialUSB.println("wake up from timer");
       if(_ledsEnabled) setLedColor(BLUE);
       wakeFromTimer = false;
       if(_gpsScanning){                                       //if we already started to send a gps fix, try to see if we have some data to send.  
          trySendGPSFix();
		      }
       else
          startGPSFix();
          setWakeUpClock();
       }
  if(_ledsEnabled) setLedColor(NONE); 
	if(!params.getIsDebugMode()) {             
     SerialUSB.println("Entering sleep mode");
     sleep();
	}

}
