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

// Select your preferred method of sending data
#define CONTAINERS
//#define CBOR
//#define BINARY

/***************************************************************************/


 
#include <ATT_LoRaWAN.h>
#include <MicrochipLoRaModem.h>
#include <Wire.h>
#include <ATT_LoRaWAN_LSM303.h>
#include <ATT_LoRaWAN_RTCZero.h>
#include <ATT_LoRaWAN_UBlox_GPS.h>
#include <ATT_LoRaWAN_TRACKTRACE.h>
#include "Config.h"
#include "BootMenu.h"

#define GPS_WAKEUP_EVERY_SEC 5            // Seconds part of the clock that wakes up the device for retrieving a GPS fix
#define GPS_WAKEUP_EVERY_MIN 0            // Minutes part of the clock that wakes up the device for retrieving a GPS fix
#define MAX_MOVEMENT_TOLERANCE 500        // Sensitivity of the accelerometer. expressed in absolute accelero values. 0 = most sensitive
#define ACCELERO_NR_CALIBRATION_STEPS 20  // The nr of iterations used during calibration of the device
#define MOVEMENT_NR_OF_SAMPLES 10         // The number of samples taken from the accelerometer to determine movement

#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'a') ? (c - 'a' + 0x0A) : ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0')))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

#define SEC_IN_A_DAY 86400

// Sodaq ONE serial communication
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

LSM303 compass;
bool _wasMoving;                // Keep track of the moving state
//bool _resendWasMoving = false;  // When true, then the system needs to try and resend
bool _gpsScanning = false;      // The gps scans for a location fix in an async way, this flag keeps track of the scan state
bool _foundGPSFix = false;      // If we have a first gps fix we can query the gps faster, if not we time out longer

int16_t x, y = 0;  // the current x, y, z acceleration values (for gravity compensation)

RTCZero rtc;
volatile bool wakeFromTimer = false;

int elapsedSeconds = 0;  // used to send over batterystatus every 24 hrs (e.g. 86400 seconds)
int gpsRetryCount = 1;

bool _ledsEnabled = false;

float latPrevious = 0;
float longPrevious = 0;

#define ACCEL_ADR 0b0011110          // accelero address !! DO NOT CHANGE !!
#define ACCELERO_COMPENSATION 0.061  // used to convert acceleration to g !! DO NOT CHANGE !!

void setThresholds(int16_t x, int16_t y)
{
  uint8_t sensitivity = params.getAcceleroSensitivity();

  debugSerial.print("base x: ");
  debugSerial.println(x);
  debugSerial.print("base y: ");
  debugSerial.println(y);
  
  x = (round((x * ACCELERO_COMPENSATION) / 16)) + sensitivity;  // 16 mg step size at 2g accuratie
  y = (round((y * ACCELERO_COMPENSATION) / 16)) + sensitivity;
  
  debugSerial.print("x: 0x");
  debugSerial.print(x, HEX);
  debugSerial.print("; 0b");
  debugSerial.println(x, BIN);
  
  debugSerial.print("y: 0x");
  debugSerial.print(y, HEX);
  debugSerial.print("; 0b");
  debugSerial.println(y, BIN);

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
    
  writeReg(0x1F, 0b10000000);  // reboot
  writeReg(0x20, 0b01010111);  // ctrl1
  writeReg(0x30, 0b10000010);  // Axes mask (gen 1)
  writeReg(0x34, 0b10001000);  // Axes mask (gen 2)

  setThresholds(x, y);

  writeReg(0x33, 0b00010010);  // Duration (gen 1)
  writeReg(0x37, 0b00010010);  // Duration (gen 2)

  enableAcceleroInterupts();
}

void onSleepDone() 
{
  wakeFromTimer = true;
}

// When a wakeupclock is initialized, this value keeps track of when the next interval impuls should be given
//unsigned long _nextIntervalAt = 0;

/****
 * Set up an interupt to wake up the device
 */
void setWakeUpClock()
{
  //debugSerial.print("get Minutes ");
  //debugSerial.println(rtc.getMinutes());
  //debugSerial.print("get seconds ");
  //debugSerial.println(rtc.getSeconds());

  elapsedSeconds = elapsedSeconds + rtc.getMinutes()*60 + rtc.getSeconds();
  debugSerial.print("elapsed seconds ");
  debugSerial.println(elapsedSeconds);

  // if 24 hrs have passed send battery status
  // this payload can also be used as keepalive signal
  if(elapsedSeconds > SEC_IN_A_DAY)
  {
    elapsedSeconds = elapsedSeconds -SEC_IN_A_DAY;
    sendBatteryState();
  }
    
  if(_gpsScanning)
  {
    if(_foundGPSFix)
    {
      rtc.setAlarmSeconds(GPS_WAKEUP_EVERY_SEC);  // schedule the wakeup interrupt
      rtc.setAlarmMinutes(GPS_WAKEUP_EVERY_MIN);
      
      debugSerial.print("set wake-up in ");
      debugSerial.print(GPS_WAKEUP_EVERY_SEC);
      debugSerial.println(" seconds");
    }
    else
    {
      rtc.setAlarmSeconds(GPS_WAKEUP_EVERY_SEC * 6);
      rtc.setAlarmMinutes(GPS_WAKEUP_EVERY_MIN);
      
      debugSerial.print("set wake-up in ");
      debugSerial.print(GPS_WAKEUP_EVERY_SEC * 6);
      debugSerial.println(" seconds");
    }
  }
  else
  {
    rtc.setAlarmSeconds(params.getFixIntervalSeconds());  // schedule the wakeup interrupt
    rtc.setAlarmMinutes(params.getFixIntervalMinutes());
    
    debugSerial.print("set wake-up in ");
    debugSerial.print(params.getFixIntervalMinutes());
    debugSerial.println(" minutes");
  }
  rtc.enableAlarm(RTCZero::MATCH_MMSS);  // MATCH_SS
  rtc.attachInterrupt(onSleepDone);      // attach handler so that we can set the battery flag when the time has passed.
  rtc.setEpoch(0);                       // this sets it to 2000-01-01
}

void setup()
{   
  initPower();

  sodaq_gps.init();
  //sodaq_gps.setDiag(debugSerial);
  params.read();  // Get previously saved configs if any otherwise use default
  
  _ledsEnabled = params.getIsLedEnabled();
  
  setPower(HIGH);  // Turn on board
  
  if(_ledsEnabled)  // BLUE led indicates the device is bootingdevice
    setLedColor(BLUE);
    
  handleSerialPort();
  
  debugSerial.println("start track and trace demo v2.0");
  delay(500);
  rtc.begin();
  connect();

  Wire.begin();

  if(_ledsEnabled)  // Indicate end of init
    setLedColor(NONE);

  sendBatteryState();
  
  if(params.getUseAccelero())
  {
    compass.init(LSM303::device_D);
    compass.enableDefault();
    calibrate(x, y);
    prepareInterrupts(x, y);
    // The default state of the accelero meter is shut-down mode
  }
  else
  {
    compass.writeReg(LSM303::CTRL1, 0);  // Disable accelerometer, power-down mode
    compass.writeReg(LSM303::CTRL5, 0);  // Zero CTRL5 (including turn off TEMP sensor)
    compass.writeReg(LSM303::CTRL7, 0b00000010);  // Disable magnetometer, power-down mode
  } 

  modem.sleep();
 
  // Start initial GPS fix after bootup
  _wasMoving = true;
  startGPSFix();
  
  setWakeUpClock();
}

void loop()
{
  if(params.getUseAccelero())
  {
    // We got woken up by the timer so check if still moving
    if (wakeFromTimer == true)
    {
      debugSerial.println("wake up from timer");

      if(_ledsEnabled)
        setLedColor(BLUE);

      wakeFromTimer = false;
      if(_wasMoving)
      {
        if(_gpsScanning)
        {
          if (sendGPSFix())
          {
            // calibrate(x, y);           
            // setThresholds(x, y);
            enableAcceleroInterupts();
            _wasMoving = false;
          }
        }
        else
          startGPSFix();
      }
      setWakeUpClock();  
    }  
    else if(hasMoved())
    {
      debugSerial.println("Movement detected");

      if(_ledsEnabled)
        setLedColor(YELLOW);

      delay(1000);

      if(_ledsEnabled)
        setLedColor(NONE);

      disableAcceleroInterupts();
      
      // Mark that there was movement. When timer elapses, data will be transmitted
      if(_wasMoving == false)
        _wasMoving = true;
    }
  }  
  else if(wakeFromTimer == true)
  {
    debugSerial.println("wake up from timer");
    
    if(_ledsEnabled)
      setLedColor(BLUE);

    wakeFromTimer = false;

    if(_gpsScanning)
      sendGPSFix();
    else
      startGPSFix();

    setWakeUpClock();
  }
  if(_ledsEnabled)
    setLedColor(NONE);

  if(!params.getIsDebugMode())
  {
    debugSerial.println("Entering sleep mode");
    sleep();
  }
}

/****
 * Start up the gps unit and set a flag so the main loop knows we need to scan the gps
 */
void startGPSFix()
{
  // only try to start it if the gps is not yet running
  // otherwise we might reset it by accident and never get a fix
  if(_gpsScanning == false)
  {
    gpsRetryCount = 1;
    uint32_t timeout = params.getGpsFixTimeout() * (_foundGPSFix ? 5000 : 30000);
    debugSerial.println(String("spinning up gps ..., timeout in ") + (timeout / 1000) + String("s"));
    sodaq_gps.startScan();
    _gpsScanning = true;
  }
}

/****
 * Stop the async scanning of the gps module
 */
void stopGPSFix()
{
  debugSerial.println("shutting down gps");
  sodaq_gps.endScan();
  _gpsScanning = false;
}

/****
 * Get the current location from the gps and send it to the cloud
 * @param withSendDelay when true, the function will pause before sending the message
 * @return true if the operation was terminated, otherwise false
 */
bool sendGPSFix()
{
  debugSerial.println("scanning gps");
  if (sodaq_gps.tryScan(10))
  {
    debugSerial.println(" found GPS fix! ");
    modem.wakeUp();

    if(_ledsEnabled)
      setLedColor(GREEN);

    stopGPSFix();

    debugSerial.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
    debugSerial.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
    debugSerial.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
    debugSerial.println(String(" alt = ") + String(sodaq_gps.getAlt(), 7));

    debugSerial.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));
       
    // calculate distance between previous and new coördinates
    if (_foundGPSFix)
    {
      float lat2 = sodaq_gps.getLat();
      float long2 = sodaq_gps.getLon();
      float lat1 = latPrevious;
      float long1 = longPrevious;
      debugSerial.println(String(" distance between 2 points = ") + String(sodaq_gps.distance_between(lat1, long1, lat2,long2)));
    }

    latPrevious = sodaq_gps.getLat();
    longPrevious = sodaq_gps.getLon();

    debugSerial.println(String(" spd = ") + String(sodaq_gps.getSpeed(), 7));

    sendValues();

    modem.sleep();
    if(_ledsEnabled)
      setLedColor(NONE);

    _foundGPSFix = true;
    return true;
  }
  else if(gpsRetryCount >= params.getGpsFixTimeout())
  {
    stopGPSFix();
    debugSerial.println("GPS module stopped: failed to find fix. new GPS fix will be attempted upon next wake-up");
    return true;
  }
  else
  {
    debugSerial.println("No GPS Fix yet");
    gpsRetryCount = gpsRetryCount +1;
    return false;
  }
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

void sendValues()
{
  #ifdef CONTAINERS
  container.addToQueue(float(sodaq_gps.getLat()), float(sodaq_gps.getLon()), float(sodaq_gps.getAlt()), 0., GPS, false); process();
  container.addToQueue((float)sodaq_gps.getSpeed(), NUMBER_SENSOR, false); process();
  #endif
  
  #ifdef CBOR
  payload.reset();
  payload.map(2);
  payload.addGps(float(sodaq_gps.getLat()), float(sodaq_gps.getLon()), float(sodaq_gps.getAlt()), "9");
  payload.addNumber((float)sodaq_gps.getSpeed(), "16");
  process();
  #endif
  
  #ifdef BINARY
  #endif
}

bool sendBatteryState()
{
  modem.wakeUp();
  
  if(_ledsEnabled)
    setLedColor(GREEN);

  debugSerial.println("reading battery level");

  int16_t batdata = batteryStatus();

  #ifdef CONTAINERS
  container.addToQueue(batdata, BATTERY_LEVEL, false);
  process();
  #endif
  
  #ifdef CBOR
  payload.reset();
  payload.map(1);
  payload.addInteger(batdata, "14");
  payload.addToQueue(false);
  #endif
  
  #ifdef BINARY
  #endif
  
  modem.sleep();
  if(_ledsEnabled)
    setLedColor(NONE);

  return true;
}

/****
 *
 */
 
void connect()
{
  uint8_t devAddr[5];
  uint8_t appSKey[17];
  uint8_t nwkSKey[17];

  debugSerial.println(params.getUseAccelero());
  bool allParametersValid = convertAndCheckHexArray((uint8_t*)devAddr, params.getDevAddrOrEUI(), sizeof(devAddr) -1)
    && convertAndCheckHexArray((uint8_t*)appSKey, params.getAppSKeyOrEUI(), sizeof(appSKey) -1)
    && convertAndCheckHexArray((uint8_t*)nwkSKey, params.getNwSKeyOrAppKey(), sizeof(nwkSKey)-1);
    
  if(!allParametersValid)
  {
    debugSerial.println("Invalid parameters for the lora connection. Can't start lora modem");
    return;
  }

  // Set baud rate of the serial connection to match the modem
  // Note: it is more power efficient to leave loraSerial running
  loraSerial.begin(modem.getDefaultBaudRate());

  modem.sleep();  // Make certain that the modem is synced and awake
  delay(50);
  modem.wakeUp();

  while(!device.initABP(devAddr, appSKey, nwkSKey));
  debugSerial.print("Modem version: ");
  debugSerial.println(modem.getSysParam("ver"));
  debugSerial.println("Ready to send data");
}

/****
 * Convert the given hex array
 * "hex" is assumed to be 2*resultSize bytes
 * @return true if it is valid hex and non-zero
 */
bool convertAndCheckHexArray(uint8_t* result, const char* hex, size_t resultSize)
{
  bool foundNonZero = false;

  uint16_t inputIndex = 0;
  uint16_t outputIndex = 0;

  // Stop at the first string termination char or at the end of the output buffer
  while(outputIndex < resultSize && hex[inputIndex] != 0 && hex[inputIndex + 1] != 0)
  {
    if(!isxdigit(hex[inputIndex]) || !isxdigit(hex[inputIndex + 1]))
      return false;

    result[outputIndex] = HEX_PAIR_TO_BYTE(hex[inputIndex], hex[inputIndex + 1]);

    if (result[outputIndex] > 0)
      foundNonZero = true;

    inputIndex += 2;
    outputIndex++;
  }

  result[outputIndex] = 0;  // Terminate the string

  return foundNonZero;
}

/****
 * Show and handle the boot menu if there is a serial port
 */
void handleSerialPort()
{
  debugSerial.begin(57600);
  
  // Main configs have been supplied, use short boot cycle
  if (params.keysAndAddrSpecified())
  {
    unsigned long start = millis();
    pinMode(BUTTON, INPUT_PULLUP);
    int sensorVal = digitalRead(BUTTON);

    while(sensorVal == HIGH && start + 5000 > millis())

    sensorVal = digitalRead(BUTTON);
    if(sensorVal == LOW)
    {
      while(!debugSerial){}  // Wait until the serial port is connected or time out if there is none
      do
      {
        showBootMenu(debugSerial);
      }
      while(!params.checkConfig(debugSerial) || !params.keysAndAddrSpecified());

      params.showConfig(&debugSerial);
    }
  }
  else
  {
    while(!debugSerial){}  // Wait until the serial port is connected or time out if there is none

    do
    {
      showBootMenu(debugSerial);
    }
    while (!params.checkConfig(debugSerial) || !params.keysAndAddrSpecified());

    params.showConfig(&debugSerial);
  }
}

/****
 * Calibrate the accelerometer
 */
void calibrate(int16_t &x, int16_t &y)
{
  x = 0;
  y = 0;
  debugSerial.println("begin calibration");
  for (int i = 1; i <= ACCELERO_NR_CALIBRATION_STEPS; i++)
  {
    compass.read();

    debugSerial.print("x: ");
    debugSerial.print(compass.a.x * ACCELERO_COMPENSATION);
    debugSerial.print(", y: ");
    debugSerial.print(compass.a.y * ACCELERO_COMPENSATION);
    debugSerial.print(", z: ");
    debugSerial.println(compass.a.z * ACCELERO_COMPENSATION);

    if (abs(compass.a.x) > x)
    {
      x = abs(compass.a.x);
      debugSerial.print("max x: ");
      debugSerial.println(x * ACCELERO_COMPENSATION);
    }
    if (abs(compass.a.y) > y)
    {
      y = abs(compass.a.y);
      debugSerial.print("max y: ");
      debugSerial.println(y * ACCELERO_COMPENSATION);
    }
    delay(100);
  }
  debugSerial.println("end calibration");
}