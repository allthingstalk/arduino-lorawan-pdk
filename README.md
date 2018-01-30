# arduino-lorawan-pdk

This repository contains examples and experiment sketches for the AllThingsTalk LoRaWAN Product Kit.

> It is to be used in conjunction with the [arduino-lorawan-sdk](https://github.com/allthingstalk/arduino-lorawan-sdk) which contains all basic functionality.

## Hardware

This library has been developed for:

- Sodaq ONE
- Microchip LoRa modem

## Installation

Download the source code and copy the content of the zip file to your arduino libraries folder (usually found at /libraries) _or_ import the .zip file directly using the Arduino IDE.

## Example sketches

* `push-button` basic push button example using the onboard button
* `track-and-trace` GPS tracking device

### Sample sketches

Short samples of how to use the onboard sensors of the Sodaq ONE

* `accelerometer`
* `battery`
* `compass`
* `magnetometer`
* `push-button`
* `rgbled`

### Sending data

There are three ways to send your data to AllThingsTalk

* `Standardized containers`
* `Cbor payload`
* `Binary payload`

You can simply select the method you prefer by (un)commenting the methods at the start of the sketch.

```
// Select your preferred method of sending data
//#define CONTAINERS
#define CBOR
//#define BINARY
```

By default, Cbor will be used. For more information on payloads, please [check here](https://github.com/allthingstalk/arduino-lorawan-sdk/blob/master/README.md)