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

* `push-button-containers` basic push button example using preset [container definitions](http://docs.allthingstalk.com/developers/data/default-payload-conversion/)
* `push-button-payload-builder` basic push button example using our [custom binary payload decoding](http://docs.allthingstalk.com/developers/data/custom-payload-conversion/)
* `track-and-trace-containers` GPS tracking device

### Sample sketches

Short samples of how to use the onboard sensors of the Sodaq ONE

* `accelerometer`
* `battery`
* `compass`
* `magnetometer`
* `push-button`
* `rgbled`