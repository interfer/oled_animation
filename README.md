# oled_animation

## Installation pre-requisites

 For communicating with the MPU6050

You need to install this version from GitHub
https://github.com/witnessmenow/Arduino-MPU6050

The library for controlling the LED Matrix
Can be installed from the library manager: https://github.com/2dom/PxMatrix

Adafruit GFX library is a dependancy for the PxMatrix Library
Can be installed from the library manager: https://github.com/adafruit/Adafruit-GFX-Library

// This was the original header from the Adafruit Learn guide this project 
//  was based on
//  https://learn.adafruit.com/matrix-led-sand/overview
// --------------------------------------------------------------------------
//  Animated 'sand' for Adafruit Feather.  Uses the following parts:
//   - Feather 32u4 Basic Proto (adafruit.com/product/2771)
//   - Charlieplex FeatherWing (adafruit.com/product/2965 - any color!)
//   - LIS3DH accelerometer (2809)
//   - 350 mAh LiPoly battery (2750)
//   - SPDT Slide Switch (805)
//
// This is NOT good "learn from" code for the IS31FL3731; it is "squeeze
// every last byte from the microcontroller" code.  If you're starting out,
// download the Adafruit_IS31FL3731 and Adafruit_GFX libraries, which
// provide functions for drawing pixels, lines, etc.
//--------------------------------------------------------------------------