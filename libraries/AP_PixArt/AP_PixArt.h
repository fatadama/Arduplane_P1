/*
  AP_PixArt.h - - Vision library for Arduino using the Wii PixArt IR camera
  - Version 0.3
  
  Original work by Kako:  http://www.kako.com/neta/2007-001/2007-001.html
                          http://www.kako.com/neta/2008-009/2008-009.html
  Derived from PVision library by Stephen Hobley:
    http://www.stephenhobley.com/blog/2009/03/01/pixartwiimote-sensor-library-for-arduino/
  PixArt library (0.1) by Andrew Beckett
  v0.2 includes sensitivity setting delay optimizations by Tim Woodbury.
  v0.3 ports the library from the Arduino Wire class to Wayne Truchsess's I2C
    library.

  This library is for use with the Wii PixArt sensor connected to the Arduino
  I2C/TWI port. The PixArt sensor is discussed in detail on the Wiibrew wiki:
  http://wiibrew.org/wiki/Wiimote#IR_Camera. This library should only be used
  when the camera has been removed from the Wiimote and provided with support
  circuitry. See links above for more information.
  
  This library differs from the PVision library in that the sensor is set for
  Basic mode rather than Extended mode and online sensitivity setting change
  is implemented.
  
  Tested with Arduino 1.0.1
*/

#ifndef AP_PixArt_h
#define AP_PixArt_h

#include <Arduino.h>
#include <I2C.h>

// I2C address of PixArt Wii sensor
#define IRsensorAddress 0xB0

class AP_PixArt {

public:
  AP_PixArt(); // default constructor

  uint8_t init();  // initializes with default (Kako's) sensitivity settings, returns error state (0 = successful)
  uint8_t setSensitivity(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3); // initializes with specified sensitivity settings, returns error state
  uint8_t setSensitivity(uint8_t px[]); // initializes with specified sensitivity array, returns error state
  uint8_t* getSensitivity(); // returns sensitivity array
  uint8_t getSensitivity(uint8_t px); // returns specified sensitivity
  int getBlobX(uint8_t blobID); // returns blob x location
  int getBlobY(uint8_t blobID); // returns blob y location
  int* getBlob(uint8_t blobID); // returns blob coordinate array in XY order

  uint8_t read();  // update the blobs and return active blob bitmask
  uint8_t getActiveBlobs(); // returns the active blob bitmask
  
private:
  int blobX[4]; // X location per blob, max val = 1023
  int blobY[4]; // Y location per blob, max val = 767
  uint8_t sensitivity[4]; // Array of sensitivity parameters
  uint8_t IRslaveAddress; // TWI-compatible sensor address
  uint8_t dataBuf[16]; // buffer to hold I2C/TWI packets
  uint8_t activeBlobs; // active blob list

  void unpack(); // unpack data_buf packets and return active blob bitmask
  uint8_t setSensitivity(); // sets the sensitivity parameters on the sensor
};

#endif
