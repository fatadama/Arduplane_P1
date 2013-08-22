/*
  AP_PixArt.cpp - - Vision library for Arduino using the Wii PixArt IR camera
  - Version 0.3
  
  Original work by Kako:  http://www.kako.com/neta/2007-001/2007-001.html
                          http://www.kako.com/neta/2008-009/2008-009.html
  Derived from PVision library by Stephen Hobley:
    http://www.stephenhobley.com/blog/2009/03/01/pixartwiimote-sensor-library-for-arduino/
  PixArt library (0.1) by Andrew Beckett
  v0.2 includes sensitivity setting delay optimizations by Tim Woodbury.
  v0.3 ports the library from the Arduino Wire class to Wayne Truchsess's I2C
    library (http://dsscircuits.com/articles/arduino-i2c-master-library.html).

  This library is for use with the Wii PixArt sensor connected to the Arduino
  I2C/TWI port. The PixArt sensor is discussed in detail on the Wiibrew wiki:
  http://wiibrew.org/wiki/Wiimote#IR_Camera. This library should only be used
  when the camera has been removed from the Wiimote and provided with support
  circuitry. See links above for more information.
  
  This library differs from the PVision library in that the sensor is set for
  Basic mode rather than Extended mode and online sensitivity setting change
  is implemented.
*/

/******************************************************************************
* Includes
******************************************************************************/
#include "AP_PixArt.h"
#include <I2C.h>
#include <Arduino.h>

/******************************************************************************
* Private methods
******************************************************************************/
void AP_PixArt::unpack() {
  activeBlobs = 0; // flush active blob list
  uint8_t s; // temp data
  char cursor = 0; // current dataBuf index

  // Unpack the packet
  for(char i=0;i<4;i++) { // for each blob
    blobX[i] = dataBuf[++cursor]; // store the X LSB
    blobY[i] = dataBuf[++cursor]; // store the Y LSB
	s = dataBuf[++cursor]; // get the MSn
	blobX[i] += (s & 0x30) <<4; // store the X MSn
	blobY[i] += (s & 0xC0) <<2; // store the Y MSn
	activeBlobs |= (blobY[i] < 768)? 0x01<<i : 0; // active blob?
  }
}

uint8_t AP_PixArt::setSensitivity() {
  uint8_t error = 0; // error state

  uint8_t registerAddress[6] = { 0x30 , 0x00 , 0x07 , 0x1A , 0x33 , 0x30 }; // register addresses to modify
  uint8_t numberBytes[6] = { 1 , 7 , 2 , 2 , 1 , 1 }; // number of significant bytes in the data array

  uint8_t dataItem[6][7] = {
    { 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 },
	{ 0x02 , 0x00 , 0x00 , 0x71 , 0x01 , 0x00 , sensitivity[0] },
	{ 0x00 , sensitivity[1] , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 },
	{ sensitivity[2] , sensitivity [3] , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 },
	{ 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 },
	{ 0x08 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 }
  };

  for(char i=0;i<7;i++) {
	error |= I2c.write(IRslaveAddress,registerAddress[i],dataItem[i],numberBytes[i]);
  }

  return error;
}

/******************************************************************************
* Constructor
******************************************************************************/
AP_PixArt::AP_PixArt() {
  IRslaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI

  // set the blobs inactive.
  for(char i=0;i<4;i++) {
    blobX[i] = 1023;
    blobY[i] = 1023;
  }
}

/******************************************************************************
* Public methods
******************************************************************************/
// init the PixArt sensor with default (Kako's) sensitivity settings in Basic mode
uint8_t AP_PixArt::init() {
  uint8_t error = 0; // error state
  
  uint8_t registerAddress[7] = { 0x30 , 0x30 , 0x06 , 0x08 , 0x1A , 0x33 , 0x33 };
  uint8_t dataItem[7] = { 0x01 , 0x08 , 0x90 , 0xC0 , 0x40 , 0x33 , 0x03 };
  
  for(char i=0;i<7;i++) {
	error |= I2c.write(IRslaveAddress,registerAddress[i],dataItem[i]);
  }
  return error;
}

// init the PixArt sensor with specified sensitivity settings in Basic mode
uint8_t AP_PixArt::setSensitivity(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3) {
  sensitivity[0] = p0;
  sensitivity[1] = p1;
  sensitivity[2] = p2;
  sensitivity[3] = p3;
  return setSensitivity();
}

// init the PixArt sensor with specified sensitivity settings in Basic mode
uint8_t AP_PixArt::setSensitivity(uint8_t px[]) {
  for(char i=0;i<4;i++) {sensitivity[i] = px[i]; }
  return setSensitivity();
}

 // returns sensitivity array
uint8_t* AP_PixArt::getSensitivity() {
  static uint8_t px[4] = { sensitivity[0] , sensitivity[1] , sensitivity[2] , sensitivity[3] };
  return px;
}

// returns specified sensitivity
uint8_t AP_PixArt::getSensitivity(uint8_t px) {
  return sensitivity[px];
}

// returns blob x location
int AP_PixArt::getBlobX(uint8_t blobID) {
  return blobX[blobID];
}

// returns blob y location
int AP_PixArt::getBlobY(uint8_t blobID) {
  return blobY[blobID];
}

// returns blob coordinate array in XY order
int* AP_PixArt::getBlob(uint8_t blobID) {
  static int temp[2] = { blobX[blobID] , blobY[blobID] };
  return temp;
}

uint8_t AP_PixArt::read() {
  uint8_t readCommand = 0x36;

  // Null the buffer
  for(char i=0;i<16;i++) {
    dataBuf[i]=0;
  }

  //IR sensor read
  uint8_t nullError = I2c.write(IRslaveAddress,readCommand);
  nullError |= I2c.read(IRslaveAddress,16,dataBuf);

  // Decode the packet
  unpack();
  return activeBlobs;
}

uint8_t AP_PixArt::getActiveBlobs() {
  // returns the active blob list
  return activeBlobs;
}
