/*
Copyright 2013 Google Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// I2C interface to serial number (i2c.c)

#include "IMU.h"

// Get the serial number from the I2C device and add it to the rawData struct
void getSerialID(void) {
  int i = 100;

  // Wait for STP bit to clear
  while (I2caRegs.I2CMDR.bit.STP == 1 || i-- ) {};

  I2caRegs.I2CSAR = 0x0050;

  // Check to see if bus is busy
  i = 100;
  while (I2caRegs.I2CSTR.bit.BB == 1 || i-- ) {};

  // Read out 9 bytes
  I2caRegs.I2CCNT = 9;

  // Start reading from address 0
  I2caRegs.I2CMDR.all = 0x6420;//0x2620;

  // Now poll to read out the serial information
  // Wait for the 8 bytes to come in
  i = 100;
  while ( I2caRegs.I2CFFRX.bit.RXFFST < 9 || i-- ) {};

  // Read out the values
  rawData.serial_id = 0x0000L;
  for ( i = 0; i < 8; i++ ) {
    rawData.serial_id = I2caRegs.I2CDRR | (rawData.serial_id << 8);
  }
}


// Initialize the serial device
void initI2C(void) {
  EALLOW;
  // Enable the I2C Bus
  SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;

  // Enable the pin pullups
  GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;
  GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;

  // Asynchronous
  GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;
  GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;

  // Configure the pins to use I2C
  GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;
  GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;
  EDIS;

  // Initialize I2C Module
  // Slave address is 0x50
  I2caRegs.I2CSAR = 0x0050;

  // 10MHz module clk - 150MHz / 15 = 10MHz
  I2caRegs.I2CPSC.all = 14;

  // Clock Divider
  I2caRegs.I2CCLKL = 10;
  I2caRegs.I2CCLKH = 5;

  // Disable all interrupts
  I2caRegs.I2CIER.all = 0x00;

  // Bring I2C out of reset
  I2caRegs.I2CMDR.all = 0x0020;

  // Enable FIFOs
  I2caRegs.I2CFFTX.all = 0x6000;
  I2caRegs.I2CFFRX.all = 0x2040;

  // Get Super Serial!
  getSerialID();
}


