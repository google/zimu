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

// SPI interface to the 3-axis accelerometer (spi.c)

#include "IMU.h"

// Reads the results from the SPI receive buffer.  This function
// is called by an interrupt when the receive FIFO contains 7 bytes.
#pragma CODE_SECTION(ReadSpi_ISR,"ramfuncs");
void ReadSpi_ISR(void) {
  char i;
  static unsigned short c[6];

  // set the chip select
  GpioDataRegs.GPASET.bit.ACCEL3_CS = 1;
  DELAY_US(1);

  // read results
  c[0] = SpiaRegs.SPIRXBUF;		// empty byte
  for ( i = 0; i < 6; i++ )
    c[i] = SpiaRegs.SPIRXBUF;

  rawData.accel3_x = (c[1] << 8) + c[0];
  rawData.accel3_y = (c[3] << 8) + c[2];
  rawData.accel3_z = (c[5] << 8) + c[4];

  // clear interrupt flags
  SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

#pragma CODE_SECTION(readLowG,"ramfuncs");
void readLowG(void) {
  unsigned char i = 0;

  // Initiate read for the 3-axis SPI low g-accel.
  GpioDataRegs.GPACLEAR.bit.ACCEL3_CS = 1;		// clear chip-select
  DELAY_US(1);
  SpiaRegs.SPITXBUF = 0xE800;	// read data registers, increment address mode
  for ( i = 0; i < 6; i++ )
    SpiaRegs.SPITXBUF = 0x0000;
}

void initSpi(void) {
  char c;
  EALLOW;

  // Enable SPI clock
  SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;

  // Enable Pull ups
  GpioCtrlRegs.GPBPUD.bit.ACCEL3_SDI = 0;
  GpioCtrlRegs.GPBPUD.bit.ACCEL3_SDO = 0;
  GpioCtrlRegs.GPBPUD.bit.ACCEL3_SPC = 0;

  // Set asynchronous
  GpioCtrlRegs.GPBQSEL2.bit.ACCEL3_SDI = 3;
  GpioCtrlRegs.GPBQSEL2.bit.ACCEL3_SDO = 3;
  GpioCtrlRegs.GPBQSEL2.bit.ACCEL3_SPC = 3;

  // Enable SPI
  GpioCtrlRegs.GPBMUX2.bit.ACCEL3_SDI = 1;
  GpioCtrlRegs.GPBMUX2.bit.ACCEL3_SDO = 1;
  GpioCtrlRegs.GPBMUX2.bit.ACCEL3_SPC = 1;

  // Setup the Chip Select and RDY/INT pints
  GpioCtrlRegs.GPADIR.bit.ACCEL3_CS = 1;
  GpioCtrlRegs.GPADIR.bit.ACCEL3_RDYINT = 0;

  EDIS;

  // Set CS high
  GpioDataRegs.GPASET.bit.ACCEL3_CS = 1;

  // Reset on, falling edge, 8-bit chars
  SpiaRegs.SPICCR.all = 0x47;

  // Enable master mode, normal phase
  SpiaRegs.SPICTL.all = 0x06;

  // Enable talk, no interrupt
  // Set the baud rate to 6.25MHz (37.5MHz / (SPIBRR + 1))
  SpiaRegs.SPIBRR = 0x05;

  // Initialize SPI FIFOs
  SpiaRegs.SPIFFTX.all = 0xE040;
  SpiaRegs.SPIFFRX.all = 0x2047;      // set fifo interrupt level to 7
  SpiaRegs.SPIFFCT.all = 0x0;

  // Start SPI
  SpiaRegs.SPICCR.bit.SPISWRESET = 1;

  // Transmit even if there's software breakpoints
  SpiaRegs.SPIPRI.bit.FREE = 1;

  // Set the accelerometer configuration registers
  GpioDataRegs.GPACLEAR.bit.ACCEL3_CS = 1;
  DELAY_US(1);
  SpiaRegs.SPITXBUF = 0x2000;
  SpiaRegs.SPITXBUF = 0x8700;
  while(SpiaRegs.SPIFFRX.bit.RXFFST <= 1 ) {}
  c = SpiaRegs.SPIRXBUF;
  c = SpiaRegs.SPIRXBUF;
  GpioDataRegs.GPASET.bit.ACCEL3_CS = 1;
  DELAY_US(1);

  GpioDataRegs.GPACLEAR.bit.ACCEL3_CS = 1;
  DELAY_US(1);
  SpiaRegs.SPITXBUF = 0x2100;
  SpiaRegs.SPITXBUF = 0x8000;
  while(SpiaRegs.SPIFFRX.bit.RXFFST <= 1 ) {}
  c = SpiaRegs.SPIRXBUF;
  c = SpiaRegs.SPIRXBUF;
  GpioDataRegs.GPASET.bit.ACCEL3_CS = 1;
  DELAY_US(1);

  // Enable fifo receive interrupts
  SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
  SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;
  SpiaRegs.SPICTL.bit.SPIINTENA = 1;

  // Enable SEQ6_INT in PIE
  PieCtrlRegs.PIEIER6.bit.INTx1 = 1;

  // Enable INT6 in IER
  IER |= 0x0020;

  c = c;  // get rid of "variable never used warning"

}


