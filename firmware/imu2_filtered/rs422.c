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

// RS422 communication interface (rs422.c)

#include "IMU.h"

#define SEND_MSG_SIZE 49
#define BUF_SIZE 64
#define REQ_BYTE 0xE3
#define REQ_BYTE_RAW 0x29
#define MAG_RESET_CMD 0xB4

extern void resetMag(void);

static char send_buf[BUF_SIZE];
static Uint16 send_cnt = 0;

// Stuffs float into the char buffer
#pragma CODE_SECTION(floatToChar,"ramfuncs");
void floatToChar( char* buf, Uint16 index, float *num ) {
  char *c = (char *)num;
  buf[index++] = *c & 0x00ff;
  buf[index++] = (*c & 0xff00) >> 8;
  c++;
  buf[index++] = *c & 0x00ff;
  buf[index] = (*c & 0xff00) >> 8;
}

// Stuffs int64 into the char buffer
#pragma CODE_SECTION(uint64ToChar,"ramfuncs");
void uint64ToChar( char* buf, Uint16 index, Uint64 *num ) {
  char *c = (char *)num;
  buf[index++] = *c & 0x00ff;
  buf[index++] = (*c & 0xff00) >> 8;
  c++;
  buf[index++] = *c & 0x00ff;
  buf[index++] = (*c & 0xff00) >> 8;
  c++;
  buf[index++] = *c & 0x00ff;
  buf[index++] = (*c & 0xff00) >> 8;
  c++;
  buf[index++] = *c & 0x00ff;
  buf[index] = (*c & 0xff00) >> 8;
}

// Stuffs an unsigned  16 bit integer into the char buffer
#pragma CODE_SECTION(uintToChar,"ramfuncs");
void uintToChar( char* buf, Uint16 index, Uint16 *num ) {
  buf[index++] = *num & 0x00ff;
  buf[index] = (*num & 0xff00) >> 8;
}

// Stuffs a signed 16 bit integer into the char buffer
#pragma CODE_SECTION(intToChar,"ramfuncs");
void intToChar( char* buf, Uint16 index, int16 *num ) {
  buf[index++] = *num & 0x00ff;
  buf[index] = (*num & 0xff00) >> 8;
}

// Calculate checksum by xor'ing each byte.  length is the length of the
// message to be processed, not including the check sum.
#pragma CODE_SECTION(calcCheckSum,"ramfuncs");
char calcCheckSum(char* buf, Uint16 len) {
  Uint16 i = 0;
  char sum = 0;

  for (i = 0; i < len; i++)
    sum ^= buf[i];
  return sum;
}

// Loads the send buffer with the current converted data
#pragma CODE_SECTION(loadSendBuffer,"ramfuncs");
void loadSendBuffer() {
  send_cnt = 0;
  floatToChar(send_buf, send_cnt, &convData.accel_x);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.accel_y);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.accel_z);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.gyro_x);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.gyro_y);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.gyro_z);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.mag_x);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.mag_y);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.mag_z);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.accel3_x);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.accel3_y);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &convData.accel3_z);
  send_cnt += 4;
  send_buf[send_cnt] = calcCheckSum(send_buf, SEND_MSG_SIZE-1);
}

// Loads the send buffer with the current raw data (data is formatted to fit in
// the same number of bytes as the loadSendBuffer packet).
#pragma CODE_SECTION(loadSendBufferRaw,"ramfuncs");
void loadSendBufferRaw() {
  send_cnt = 0;
  uintToChar(send_buf, send_cnt, &rawData.accel_x);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.accel_y);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.accel_z);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.gyro_x);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.gyro_y);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.gyro_z);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.mag_x);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.mag_y);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.mag_z);
  send_cnt += 2;
  intToChar(send_buf, send_cnt, &rawData.accel3_x);
  send_cnt += 2;
  intToChar(send_buf, send_cnt, &rawData.accel3_y);
  send_cnt += 2;
  intToChar(send_buf, send_cnt, &rawData.accel3_z);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.accel_x_t);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.accel_y_t);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.accel_z_t);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.gyro_x_t);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.gyro_y_t);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.gyro_z_t);
  send_cnt += 2;
  uintToChar(send_buf, send_cnt, &rawData.ref_mon);
  send_cnt += 2;
  uint64ToChar(send_buf, send_cnt, &rawData.serial_id);
  send_cnt += 8;
  while (send_cnt < SEND_MSG_SIZE-1)
    send_buf[send_cnt++] = 0x00;  // fill the rest of the packet with zeros
  send_buf[send_cnt] = calcCheckSum(send_buf, SEND_MSG_SIZE-1);
}

// Sends raw and filtered data for debugging.
#pragma CODE_SECTION(loadSendBufferFilt,"ramfuncs");
void loadSendBufferFilt() {
  send_cnt = 0;
  floatToChar(send_buf, send_cnt, &filterInputData.accel_x);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterInputData.accel_y);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterInputData.accel_z);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterInputData.gyro_x);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterInputData.gyro_y);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterInputData.gyro_z);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterData.accel_x);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterData.accel_y);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterData.accel_z);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterData.gyro_x);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterData.gyro_y);
  send_cnt += 4;
  floatToChar(send_buf, send_cnt, &filterData.gyro_z);
  send_cnt += 4;
  send_buf[send_cnt] = calcCheckSum(send_buf, SEND_MSG_SIZE-1);
}

// Load the transmit buffer and initiate a transmission.
// Led flashs when communicating
#pragma CODE_SECTION(initiateTx,"ramfuncs");
void initiateTx(char mode) {
  GpioDataRegs.GPCCLEAR.bit.nLED0 = 1;

  // load send buffer with a packet
  switch (mode) {
  case 1:
    loadSendBuffer();
    break;
  case 0:
    loadSendBufferRaw();
    break;
  case 2:
    loadSendBufferFilt();
    break;
  default:
    loadSendBuffer();
  }

  // reset the buffer pointer
  send_cnt = 0;

  // enable the Tx interrupt
  ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
  ScibRegs.SCIFFTX.bit.TXFFIENA = 1;

  GpioDataRegs.GPCSET.bit.nLED0 = 1;
}

// ISR gets called when the receive FIFO has one or more bytes.
// Check that the byte is the correct request byte, setup the send
// buffer with a fresh packet and begin transmitting.
// If the debug mode is detected ('db0', 'db1', etc) the debug_mode
// flag is set.  The IMU exits debug mode when a valid request packet is
// received.
#pragma CODE_SECTION(ReceiveData_ISR,"ramfuncs");
void ReceiveData_ISR() {
  static char inbuf[3];	// stores the last three received bytes

  // Clear RX interrupt flags
  ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  while ( ScibRegs.SCIFFRX.bit.RXFFST != 0 ) {
    inbuf[2] = inbuf[1];
    inbuf[1] = inbuf[0];
    inbuf[0] = ScibRegs.SCIRXBUF.bit.RXDT;
    if (inbuf[0] == REQ_BYTE) {
      ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;		// clear rxfifo
      ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
      initiateTx(1);
      debug_mode = -1;
      break;
    } else if (inbuf[0] == REQ_BYTE_RAW) {
      ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;		// clear rxfifo
      ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
      initiateTx(0);
      debug_mode = -1;
      break;
    } else if (inbuf[0] == MAG_RESET_CMD) {
      resetMag();
      break;
    } else if (inbuf[2] == 'd' && inbuf[1] == 'b') {
      if (inbuf[0] == '0') {
        debug_mode = 0;
      } else if (inbuf[0] == '1') {
        debug_mode = 1;
      } else if (inbuf[0] == '2') {
        debug_mode = 2;
      }
    }
  }
}

// Load 16 bytes into the tx fifo from the send buffer when the tx fifo empty
// interrupt is called.  When the entire send buffer is sent, disable the
// interrupt.
#pragma CODE_SECTION(SendData_ISR,"ramfuncs");
void SendData_ISR() {
  // Clear TX interrupt flags
  ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // load the transmitt FIFO
  while( ScibRegs.SCIFFTX.bit.TXFFST < 16 ) {
    // disable TX interrupt when the packet is complete
    if (send_cnt >= SEND_MSG_SIZE) {
      ScibRegs.SCIFFTX.bit.TXFFIENA = 0;
      break;
    }
    ScibRegs.SCITXBUF = send_buf[send_cnt++];
  }
}

void initSerial(void) {
  // Setup the DSP to enable Serial Communication module and the required pins

  EALLOW;
  // Enable SCI by sending clock signals to it
  SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 1;

  // Setup the pins
  // Half / Full Duplex select pin - output
  GpioCtrlRegs.GPADIR.bit.SER_HnF = 1;

  // Driver / Receiver Output Enable - output
  GpioCtrlRegs.GPADIR.bit.SER_DE = 1;
  GpioCtrlRegs.GPADIR.bit.SER_nRE = 1;

  // Enable pull-ups for the RX/TX pins
  GpioCtrlRegs.GPAPUD.bit.SER_DI = 0;
  GpioCtrlRegs.GPAPUD.bit.SER_RO = 0;

  // Set to asynchronous for the receive pin
  GpioCtrlRegs.GPAQSEL2.bit.SER_RO = 3;

  // Configure for SCI function
  GpioCtrlRegs.GPAMUX2.bit.SER_DI = 3;
  GpioCtrlRegs.GPAMUX2.bit.SER_RO = 3;

  EDIS;

  // Now, start setting everything up
  // Full Duplex Mode
  GpioDataRegs.GPACLEAR.bit.SER_HnF = 1;

  // Set the output enables for the driver and receiver
  GpioDataRegs.GPASET.bit.SER_DE = 1;
  GpioDataRegs.GPACLEAR.bit.SER_nRE = 1;

  // Setup the SCI modules
  // 8-N-1 with no flow control
  ScibRegs.SCICCR.all = 0x07;

  // No error irqs, enable transmitters and buffers
  ScibRegs.SCICTL1.all = 0x03;

  // Low Speed Peripheral clock is SYSCLK/4 = 37.5 MHz
  // 921600 bps => 37500000 / ((1500000 + 1) * 8) = 4.086 = 4
  // (The real baud rate will be 937500 bps)
  ScibRegs.SCIHBAUD = 0x00;
  ScibRegs.SCILBAUD = 0x04;

  //Disable TX / RX interrupts - use fifo interrupts instead
  ScibRegs.SCICTL2.all = 0x00;

  // Enable transmit FIFO, disable interrupt, set fifo interrupt level to 0
  ScibRegs.SCIFFTX.all = 0xE040;

  // Enable receive FIFO, enable interrupt when fifo has one or more bytes
  ScibRegs.SCIFFRX.all = 0x2061;

  // No auto baud rate detection, zero transfer delay
  ScibRegs.SCIFFCT.all = 0x0000;

  // Immediate stop in emulation if breakpoint is hit
  ScibRegs.SCIPRI.all = 0x00;

  // Bring up SCI from reset
  ScibRegs.SCICTL1.bit.SWRESET = 1;

  // Enable SEQ9_INT in PIE
  PieCtrlRegs.PIEIER9.bit.INTx4 = 1;	// SCITXINTB
  PieCtrlRegs.PIEIER9.bit.INTx3 = 1;	// SCIRXINTB

  // Enable INT9 in IER
  IER |= 0x0100;
}

