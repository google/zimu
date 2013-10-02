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

// IMU main (main.c)

#include "IMU.h"

extern void initAdc();
extern void initSerial();
extern void initSpi();
extern void initI2C();
void resetMag();

extern void initiateTx();

// Required variables to allow booting from flash
extern unsigned int hwi_vec_loadstart;
extern unsigned int hwi_vec_loadend;
extern unsigned int hwi_vec_runstart;
extern unsigned int trcdata_loadstart;
extern unsigned int trcdata_loadend;
extern unsigned int trcdata_runstart;
extern unsigned int ramfuncs_loadstart;
extern unsigned int ramfuncs_loadend;
extern unsigned int ramfuncs_runstart;
extern unsigned int econst_loadstart;
extern unsigned int econst_loadend;
extern unsigned int econst_runstart;

extern void DSP28x_usDelay(unsigned long Count);

IMURawData rawData;
IMUConvData convData;
IMUFilterData filterData;
IMUFilterData filterInputData;		// used for debug

// Debug mode can be set by sending 'db' plus the mode number:
// 		db0: autotransmit raw data
//		db1: autotransmit converted data
//		db2: autotransmit filtered data
char debug_mode = -1;

Uint32 led_state;
Uint32 led1_pattern;

// Pre-main function
void UserInit(void) {
  memcpy(&trcdata_runstart, &trcdata_loadstart,
         &trcdata_loadend - &trcdata_loadstart);
}

// Main function contains initialization routines then passes control
// to DSP/Bios scheduler
int main(void) {
  // Initialize the hwi vector
  EALLOW;
  memcpy(&hwi_vec_runstart, &hwi_vec_loadstart,
         &hwi_vec_loadend - &hwi_vec_loadstart);
  EDIS;

  // Copy the ram functions
  memcpy(&ramfuncs_runstart, &ramfuncs_loadstart,
         &ramfuncs_loadend - &ramfuncs_loadstart);

  // Copy the constant table
  memcpy(&econst_runstart, &econst_loadstart,
         &econst_loadend - &econst_loadstart);

  // Some initializations first
  DINT;

  EALLOW;
  // Enable the peripherals
  DevEmuRegs.PROTSTART = 0x0100;
  DevEmuRegs.PROTRANGE = 0x00FF;

  // Configure the clocks
  // High speed peripheral clock prescaler = 1
  SysCtrlRegs.HISPCP.all = 0x0000;
  // Low speed peripheral clock prescaler = 4
  SysCtrlRegs.LOSPCP.all = 0x0002;

  // Enable the ADC and send clock signals there
  SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;

  // Initialize PIE
  PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

  // Disable and clear all PIE
  PieCtrlRegs.PIEIER1.all =  0x0000;
  PieCtrlRegs.PIEIER2.all =  0x0000;
  PieCtrlRegs.PIEIER3.all =  0x0000;
  PieCtrlRegs.PIEIER4.all =  0x0000;
  PieCtrlRegs.PIEIER5.all =  0x0000;
  PieCtrlRegs.PIEIER6.all =  0x0000;
  PieCtrlRegs.PIEIER7.all =  0x0000;
  PieCtrlRegs.PIEIER8.all =  0x0000;
  PieCtrlRegs.PIEIER9.all =  0x0000;
  PieCtrlRegs.PIEIER10.all = 0x0000;
  PieCtrlRegs.PIEIER11.all = 0x0000;
  PieCtrlRegs.PIEIER12.all = 0x0000;

  PieCtrlRegs.PIEIFR1.all  = 0x0000;
  PieCtrlRegs.PIEIFR2.all  = 0x0000;
  PieCtrlRegs.PIEIFR3.all  = 0x0000;
  PieCtrlRegs.PIEIFR4.all  = 0x0000;
  PieCtrlRegs.PIEIFR5.all  = 0x0000;
  PieCtrlRegs.PIEIFR6.all  = 0x0000;
  PieCtrlRegs.PIEIFR7.all  = 0x0000;
  PieCtrlRegs.PIEIFR8.all  = 0x0000;
  PieCtrlRegs.PIEIFR9.all  = 0x0000;
  PieCtrlRegs.PIEIFR10.all = 0x0000;
  PieCtrlRegs.PIEIFR11.all = 0x0000;
  PieCtrlRegs.PIEIFR12.all = 0x0000;

  // Acknowlege all PIE interrupt groups
  PieCtrlRegs.PIEACK.all = 0xFFFF;

  // Enable PIE
  PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

  // Setup GPIOs
  GpioCtrlRegs.GPAMUX1.all = 0x0000;     // GPIO functionality GPIO0-GPIO15
  GpioCtrlRegs.GPAMUX2.all = 0x0000;     // GPIO functionality GPIO16-GPIO31
  GpioCtrlRegs.GPCMUX1.all = 0x0000;     // GPIO functionality GPIO64-GPIO79
  GpioCtrlRegs.GPCMUX2.all = 0x0000;     // GPIO functionality GPIO80-GPIO95

  GpioCtrlRegs.GPADIR.all = 0x0000;      // GPIO0-GPIO31 are inputs
  GpioCtrlRegs.GPBDIR.all = 0x0000;      // GPIO32-GPIO63 are inputs
  GpioCtrlRegs.GPCDIR.all = 0x0000;      // GPIO65-GPIO95 are inputs

  // Setup LED outputs
  GpioCtrlRegs.GPCDIR.bit.nLED0 = 1;
  GpioCtrlRegs.GPCDIR.bit.nLED1 = 1;
  GpioDataRegs.GPCSET.bit.nLED0 = 1;
  GpioDataRegs.GPCCLEAR.bit.nLED1 = 1;
  led_state = 1;			 // initialized state variable

  // Setup mag reset pin
  GpioCtrlRegs.GPAPUD.bit.MAG_RESET = 1;	// disable pullup
  GpioCtrlRegs.GPADIR.bit.MAG_RESET = 1; 	// set as output

  EDIS;

  // Initialize magnetometers
  resetMag();
  // Initialize ADC
  initAdc();
  // Initialize Serial
  initSerial();
  // Initialize SPI for 3-axis accel.
  initSpi();
  // Initialize I2C for serial number and copy into the data structure
  initI2C();

  EINT;
  ERTM;

  // Return control to DSP/BIOS
}

// Resets the magetometers using the built-in set/reset straps.
void resetMag() {
  GpioDataRegs.GPASET.bit.MAG_RESET = 1;
  DELAY_US(10000);
  GpioDataRegs.GPACLEAR.bit.MAG_RESET = 1;
  DELAY_US(1000);
  GpioDataRegs.GPASET.bit.MAG_RESET = 1;
}

#pragma CODE_SECTION(SensorUpdate,"ramfuncs");
void SensorUpdate(void) {
  static Uint16 sample;

  // Initiate adc conversion
  AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;

  // Autotransmit during debug mode
  // Send raw and converted data at the normal ouput rate.
  // Send filter debug data at the full internal rate.
  if (debug_mode == -1);	// normal mode - do nothing
  else if (debug_mode == 0 || debug_mode == 1) {
    if (sample >= OVERSAMPLE_RATE-1) {
      sample = 0;
      initiateTx(debug_mode);
    } else {
      sample++;
    }
  } else if (debug_mode == 2) {
    initiateTx(2);
  } else {
    debug_mode = -1;	// reset to normal mode
  }
}

// The led blink pattern is defined over 32 states where each bit in the
// pattern variable determines whether the led is on of off during the current
// state.  (1 = on, 0 = off).  The state variable is a one-hot encoded Uint32.
#pragma CODE_SECTION(LedBlink,"ramfuncs");
void LedBlink(void) {

  if (led_state & LED1_PATTERN)
    GpioDataRegs.GPCCLEAR.bit.nLED1 = 1;
  else
    GpioDataRegs.GPCSET.bit.nLED1 = 1;

  // increment state
  if (led_state == 0x80000000)
    led_state = 1;
  else
    led_state = led_state << 1;

}


