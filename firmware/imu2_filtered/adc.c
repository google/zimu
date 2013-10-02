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

// ADC interface (adc.c)
// The ADC is initialted by the DSP/Bios task
#include "IMU.h"
#include "EU_info.h"

#define ADC_usDELAY 5000L

extern void ADC_cal(void);
extern void initFilter(void);
extern Uint16 updateFilter(void);
extern void readLowG(void);

#pragma CODE_SECTION(EUConv,"ramfuncs");
float EUConv(Uint16 data, float gain, float offset) {
  return data * gain + offset;
}

#pragma CODE_SECTION(EUConvSigned,"ramfuncs");
float EUConvSigned(int16 data, float gain, float offset) {
  return data * gain + offset;
}

// Calibration and temperature compensation function for EU converted data.
#pragma CODE_SECTION(convCal,"ramfuncs");
float convCal(float eu_data, float cal_gain, float cal_bias, float temp0,
              Uint16 temp, float temp_sen) {
  return eu_data * cal_gain - (cal_bias + temp_sen * ((float)temp - temp0));
}

// Convert the data to engineering units and apply calibration.
#pragma CODE_SECTION(convertData,"ramfuncs");
void convertData(void) {
  convData.accel_x = convCal(filterData.accel_x, AX_CGF, AX_CBO, AX_CTN,
                             rawData.accel_x_t, AX_CTS);
  convData.accel_y = convCal(filterData.accel_y, AY_CGF, AY_CBO, AY_CTN,
                             rawData.accel_y_t, AY_CTS);
  convData.accel_z = convCal(filterData.accel_z, AZ_CGF, AZ_CBO, AZ_CTN,
                             rawData.accel_z_t, AZ_CTS);
  convData.gyro_x = convCal(filterData.gyro_x, GX_CGF, GX_CBO, GX_CTN,
                            rawData.gyro_x_t, GX_CTS);
  convData.gyro_y = convCal(filterData.gyro_y, GY_CGF, GY_CBO, GY_CTN,
                            rawData.gyro_y_t, GY_CTS);
  convData.gyro_z = convCal(filterData.gyro_z, GZ_CGF, GZ_CBO, GZ_CTN,
                            rawData.gyro_z_t, GZ_CTS);
  convData.mag_x = convCal(EUConv(rawData.mag_x, MX_EUG, MX_EUO), MX_CGF,
                            MX_CBO, MX_CTN, rawData.gyro_y_t, MX_CTS);
  convData.mag_y = convCal(EUConv(rawData.mag_y, MY_EUG, MY_EUO), MY_CGF,
                            MY_CBO, MY_CTN, rawData.gyro_x_t, MY_CTS);
  convData.mag_z = convCal(EUConv(rawData.mag_z, MZ_EUG, MZ_EUO), MZ_CGF,
                            MZ_CBO, MZ_CTN, rawData.gyro_x_t, MZ_CTS);
  convData.accel3_x = convCal(EUConvSigned(rawData.accel3_x, A3X_EUG, A3X_EUO),
                            A3X_CGF, A3X_CBO, A3X_CTN, 0, A3X_CTS);
  convData.accel3_y = convCal(EUConvSigned(rawData.accel3_y, A3Y_EUG, A3Y_EUO),
                            A3Y_CGF, A3Y_CBO, A3Y_CTN, 0, A3Y_CTS);
  convData.accel3_z = convCal(EUConvSigned(rawData.accel3_z, A3Z_EUG, A3Z_EUO),
                            A3Z_CGF, A3Z_CBO, A3Z_CTN, 0, A3Z_CTS);
  convData.ref_mon = EUConv(rawData.ref_mon, RM_EUG, RM_EUO);
}

#pragma CODE_SECTION(ADC_INT_ISR,"ramfuncs");
void ADC_INT_ISR(void) {

  // Reset interrupt flags
  AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
  AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Read ADC results
  rawData.gyro_z_t = AdcRegs.ADCRESULT0 >> 4;
  rawData.ref_mon = AdcRegs.ADCRESULT1 >> 4;
  rawData.mag_y = AdcRegs.ADCRESULT2 >> 4;
  rawData.accel_x_t = AdcRegs.ADCRESULT3 >> 4;
  rawData.gyro_x_t = AdcRegs.ADCRESULT4 >> 4;
  rawData.accel_x = AdcRegs.ADCRESULT5 >> 4;
  rawData.gyro_x = AdcRegs.ADCRESULT6 >> 4;
  rawData.mag_z = AdcRegs.ADCRESULT7 >> 4;
  rawData.gyro_z = AdcRegs.ADCRESULT8 >> 4;
  rawData.mag_x = AdcRegs.ADCRESULT9 >> 4;
  rawData.gyro_y = AdcRegs.ADCRESULT10 >> 4;
  rawData.gyro_y_t = AdcRegs.ADCRESULT11 >> 4;
  rawData.accel_z = AdcRegs.ADCRESULT12 >> 4;
  rawData.accel_z_t = AdcRegs.ADCRESULT13 >> 4;
  rawData.accel_y = AdcRegs.ADCRESULT14 >> 4;
  rawData.accel_y_t = AdcRegs.ADCRESULT15 >> 4;

  // Pass data to the filter.  If a decimation cycle is up, convert the data
  // and start a low g conversion.
  if (updateFilter()) {
    convertData();
    readLowG();
  }
}

void initAdc(void) {
  // Load the calibration value
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
  EDIS;

  DELAY_US( ADC_usDELAY );

  // Reset the ADC Module
  AdcRegs.ADCTRL1.all = 0x4000;
  asm(" NOP");
  asm(" NOP");
  DELAY_US( ADC_usDELAY );

  // Power up bandgap / reference circuitry
  AdcRegs.ADCTRL3.bit.ADCBGRFDN = 0x03;
  DELAY_US( ADC_usDELAY );

  // Power up rest of ADC
  AdcRegs.ADCTRL3.bit.ADCPWDN = 1;
  DELAY_US( ADC_usDELAY );

  // Setup ADCTRL 3 Core Clock Divider to divide by 8
  AdcRegs.ADCTRL3.bit.ADCCLKPS = 8;

  // Setup ADCTRL1 Register
  //  Acquisition window size of 16
  //  Core clock prescaler set (/2)
  //  Cascaded mode
  AdcRegs.ADCTRL1.all = 0x0F90;

  // Setup ADCTRL2 Register
  //  Reset Sequencer
  //  Interrupt enabled (INT SEQ1)
  AdcRegs.ADCTRL2.all = 0x4800;

  // Specify 16 conversions
  AdcRegs.ADCMAXCONV.all = 0x000F;

  // Configure channel selection;
  AdcRegs.ADCCHSELSEQ1.all = 0x3210;
  AdcRegs.ADCCHSELSEQ2.all = 0x7654;
  AdcRegs.ADCCHSELSEQ3.all = 0xBA98;
  AdcRegs.ADCCHSELSEQ4.all = 0xFEDC;

  // Run the calibration routines
  EALLOW;
  ADC_cal();
  EDIS;

  // Setup the ADC reference select register
  AdcRegs.ADCREFSEL.bit.REF_SEL = 0;

  // 5ms delay before any conversion is done
  DELAY_US( ADC_usDELAY );

  // Enable SEQ1_INT in PIE
  PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

  // Enable INT1 in IER
  IER |= 0x0001;

  initFilter();
}

/*
// Used for triggering the ADC without the dspbios
// [not yet implemented]
void initPWM(void)
{
	// Set TBCLK to SYSCLK / 40 = 3.75 MHz
	EPwm1Regs.TBCTL.bit.CLKDIV = 0x2; 		// Div by 4
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x5;	// Div by 10

	// Set counter mode to up-count
	EPwm1Regs.TBCTL.bit.CTRMODE = 0;

	// Set pwm period
	// 250 Hz => 15000
	// 2500 Hz => 1500 (over sample by 10)
	// 3750 Hz => 1000 (over sample by 15)
	EPwm1Regs.TBPRD = 15000;

}
*/

