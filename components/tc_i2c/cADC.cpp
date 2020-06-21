// cADC library
// Version date: August 20, 2010
// interface with MCP3424 18-bit ADC

// *** BSD License ***
// ------------------------------------------------------------------------------------------
// Copyright (c) 2010, MLG Properties, LLC
// All rights reserved.
//
// Contributor:  Jim Gallt
//
// Redistribution and use in source and binary forms, with or without modification, are
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list
//   of conditions and the following disclaimer in the documentation and/or other materials
//   provided with the distribution.
//
//   Neither the name of the MLG Properties, LLC nor the names of its contributors may be
//   used to endorse or promote products derived from this software without specific prior
//   written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

// Acknowledgement is given to Bill Welch for his development of the prototype hardware and software
// upon which much of this library is based.

// 20110609  Significant revision for flexibility in selecting modes of operation
// 20120126  Arduino 1.0 compatibility
//  (thanks and acknowledgement to Arnaud Kodeck for his code contributions).

#include "cADC.h"
#include <stdint.h>
#include <math.h>
#include <binary.h>
#include <driver/i2c.h>

// --------------------------------------------------- dFilterRC
filterRC::filterRC()
{
  level = 0;
  y = 0;
  first = true;
};

// ----------------------------------------------------
void filterRC::init(int32_t percent)
{
  level = percent;
  first = true;
};

// ------------------------------------
int32_t filterRC::doFilter(int32_t xi)
{
  if (first)
  {
    y = xi;
    first = false;
    return y;
  }
  float yy = (float)(100 - level) * (float)xi * 0.01;
  float yyy = (float)level * (float)y * 0.01;
  yy += yyy;
  return y = round(yy);
};

// ----------------------------------------------------------
cADC::cADC(uint8_t addr)
{
  a_adc = addr; // address of ADC chip
  cal_gain = CAL_GAIN;
  cal_offset = CAL_OFFSET;
  setCfg(ADC_BITS_18, ADC_GAIN_8, ADC_CONV_1SHOT); // 18 bit, 8X, 1 shot
}

// --------------------------------------------------setCfg
void cADC::setCfg(uint8_t res, uint8_t gain, uint8_t conv)
{
  cfg = res | gain | conv;
  // set uV value of the LSB for conversions to uV
  //printf("cfg set to : %#x\n", cfg);
  switch (res)
  {
  case ADC_BITS_12:
    convTime = _CONV_TIME_12;
    nLSB = 0; // bit shift count = ADC_BITS - 12
    break;
  case ADC_BITS_14:
    convTime = _CONV_TIME_14;
    nLSB = 2; // bit shift count = ADC_BITS - 12
    break;
  case ADC_BITS_16:
    convTime = _CONV_TIME_16;
    nLSB = 4; // bit shift count = ADC_BITS - 12
    break;
  case ADC_BITS_18:
    convTime = _CONV_TIME_18;
    nLSB = 6; // bit shift count = ADC_BITS - 12
    break;
  default:
    convTime = _CONV_TIME_18;
    nLSB = 6; // bit shift count = ADC_BITS - 12
  }
}

// -------------------------------------------------- getConvTime
uint16_t cADC::getConvTime()
{
  return convTime;
}

// --------------------------------------------------setCal
void cADC::setCal(float gain, int8_t offs)
{
  cal_gain = gain - 1.0; // to reduce loss of significance
  cal_offset = offs;
};

// -----------------------------------------------------------
int32_t cADC::readuV()
{
  int32_t v;
  uint8_t a, b, c, stat;
  esp_err_t espRc;
  // resolution determines number of bytes requested
  if ((cfg & ADC_RES_MASK) == ADC_BITS_18)
  { // 3 data bytes
    //Wire.requestFrom(a_adc, (uint8_t)4, true);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, a_adc << 1 | I2C_MASTER_READ, I2C_MASTER_NACK);
    i2c_master_read_byte(cmd, &a, I2C_MASTER_ACK);     //read 4 bytes
    i2c_master_read_byte(cmd, &b, I2C_MASTER_ACK);     //read 4 bytes
    i2c_master_read_byte(cmd, &c, I2C_MASTER_ACK);     //read 4 bytes
    i2c_master_read_byte(cmd, &stat, I2C_MASTER_NACK); //read 4 bytes
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    //printf("I2C Read Returned : %#x\n", espRc);
    i2c_cmd_link_delete(cmd);

    //printf("data: %d %d %d %d \n", a, b, c, stat);

    /*uint8_t a = Wire._READ(); // first data byte
    uint8_t b = Wire._READ(); // second data byte
    uint8_t c = Wire._READ(); // 3rd data byte*/

    v = a;
    v <<= 24; // v = a : 0 : 0 : 0
    v >>= 16; // v = s : s : a : 0
    v |= b;   //   v = s : s : a : b
    v <<= 8;  //  v = s : a : b : 0
    v |= c;   //   v = s : a : b : c
  }
  else
  { // 2 data bytes
    /*Wire.requestFrom(a_adc, (uint8_t)3, true);
    uint8_t a = Wire._READ(); // first data byte
    uint8_t b = Wire._READ(); // second data byte*/

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, a_adc << 1 | I2C_MASTER_READ, I2C_MASTER_NACK);
    i2c_master_read_byte(cmd, &a, I2C_MASTER_ACK);     //read 3 bytes
    i2c_master_read_byte(cmd, &b, I2C_MASTER_ACK);     //read 3 bytes
    i2c_master_read_byte(cmd, &stat, I2C_MASTER_NACK); //read 3 bytes
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    v = a;
    v <<= 24; // v = a : 0 : 0 : 0
    v >>= 16; // v = s : s : a : 0
    v |= b;   //   v = s : s : a : b
  }
  //uint8_t stat = Wire._READ(); // read the status byte returned from the ADC
  v *= 1000; // convert to uV.  This cannot overflow ( 10 bits + 18 bits < 31 bits )
  // bit shift count for ADC gain
  uint8_t gn = stat & ADC_GAIN_MASK;
  // shift based on ADC resolution plus ADC gain
  /*Serial.print(v);
  Serial.print(" ");
  Serial.println(stat,BIN);*/
  if ((stat & ADC_CONV_1SHOT) == 0x80)
  {
    printf("rejected");
    return 0xFFFFFFFF;
  }
  v >>= (nLSB + gn); // v = raw reading, uV

  // calculate effect of external calibration gain; minimize loss of significance
  // int32_t deltaV = round((float)v * cal_gain);

  //return v + deltaV;  // returns corrected, unfiltered value of uV
  return v; // returns corrected, unfiltered value of uV
};

// -------------------------------------
void cADC::nextConversion(uint8_t chan)
{
  esp_err_t espRc;
  uint8_t tx = cfg | ((chan & B00000011) << ADC_C0);
 
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (a_adc << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
  i2c_master_write_byte(cmd, tx, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS); //could check success
  i2c_cmd_link_delete(cmd);
};

// -------------------------------------
void cADC::reset()
{
  esp_err_t espRc;
  uint8_t tx = 0x6;
 
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, 0 | I2C_MASTER_WRITE, I2C_MASTER_NACK);//general call
  i2c_master_write_byte(cmd, tx, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS); //could check success
  i2c_cmd_link_delete(cmd);
};

int32_t cADC::C100_to_uV(int16_t T)
{
  float T0 = 2.5000000E+03; //*100
  float V0 = 1.0003453E+03; //*1000

  float p[4] = {4.0514854E-04, -3.8789638E-09, -2.8608478E-12, -9.5367041E-18};
  float q[2] = {-1.3948675E-05, -6.7976627E-09};

  float TT0 = (float)T - T0;
  float top = (TT0 * (p[0] + TT0 * (p[1] + TT0 * (p[2] + TT0 * p[3]))));
  float bottom = (1 + TT0 * (q[0] + q[1] * TT0));
  return V0 + top / bottom * 1000;
}

float cADC::uV_to_C(int32_t V)
{
  float V_range[2][2] = {
      {-3554, 4096},
      {4096, 16397}};

  float T0[2] = {-8.7935962E+00, 3.1018976E+02};
  float V0[2] = {-3.4489914E+02, 1.2631386E+04};
  float p[4][2] = {
      {2.5678719E-02, 2.4061949E-02},
      {-4.9887904E-07, 4.0158622E-06},
      {-4.4705222E-10, 2.6853917E-10},
      {-4.4869203E-14, -9.7188544E-15}};

  float q[3][2] = {
      {2.3893439E-07, 1.6995872E-04},
      {-2.0397750E-08, 1.1413069E-08},
      {-1.8424107E-12, -3.9275155E-13}};

  // first figure out which range of values
  uint8_t j;
  uint8_t ind = 0;
  for (j = 0; j < 2; j++)
  {
    if (((float)V >= V_range[0][j]) &&
        ((float)V <= V_range[1][j]))
      ind = j;
  };

  float VV0 = (float)V - V0[ind];

  float T = T0[ind] + (VV0 * (p[0][ind] + VV0 * (p[1][ind] + VV0 * (p[2][ind] + VV0 * p[3][ind])))) / (1 + VV0 * (q[0][ind] + VV0 * (q[1][ind] + VV0 * q[2][ind])));
  return T;
}


// ----------------------------------------------------------- ambSensor
ambSensor::ambSensor(uint8_t addr)
{
  a_amb = addr; // I2C address
  filter.init(0);
  setCfg(AMB_BITS_12); // default is 12 bits
}

// -----------------------------------------
// sets up the configuration byte
void ambSensor::setCfg(uint8_t res)
{
  cfg = res | convMode;
  switch (res)
  {
  case AMB_BITS_9:
    convTime = _AMB_CONV_TIME_9;
    nLSB = 7; // shift count
    break;
  case AMB_BITS_10:
    convTime = _AMB_CONV_TIME_10;
    nLSB = 6; // shift count
    break;
  case AMB_BITS_11:
    convTime = _AMB_CONV_TIME_11;
    nLSB = 5; // shift count
    break;
  case AMB_BITS_12:
    convTime = _AMB_CONV_TIME_12;
    nLSB = 4; // shift count
    break;
  }
}

// -----------------------------------------
// returns minimum time required for a conversion
uint16_t ambSensor::getConvTime()
{
  return convTime;
}

// -----------------------------------------
// puts the 9800 in shutdown.  Required for one-shot mode
void ambSensor::ambShutdown()
{
  /*Wire.beginTransmission(a_amb);
  Wire._WRITE(AMB_REGSEL_CFG); // point to config reg
  Wire._WRITE((uint8_t)AMB_SHUTDOWN);
  Wire.endTransmission(true);*/

  esp_err_t espRc;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (a_amb << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
  i2c_master_write_byte(cmd, AMB_REGSEL_CFG, I2C_MASTER_NACK); //evtl ACK here
  i2c_master_write_byte(cmd, AMB_SHUTDOWN, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS); //could check success
  i2c_cmd_link_delete(cmd);

  // delay needed here?
}

// ------------------------------------------------
void ambSensor::init(int fpercent, uint8_t cmode)
{
  filter.init(fpercent);
  convMode = cmode;
  if (cmode == AMB_CONV_1SHOT)
    ambShutdown();
}

// -----------------------------------------
void ambSensor::nextConversion()
{
  /*Wire.beginTransmission(a_amb);
  Wire._WRITE((uint8_t)AMB_REGSEL_CFG); // configuration register
  Wire._WRITE(cfg);                     // request a conversion
  Wire.endTransmission(true);*/
  esp_err_t espRc;
  uint8_t tx = AMB_REGSEL_CFG;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (a_amb << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
  i2c_master_write_byte(cmd, tx, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS); //could check success
  i2c_cmd_link_delete(cmd);
}

// -----------------------------------------
int32_t ambSensor::readSensor()
{
  /*uint8_t a, b;
  Wire.beginTransmission(a_amb);
  Wire._WRITE((uint8_t)AMB_REGSEL_TMP); // point to temperature reg.
  Wire.endTransmission(true);
  Wire.requestFrom(a_amb, (uint8_t)2, true);
  a = Wire._READ();
  b = Wire._READ();*/
  uint8_t a, b;
  esp_err_t espRc;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, a_amb << 1 | I2C_MASTER_READ, I2C_MASTER_NACK);
  i2c_master_read_byte(cmd, &a, I2C_MASTER_ACK);  //read 1. byte
  i2c_master_read_byte(cmd, &b, I2C_MASTER_NACK); //read 2. byte
  i2c_master_stop(cmd);
  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  raw = a;    //  0 : 0 : 0 : a
  raw <<= 24; // a : 0 : 0 : 0
  raw >>= 16; //  s : s : a : 0
  raw |= b;   //  s : s : a : b
  // 12-bit, nLSB = 4
  // 11-bit, nLSB = 5
  // 10-bit, nLSB = 6
  //  9-bit, nLSB = 7
  raw >>= nLSB;                                  // first nLSB bits in b are undefined
  raw <<= nLSB;                                  // they are gone now, replaced by zero
  raw >>= 4;                                     // move bits to right to form raw code
  filtered = filter.doFilter(raw << AMB_FACTOR); // create more resolution for filter
  ambC = (float)filtered;
  ambC += temp_offset * AMB_LSB_INV; // calibration correction
  ambC *= AMB_LSB;                   // Ta = code / 16 per MCP9800 datasheet
  ambF = 1.8 * ambC + 32.0;
  return filtered;
};

// -----------------------------------
float ambSensor::getAmbC() { return ambC; }
float ambSensor::getAmbF() { return ambF; }

// ------------------------------------
float ambSensor::getOffset()
{
  return temp_offset;
};

// ---------------------------------------
void ambSensor::setOffset(float tempC)
{
  temp_offset = tempC;
};
