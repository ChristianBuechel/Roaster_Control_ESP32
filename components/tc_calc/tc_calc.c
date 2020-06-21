
#include "tc_calc.h"

// constants for C100_to_uV http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/thermocouple/calibration-table#computing-cold-junction-voltages
// cave, these are adjusted because DS1820 gets us °C * 100
// valid for -20 to 70°C 
static float C_T0 = 2.5000000E+03; //*100
static float C_V0 = 1.0003453E+03; //*1000
static float C_p[4] = {4.0514854E-04, -3.8789638E-09, -2.8608478E-12, -9.5367041E-18};
static float C_q[2] = {-1.3948675E-05, -6.7976627E-09};

// constants for uV_to_C
static float u_V_range[2][2] = {
    {-3554, 4096}, //-100 to 100°C 
    {4096, 16397}}; //100 to 400°C   --> can be extended http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/thermocouple/type-k-calibration-table


static float u_T0[2] = {-8.7935962E+00, 3.1018976E+02};
static float u_V0[2] = {-3.4489914E+02, 1.2631386E+04};
static float u_p[4][2] = {
    {2.5678719E-02, 2.4061949E-02},
    {-4.9887904E-07, 4.0158622E-06},
    {-4.4705222E-10, 2.6853917E-10},
    {-4.4869203E-14, -9.7188544E-15}};

static float u_q[3][2] = {
    {2.3893439E-07, 1.6995872E-04},
    {-2.0397750E-08, 1.1413069E-08},
    {-1.8424107E-12, -3.9275155E-13}};

int32_t C100_to_uV(int16_t T)
{
  /*float T0 = 2.5000000E+03; *100
  float V0 = 1.0003453E+03; *1000

  float p[4] = {4.0514854E-04, -3.8789638E-09, -2.8608478E-12, -9.5367041E-18};
  float q[2] = {-1.3948675E-05, -6.7976627E-09};
*/
  float TT0 = (float)T - C_T0;
  float top = (TT0 * (C_p[0] + TT0 * (C_p[1] + TT0 * (C_p[2] + TT0 * C_p[3]))));
  float bottom = (1 + TT0 * (C_q[0] + C_q[1] * TT0));
  return C_V0 + top / bottom * 1000;
}

float uV_to_C(int32_t V)
{
  /*  float V_range[2][2] = {
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
*/
  // first figure out which range of values
  uint8_t j;
  uint8_t ind = 0;
  for (j = 0; j < 2; j++)
  {
    if (((float)V >= u_V_range[0][j]) &&
        ((float)V <= u_V_range[1][j]))
      ind = j;
  };

  float VV0 = (float)V - u_V0[ind];

  float T = u_T0[ind] + (VV0 * (u_p[0][ind] + VV0 * (u_p[1][ind] + VV0 * (u_p[2][ind] + VV0 * u_p[3][ind])))) / (1 + VV0 * (u_q[0][ind] + VV0 * (u_q[1][ind] + VV0 * u_q[2][ind])));
  int temp_int = (int)(T*10);
  T = (float)temp_int; //round to 1 decimal
  return T/10;
}

int32_t uV_to_C1000(int32_t V)
{
  // first figure out which range of values
  uint8_t j;
  uint8_t ind = 0;
  for (j = 0; j < 2; j++)
  {
    if (((float)V >= u_V_range[0][j]) &&
        ((float)V <= u_V_range[1][j]))
      ind = j;
  };

  float VV0 = (float)V - u_V0[ind];

  float T = u_T0[ind] + (VV0 * (u_p[0][ind] + VV0 * (u_p[1][ind] + VV0 * (u_p[2][ind] + VV0 * u_p[3][ind])))) / (1 + VV0 * (u_q[0][ind] + VV0 * (u_q[1][ind] + VV0 * u_q[2][ind])));
  int32_t temp_int = (int32_t)(T*1000);
  return temp_int;
}
