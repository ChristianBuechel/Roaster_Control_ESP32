#ifndef _TC_CALC_H_
#define _TC_CALC_H_ 

#ifdef __cplusplus
extern "C"
{
#endif 

#include <stdint.h>

int32_t C100_to_uV(int16_t T);
float uV_to_C(int32_t V);
int32_t uV_to_C1000(int32_t V);

#ifdef __cplusplus
}
#endif

#endif 