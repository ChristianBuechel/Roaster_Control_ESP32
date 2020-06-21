#ifndef _TRIAC_H_
#define _TRIAC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt.h"

//#include "driver/gpio.h"
#include "esp_log.h"
#include <stdint.h>

#define ESP_INTR_FLAG_DEFAULT 0

#define RMT_TX_CHANNEL_0 RMT_CHANNEL_2 //channels 0 and 1 are used by the DS 1820 module
#define RMT_TX_CHANNEL_1 RMT_CHANNEL_3 //channels 0 and 1 are used by the DS 1820 module

// Pin configuration //////////////////////////////////////////////////////////////
#define ZERO_CROSSING_PIN GPIO_NUM_35

//#define OUT_0 GPIO_NUM_13 // output phase angle control (PAC)
//#define OUT_1 GPIO_NUM_14 // output integral cycle control
// had to switch those due to different Triacs BTA06 BTA24
#define OUT_0 GPIO_NUM_14 // output phase angle control (PAC)
#define OUT_1 GPIO_NUM_13 // output integral cycle control

#define ZC_LEAD 500             // zero cross signal is about 1000us and leads the actual crossing by approx 500us
#define TRIAC_PULSE_WIDTH 2000  // reasonable, but fan might require more
#define RATIO_M 100             // resolution of quantization of output levels

    typedef struct
    {
        uint64_t time; //time of ZC
    } triac_event_t;

    QueueHandle_t *triac_init();


    // these are needed globally
    extern volatile int8_t duty_2; // in %
    extern volatile bool newN;     // signals that output level has changed
    // just here

#ifdef __cplusplus
}
#endif

#endif