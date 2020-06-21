#ifndef _ROTARY_H_
#define _ROTARY_H_

#ifdef __cplusplus
extern "C"
{
#endif

    //#include <string.h>
    //#include <stdbool.h>
    //#include <stdio.h>

#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/pcnt.h"

//#include "driver/gpio.h"
#include "esp_log.h"
#include <stdint.h>

#define PCNT_TEST_UNIT_0 PCNT_UNIT_0
#define PCNT_H_LIM_VAL_0 4
#define PCNT_L_LIM_VAL_0 -4
#define PCNT_INPUT_0_A 39 // Pulse Input GPIO
#define PCNT_INPUT_0_B 34 // Control GPIO HIGH=count up, LOW=count down

#define PCNT_TEST_UNIT_1 PCNT_UNIT_1
#define PCNT_H_LIM_VAL_1 4
#define PCNT_L_LIM_VAL_1 -4
#define PCNT_INPUT_1_A 26 // Pulse Input GPIO
#define PCNT_INPUT_1_B 27 // Control GPIO HIGH=count up, LOW=count down

    typedef struct
    {
        int unit;        // the PCNT unit that originated an interrupt
        uint32_t status; // information on the event type that caused the interrupt
        uint32_t time;
    } rotary_event_t;

    QueueHandle_t *rotary_init();

#ifdef __cplusplus
}
#endif

#endif