#ifndef _TC_I2C_H_
#define _TC_I2C_H_ 

#ifdef __cplusplus
extern "C"
{
#endif 


#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdint.h>

typedef struct {
	int16_t temp_uV;
	float temp_C;
    uint8_t ch;
    uint32_t time;
} tc_i2c_event_t;

QueueHandle_t *tc_i2c_init();

#ifdef __cplusplus
}
#endif

#endif 