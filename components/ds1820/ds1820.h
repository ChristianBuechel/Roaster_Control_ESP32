#ifndef _DS1820_H_
#define _DS1820_H_ 

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
	int16_t temp100;
    uint32_t time;
} ds1820_event_t;

QueueHandle_t * ds1820_init(uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif 