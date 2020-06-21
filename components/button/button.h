#ifndef _BUTTON_H_
#define _BUTTON_H_ 

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

#define PIN_BIT(x) (1ULL<<x)

#define BUTTON_DOWN (1)
#define BUTTON_UP (2)

typedef struct {
	uint8_t pin;
    uint8_t event;
} button_event_t;

QueueHandle_t * button_init(unsigned long long pin_select);

#ifdef __cplusplus
}
#endif

#endif 