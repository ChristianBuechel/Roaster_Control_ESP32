
#include "ds1820.h"
#include "owb.h"

#define TAG "DS1820"

#define DS18B20_FUNCTION_TEMP_CONVERT 0x44
#define DS18B20_FUNCTION_SCRATCHPAD_READ 0xBE
#define DS18B20_DLY 750 //ms it takes to get a temperature value

int ds1820_initialized = -1;

QueueHandle_t *queue;
OneWireBus *owb;
owb_rmt_driver_info rmt_driver_info; //put this global asa a pointer, then it should work !!!

static uint32_t millis()
{
    return esp_timer_get_time() / 1000;
}

static void send_event(int16_t temp)
{
    
    ds1820_event_t event = {
        .temp100 = temp,
        .time = millis(),
    };
    xQueueSend(queue, &event, portMAX_DELAY);
}

static void ds1820_task(void *pvParameter)
{
    int16_t TReading, Tc_100;
    uint8_t all_data[9]; //9 bytes come in last one is CRC
    bool present = false;
    
    while (1)
    {
        owb_reset(owb, &present);
        owb_write_byte(owb, OWB_ROM_SKIP);
        owb_write_byte(owb, DS18B20_FUNCTION_TEMP_CONVERT);
        vTaskDelay(DS18B20_DLY / portTICK_PERIOD_MS);
        owb_reset(owb, &present);
        owb_write_byte(owb, OWB_ROM_SKIP);
        owb_write_byte(owb, DS18B20_FUNCTION_SCRATCHPAD_READ);
        owb_read_bytes(owb, all_data, 9);

        if (owb_crc8_bytes(0, all_data, 9) == 0)
        {
            TReading = (all_data[1] << 8) + all_data[0];
            Tc_100 = (6 * TReading) + TReading / 4; //*6.25 as 1 = 0.0625°C --> T in C * 100
            send_event(Tc_100);
        }
        else
        {
            ESP_LOGE(TAG, "CRC does not match");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

QueueHandle_t *ds1820_init(uint8_t pin)
{

    if (ds1820_initialized != -1)
    {
        ESP_LOGE(TAG, "Already initialized");
        return NULL;
    }
    // Initialize global state and queue
    queue = xQueueCreate(4, sizeof(ds1820_event_t));
    ESP_LOGI(TAG, "Queue created");
    ESP_LOGI(TAG, "Using GPIO %d", pin);
    
    owb = owb_rmt_initialize(&rmt_driver_info, pin, RMT_CHANNEL_1, RMT_CHANNEL_0);

    // Spawn a task and hand over the pin for the DS1820
    xTaskCreate(&ds1820_task, "ds1820_task", 4096, NULL, 5, NULL);
    return queue;
}
