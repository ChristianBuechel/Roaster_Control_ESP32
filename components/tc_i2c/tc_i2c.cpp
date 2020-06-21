
#include "tc_i2c.h"
#include "cADC.h"
#include <driver/i2c.h>

#define TAG "TC_I2C"
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

int tc_i2c_initialized = -1;

QueueHandle_t *i2c_queue;
i2c_config_t conf;
cADC adc(0x68); // MCP3424 just initialize the object

static uint32_t millis()
{
    return esp_timer_get_time() / 1000;
}

static void send_event(int16_t temp, uint8_t chan)
{
    tc_i2c_event_t event = {};
    event.ch = chan;
    event.temp_uV = temp;
    event.temp_C = adc.uV_to_C(temp);
    event.time = millis();

    xQueueSend(i2c_queue, &event, portMAX_DELAY);
}

static void tc_i2c_task(void *pvParameter)
{
    int32_t v = 0;
    int32_t old_v = 0;;
    uint8_t ch = 0;
    uint16_t dly = adc.getConvTime(); //300 ms

    while (1)
    {
        adc.nextConversion(ch);
        //vTaskDelay(dly / portTICK_PERIOD_MS);
        vTaskDelay(adc.getConvTime() / portTICK_PERIOD_MS);
        v = adc.readuV();
        if (v != old_v)
        {
        send_event(v, ch);
        old_v = v;
        }
        switch (ch)
        { // clumsy
        case 0:
            ch = 1;
            adc.setCfg(ADC_BITS_18, ADC_GAIN_8, ADC_CONV_1SHOT); // 
            break;
        case 1:
            ch = 0;
            adc.setCfg(ADC_BITS_18, ADC_GAIN_8, ADC_CONV_1SHOT); // 
            break;
        }
    }
}

QueueHandle_t *tc_i2c_init()
{

    if (tc_i2c_initialized != -1)
    {
        ESP_LOGE(TAG, "Already initialized");
        return NULL;
    }
    tc_i2c_initialized = 1;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000; //seems to work both with 100 kHz and 400 kHz
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // Initialize global state and queue
    i2c_queue = (void **)xQueueCreate(4, sizeof(tc_i2c_event_t));
    ESP_LOGI(TAG, "Queue created");

    // Spawn a task and hand over the pin for the DS1820
    xTaskCreate(&tc_i2c_task, "tc_i2c_task", 4096, NULL, 5, NULL);
    return i2c_queue;
}
