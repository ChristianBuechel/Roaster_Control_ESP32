
#include "triac.h"
#define TAG "TRIAC"

volatile int8_t curr;       // current value in N:M sequence
volatile bool newN = true;  // signals that output level has changed
volatile int8_t duty_2 = 0; // in %

int triac_initialized = -1;

QueueHandle_t *triac_queue;

static void IRAM_ATTR zero_crossing_interrupt(void *arg)
{
    uint32_t status;
    triac_event_t evt;
    evt.time = esp_timer_get_time();
    portBASE_TYPE HPTaskAwoken = pdFALSE;
    rmt_get_status(RMT_TX_CHANNEL_0, &status);
    if ((status & RMT_STATE_CH2) == 0)        //0 : idle, 1 : send, 2 : read memory, 3 : receive, 4 : wait
        rmt_tx_start(RMT_TX_CHANNEL_0, true); //send only if idle , if sending a new item while still sending screws everything up big time
 
    // send ZC time and clean up
    xQueueSendFromISR(triac_queue, &evt, &HPTaskAwoken);
    if (HPTaskAwoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }

    // let's do ICC using fixed blocks to reduce TRIAC switching
    if (newN)
    {
        curr = RATIO_M;
        newN = false;
    }
    curr -= 1;
    if (curr < 1) //curr == 0 ?
    {
        curr = RATIO_M;
    }
    if (curr > duty_2)
    {
        //do nothing?
    }
    else
    { //start pulse
    rmt_get_status(RMT_TX_CHANNEL_1, &status);
    if ((status & RMT_STATE_CH3) == 0)        //0 : idle, 1 : send, 2 : read memory, 3 : receive, 4 : wait
        rmt_tx_start(RMT_TX_CHANNEL_1, true); //send only if idle , if sending a new item while still sending screws everything up big time
    }
}

QueueHandle_t *triac_init()
{
    if (triac_initialized != -1)
    {
        ESP_LOGI(TAG, "Already initialized");
        return NULL;
    }

    triac_queue = xQueueCreate(10, sizeof(triac_event_t));
    ESP_LOGI(TAG, "Queue created");

    // set pull-down on FAN and HEATER pin
    // this turned out to be important so that the signal
    // comes down to LOW after RMT fired its pulse sequence
    gpio_config_t gp;
    gp.mode = GPIO_MODE_OUTPUT;
    gp.pull_up_en = GPIO_PULLUP_DISABLE;
    gp.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gp.pin_bit_mask = (1ULL << OUT_0) | (1ULL << OUT_1); //GPIO_SEL_13;
    gpio_config(&gp);

    //configure RMT for FAN (phase cut)
    rmt_config_t config;
    config.channel = RMT_TX_CHANNEL_0;
    config.gpio_num = OUT_0;
    config.mem_block_num = 1; // use default of 1 memory block
    config.tx_config.carrier_en = false;
    config.tx_config.loop_en = false;
    config.tx_config.idle_output_en = true;
    config.rmt_mode = RMT_MODE_TX;
    config.clk_div = 80; //us resolution
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    ESP_LOGI(TAG, "Installed RMT driver 0.\n");

    // configure RMT for HEATER
    config.channel = RMT_TX_CHANNEL_1;
    config.gpio_num = OUT_1;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    ESP_LOGI(TAG, "Installed RMT driver 1.\n");


    /*gpio_config_t out;
    out.mode = GPIO_MODE_OUTPUT;
    out.pin_bit_mask = 1ULL << OUT_1;
    gpio_config(&out);
    */

    gpio_config_t zc;
    zc.intr_type = GPIO_INTR_NEGEDGE;
    zc.mode = GPIO_MODE_INPUT;
    zc.pull_up_en = GPIO_PULLUP_ENABLE;
    //zc.pull_down_en = GPIO_PULLDOWN_DISABLE;
    zc.pin_bit_mask = GPIO_SEL_35; //try 1ULL << ZERO_CROSSING_PIN
    gpio_config(&zc);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ZERO_CROSSING_PIN, zero_crossing_interrupt, (void *)ZERO_CROSSING_PIN);
    ESP_LOGI(TAG, "Enabled zero crossing interrupt.\n");

    triac_initialized = 1;
    return triac_queue;
}
