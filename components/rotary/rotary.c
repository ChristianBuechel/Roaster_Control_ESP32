
#include "rotary.h"

#define TAG "ROTARY"

int rotary_initialized = -1;
QueueHandle_t *rotary_queue;

pcnt_config_t pcnt_configa0 = {};
pcnt_config_t pcnt_configb0 = {};
pcnt_config_t pcnt_configa1 = {};
pcnt_config_t pcnt_configb1 = {};

pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    rotary_event_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;
    for (i = 0; i < PCNT_UNIT_MAX; i++)
    {
        if (intr_status & (BIT(i)))
        {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            evt.time   = esp_timer_get_time() / 1000;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(rotary_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }
        }
    }
}

QueueHandle_t *rotary_init()
{
    if (rotary_initialized != -1)
    {
        ESP_LOGI(TAG, "Already initialized");
        return NULL;
    }
    // initialize pcnt_configa0
    pcnt_configa0.pulse_gpio_num = PCNT_INPUT_0_A;
    pcnt_configa0.ctrl_gpio_num = PCNT_INPUT_0_B;
    pcnt_configa0.channel = PCNT_CHANNEL_0;
    pcnt_configa0.unit = PCNT_TEST_UNIT_0;
    pcnt_configa0.pos_mode = PCNT_COUNT_INC;      // Count up on the positive edge
    pcnt_configa0.neg_mode = PCNT_COUNT_DEC;      // Keep the counter value on the negative edge
    pcnt_configa0.lctrl_mode = PCNT_MODE_KEEP;    // Reverse counting direction if low
    pcnt_configa0.hctrl_mode = PCNT_MODE_REVERSE; // Keep the primary counter mode if high
    pcnt_configa0.counter_h_lim = PCNT_H_LIM_VAL_0;
    pcnt_configa0.counter_l_lim = PCNT_L_LIM_VAL_0;
    // initialize pcnt_configb0
    pcnt_configb0.pulse_gpio_num = PCNT_INPUT_0_B;
    pcnt_configb0.ctrl_gpio_num = PCNT_INPUT_0_A;
    pcnt_configb0.channel = PCNT_CHANNEL_1;
    pcnt_configb0.unit = PCNT_TEST_UNIT_0;
    pcnt_configb0.pos_mode = PCNT_COUNT_DEC;      // Count up on the positive edge
    pcnt_configb0.neg_mode = PCNT_COUNT_INC;      // Keep the counter value on the negative edge
    pcnt_configb0.lctrl_mode = PCNT_MODE_KEEP;    // Reverse counting direction if low
    pcnt_configb0.hctrl_mode = PCNT_MODE_REVERSE; // Keep the primary counter mode if high
    pcnt_configb0.counter_h_lim = PCNT_H_LIM_VAL_0;
    pcnt_configb0.counter_l_lim = PCNT_L_LIM_VAL_0;
    // initialize pcnt_configa1
    pcnt_configa1.pulse_gpio_num = PCNT_INPUT_1_A;
    pcnt_configa1.ctrl_gpio_num = PCNT_INPUT_1_B;
    pcnt_configa1.channel = PCNT_CHANNEL_0;
    pcnt_configa1.unit = PCNT_TEST_UNIT_1;
    pcnt_configa1.pos_mode = PCNT_COUNT_INC;      // Count up on the positive edge
    pcnt_configa1.neg_mode = PCNT_COUNT_DEC;      // Keep the counter value on the negative edge
    pcnt_configa1.lctrl_mode = PCNT_MODE_KEEP;    // Reverse counting direction if low
    pcnt_configa1.hctrl_mode = PCNT_MODE_REVERSE; // Keep the primary counter mode if high
    pcnt_configa1.counter_h_lim = PCNT_H_LIM_VAL_1;
    pcnt_configa1.counter_l_lim = PCNT_L_LIM_VAL_1;
    // initialize pcnt_configb1
    pcnt_configb1.pulse_gpio_num = PCNT_INPUT_1_B;
    pcnt_configb1.ctrl_gpio_num = PCNT_INPUT_1_A;
    pcnt_configb1.channel = PCNT_CHANNEL_1;
    pcnt_configb1.unit = PCNT_TEST_UNIT_1;
    pcnt_configb1.pos_mode = PCNT_COUNT_DEC;      // Count up on the positive edge
    pcnt_configb1.neg_mode = PCNT_COUNT_INC;      // Keep the counter value on the negative edge
    pcnt_configb1.lctrl_mode = PCNT_MODE_KEEP;    // Reverse counting direction if low
    pcnt_configb1.hctrl_mode = PCNT_MODE_REVERSE; // Keep the primary counter mode if high
    pcnt_configb1.counter_h_lim = PCNT_H_LIM_VAL_1;
    pcnt_configb1.counter_l_lim = PCNT_L_LIM_VAL_1;
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_configa0);
    pcnt_unit_config(&pcnt_configb0);
    pcnt_unit_config(&pcnt_configa1);
    pcnt_unit_config(&pcnt_configb1);
    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT_0, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT_0);
    pcnt_set_filter_value(PCNT_TEST_UNIT_1, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT_1);
    /* Define limit events for interrupt */
    pcnt_event_enable(PCNT_TEST_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT_0, PCNT_EVT_L_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT_1, PCNT_EVT_L_LIM);
    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT_0);
    pcnt_counter_pause(PCNT_TEST_UNIT_1);
    pcnt_counter_clear(PCNT_TEST_UNIT_0);
    pcnt_counter_clear(PCNT_TEST_UNIT_1);
    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT_0);
    pcnt_intr_enable(PCNT_TEST_UNIT_1);
    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT_0);
    pcnt_counter_resume(PCNT_TEST_UNIT_1);
    // Initialize global state and queue
    rotary_queue = xQueueCreate(10, sizeof(rotary_event_t));
    ESP_LOGI(TAG, "Queue created");

    rotary_initialized = 1;
    return rotary_queue;
}
