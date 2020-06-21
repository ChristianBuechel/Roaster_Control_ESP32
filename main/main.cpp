/* Single DS1820 readout in task and data transfer through queue
*/

#include "ds1820.h"
#include "tc_calc.h"
#include "tc_i2c.h"
#include "button.h"
#include "rotary.h"
#include "tftspi.h"
#include "tft.h"
#include "SerialCommand.h"
#include "uart.h"
#include "triac.h"

#define GPIO_DS18B20 GPIO_NUM_4
#define BUTTON_0 GPIO_NUM_36
#define BUTTON_1 GPIO_NUM_25

/*
// Overview of all GPIOs used
// Currently all defined somewhere (.c, .h)
// TFT
PIN_NUM_CS GPIO_NUM_5
PIN_NUM_MISO GPIO_NUM_19
PIN_NUM_MOSI GPIO_NUM_23
PIN_NUM_CLK GPIO_NUM_18
PIN_NUM_DC GPIO_NUM_17

// Triac
#define ZERO_CROSSING_PIN GPIO_NUM_35
#define OUT_0 GPIO_NUM_13 // output phase angle control (PAC)
#define OUT_1 GPIO_NUM_14 // output integral cycle control

//I2C
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

//Rotary
#define PCNT_INPUT_0_A GPIO_NUM_39 // Pulse Input GPIO
#define PCNT_INPUT_0_B GPIO_NUM_34 // Control GPIO HIGH=count up, LOW=count down
#define PCNT_INPUT_1_A GPIO_NUM_26 // Pulse Input GPIO
#define PCNT_INPUT_1_B GPIO_NUM_27 // Control GPIO HIGH=count up, LOW=count down


// The following are already defined in main.cpp

//Button
#define BUTTON_0 GPIO_NUM_36
#define BUTTON_1 GPIO_NUM_25

//DS18B20
#define GPIO_DS18B20 GPIO_NUM_4

*/



#define X_FAN 0
#define Y_FAN 107
#define X_PWR 80
#define Y_PWR 107

#define Y_AT 2
#define Y_ET 37
#define Y_BT 72

#define LINE_1 1 //2
#define LINE_2 31 //37
#define LINE_3 61 //72
#define LINE_4 91 //107
#define LINE_5 110 //107
#define LINE_END DEFAULT_TFT_DISPLAY_WIDTH

#define QUART_1 0
#define QUART_2 (DEFAULT_TFT_DISPLAY_HEIGHT / 4)
#define QUART_3 (2 * (DEFAULT_TFT_DISPLAY_HEIGHT / 4))
#define QUART_4 (3 * (DEFAULT_TFT_DISPLAY_HEIGHT / 4))
#define QUART_END DEFAULT_TFT_DISPLAY_HEIGHT

#define QT_1 0
#define QT_2 (DEFAULT_TFT_DISPLAY_HEIGHT / 4)
#define QT_3 ((DEFAULT_TFT_DISPLAY_HEIGHT / 4) + (DEFAULT_TFT_DISPLAY_HEIGHT * 3 / 8))
#define QT_END DEFAULT_TFT_DISPLAY_HEIGHT

#define THIRD_1 0
#define THIRD_2 (DEFAULT_TFT_DISPLAY_HEIGHT / 3)
#define THIRD_3 (2 * (DEFAULT_TFT_DISPLAY_HEIGHT / 3))
#define THIRD_END DEFAULT_TFT_DISPLAY_HEIGHT

#define HALF_1 0
#define HALF_2 (DEFAULT_TFT_DISPLAY_HEIGHT / 2)
#define HALF_END DEFAULT_TFT_DISPLAY_HEIGHT

#define BG_DEFAULT TFT_WHITE
#define BG_CONFIRM TFT_WHITE
#define FG_CONFIRM TFT_BLACK
#define BG_CHANGE TFT_RED
#define FG_CHANGE TFT_WHITE

#define FG_AT TFT_BLUE
#define FG_BT TFT_RED
#define FG_ET TFT_RED

#define BLINK_GPIO GPIO_NUM_2

#define TAG "MAIN"

extern "C"
{
	void app_main();
}

template <class T>
T check_range(T value, T min, T max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}
//entry 0 of phase delay will never be used as wave.level1 = 0
uint16_t phase_delay[101] = {0, 8840, 8531, 8311, 8133, 7981, 7846, 7724, 7613, 7509,
							 7411, 7319, 7231, 7148, 7067, 6990, 6915, 6843, 6772, 6704,
							 6637, 6572, 6508, 6446, 6384, 6324, 6265, 6206, 6149, 6092,
							 6036, 5981, 5926, 5872, 5818, 5765, 5712, 5660, 5608, 5556,
							 5504, 5453, 5402, 5352, 5301, 5251, 5201, 5150, 5100, 5050,
							 5000, 4950, 4900, 4850, 4800, 4750, 4699, 4649, 4598, 4547,
							 4496, 4445, 4393, 4341, 4289, 4236, 4182, 4129, 4075, 4020,
							 3964, 3908, 3852, 3794, 3736, 3677, 3616, 3555, 3492, 3429,
							 3363, 3297, 3228, 3158, 3085, 3011, 2933, 2853, 2769, 2681,
							 2589, 2492, 2388, 2276, 2154, 2020, 1868, 1690, 1470, 1160,
							 0}; //TRIAC goes off at about 94..95

// we need to deal with cases of 0 and 100% power...
// there is a 8us delay in the ISR, ie the delay before the TRIAC trigger is 8us longer  

uint32_t duty_1 = 0;			 // in %
uint32_t pw = TRIAC_PULSE_WIDTH; // in us

rmt_item32_t wave0[] = { // use for FAN
	{{{0, 0, 0, 1}}},   // dummy values will never be used
	{{{0, 1, 0, 0}}}}; 	// RMT end marker
rmt_item32_t wave1[] = { //use for HEATER
	{{{0, 0, 0, 1}}},   // dummy values will never be used
	{{{0, 1, 0, 0}}}}; 	// RMT end marker

struct window
{
	uint16_t x1, x2, y1, y2;
	color_t fg, bg, hfg, hbg; //foreground, back, highlights
	char align_x, align_y;	// cave : display is sotated so alignments as well!!
	uint8_t font, font_dec;
};

window windows[] = {
	{QT_1, QT_2, LINE_1, LINE_2, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'l', 't', BAHN28_FONT, BAHN20_FONT},		 //0 line 1 left
	{QT_2, QT_3, LINE_1, LINE_2, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'r', 't', BAHN28_FONT, BAHN20_FONT},	     //1 line 1 middle
	{QT_3, QT_END, LINE_1, LINE_2, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'r', 't', BAHN28_FONT, BAHN20_FONT},	 //2 line 1 right

	{QT_1, QT_2, LINE_2, LINE_3, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'l', 't', BAHN28_FONT, BAHN20_FONT},		 //3 line 2 left
	{QT_2, QT_3, LINE_2, LINE_3, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'r', 't', BAHN28_FONT, BAHN20_FONT},	     //4 line 2 middle
	{QT_3, QT_END, LINE_2, LINE_3, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'r', 't', BAHN28_FONT, BAHN20_FONT},	 //5 line 2 right

	{QT_1, QT_2, LINE_3, LINE_4, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'l', 't', BAHN28_FONT, BAHN20_FONT},		  //6 line 3 left
	{QT_2, QT_3, LINE_3, LINE_4, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'r', 't', BAHN28_FONT, BAHN20_FONT},	      //7 line 3 middle
	{QT_3, QT_END, LINE_3, LINE_4, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'r', 't', BAHN28_FONT, BAHN20_FONT},	  //8 line 3 right

	{QUART_1, QUART_END, LINE_4, LINE_5, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'l', 'c', CONSOLA14_FONT, CONSOLA14_FONT}, //9 line 4 4

	{QUART_1, QUART_2, LINE_5, LINE_END, TFT_BLACK, TFT_WHITE, TFT_BLACK, TFT_LIGHTGREY, 'l', 'c', BAHN18_FONT, BAHN16_FONT}, //10 line 4 1
	{QUART_2, QUART_3, LINE_5, LINE_END, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'c', 'c', BAHN18_FONT, BAHN16_FONT},	  //11 line 4 2
	{QUART_3, QUART_4, LINE_5, LINE_END, TFT_BLACK, TFT_WHITE, TFT_BLACK, TFT_LIGHTGREY, 'l', 'c', BAHN18_FONT, BAHN16_FONT}, //12 line 4 3
	{QUART_4, QUART_END, LINE_5, LINE_END, TFT_BLACK, TFT_WHITE, TFT_WHITE, TFT_BLACK, 'c', 'c', BAHN18_FONT, BAHN16_FONT},   //13 line 4 4

};

#define BUFF_SIZE (16U) //16 elemnts
#define BUFF_SIZE_MASK (BUFF_SIZE-1U)

const int32_t weights[16] = {1, 13, 77, 273, 637, 1001, 1001, 429, -429, -1001, -1001, -637, -273, -77, -13, -1};

struct buffer {
    int32_t buff[2][BUFF_SIZE]; 
    uint8_t writeIndex[2] = {0,0};
};

buffer temp_buffer;  //FIFO holding past 16 values of temps in C * 1000
buffer deriv_buffer; //FIFO holding past 16 values of deriv in C/min * 10


void write(struct buffer *buffer, uint8_t chan, int32_t value) //write to buffer
{
	//ESP_LOGI(TAG, "Writing chan %d, index %d Value : %d", chan, buffer->writeIndex[chan]& BUFF_SIZE_MASK, value);
    buffer->buff[chan][(++buffer->writeIndex[chan]) & BUFF_SIZE_MASK] = value;
	//ESP_LOGI(TAG, "Index now %d", buffer->writeIndex[chan]& BUFF_SIZE_MASK);
}

int32_t readn(struct buffer *buffer, uint8_t chan, unsigned Xn) //read from buffer Xn=0 will read last written value 
{
    return buffer->buff[chan][(buffer->writeIndex[chan] - Xn) & BUFF_SIZE_MASK];
}

bool is_diff(int32_t a, int32_t b, int32_t range)
{
	return (a/range) != (b/range);
}

int32_t get_deriv(struct buffer *buffer, uint8_t chan)
{
	int32_t sum_up = 0;
	for (uint8_t i = 0; i < BUFF_SIZE - 1; ++i)
	{
		sum_up += readn(buffer, chan, i) * weights[i];
	}

	sum_up = sum_up >> 14; // divide by 16384
	                       // sampling is 600ms (0.01 minute); units are degC *1000 
	return sum_up;         // is now the derivative in deg C * 10 
}

float ET, BT, ETd, BTd; //current temps in float
int16_t current_temp100_ds_C = 0;
float current_temp_ds_C = 0;
float old_temp_ds_C = 0;
int16_t fan = 0;
int16_t pwr = 0;
int16_t t_fan = 0;
int16_t t_pwr = 0;

SerialCommand myCMD; // The  SerialCommand object

static char tmp_buff[64];

void print_window(char *st, uint8_t win, uint8_t hl)
{
	int x_c = 0;
	int y_c = 0;
	int w_pre = 0;
	int w_post = 0;
	char str[128];
	char *deci;

	strcpy(str, st);
	TFT_setclipwin(windows[win].x1, windows[win].y1, windows[win].x2, windows[win].y2);
	strtok_r(str, ".", &deci); //divide string
	if (deci != NULL)
	{
		// get length
		TFT_setFont(windows[win].font_dec, NULL);
		w_post = TFT_getStringWidth(deci);
	}

	TFT_setFont(windows[win].font, NULL);
	w_pre = TFT_getStringWidth(str);

	if (hl) // highlight
	{
		_bg = windows[win].hbg;
		_fg = windows[win].hfg;
		TFT_fillWindow(windows[win].hbg);
	}
	else
	{
		_bg = windows[win].bg;
		_fg = windows[win].fg;
		TFT_fillWindow(windows[win].bg);
	}
	switch (windows[win].align_x)
	{
	case 'l':
		x_c = 0;
		break;
	case 'c':
		x_c = CENTER;
		break;
	case 'r':
		x_c = windows[win].x2 - windows[win].x1 - w_pre - w_post - 2; //debug -2
		break;
	}
	switch (windows[win].align_y)
	{
	case 't':
		y_c = 0;
		break;
	case 'c':
		y_c = CENTER;
		break;
	case 'b':
		y_c = BOTTOM;
		break;
	}

	TFT_setFont(windows[win].font, NULL);
	TFT_print(str, x_c, y_c); //use window width

	if (deci != NULL)
	{
		TFT_setFont(windows[win].font_dec, NULL);
		//TFT_print(deci, x_c + w_pre, y_c); //use window width
		TFT_print(deci, LASTX, y_c); //use window width
	}
	TFT_resetclipwin();
}

void set_duty_fan(int16_t fan)
{
	wave0[0].duration0 = phase_delay[fan] + uint16_t(ZC_LEAD);
	if (fan > 0)
	{
		wave0[0].level1 = 1;
	}
	else
	{
		wave0[0].level1 = 0;
	}
	rmt_fill_tx_items(RMT_TX_CHANNEL_0, wave0, 2, 0);
	ESP_LOGI(TAG, "Set FAN duty to %d%%", fan);
}
void set_duty_pwr(int16_t pwr)
{
	duty_2 = pwr;
	newN = true;
	ESP_LOGI(TAG, "Set ICC duty to %d%%", duty_2);
}

void confirm_fan()
{
	fan = t_fan;
	sprintf(tmp_buff, "%d ", fan);
	print_window(tmp_buff, 11, 0);
	set_duty_fan(fan);
}

void fan_chng(int8_t x)
{
	//t_fan = t_fan + x;
	t_fan = check_range((int16_t)(t_fan + x), (int16_t)0, (int16_t)100);
	sprintf(tmp_buff, "%d ", t_fan);
	print_window(tmp_buff, 11, 1);
}

void confirm_pwr()
{
	pwr = t_pwr;
	sprintf(tmp_buff, "%d ", pwr);
	print_window(tmp_buff, 13, 0);
	set_duty_pwr(pwr);
}

void pwr_chng(int8_t x)
{
	//t_pwr = t_pwr + x;
	t_pwr = check_range((int16_t)(t_pwr + x), (int16_t)0, (int16_t)RATIO_M);
	sprintf(tmp_buff, "%d ", t_pwr);
	print_window(tmp_buff, 13, 1);
}

void unrecognized(const char *command)
{
	char *arg;
	arg = myCMD.next(); // Get the next argument from the SerialCommand object buffer
	if (arg != NULL)	// As long as it existed, take it
	{
		sprintf(tmp_buff, "> %s;%s", command,arg); //CAVE only upcase letters in fonts ...
	    ESP_LOGI(TAG, "> %s;%s", command,arg);
	}
	else
	{
		sprintf(tmp_buff, "> %s", command); //CAVE only upcase letters in fonts ...
	    ESP_LOGI(TAG, "> %s", command);
	}
	print_window(tmp_buff, 9, 0);
}


void process_DUR()
{
	char *arg;
	uint32_t d = 0;		// between 0 and 10000 us
	arg = myCMD.next(); // Get the next argument from the SerialCommand object buffer
	if (arg != NULL)	// As long as it existed, take it
	{
		d = atol(arg);
		pw = check_range(d, (uint32_t)0, (uint32_t)4000); // max 4000 us
		wave0[0].duration1 = pw;
		wave1[0].duration1 = pw;
		rmt_fill_tx_items(RMT_TX_CHANNEL_0, wave0, 2, 0);
		rmt_fill_tx_items(RMT_TX_CHANNEL_1, wave1, 2, 0);
		ESP_LOGI(TAG, "Set PW to %d", pw);
	}
}

void process_FAN()
{
	char *arg;
	int32_t d = 0;		// between 0 and 10000 us
	arg = myCMD.next(); // Get the next argument from the SerialCommand object buffer
	if (arg != NULL)	// As long as it existed, take it
	{
		d = atol(arg);
		t_fan = check_range(d, (int32_t)0, (int32_t)100);
		ESP_LOGI(TAG, "Set PAC to %d%%", t_fan);
		confirm_fan();
	}
}

void process_PWR()
{
	char *arg;
	int32_t d = 0;		// between 0 and 100%
	arg = myCMD.next(); // Get the next argument from the SerialCommand object buffer
	if (arg != NULL)	// As long as it existed, take it
	{
		d = atol(arg);
		t_pwr = check_range(d, (int32_t)0, (int32_t)RATIO_M);
		ESP_LOGI(TAG, "Set ICC to %d%%", t_pwr);
		confirm_pwr();
	}
}

void process_READ()
{
	printf("0.0,%1.1f,%1.1f,%d,%d,55,%1.1f\n", ET, BT, pwr, fan, current_temp_ds_C);
	//Artisan says: response: list ["t0","t1","t2","t3","t4"]  with t0 = internal temp; t1 = ET; t2 = BT, t3 = chan3, t4 = chan4 on "CHAN;1234" if ArduinoTC4_34 is configured
    //after PID_ON: + [,"Heater", "Fan", "SV"] 
}

void process_LOG()
{
	char *arg;
	uint8_t log_level = 0;
	arg = myCMD.next(); // Get the next argument from the SerialCommand object buffer
	if (arg != NULL)	// As long as it existed, take it
	{
		log_level = atoi(arg);
		if (log_level < 1)
		{
			esp_log_level_set("*", ESP_LOG_NONE);
		}
		else
		{
			esp_log_level_set("*", ESP_LOG_INFO);
		}
	}
}

void process_CHAN()
{
	char *arg;
	arg = myCMD.next(); // Get the next argument from
	if (arg != NULL)	// As long as it existed, take it
	{
		printf("# Active channels set to %s\n", arg);
	}
}


static void gpio_test_signal(void *arg)
{
	ESP_LOGI(TAG, "initializing fake ZC signal...");
	gpio_config_t gp;
	//gp.intr_type = GPIO_INTR_DISABLE;
	gp.mode = GPIO_MODE_OUTPUT;
	//gp.pull_up_en = GPIO_PULLUP_ENABLE;
	//gp.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gp.pin_bit_mask = GPIO_SEL_12;
	gpio_config(&gp);

	while (1)
	{
		//here the period of test signal is 10ms --> emulate ZC
		gpio_set_level(GPIO_NUM_12, 0); // Set low
		ets_delay_us(1000);
		gpio_set_level(GPIO_NUM_12, 1); // Set high
		vTaskDelay(1);					// wait for about 10ms
	}
}


void app_main()
{
	esp_log_level_set("*", ESP_LOG_NONE); //can be changed using UART cmd LOG;0 or 1
	gpio_pad_select_gpio(BLINK_GPIO);	  //to see when ESP32 is running
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(BLINK_GPIO, 1);
	// setup ds1820
	ds1820_event_t ds1820ev;
	QueueHandle_t ds1820_events = ds1820_init(GPIO_DS18B20);
	//setup thermocouples
	tc_i2c_event_t tci2cev;
	QueueHandle_t tc_i2c_events = tc_i2c_init(); //init with GPIOs would be nice
	//setup rotary encoders
	rotary_event_t rotaryev;
	QueueHandle_t rotary_events = rotary_init(); //init with GPIOs would be nice
	//setup buttons on rotary encoders
	button_event_t buttonev;
	QueueHandle_t button_events = button_init(PIN_BIT(BUTTON_0) | PIN_BIT(BUTTON_1));

	// start fake ZC signal
	xTaskCreate(gpio_test_signal, "gpio_test_signal", 4096, NULL, 5, NULL); //comment if you don't want to use capture module

	//setup triac routines
	triac_event_t triacev;
	QueueHandle_t triac_events = triac_init(); //init with GPIOs would be nice
	
	wave0[0].duration0 = phase_delay[fan] + uint16_t(ZC_LEAD);
	wave0[0].level1    = 0;
	wave0[0].duration1 = pw;
	rmt_fill_tx_items(RMT_TX_CHANNEL_0, wave0, 2, 0);

	wave1[0].duration0 = uint16_t(ZC_LEAD);
	wave1[0].duration1 = pw;
	rmt_fill_tx_items(RMT_TX_CHANNEL_1, wave1, 2, 0);
	//initilize tft
	TFT_init();
	sprintf(tmp_buff, "AT"); // window 0
	print_window(tmp_buff, 0, 0);
	sprintf(tmp_buff, "ET"); // window 2
	print_window(tmp_buff, 3, 0);
	sprintf(tmp_buff, "BT"); // window 4
	print_window(tmp_buff, 6, 0);

	sprintf(tmp_buff, "FAN"); // window 4
	print_window(tmp_buff, 10, 0);

	sprintf(tmp_buff, "PWR"); // window 4
	print_window(tmp_buff, 12, 0);

	sprintf(tmp_buff, "%d ", t_fan);
	print_window(tmp_buff, 11, 0);
	sprintf(tmp_buff, "%d ", t_pwr);
	print_window(tmp_buff, 13, 0);

	//initialite UART
	uart_init();
	myCMD.setDefaultHandler(unrecognized);	//
	myCMD.addCommand("PWR", process_PWR);	//
	myCMD.addCommand("FAN", process_FAN);	//
	myCMD.addCommand("DUR", process_DUR);	//
	myCMD.addCommand("LOG", process_LOG);   //
	myCMD.addCommand("READ", process_READ); //
	myCMD.addCommand("CHAN", process_CHAN); // register parse handlers
	myCMD.clearBuffer();

	while (1) //main loop
	{
		//int messagesWaiting = uxQueueMessagesWaiting(triac_events);
		//ESP_LOGI(TAG, "%d messages in triac queue", messagesWaiting);

		myCMD.readSerial();
		if (xQueueReceive(triac_events, &triacev, 0)) //do not wait
		{
			//ESP_LOGI(TAG, "Last ZC(ms): %d", uint32_t(triacev.time / 1000));
		}

		if (xQueueReceive(ds1820_events, &ds1820ev, 0)) //do not wait
		{
			current_temp100_ds_C = ds1820ev.temp100;
			current_temp_ds_C = (float)ds1820ev.temp100 / 100;
			if (current_temp_ds_C != old_temp_ds_C)
			{
				sprintf(tmp_buff, "%3.1f", current_temp_ds_C); // window 2 AT
				print_window(tmp_buff, 2, 0);
				old_temp_ds_C = current_temp_ds_C;
			}
		}
		if (xQueueReceive(tc_i2c_events, &tci2cev, 0)) //do not wait
		{
			write(&temp_buffer, tci2cev.ch, uV_to_C1000(C100_to_uV(current_temp100_ds_C) + (int32_t)tci2cev.temp_uV));
			write(&deriv_buffer, tci2cev.ch, get_deriv(&temp_buffer,tci2cev.ch));
            ET  = (float)readn(&temp_buffer,0,0)/1000; //last values
            BT  = (float)readn(&temp_buffer,1,0)/1000; //here you could also define means etc
            ETd = (float)readn(&deriv_buffer,0,0)/10;
            BTd = (float)readn(&deriv_buffer,1,0)/10;

			ESP_LOGI(TAG, "Channel %d sampled at %d ms", tci2cev.ch, tci2cev.time);
			ESP_LOGI(TAG, "AT: %3.2f ET: %3.2f°C BT: %3.2f°C ETd: %3.2f°C BTd: %3.2f°C", current_temp_ds_C, ET, BT, ETd, BTd);

			if (is_diff(readn(&temp_buffer, 0, 0),readn(&temp_buffer, 0, 1),100)) //temp is in C*1000 so cmp to 1 decimal place
			{
				sprintf(tmp_buff, "%3.1f", ET); // window 5
				print_window(tmp_buff, 5, 0);
			}
			if (readn(&deriv_buffer, 0, 0) != readn(&deriv_buffer, 0, 1)) //deriv is in C*10 so cmp to 1 decimal place
			{
				sprintf(tmp_buff, "%3.1f", ETd); // window 7
				print_window(tmp_buff, 4, 0);
			}

			if (is_diff(readn(&temp_buffer, 1, 0),readn(&temp_buffer, 1, 1),100)) //temp is in C*1000 so cmp to 1 decimal place
			{
				sprintf(tmp_buff, "%3.1f", BT); // window 8
				print_window(tmp_buff, 8, 0);
			}
			if (readn(&deriv_buffer, 1, 0) != readn(&deriv_buffer, 1, 1)) //deriv is in C*10 so cmp to 1 decimal place
			{
				sprintf(tmp_buff, "%3.1f", BTd); // window 7
				print_window(tmp_buff, 7, 0);
			}
		}
		if (xQueueReceive(button_events, &buttonev, 0)) //do not wait
		{
			if ((buttonev.pin == BUTTON_0) && (buttonev.event == BUTTON_UP))
			{
				//FAN
				ESP_LOGI(TAG, "Rotary 0 RELEASED");
				confirm_fan();
			}
			if ((buttonev.pin == BUTTON_1) && (buttonev.event == BUTTON_UP))
			{
				//PWR
				ESP_LOGI(TAG, "Rotary 1 RELEASED");
				confirm_pwr();
			}
		}

		if (xQueueReceive(rotary_events, &rotaryev, 0)) //do not wait
		{
			if ((rotaryev.status & PCNT_STATUS_L_LIM_M) && (rotaryev.unit == PCNT_TEST_UNIT_0))
			{
				ESP_LOGI(TAG, "Rotary 0 UP");
				fan_chng(1);
			}
			if ((rotaryev.status & PCNT_STATUS_H_LIM_M) && (rotaryev.unit == PCNT_TEST_UNIT_0))
			{
				ESP_LOGI(TAG, "Rotary 0 DOWN");
				fan_chng(-1);
			}

			if ((rotaryev.status & PCNT_STATUS_L_LIM_M) && (rotaryev.unit == PCNT_TEST_UNIT_1))
			{
				ESP_LOGI(TAG, "Rotary 1 UP");
				pwr_chng(1);
			}
			if ((rotaryev.status & PCNT_STATUS_H_LIM_M) && (rotaryev.unit == PCNT_TEST_UNIT_1))
			{
				ESP_LOGI(TAG, "Rotary 1 DOWN");
				pwr_chng(-1);
			}
		}
		vTaskDelay(10 / portTICK_RATE_MS); //wait --> see whether this is OK for the ZC queue
										   //tft_demo();
	}
}