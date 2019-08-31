/* BianFa Lift Arrival Lighting System
	 Based on gpio_example_main.c 
   CopyRight @ BianFa Techonoly
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include <math.h>
#include "audio.h"
//#include "pcm_code.h"
#include "pcm_code_16k.h"
#include "dingdong_16k.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "nvs_flash.h"
#include "nvs.h"

//#include "myprivate.h"

extern void bt_ssp_init();

#define STORAGE_NAMESPACE "storage"
/**
 * Brief:
 * Detect the evevator up/dn arrival, then, turn on and flash the LED lights, play the voice
 *
 */

//#define LED_NUM	   90

#define ZERO_CODE_H   7 
#define ZERO_CODE_L   43

#define ONE_CODE_H    24
#define ONE_CODE_L    26

#define GPIO_DT1_UP  34     //Lift1 Up Signal
#define GPIO_DT1_DN  35     //Lift1 Down Signal

#define GPIO_DT2_UP    32   //Lift2 Up Signal
#define GPIO_DT2_DN    33   //Lift2 Down Signal

#define GPIO_LED_UP    22     //LED DATA IO 0
#define GPIO_LED_DN    21     //LED DATA IO 1

#define GPIO_PA_EN		 23

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_LED_UP) | (1ULL<<GPIO_LED_DN) | (1ULL<<GPIO_PA_EN) | (1ULL<<0))

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_DT1_UP) | (1ULL<<GPIO_DT1_DN)|(1ULL<<GPIO_DT2_UP) | (1ULL<<GPIO_DT2_DN))
#define ESP_INTR_FLAG_DEFAULT 0

#define BUF_SIZE (4096)

#define UART_CMD_SET_LIGNTS_NUM       0
#define UART_CMD_SET_BRIGHTNESS       1
#define UART_CMD_SET_FRESH_FREQ       2
#define UART_CMD_SET_ADVISE_DAT       3
#define UNDEFINED_UART_CMD						255

#define VOICE_DATA_LENG               5920 * 16 * 2

/*
* Assign HSI SPI pin
*/
#define PIN_NUM_MISO 15
#define PIN_NUM_MOSI 21
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   18

/*
* Assign VSI SPI pin
*/
#define PIN_NUM_MISO_1 27
#define PIN_NUM_MOSI_1 22
#define PIN_NUM_CLK_1  14
#define PIN_NUM_CS_1   12
/*
* Assign I2S Pin
*/
#define SAMPLE_RATE     (16000)
#define I2S_NUM         (0)
#define I2S_BCK_IO      (GPIO_NUM_5)
#define I2S_WS_IO       (GPIO_NUM_25)
#define I2S_DO_IO       (GPIO_NUM_26)
#define I2S_DI_IO       (-1)
#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)
/*
* Assign UART pin
*/
#define ECHO_TEST_TXD  (GPIO_NUM_4)
#define ECHO_TEST_RXD  (GPIO_NUM_5)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define EX_UART_NUM UART_NUM_0
/*
* declare two spi control handle
*/
spi_device_handle_t spi;
spi_device_handle_t spi_1;
/*
* declare nvs flash handle
*/
nvs_handle my_nvs_handle;
/*
* declare semaphore
*/
SemaphoreHandle_t xElevator_UpArrival_Semaphore = NULL;
SemaphoreHandle_t xElevator_DnArrival_Semaphore = NULL;
SemaphoreHandle_t xLED_Flash_Semaphore = NULL;
SemaphoreHandle_t xVoicePlay_Semaphore = NULL;
QueueHandle_t uart0_queue;
SemaphoreHandle_t xUART0_Semaphore = NULL;

SemaphoreHandle_t xLIGHTNUM_Semaphore = NULL;
SemaphoreHandle_t xVoiceIndex_Semaphore = NULL;
SemaphoreHandle_t xDoorDly_Semaphore = NULL;
SemaphoreHandle_t xLEDWait_Semaphore = NULL;
/*
* declare uart event tag
*/
const char *MY_UART_TAG = "uart_events";
/*
* The Private Structor for Lift Arrival Light System
*/
typedef struct {
	uint8_t light_num;
	uint8_t brightness;
	uint8_t frequency;
  	uint8_t voice_index; 
  	uint16_t door_delay;
  	uint16_t led_wait;
  	uint8_t voice_num;
  	uint16_t voice_delay;
  	uint8_t  work_mode;
} BF_PRIV;
BF_PRIV myprivate;
/*
* The Structor for uart command
*/
typedef struct {
	uint8_t uart_cmd;
  	uint8_t uart_params;
  	long int data; 
} UART_PRIV;
UART_PRIV uart_priv;

/*
* UART COMMAND LIST
*/
typedef struct {
	const char *cmd;
	const char *num;
} uart_command_config;
const uart_command_config uart_command[] = {
	{"BFSET", "0"},
	{"BFADV", "1"},
	{"BFTST", "2"},
	{"BFGET", "3"},
	{NULL, NULL}
};	
/*
* UART Parameters List
*/
const uart_command_config uart_params[] = {
	{"-N", "0"},   //light number
	{"-B", "1"},	 //Brightness
	{"-F", "2"},	 //Refresh frequency
	{"-D", "3"},	 //Door delay time
	{"-L", "4"},   //Data Length
	{"-T", "5"},	 //LED Wait time
	{"-V", "6"},   //Voice index
	{"-O", "7"},   //Voice repeat number
	{"-W", "8"},   //Voice wait time
	{"-M", "9"},   //DIVIDER MODE
	{NULL, NULL}
};	
/*
* Brief:
* 	compare two strings
*/
int strcmp_private( char *s1, const char *s2)
{
	while (*s1 == *s2++) {
        	if (*s1++ == 0 && *s2 == 0) {
           		 return 0;
        	}
    	}	
    	return 1;
}
/*
* Brief:
* 	config uart0 port
*/
void uart0_config(void)
{
    	/* Configure parameters of an UART driver,
     	* communication pins and install the driver */
  	uart_config_t uart_config = {
       		.baud_rate = 115200,
       		.data_bits = UART_DATA_8_BITS,
       		.parity = UART_PARITY_DISABLE,
       		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       		.rx_flow_ctrl_thresh = 122,
  	};
	//Set UART parameters
	uart_param_config(EX_UART_NUM, &uart_config);
	//Set UART log level
	esp_log_level_set(MY_UART_TAG, ESP_LOG_INFO);
	//Install UART driver, and get the queue.
	uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart0_queue, 0);
	//Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//Set uart pattern detect function.
	uart_enable_pattern_det_intr(EX_UART_NUM, '+', 3, 10000, 10, 10); 
}
/*
* Brief:
* 	play "dingdong" voice 
*/
void dingdong_play(void)
{
#if 0
    	uint8_t *samples_data = malloc(7282 * 16);
    	size_t i2s_bytes_write = 0;
    	memset(samples_data, 0, 7282*16);
    	memcpy(samples_data, dingdong_sample_16k, 7282 * 16);
    	i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2); 
    	i2s_write(I2S_NUM, samples_data, 7282 * 16, &i2s_bytes_write, portMAX_DELAY);
    	free(samples_data);
#else    	
	uint8_t *samples_data = malloc(3641 * 16);
    	size_t i2s_bytes_write = 0;
    	memset(samples_data, 0, 3641*16);
	//printf("dingdong----1\n");
    	memcpy(samples_data, dingdong_sample_16k, 3641 * 16);
    	i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2); 
    	i2s_write(I2S_NUM, samples_data, 3641 * 16, &i2s_bytes_write, portMAX_DELAY);
    	memcpy(samples_data, dingdong_sample_16k+3641 * 16, 3641 * 16);
    	i2s_write(I2S_NUM, samples_data, 3641 * 16, &i2s_bytes_write, portMAX_DELAY);
	//printf("dingdong----2\n");
	free(samples_data);

#endif
}
/*
* Brief:
* 	play "bianfa advise" voice 
*/
void voice_play(void)
{
#if 0 
    	uint8_t *samples_data = malloc(5920 * 16);
    	uint8_t *samples_data1 = malloc(5920 * 16);
    	size_t i2s_bytes_write = 0;
    	memset(samples_data, 0, 5920*16);
    	memset(samples_data1, 0, 5920*16);
    	memcpy(samples_data, pcm_sample_16k, 5920 * 16);
    	memcpy(samples_data1, pcm_sample_16k+5920 * 16, 5920 * 16);
    	i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2); 
    	i2s_write(I2S_NUM, samples_data, 5920 * 16, &i2s_bytes_write, portMAX_DELAY);
    	i2s_write(I2S_NUM, samples_data1, 5920 * 16, &i2s_bytes_write, portMAX_DELAY);
    	free(samples_data);
    	free(samples_data1);
#else
    	uint8_t *samples_data = malloc(5920 * 16);
    	size_t i2s_bytes_write = 0;
    	memset(samples_data, 0, 5920*16);
    	memcpy(samples_data, pcm_sample_16k, 5920 * 16);
    	i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2); 
    	i2s_write(I2S_NUM, samples_data, 5920 * 16, &i2s_bytes_write, portMAX_DELAY);
    	memcpy(samples_data, pcm_sample_16k+5920 * 16, 5920 * 16);
    	i2s_write(I2S_NUM, samples_data, 5920 * 16, &i2s_bytes_write, portMAX_DELAY);
    	free(samples_data);
#endif
}
/*
* Initialize I2S for 16kHz sample rate and 16bit data length
* set I2S IO port, Enable clock
* 
*/
void i2s_setup(void)
{
    	i2s_config_t i2s_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX,                                 
		.sample_rate = SAMPLE_RATE,
		.bits_per_sample = 16,
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           
		.communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
		.dma_buf_count = 6,
		.dma_buf_len = 60,
		.use_apll = false,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                
    	};
    	i2s_pin_config_t pin_config = {
		.bck_io_num = I2S_BCK_IO,
		.ws_io_num = I2S_WS_IO,
		.data_out_num = I2S_DO_IO,
		.data_in_num = I2S_DI_IO                                               
 	};
    	i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    	i2s_set_pin(I2S_NUM, &pin_config);
    	WRITE_PERI_REG(PIN_CTRL,(CLK_OUT3<<CLK_OUT3_S) | (CLK_OUT2 << CLK_OUT2_S) | (0 << CLK_OUT1_S) );
    	PIN_INPUT_DISABLE(PERIPHS_IO_MUX_GPIO0_U);
    	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
}
void perform_uart_command(void)
{

	esp_err_t err;
	switch(uart_priv.uart_cmd) {
		case 0:			//BFSET COMMAND
			switch(uart_priv.uart_params)
			{
				case 0:
					printf("BFSET -N %ld!\n", uart_priv.data);
					xSemaphoreTake(xLIGHTNUM_Semaphore, portMAX_DELAY);
    					myprivate.light_num = (uint8_t)uart_priv.data;
    					xSemaphoreGive(xLIGHTNUM_Semaphore);
    					printf("Updating light number in NVS Flash................\n");
    					int8_t lightnum = (int8_t)myprivate.light_num;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle);
      					err = nvs_set_i8(my_nvs_handle, "light_number", lightnum);
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      					nvs_close(my_nvs_handle);
					break;
				case 1:
					printf("BFSET -B %ld!\n", uart_priv.data);
					xSemaphoreTake(xLIGHTNUM_Semaphore, portMAX_DELAY);
    					myprivate.brightness = (uint8_t)uart_priv.data;
    					xSemaphoreGive(xLIGHTNUM_Semaphore);	
    					printf("Updating brightness in NVS Flash................\n");
    					int8_t brightness = (int8_t)myprivate.brightness;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle);    						
      					err = nvs_set_i8(my_nvs_handle, "brightness", brightness);  
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      					nvs_close(my_nvs_handle);												
					break;
				case 2:
					printf("BFSET -F %ld!\n", uart_priv.data);
					xSemaphoreTake(xLIGHTNUM_Semaphore, portMAX_DELAY);
    					myprivate.frequency = (uint8_t)uart_priv.data;
    					xSemaphoreGive(xLIGHTNUM_Semaphore);
    					printf("Updating refresh frequency in NVS Flash................\n");
    					int8_t frequency = (int8_t)myprivate.frequency;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle); 
      					err = nvs_set_i8(my_nvs_handle, "frequency", frequency);   
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      					nvs_close(my_nvs_handle);		  						
					break;
				case 3: 
					printf("BFSET -D %ld!\n", uart_priv.data);
					xSemaphoreTake(xDoorDly_Semaphore, portMAX_DELAY);
    					myprivate.door_delay = (uint16_t)uart_priv.data;
    					xSemaphoreGive(xDoorDly_Semaphore);
    					printf("Updating door delay time in NVS Flash................\n");
    					int16_t door_delay = (int16_t)myprivate.door_delay;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle); 
      					err = nvs_set_i16(my_nvs_handle, "door_delay", door_delay);   
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      					nvs_close(my_nvs_handle);	    						
					break;
				case 5: 
					printf("BFSET -T %ld!\n", uart_priv.data);
					xSemaphoreTake(xLIGHTNUM_Semaphore, portMAX_DELAY);
    					myprivate.led_wait = (uint16_t)uart_priv.data;
    					xSemaphoreGive(xLIGHTNUM_Semaphore);
    					printf("Updating led wait time in NVS Flash................\n");
    					int16_t led_wait = (int16_t)myprivate.led_wait;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle); 
      					err = nvs_set_i16(my_nvs_handle, "led_wait", led_wait);   
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      					nvs_close(my_nvs_handle);	    						
					break;								
				case 6: 
					printf("BFSET -V %ld!\n", uart_priv.data);
					xSemaphoreTake(xVoiceIndex_Semaphore, portMAX_DELAY);
    					myprivate.voice_index = (uint8_t)uart_priv.data;
    					xSemaphoreGive(xVoiceIndex_Semaphore);
    					printf("Updating voice index in NVS Flash................\n");
    					int8_t voice_index = (int8_t)myprivate.voice_index;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle); 
      					err = nvs_set_i8(my_nvs_handle, "voice_index", voice_index);   
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      					nvs_close(my_nvs_handle);										
					break;			
				case 7: 
					printf("BFSET -O %ld!\n", uart_priv.data);
					xSemaphoreTake(xDoorDly_Semaphore, portMAX_DELAY);
    					myprivate.voice_num = (uint8_t)uart_priv.data;
    					xSemaphoreGive(xDoorDly_Semaphore);
    					printf("Updating voice number in NVS Flash................\n");
    					int8_t voice_num = (int8_t)myprivate.voice_num;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle); 
      					err = nvs_set_i8(my_nvs_handle, "voice_num", voice_num);   
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      					nvs_close(my_nvs_handle);
					break;
				case 8: 
					printf("BFSET -W %ld!\n", uart_priv.data);
					xSemaphoreTake(xVoicePlay_Semaphore, portMAX_DELAY);
    					myprivate.voice_delay = (uint16_t)uart_priv.data;
    					xSemaphoreGive(xVoicePlay_Semaphore);
    					printf("Updating voice delay time in NVS Flash................\n");
    					int16_t voice_delay = (int16_t)myprivate.voice_delay;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle); 
      					err = nvs_set_i16(my_nvs_handle, "voice_delay", voice_delay);   
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      					nvs_close(my_nvs_handle);	
					break;
				case 9: 
					printf("BFSET -M %ld!\n", uart_priv.data);
					xSemaphoreTake(xLIGHTNUM_Semaphore, portMAX_DELAY);
    					myprivate.work_mode = (uint8_t)uart_priv.data;
    					xSemaphoreGive(xLIGHTNUM_Semaphore);
    					printf("Set Work Mode into NVS Flash................\n");
    					int8_t lightdiv = (int8_t)myprivate.work_mode;
    					err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle); 
      					err = nvs_set_i8(my_nvs_handle, "lightdivide", lightdiv);   
      					printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      					nvs_close(my_nvs_handle);	
					break;
				default: 
					break;
			}				
			uart_priv.uart_cmd = 255;
			break;
		case 1:			//BFADV -L COMMAND
			printf("BFADV -L %ld!\n", uart_priv.data);
			break;
		case 2: //BFTST -T COMMAND
			if(uart_priv.uart_params ==  5) {
				printf("BFTST -T %ld!\n", uart_priv.data);
				switch(uart_priv.data)
				{
					case 0:
						xSemaphoreGive(xElevator_UpArrival_Semaphore);
						break;
			  		case 1:
						xSemaphoreGive(xElevator_DnArrival_Semaphore);
						break;
					case 2:
						xSemaphoreGive(xVoicePlay_Semaphore);
						break;
					default:
						break;
				}
			}
			uart_priv.uart_cmd = 255;
			break;
		case 3:  //BFGET COMMAND
			err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle);  
			int8_t temp;			
			switch(uart_priv.uart_params)
			{
				case 0:
					err = nvs_get_i8(my_nvs_handle, "light_number", &temp);
					break;
				case 1:  					
					err = nvs_get_i8(my_nvs_handle, "brightness", &temp);
					break;
				case 2:
					err = nvs_get_i8(my_nvs_handle, "frequency", &temp);
					break;
				case 3: 
					err = nvs_get_i8(my_nvs_handle, "door_delay", &temp);
					break;
				case 5: 	
					err = nvs_get_i8(my_nvs_handle, "led_wait", &temp);
					break;								
				case 6: 		
					err = nvs_get_i8(my_nvs_handle, "voice_index", &temp);
					break;			
				case 7: 
					err = nvs_get_i8(my_nvs_handle, "voice_num", &temp);
					break;
				case 8:       					
					err = nvs_get_i8(my_nvs_handle, "voice_delay", &temp);
					break;
				case 9:       					
					err = nvs_get_i8(my_nvs_handle, "lightdivide", &temp);
					break;	
		  		default:
		  			break;				
		  	}
			switch (err) 
			{
  				case ESP_OK:
    					printf("\n %d \n", (uint8_t)temp);
      					break;
    				case ESP_ERR_NVS_NOT_FOUND:
      					printf("BFGET failed to read!\n");
      					break;
    				default :
      					printf("Error (%s) reading!\n", esp_err_to_name(err));
      					break;
  			}		
  			nvs_close(my_nvs_handle);
			uart_priv.uart_cmd = 255;
			break;
		default:
			uart_priv.uart_cmd = 255;
			break;
	}
}
/*
*	echo_task
*	get the uart command, check and perform uart command
*/
void echo_task(void* arg)
{ 
	uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
  	while(1) {
		//printf("echo_task().........\n");
		if(xSemaphoreTake(xUART0_Semaphore, portMAX_DELAY)) {
      			int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE, 20/  portTICK_RATE_MS);
			int command_delimiter,index;		
			char *command_buf = (char*)malloc(10);
			char *params_buf = (char*)malloc(10);
			
			int uart_command_num = 0;
			int uart_params_num = 0;

			if(len > BUF_SIZE || len == 0) {
				if(len)
					printf("command is too long!!, command length = %d\n", len);
				else
					printf("no command!!, command length = %d\n", len);
			}
			else
			{
				if(uart_priv.uart_cmd == 1)
				{
					printf("Receiving advertisement data, uart_priv.data = %ld, length = %d\n", 
									uart_priv.data, len);
					uart_priv.data -= len;
					if(uart_priv.data <= 0) {
						uart_priv.uart_cmd = 255;	
						printf("Received all advertisement data, Done!\n");
					}
				}
				if(uart_priv.uart_cmd == 255) 
				{
					command_delimiter = 0;
					/*
					*	To get the position of command delimiter
					*	copy the command string
					*/
					for(index = 0; index < len; index++) {	
						if(*(data + index) == ' ') {
							break;
						}
						*(command_buf+index) = *(data + index);
					}
					*(command_buf + index) = '\0';	//command EOF 
	
					/*
					*	copy the argument of command
					*/
					//value_len = 0;
					index++;
					command_delimiter =  index;
					for(; index < len; index++) {
						if(*(data + index) == ' ' || (*(data + index) == 0x0d && *(data + index + 1) == 0x0a)) {
							break;
						}
						*(params_buf + index - command_delimiter) = *(data + index);
					}
					*(params_buf + index - command_delimiter) = '\0'; //argu EOF 
					index++;
					uart_priv.data = 0;
					for(; index < len; index++) {
						if((*(data + index) == ' ')|| (*(data + index) == 0x0d && *(data + index + 1) == 0x0a)) {
							break;
						}
						printf("uart received data =  %x\n", *(data + index));
						uart_priv.data = uart_priv.data * 10;
						uart_priv.data += *(data + index)-0x30;
					}					
					 
					/*
					*	To check the command type
					*/
					index = 0;
					uart_priv.uart_cmd =  255;
					while((uart_command[index].cmd) != NULL) {
						if(strcmp_private(command_buf, uart_command[index].cmd)) {
							index++;
						}
						else
						{	
							command_delimiter = 0;
							while(*(uart_command[index].num + command_delimiter)) {
								uart_command_num = uart_command_num * 10 + 
									  *(uart_command[index].num + command_delimiter) - 0x30;
								command_delimiter++;
							}
							uart_priv.uart_cmd = uart_command_num;
							break;
						}
					}
					/*
					*	To check the parameters type
					*/
					index = 0;
					while((uart_params[index].cmd) != NULL) {
						if(strcmp_private(params_buf, uart_params[index].cmd)) {
							index++;
						}
						else
						{	
							command_delimiter = 0;
							while(*(uart_params[index].num + command_delimiter)) {
								uart_params_num = uart_params_num * 10 + 
									  *(uart_params[index].num + command_delimiter) - 0x30;
								command_delimiter++;
							}
							uart_priv.uart_params = uart_params_num;
							break;
						}
					}					
					/*
					*	To perform the uart command
					*/
					perform_uart_command();
				}
			}
			for(index = 0; index < len; index++) {
				*(data + index) = 0;
			}
			free(params_buf);
			free(command_buf);
		}
		vTaskDelay(100/portTICK_RATE_MS);
  	}
}
/*
* Brief:
* 	uart reveive and transmit data
*/
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    xUART0_Semaphore = xSemaphoreCreateMutex();
    for(;;) {
        //Waiting for UART event.
	//printf("uart_event_task().............\n");
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.
                in this example, we don't process data in event, but read data outside.*/
                case UART_DATA:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
										xSemaphoreGive(xUART0_Semaphore);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(MY_UART_TAG, "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(EX_UART_NUM);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(MY_UART_TAG, "ring buffer full\n");
                    //If buffer full happened, you should consider encreasing your buffer size
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(EX_UART_NUM);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(MY_UART_TAG, "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(MY_UART_TAG, "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(MY_UART_TAG, "uart frame error\n");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(MY_UART_TAG, "uart pattern detected\n");
                    break;
                //Others
                default:
                    ESP_LOGI(MY_UART_TAG, "uart event type: %d\n", event.type);
                    break;
            }
        }
		vTaskDelay(10/portTICK_RATE_MS);
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}	
/*
* Brief:
* 	when semaphore is activity, Play the voice via I2S
*/
static void task_voice_play(void* arg)
{
	for(;;)
	{
	//	printf("task_voice_play().......\n");
		if(xSemaphoreTake(xVoicePlay_Semaphore, 0)) 
		{
			xSemaphoreTake(xVoiceIndex_Semaphore,portMAX_DELAY);
			uint8_t voice_index = myprivate.voice_index;
			uint16_t voice_delay = myprivate.voice_delay;
			xSemaphoreGive(xVoiceIndex_Semaphore);
			gpio_set_level(GPIO_PA_EN, 1);
			vTaskDelay(30 / portTICK_RATE_MS);
			if(voice_index == 0){
				voice_play();
			}
			else {
				dingdong_play();
			}
			gpio_set_level(GPIO_PA_EN, 0);
			vTaskDelay(voice_delay / portTICK_RATE_MS);
		}
		vTaskDelay(30 / portTICK_RATE_MS);
	}
}
/*
* Brief:
*	detect the up/dn arrival of elevator 1, and send signal to led task and voice task.
*	waiting until the led task is over
*/
static void task_elevator_1_arrival(void* arg)
{
    uint8_t timecnt_up = 0;
    uint8_t timecnt_dn = 0;
    uint16_t timecnt_up_hld = 0;
    uint16_t timecnt_dn_hld = 0;
    uint8_t voice_start_u = 0;
    uint8_t voice_start_d = 0;
    uint16_t doordlytime;
    uint8_t voice_num = 0;
    for(;;) {
	  //  	printf("task_elevator_1_arrival()............\n");
    		xSemaphoreTake(xDoorDly_Semaphore, portMAX_DELAY);
    		doordlytime = myprivate.door_delay;
    		voice_num = myprivate.voice_num;
    		xSemaphoreGive(xDoorDly_Semaphore);
#if 0
		while(1){
    			if((gpio_get_level(GPIO_DT1_UP) == 0) || (gpio_get_level(GPIO_DT2_UP) == 0) ) {
    				timecnt_up++;
    				if(timecnt_up ==10) break;
    			}
    			else {
    				if(timecnt_up_hld < doordlytime) {
    					timecnt_up_hld++;					
    				}
    				else {
    					timecnt_up = 0;
    					voice_start_u = 0;
    				}
    			} 
    			if((gpio_get_level(GPIO_DT1_DN) ==0 ) || (gpio_get_level(GPIO_DT2_DN) ==0 )){
    				timecnt_dn++;
    				if(timecnt_dn==10) break;
    			}
    			else {
    				if(timecnt_dn_hld < doordlytime) {
    					timecnt_dn_hld++;
    				}
    				else {
    					timecnt_dn = 0;
    					voice_start_d = 0;
    				}
    			}
    			vTaskDelay(10 / portTICK_RATE_MS);
    		}
    		if(timecnt_up == 10) {
    			timecnt_up = 0;
    			timecnt_up_hld = 0;
    			xSemaphoreGive(xElevator_UpArrival_Semaphore);
    			vTaskDelay(10 / portTICK_RATE_MS);
    			if(voice_start_u < voice_num) {
    				xSemaphoreGive(xVoicePlay_Semaphore);
    				voice_start_u++;
    			}
    			xSemaphoreTake(xLED_Flash_Semaphore, portMAX_DELAY);
    			vTaskDelay(10 / portTICK_RATE_MS);
    		}
    		else {
    			if(timecnt_dn == 10) {
    				timecnt_dn = 0;
    				timecnt_dn_hld = 0;
    				xSemaphoreGive(xElevator_DnArrival_Semaphore);
    				vTaskDelay(10 / portTICK_RATE_MS);
    				if(voice_start_d < voice_num) {
    					xSemaphoreGive(xVoicePlay_Semaphore);
    					voice_start_d++;
    				}
    				xSemaphoreTake(xLED_Flash_Semaphore, portMAX_DELAY);
    				vTaskDelay(10 / portTICK_RATE_MS);
    			}
    		}
#endif
    		vTaskDelay(10 / portTICK_RATE_MS);
    }
}
void turnoff_led(uint16_t startpos, uint16_t light_num) 
{
	spi_transaction_t t, t_1;
	uint8_t tmp[8], tmp_1[8];
	uint16_t index, indey;
	esp_err_t ret;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length= 8 *  8;                 //Len is in bytes, transaction length is in bits.
	memset(&t_1, 0, sizeof(t_1));       //Zero out the transaction
	t_1.length= 8 *  8;                 //Len is in bytes, transaction length is in bits.
	for(index = startpos; index < 3 * light_num; index++) 
	{
		for(indey = 0; indey < 8; indey++) 
		{
			tmp[indey] = 0x80;
			tmp_1[indey] = 0x80;
		}
		t.tx_buffer=&tmp;               //Data
		t_1.tx_buffer=&tmp_1;               //Data
    		ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    		ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    		assert(ret==ESP_OK); 			 
	}
}
void led1_flash(void)
{
	uint16_t index, cnt, indey;
	esp_err_t ret;
	spi_transaction_t t, t_1;
	uint8_t tmp[8], tmp_1[8];
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length= 8 *  8;                 //Len is in bytes, transaction length is in bits.
	memset(&t_1, 0, sizeof(t_1));       //Zero out the transaction
	t_1.length= 8 *  8;                 //Len is in bytes, transaction length is in bits.
	uint8_t light_num, brightness, frequency, divider;
	uint16_t led_wait;
    
	if(xSemaphoreTake(xElevator_UpArrival_Semaphore, 0)) 
	{
		xSemaphoreTake(xLIGHTNUM_Semaphore, 0);
	    	light_num = myprivate.light_num;
	    	brightness = myprivate.brightness;
	    	frequency = myprivate.frequency;
	    	led_wait = myprivate.led_wait;
	    	divider = myprivate.work_mode;
	    	xSemaphoreGive(xLIGHTNUM_Semaphore);
	    	printf("Elevator go up,  light_num = %d, brightness = %d, frequency = %d, divider = %d\n",
    					light_num, brightness, frequency, divider);

	    	switch(divider) {
			case 0: //default mode
			case 1:
#if 0
				for(index = 0; index < 3 * light_num; index++) 
				{
					for(indey = 0; indey < 8; indey++) 
					{
						tmp[indey] = 0x80;
						tmp_1[indey] = 0x80;
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;               //Data
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    					assert(ret==ESP_OK); 			 
				}
#endif
				turnoff_led(0, light_num);
				vTaskDelay(50 / portTICK_RATE_MS);
				for(index = 3 * light_num; index > 0; index--) 
				{
					for(cnt = 0; cnt < 3 * light_num - index + 1; cnt++)
					{
	    					uint8_t brtmp = brightness;
		    				for(indey = 0; indey < 8; indey++) 
		    				{ 
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
							//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1);
    						assert(ret==ESP_OK); 
	    				}
#if 0
	   				for(; cnt < 3 * light_num; cnt++) 
	   				{	
		    				for(indey = 0; indey < 8; indey++) 
		    				{
		    					tmp[indey]=0x80;
		    					tmp_1[indey]=0x80;
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 		    	
	    				}	 
#endif
					turnoff_led(cnt, light_num);

					vTaskDelay((1000/frequency) / portTICK_RATE_MS);						
				}	
				if(divider == 1) {
	    				vTaskDelay((1000) / portTICK_RATE_MS);
#if 0
					for(index = 0; index < 3 * light_num; index++) 
					{
						for(indey = 0; indey < 8; indey++) 
						{
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;               //Data
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    						assert(ret==ESP_OK); 			 
					}
#endif
					turnoff_led(0, light_num);
				}
			break;			
			case 2:    //led on in turns
			case 3:
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{
						if(divider == 2) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;               //Data
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    					assert(ret==ESP_OK); 			 
				}
				vTaskDelay(50 / portTICK_RATE_MS);					
				for(index = 3 * light_num; index > 0; index--) 
				{
					if(index > 3 * light_num / 2 ) {
	    					for(cnt = 0; cnt < 3 * light_num - index + 1; cnt++)
	    					{	
	    						uint8_t brtmp = brightness;
		    					for(indey = 0; indey < 8; indey++) 
		    					{ 
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}
							//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    					}
		    					t.tx_buffer=&tmp;               //Data
		    					t_1.tx_buffer=&tmp_1; 
    							ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    							ret=spi_device_polling_transmit(spi_1, &t_1); 
    							assert(ret==ESP_OK); 
	    					}
#if 0
	   					for(; cnt < 3 * light_num / 2; cnt++) 
	   					{
		    					for(indey = 0; indey < 8; indey++) 
		    					{
		    						tmp[indey]=0x80;
		    						tmp_1[indey]=0x80;
		    					}
		    					t.tx_buffer=&tmp;               //Data
		    					t_1.tx_buffer=&tmp_1; 
    							ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    							ret=spi_device_polling_transmit(spi_1, &t_1); 
    							assert(ret==ESP_OK); 		    	
	    					}
#endif
						turnoff_led(cnt, light_num/2);
					}		
	    				else {
	    					for(cnt = 0; cnt < 3 * light_num / 2; cnt++)
	    					{
	    						uint8_t brtmp = brightness;
		    					for(indey = 0; indey < 8; indey++) 
		    					{ 
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}
								//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    					}
		    					t.tx_buffer=&tmp;               //Data
		    					t_1.tx_buffer=&tmp_1;
    							ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    							ret=spi_device_polling_transmit(spi_1, &t_1);
    							assert(ret==ESP_OK); 
	    					}	    				
	    				}
   					for(cnt = 3 * light_num / 2; cnt < 3 * light_num; cnt++) 
   					{			
   						uint8_t brtmp = brightness;
	    					for(indey = 0; indey < 8; indey++) 
	    					{
							if(divider == 2) {
	    							if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
	    					}
	    					t.tx_buffer=&tmp;               //Data
	    					t_1.tx_buffer=&tmp_1; 
						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
   						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 		    	
					}  
	    				vTaskDelay((1000/frequency) / portTICK_RATE_MS);						
				}
				if(divider == 3) {
	    				vTaskDelay((1000) / portTICK_RATE_MS);
#if 0
					for(index = 0; index < 3 * light_num; index++) 
					{
		    				for(indey = 0; indey < 8; indey++) 
		    				{ 
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;               //Data
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    						assert(ret==ESP_OK); 			 
					}
#endif
					turnoff_led(0, light_num);
				}
			break;
			case 4:
			case 5:
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{
						if(divider == 4) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;               //Data
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    					assert(ret==ESP_OK); 			 
				}
				vTaskDelay(50 / portTICK_RATE_MS);					
				for(index = 3 * light_num; index > 0; index--) 
				{
					if(index > 3 * light_num / 2 ) {
	    					for(cnt = 3 * light_num; cnt > 3 * light_num - index + 1 ; cnt--)
	    					{	
		    					for(indey = 0; indey < 8; indey++) 
		    					{ 
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
		    					}
		    					t.tx_buffer=&tmp;               //Data
		    					t_1.tx_buffer=&tmp_1; 
    							ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    							ret=spi_device_polling_transmit(spi_1, &t_1); 
    							assert(ret==ESP_OK); 
	    					}
	   					for(; cnt > 0; cnt--) 
	   					{
	    						uint8_t brtmp = brightness;
		    					for(indey = 0; indey < 8; indey++) 
		    					{
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}
		    					}
		    					t.tx_buffer=&tmp;               //Data
		    					t_1.tx_buffer=&tmp_1; 
    							ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    							ret=spi_device_polling_transmit(spi_1, &t_1); 
    							assert(ret==ESP_OK); 		    	
	    					}
	    				}		
	    				else {
	    					for(cnt = 0; cnt < 3 * light_num / 2; cnt++)
	    					{
	    						uint8_t brtmp = brightness;
		    					for(indey = 0; indey < 8; indey++) 
		    					{ 
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}
								//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    					}
		    					t.tx_buffer=&tmp;               //Data
		    					t_1.tx_buffer=&tmp_1;
    							ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    							ret=spi_device_polling_transmit(spi_1, &t_1);
    							assert(ret==ESP_OK); 
	    					}	    				
	    				}
   					for(cnt = 3 * light_num / 2; cnt < 3 * light_num; cnt++) 
   					{			
   						uint8_t brtmp = brightness;
	    					for(indey = 0; indey < 8; indey++) 
	    					{
							if(divider == 4) {
	    							if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
	    					}
	    					t.tx_buffer=&tmp;               //Data
	    					t_1.tx_buffer=&tmp_1; 
						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
   						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 		    	
					}  
	    				vTaskDelay((1000/frequency) / portTICK_RATE_MS);						
				}
				if(divider == 5) {
	    				vTaskDelay((1000) / portTICK_RATE_MS);
#if 0
					for(index = 0; index < 3 * light_num; index++) 
					{
		    				for(indey = 0; indey < 8; indey++) 
		    				{ 
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;               //Data
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    						assert(ret==ESP_OK); 			 
					}
#endif
					turnoff_led(0, light_num);
				}
			
			break;
			case 6:    //shine 10 times
			case 7:
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{
						if(divider == 6) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
						//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    			}
		    			
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;               //Data
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    					assert(ret==ESP_OK); 			 
				}
				vTaskDelay(50 / portTICK_RATE_MS);					
				//for(index = 0; index < 10; index++) 
				{
	    				for(cnt = 0; cnt < 3 * light_num / 2; cnt++)
	    				{
	    					uint8_t brtmp = brightness;
		    				for(indey = 0; indey < 8; indey++) 
		    				{ 
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
							//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 
	    				}
	   				for(; cnt < 3 * light_num; cnt++) 
	   				{
	    					uint8_t brtmp = brightness;
		    				for(indey = 0; indey < 8; indey++) 
		    				{
							if(divider == 6) {
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
							//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 		    	
	    				}
	    				vTaskDelay((1000/frequency) / portTICK_RATE_MS);
#if 0
	    				for(cnt = 0; cnt < 3 * light_num / 2; cnt++)
	    				{
		    				for(indey = 0; indey < 8; indey++) 
		    				{ 
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 
	    				}
#endif
					turnoff_led(0, light_num / 2);
					for(; cnt < 3 * light_num; cnt++) 
	   				{
	    					uint8_t brtmp = brightness;
		    				for(indey = 0; indey < 8; indey++) 
		    				{
							if(divider == 6) {
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 		    	
	    				}	
	    				vTaskDelay((1000/frequency) / portTICK_RATE_MS);    				
	    			}	
	   			//	vTaskDelay((1000) / portTICK_RATE_MS);  														
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{
						if(divider == 6) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
						//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    			}
		    			
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;               //Data
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    					assert(ret==ESP_OK); 			 
				}	
			break;
			case 8:
			case 9:
	    			for(cnt = 0; cnt < 3 * light_num / 2; cnt++)
	    			{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{
						if(divider == 8) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
						//tmp[indey] = 0x80;
						//tmp_1[indey] = 0x80;
		    			}
		    			t.tx_buffer=&tmp;               //Data
		    			t_1.tx_buffer=&tmp_1; 
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 
	    			}
	   			for(; cnt < 3 * light_num; cnt++) 
	   			{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{
						if(divider == 9) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
		    			}
		    			t.tx_buffer=&tmp;               //Data
		    			t_1.tx_buffer=&tmp_1; 
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 		    	
	    			}	

			break;
			default:				
				break;
		}
		vTaskDelay(led_wait / portTICK_RATE_MS);	
		xSemaphoreGive(xLED_Flash_Semaphore);		
		printf("Elevator go up, Done!\n");
	}


	if(xSemaphoreTake(xElevator_DnArrival_Semaphore, 0)) 
	{
		xSemaphoreTake(xLIGHTNUM_Semaphore, 0);
		light_num = myprivate.light_num;
    		brightness = myprivate.brightness;
    		frequency = myprivate.frequency;
    		led_wait = myprivate.led_wait;
    		divider = myprivate.work_mode;
    		xSemaphoreGive(xLIGHTNUM_Semaphore);		
    		printf("Elevator go down,  light_num = %d, brightness = %d, frequency = %d, divider = %d\n",
    					light_num, brightness, frequency, divider);   
    		switch(divider)
    		{
    			case 0: //default
			case 1:
#if 0
				for(index = 0; index < 3 * light_num; index++) 
				{
					for(indey = 0; indey < 8; indey++) 
					{
    						tmp[indey] = 0x80;  			
    						tmp_1[indey] = 0x80; 
		  			}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;  
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 		  	
	  			}	
#endif
				turnoff_led(0, light_num);
				vTaskDelay(50 / portTICK_RATE_MS); 
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				for(cnt = 0; cnt < 3 * light_num - index - 1; cnt++)
	   			 	{
						for(indey = 0; indey < 8; indey++) 
						{
		 					tmp[indey]=0x80;
		 					tmp_1[indey]=0x80;
	    					}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 	    			
					}
	    				for(; cnt < 3 * light_num; cnt++) 
	    				{
	    					uint8_t brtmp = brightness;
						for(indey = 0; indey < 8; indey++) 
						{
							//printf("brightness = %x\n", brightness);
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}	
							//printf("tmp[%d] = %x\n", indey, tmp[indey]);
	    					}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1);  
    						assert(ret==ESP_OK); 	    			
	    				}	  
	    				vTaskDelay((1000/frequency) / portTICK_RATE_MS);						
				}
				if(divider == 1) {
	    				vTaskDelay((1000) / portTICK_RATE_MS);	
#if 0
					for(index = 0; index < 3 * light_num; index++) 
					{
						for(indey = 0; indey < 8; indey++) 
						{
    							tmp[indey] = 0x80;  			
    							tmp_1[indey] = 0x80; 
		  				}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;  
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 		  
					}
#endif
					turnoff_led(0, light_num);
				}
    			break;
    			case 2:  //lights turn on in turns
			case 3:
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
					for(indey = 0; indey < 8; indey++) 
					{
						//printf("brightness = %x\n", brightness);
						if(divider == 2) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}	
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;  
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 		  	
	  			}	  
	  			vTaskDelay(50 / portTICK_RATE_MS);   			
				for(index = 3 * light_num; index > 0; index--) 
				{
	    				for(cnt = 0; cnt < 3 * light_num - index + 1; cnt++)
	   			 	{
	   			 		if(cnt < 3 * light_num / 2) {
	   			 			uint8_t brtmp = brightness;
							for(indey = 0; indey < 8; indey++) 
							{
								if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}	
							}
	    					}
	    					else {
							for(indey = 0; indey < 8; indey++) 
							{
		 						tmp[indey]=0x80;
		 						tmp_1[indey]=0x80;
	    						}	    				
	    					}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 	    			
	    				}
#if 0					
	    				for(; cnt < 3 * light_num/2; cnt++) 
	    				{
	    					uint8_t brtmp = brightness;
						for(indey = 0; indey < 8; indey++) 
						{
							//printf("brightness = %x\n", brightness);
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
	    					}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1);
    						assert(ret==ESP_OK);
	    				}
#endif
					turnoff_led(cnt, light_num/2);
	    				for(; cnt < 3 * light_num; cnt++) 
	    				{
	    					uint8_t brtmp = brightness;
						for(indey = 0; indey < 8; indey++) 
						{
							//printf("brightness = %x\n", brightness);
							if(divider == 2) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}	
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
							//printf("tmp[%d] = %x\n", indey, tmp[indey]);
	    			}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1);
    						assert(ret==ESP_OK); 	    			
	    				}  
	    				vTaskDelay((1000/frequency) / portTICK_RATE_MS);						
				}    			
	    			vTaskDelay((1000) / portTICK_RATE_MS);						
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
					for(indey = 0; indey < 8; indey++) 
					{
						//printf("brightness = %x\n", brightness);
						if(divider == 2) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}	
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;  
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 			 
				}
    			break;
			case 4:
			case 5:
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
					for(indey = 0; indey < 8; indey++) 
					{
						if(divider == 4) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}	
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;  
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 		  	
	  			}	  
	  			vTaskDelay(50 / portTICK_RATE_MS);   			
				for(index = 0; index < 3 * light_num; index++) 
				{
					for(cnt =0; cnt < 3*light_num/2; cnt++)
					{
	   			 		uint8_t brtmp = brightness;
						for(indey = 0; indey < 8; indey++) 
						{
							if(divider == 4) {
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}	
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
	    					}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 	    			
					}
					for(cnt = 0; cnt < index + 1; cnt++) 
					{
	   			 		uint8_t brtmp = brightness;
						for(indey = 0; indey < 8; indey++) 
						{
							if(divider == 4) {
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}	
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
	    					}
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 	    			

					}
					for(cnt = 0; cnt < 3 * light_num /2 -index -1; cnt++)
					{
						for(indey = 0; indey < 8; indey++) 
						{
		 					tmp[indey]=0x80;
		 					tmp_1[indey]=0x80;
	    					}	    				
						t.tx_buffer=&tmp;               //Data
						t_1.tx_buffer=&tmp_1;
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 	    			
					}
	    				vTaskDelay((1000/frequency) / portTICK_RATE_MS);						
				}    			
	    			vTaskDelay((1000) / portTICK_RATE_MS);						
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
					for(indey = 0; indey < 8; indey++) 
					{
						//printf("brightness = %x\n", brightness);
						if(divider == 4) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}	
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;  
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 			 
				}
			break;
    			case 6:  //lights shine in 10 times
			case 7:
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{ 
						if(divider == 6) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}	
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;               //Data
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    					assert(ret==ESP_OK); 			 
				}
				vTaskDelay(50 / portTICK_RATE_MS);					
				//for(index = 0; index < 10; index++) 
				{
	    				for(cnt = 0; cnt < 3 * light_num / 2; cnt++)
	    				{
	    					uint8_t brtmp = brightness;
		    				for(indey = 0; indey < 8; indey++) 
		    				{ 
							if(divider == 6) {
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}	
							}	
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
							//printf("tmp[%d] = %x\n", indey, tmp[indey]);
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 
	    				}
	   				for(; cnt < 3 * light_num; cnt++) 
	   				{
	    					uint8_t brtmp = brightness;
		    				for(indey = 0; indey < 8; indey++) 
		    				{ 
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 		    	
	    				}
	    				vTaskDelay((1000/frequency) / portTICK_RATE_MS);	
	    				for(cnt = 0; cnt < 3 * light_num / 2; cnt++)
	    				{
	    					uint8_t brtmp = brightness;
		    				for(indey = 0; indey < 8; indey++) 
		    				{ 
							if(divider == 6) {
		    						if((brtmp << indey) & 0x80) {
									tmp[indey]=0xf0;
									tmp_1[indey]=0xf0;
								}
								else {
									tmp[indey] = 0x80;
									tmp_1[indey] = 0x80;
								}	
							}	
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 
	    				}
#if 0
	   				for(; cnt < 3 * light_num; cnt++) 
	   				{
		    				for(indey = 0; indey < 8; indey++) 
		    				{
		    					tmp[indey]=0x80;
		    					tmp_1[indey]=0x80;
		    				}
		    				t.tx_buffer=&tmp;               //Data
		    				t_1.tx_buffer=&tmp_1; 
    						ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    						ret=spi_device_polling_transmit(spi_1, &t_1); 
    						assert(ret==ESP_OK); 		    	
	    				}	
#endif
					turnoff_led(cnt, light_num);
					vTaskDelay((1000/frequency) / portTICK_RATE_MS);    				
	    			}	   	

	    			//vTaskDelay((1000) / portTICK_RATE_MS);    				
				for(index = 0; index < 3 * light_num; index++) 
				{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{ 
						if(divider == 6) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}	
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
					}
					t.tx_buffer=&tmp;               //Data
					t_1.tx_buffer=&tmp_1;               //Data
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1);  //Transmit!
    					assert(ret==ESP_OK);
				}
    			break;
			case 8:
			case 9:
	    			for(cnt = 0; cnt < 3 * light_num / 2; cnt++)
	    			{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{
						if(divider == 9) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
						//tmp[indey] = 0x80;
						//tmp_1[indey] = 0x80;
		    			}
		    			t.tx_buffer=&tmp;               //Data
		    			t_1.tx_buffer=&tmp_1; 
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 
	    			}
	   			for(; cnt < 3 * light_num; cnt++) 
	   			{
	    				uint8_t brtmp = brightness;
		    			for(indey = 0; indey < 8; indey++) 
		    			{
						if(divider == 8) {
		    					if((brtmp << indey) & 0x80) {
								tmp[indey]=0xf0;
								tmp_1[indey]=0xf0;
							}
							else {
								tmp[indey] = 0x80;
								tmp_1[indey] = 0x80;
							}
						}
						else {
							tmp[indey] = 0x80;
							tmp_1[indey] = 0x80;
						}
		    			}
		    			t.tx_buffer=&tmp;               //Data
		    			t_1.tx_buffer=&tmp_1; 
    					ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    					ret=spi_device_polling_transmit(spi_1, &t_1); 
    					assert(ret==ESP_OK); 		    	
	    			}	
			break;
    			default:
    			break;
    		
    		} 		
						
		vTaskDelay(led_wait / portTICK_RATE_MS);
		xSemaphoreGive(xLED_Flash_Semaphore);		
		printf("Elevator go down, Done!\n");
	}
}

static void task_led_flash(void* arg)
{
	for(;;)
	{
		//printf("task_led_flash().........\n");
		led1_flash();
  		vTaskDelay(10 / portTICK_RATE_MS);
	}
}
void app_main()
{
	uint16_t cnt, indey;
	gpio_config_t io_conf;
	esp_err_t ret;
	uint8_t tmp[8], tmp_1[8];
	spi_transaction_t t, t_1;
	esp_err_t err;
		
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
	
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE; //GPIO_PIN_INTR_POSEDGE;
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	
	WRITE_PERI_REG(PIN_CTRL,(CLK_OUT3<<CLK_OUT3_S) | (CLK_OUT2 << CLK_OUT2_S) | (0 << CLK_OUT1_S) );
	PIN_INPUT_DISABLE(PERIPHS_IO_MUX_GPIO0_U);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
		
	gpio_config(&io_conf);
	uart0_config();
	uart_priv.uart_cmd = 255;
	i2s_setup();
	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
	 	.sclk_io_num=PIN_NUM_CLK,
	 	.quadwp_io_num=-1,
	 	.quadhd_io_num=-1,
	 	.max_transfer_sz=96 * 8 *16
	};
	spi_bus_config_t buscfg_1={
		.miso_io_num=PIN_NUM_MISO_1,
	 	.mosi_io_num=PIN_NUM_MOSI_1,
	 	.sclk_io_num=PIN_NUM_CLK_1,
	 	.quadwp_io_num=-1,
	 	.quadhd_io_num=-1,
	 	.max_transfer_sz=96 * 8 *16
	};	

	spi_device_interface_config_t devcfg={
	 	.clock_speed_hz=3200000,           //Clock out at 3.2 MHz
	  	.mode=0,                                //SPI mode 0
	  	.spics_io_num=PIN_NUM_CS,               //CS pin
	  	.queue_size=7,                          //We want to be able to queue 7 transactions at a time
	};

	spi_device_interface_config_t devcfg_1={
	 	.clock_speed_hz=3200000,           //Clock out at 3.2 MHz
	  	.mode=0,                                //SPI mode 0
	  	.spics_io_num=PIN_NUM_CS_1,               //CS pin
	  	.queue_size=7,                          //We want to be able to queue 7 transactions at a time
	};	
	//Initialize the SPI bus
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	ESP_ERROR_CHECK(ret);
	ret=spi_bus_initialize(VSPI_HOST, &buscfg_1, 2);
	ESP_ERROR_CHECK(ret);
	
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ret=spi_bus_add_device(VSPI_HOST, &devcfg_1, &spi_1);
	printf("===================================================================================\n");  
	printf("                              Dian Ti Dao Zhan Deng                                \n");  
	printf("                              CopyRight@Bian Fa Ke Ji                              \n");
	printf("===================================================================================\n");  
  	myprivate.light_num  = 55;
  	myprivate.brightness = 255;
  	myprivate.frequency  = 60; 
  	myprivate.voice_index = 1;
  	myprivate.door_delay=3000;
  	myprivate.led_wait = 10;
  	myprivate.voice_num = 2;
  	myprivate.voice_delay = 5000;
  	myprivate.work_mode = 0;
	printf("Ready to init nvs flash!\n");
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		//  Retry nvs_flash_init
	     	printf("No room for NVS flash, erase all, and retry init nvs flash!\n");
	     	ESP_ERROR_CHECK(nvs_flash_erase());
	     	err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_nvs_handle);
	//if (err != ESP_OK) return err;
	int8_t lightnum = 0;
	int8_t brightness = 0;
	int8_t frequency  = 0;
	int8_t voice_index = 0;
	int16_t door_delay = 0;
	int16_t led_wait = 0;
	int8_t voice_num = 0;
	int16_t voice_delay = 0;
	int8_t divider = 0;
	printf("Ready to read the lignt number value from NVS Flash........................\n");
	err = nvs_get_i8(my_nvs_handle, "light_number", &lightnum);
  	switch (err) {
  		case ESP_OK:
    			printf("Light Number Read Done! lignt number = %d\n", (uint8_t)lightnum);
      			myprivate.light_num = lightnum;
      		break;
    		case ESP_ERR_NVS_NOT_FOUND:
			printf("The lignt number value is not initialized yet!\n");
			printf("The lignt number value is initializing now!\n");
			lightnum = myprivate.light_num;
			err = nvs_set_i8(my_nvs_handle, "light_number", lightnum);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
   		default :
			printf("Error (%s) reading!\n", esp_err_to_name(err));
		break;
  	}	
	printf("Ready to read the brightness value from NVS Flash........................\n");
	err = nvs_get_i8(my_nvs_handle, "brightness", &brightness);
  	switch (err) {
  		case ESP_OK:
    			printf("Brightness Read Done! brightness = %d\n", (uint8_t)brightness);
      			myprivate.brightness = brightness;
      		break;
    		case ESP_ERR_NVS_NOT_FOUND:
      			printf("The brightness value is not initialized yet!\n");
      			printf("The brightness value is initializing now!\n");
      			brightness = myprivate.brightness;
      			err = nvs_set_i8(my_nvs_handle, "brightness", brightness);
      			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
    		default :
      			printf("Error (%s) reading!\n", esp_err_to_name(err));
		break;
  	}	
  	printf("Ready to read the light refresh frequency value from NVS Flash........................\n");
	err = nvs_get_i8(my_nvs_handle, "frequency", &frequency);
  	switch (err) {
  		case ESP_OK:
    			printf("Frequency Read Done! frequency = %d\n", (uint8_t)frequency);
      			myprivate.frequency = frequency;
      		break;
    		case ESP_ERR_NVS_NOT_FOUND:
      			printf("The frequency value is not initialized yet!\n");
      			printf("The frequency value is initializing now!\n");
      			frequency = myprivate.frequency;
      			err = nvs_set_i8(my_nvs_handle, "frequency", frequency);
      			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
    		default :
      			printf("Error (%s) reading!\n", esp_err_to_name(err));
		break;
  	}	
  	printf("Ready to read the voice index from NVS Flash........................\n");
	err = nvs_get_i8(my_nvs_handle, "voice_index", &voice_index);
  	switch (err) {
  		case ESP_OK:
    			printf("Voice index Read Done! voice index = %d\n", (uint8_t)voice_index);
      			myprivate.voice_index = voice_index;
     		break;
    		case ESP_ERR_NVS_NOT_FOUND:
      			printf("The voice index is not initialized yet!\n");
      			printf("The voice index is initializing now!\n");
      			voice_index = myprivate.voice_index;
      			err = nvs_set_i8(my_nvs_handle, "voice_index", voice_index);
      			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
    		default :
      			printf("Error (%s) reading!\n", esp_err_to_name(err));
			break;
  	}	
  	printf("Ready to read the door delay time from NVS Flash........................\n");
	err = nvs_get_i16(my_nvs_handle, "door_delay", &door_delay);
  	switch (err) {
  		case ESP_OK:
    			printf("Door delay Read Done! door delay = %d\n", (uint16_t)door_delay);
      			myprivate.door_delay = door_delay;
      		break;
    		case ESP_ERR_NVS_NOT_FOUND:
      			printf("The door delay is not initialized yet!\n");
      			printf("The door delay is initializing now!\n");
      			door_delay = (int16_t)myprivate.door_delay;
      			err = nvs_set_i16(my_nvs_handle, "door_delay", door_delay);
      			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
    		default :
      			printf("Error (%s) reading!\n", esp_err_to_name(err));
			break;
  	}		
  	printf("Ready to read the led wait time from NVS Flash........................\n");
	err = nvs_get_i16(my_nvs_handle, "led_wait", &led_wait);
  	switch (err) {
  		case ESP_OK:
    			printf("Led wait Read Done! led wait = %d\n", (uint16_t)led_wait);
      			myprivate.led_wait = led_wait;
      		break;
    		case ESP_ERR_NVS_NOT_FOUND:
      			printf("The led wait is not initialized yet!\n");
      			printf("The led wait is initializing now!\n");
      			led_wait = (int16_t)myprivate.led_wait;
      			err = nvs_set_i16(my_nvs_handle, "led_wait", led_wait);
      			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
    		default :
      			printf("Error (%s) reading!\n", esp_err_to_name(err));
		break;
  	}		
  	printf("Ready to read the voice number from NVS Flash........................\n");
	err = nvs_get_i8(my_nvs_handle, "voice_num", &voice_num);
  	switch (err) {
  		case ESP_OK:
    			printf("voice number Read Done! voice number = %d\n", (uint8_t)voice_num);
      			myprivate.voice_num = voice_num;
      		break;
    		case ESP_ERR_NVS_NOT_FOUND:
      			printf("The voice number is not initialized yet!\n");
      			printf("The voice number is initializing now!\n");
      			voice_num = (int8_t)myprivate.voice_num;
      			err = nvs_set_i8(my_nvs_handle, "voice_num", voice_num);
      			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
    		default :
      			printf("Error (%s) reading!\n", esp_err_to_name(err));
		break;
  	}	   
  	printf("Ready to read the voice delay time from NVS Flash........................\n");
	err = nvs_get_i16(my_nvs_handle, "voice_delay", &voice_delay);
  	switch (err) {
  		case ESP_OK:
    			printf(" voice delay Read Done! voice delay = %d\n", (uint16_t)voice_delay);
      			myprivate.voice_delay = voice_delay;
      		break;
    		case ESP_ERR_NVS_NOT_FOUND:
      			printf("The voice delay is not initialized yet!\n");
      			printf("The voice delay is initializing now!\n");
      			voice_delay = (int16_t)myprivate.voice_delay;
      			err = nvs_set_i16(my_nvs_handle, "voice_delay", voice_delay);
      			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
    		default :
      			printf("Error (%s) reading!\n", esp_err_to_name(err));
		break;
  	}	 
  	printf("Ready to read the light divide mode from NVS Flash........................\n");
	err = nvs_get_i8(my_nvs_handle, "lightdivide", &divider);
  	switch (err) {
  		case ESP_OK:
    			printf("work mode Read Done! work mode = %d\n", (uint8_t)divider);
      			myprivate.work_mode = divider;
      		break;
    		case ESP_ERR_NVS_NOT_FOUND:
      			printf("The work mode  is not initialized yet!\n");
      			printf("The work mode  is initializing now!\n");
      			divider = (int8_t)myprivate.work_mode;
      			err = nvs_set_i8(my_nvs_handle, "lightdivide", divider);
      			printf((err != ESP_OK) ? "Failed!\n" : "Done\n"); 
      		break;
    		default :
     			 printf("Error (%s) reading!\n", esp_err_to_name(err));
		break;
  	}	  	    	
#if 0
	    // Read run time blob
  size_t required_size = 0;  // value will default to 0, if not set yet in NVS
  // obtain required memory space to store blob being read from NVS
  err = nvs_get_blob(my_nvs_handle, advertise_str[0], NULL, &required_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

  if (required_size == 0) {
      printf("Nothing advise data saved yet!, required_size = %d\n", required_size);
      printf("Ready to copy voice data from pcm_sample_16k[] to nvs flash!\n");
      uint8_t* run_time = malloc(1024);
      for(index =0; index < 640; index++) {
      	memset(run_time, 0, 1024);
      	for (uint32_t i = 0; i < 1024; i++) {
      			if(index * 1024 + i < VOICE_DATA_LENG) {
          		run_time[i] = pcm_sample_16k[index * 1024 + i];
          	}
          	else {        		
          		run_time[i] = 0;
          	}
      	} 
      	required_size = 1024; 
      	printf("Writing to advise data %d!\n", index); 
      	err = nvs_set_blob(my_nvs_handle, advertise_str[index], run_time, required_size); 

      	// Commit
    		err = nvs_commit(my_nvs_handle);
    		vTaskDelay(50 / portTICK_RATE_MS);
   			if (err != ESP_OK) return err;

#if 0
      	for(uint32_t p = 0; p < 1024; p++) {
          		printf("%d: %d\n", p, run_time[p]);
      	}
#endif  		       	
      }
      free(run_time);
  } else {
  	  printf("Advise data has been saved!\n");
  	  uint8_t* run_time = malloc(1024);
  	  for(index = 0; index < 640; index++) {
        err = nvs_get_blob(my_nvs_handle, advertise_str[index], run_time, &required_size);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
        if(required_size == 0) {
        		printf("Nothing Advise data has been saved!, index = %d, required_size = %d\n", 
        			index, required_size);
        }
        else {
        	printf("Advise data has been saved!, index = %d, required_size = %d\n", 
        			index, required_size);
        	printf("%s\n", advertise_str[index]);
#if 0
        	for(uint16_t p=0; p<1024; p++) {
        		if(p % 32 == 0) printf("\n\r");
        		printf(" %2x ", run_time[p]);
        	}
        	vTaskDelay(10 / portTICK_RATE_MS);
#endif
        }
      }
      free(run_time);
  }
#endif
	nvs_close(my_nvs_handle);
	
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	memset(&t_1, 0, sizeof(t_1));
	t.length= 8  *  8;  
	t_1.length= 8  *  8;  	
        
	if(myprivate.work_mode % 2 == 0) {	
		printf("turn on all the led lights\n");
		for(cnt = 0; cnt < 3 * myprivate.light_num; cnt++) 
		{
			uint8_t brtmp = myprivate.brightness;
			for(indey =0; indey < 8; indey++){
				//tmp[indey] = 0xf0;
				if((brtmp << indey) & 0x80) {
					tmp[indey]=0xf0;
					tmp_1[indey]=0xf0;
				}
				else {
					tmp[indey] = 0x80;
					tmp_1[indey] = 0x80;
				}	
			}
			t.tx_buffer=&tmp;
			t_1.tx_buffer=&tmp_1;
			ret=spi_device_polling_transmit(spi, &t);
			ret=spi_device_polling_transmit(spi_1, &t_1);
			assert(ret==ESP_OK);
		}	
	}
	else {
		printf("turn off all the led lights\n");
#if 0
		for(cnt = 0; cnt < 3 * myprivate.light_num; cnt++) 
		{
			for(indey =0; indey < 8; indey++)
			{
				tmp[indey] = 0x80;
				tmp_1[indey] = 0x80;
			}
			t.tx_buffer=&tmp;
			t_1.tx_buffer=&tmp_1;			
			ret = spi_device_polling_transmit(spi, &t);
			ret = spi_device_polling_transmit(spi_1, &t_1);
			assert(ret==ESP_OK);
		}	
#endif
		turnoff_led(0, myprivate.light_num);
	}
	bt_ssp_init();	
  	//create Semaphore and Mutex
	xElevator_UpArrival_Semaphore = xSemaphoreCreateBinary();
	xElevator_DnArrival_Semaphore = xSemaphoreCreateBinary();
	xLED_Flash_Semaphore =  xSemaphoreCreateBinary();
	xVoicePlay_Semaphore =  xSemaphoreCreateBinary();	
	
	xLIGHTNUM_Semaphore = xSemaphoreCreateMutex();	
	xVoiceIndex_Semaphore = xSemaphoreCreateMutex();	
	xDoorDly_Semaphore = xSemaphoreCreateMutex();	
	xLEDWait_Semaphore = xSemaphoreCreateMutex();	
		
	xTaskCreate(task_elevator_1_arrival, "elevator 1 Arrival", 4096, NULL, 10, NULL);
	xTaskCreate(task_led_flash, "led on", 4096, NULL, 11, NULL);
	xTaskCreate(task_voice_play, "voice play", 4096, NULL, 9, NULL);
	xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 8, NULL);
	xTaskCreate(echo_task, "echo_task", 4096, NULL, 7, NULL);
	while(1) 
	{
		printf("main().....waiting........\n");
	      	vTaskDelay(10000 / portTICK_RATE_MS);
	}	
}
