/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXCAMPLE_DEVICE_NAME "BF_CTL_SSCOM"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
//#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/
#define SPP_SHOW_MODE SPP_SHOW_DATA

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

uint8_t bt_start_flag = 0;

#define SPP_DATA_LEN 100
static uint8_t spp_data[SPP_DATA_LEN];
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
	uint16_t  datecode;
	uint16_t bledev_number;
} BF_PRIV;
extern BF_PRIV myprivate;
/*
* The Structor for uart command
*/
typedef struct {
        uint8_t uart_cmd;
  uint8_t uart_params;
  long int data;
} UART_PRIV;
extern UART_PRIV uart_priv;

/*
* UART COMMAND LIST
*/
typedef struct {
        const char *cmd;
        const char *num;
} uart_command_config;
extern const uart_command_config uart_command[]; 
extern const uart_command_config uart_params[];
extern int strcmp_private( char *,  const char *);
extern void perform_uart_command(void);

const uint8_t info[8] = {"success!"};

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	char *devname_profile = "BFBLE";
	char* devname = (char*)malloc(30);
	uint8_t index, tmp,tmp1,tmp2,tmp3,tmp4;
	switch (event) {
		case ESP_SPP_INIT_EVT:
			for(index = 0; ; index++) {
				if(*(devname_profile+index) != 0) {
					*(devname + index) = *(devname_profile + index);
					printf("*(devname + %d) = 0x%x\n", index, *(devname+index));
				}
				else {
					break;
				}
			}
                       	tmp = myprivate.datecode / 10000;		
			*(devname + index) = tmp + 0x30;
			index++;
			tmp1 = (myprivate.datecode - tmp * 10000) / 1000;
			*(devname + index) = tmp1 + 0x30;
			index++;
			tmp2 = (myprivate.datecode - tmp * 10000 - tmp1 * 1000) / 100;
			*(devname + index) = tmp2 + 0x30;
			index++;
			tmp3 = (myprivate.datecode - tmp * 10000 - tmp1 * 1000 - tmp2 * 100) / 10;
			*(devname + index) = tmp3 + 0x30;
			index++;
			tmp4 = (myprivate.datecode - tmp * 10000 - tmp1 * 1000 - tmp2 * 100 - tmp3 * 10);
			*(devname + index) = tmp4 + 0x30;
			index++;
			tmp = 0;
			tmp1 = 0;
			tmp2 = 0;
			tmp3 = 0;
			tmp4 = 0;
                       	tmp = myprivate.bledev_number / 10000;		
			*(devname + index) = tmp + 0x30;
			index++;
			tmp1 = (myprivate.bledev_number - tmp * 10000) / 1000;
			*(devname + index) = tmp1 + 0x30;
			index++;
			tmp2 = (myprivate.bledev_number - tmp * 10000 - tmp1 * 1000) / 100;
			*(devname + index) = tmp2 + 0x30;
			index++;
			tmp3 = (myprivate.bledev_number - tmp * 10000 - tmp1 * 1000 - tmp2 * 100) / 10;
			*(devname + index) = tmp3 + 0x30;
			index++;
			tmp4 = (myprivate.bledev_number - tmp * 10000 - tmp1 * 1000 - tmp2 * 100 - tmp3 * 10);
			*(devname + index) = tmp4 + 0x30;
			index++;
			*(devname + index) = 0;
                       
			ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
			//esp_bt_dev_set_device_name(EXCAMPLE_DEVICE_NAME);
			esp_bt_dev_set_device_name(devname);
			esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
			//if(bt_start_flag == 0) {
			ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT-1");
			esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
			//}
			break;
		case ESP_SPP_DISCOVERY_COMP_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
			break;
		case ESP_SPP_OPEN_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        		esp_spp_write(param->srv_open.handle, SPP_DATA_LEN, spp_data);
			break;
		case ESP_SPP_CLOSE_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
			break;
		case ESP_SPP_START_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
			break;
		case ESP_SPP_CL_INIT_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
			break;
		case ESP_SPP_DATA_IND_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
			param->data_ind.len, param->data_ind.handle);
			esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
			if(uart_priv.uart_cmd == 1)
			{
			}
			if(uart_priv.uart_cmd == 255) 
			{
				char *command_buf = (char*)malloc(10);
				char *params_buf = (char*)malloc(10);
				int command_delimiter,index;		
				int uart_command_num = 0;
				int uart_params_num = 0;
				command_delimiter = 0;
				/*
				*	To get the position of command delimiter
				*	copy the command string
				*/
				for(index = 0; index < param->data_ind.len; index++) {	
					if(*(param->data_ind.data + index) == ' ') {
						break;
					}
					*(command_buf+index) = *(param->data_ind.data + index);
				}
				*(command_buf + index) = '\0';	//command EOF 
				/*
				*	copy the argument of command
				*/
				//value_len = 0;
				index++;
				command_delimiter =  index;
				for(; index < param->data_ind.len; index++) {
					if(*(param->data_ind.data + index) == ' ' || (*(param->data_ind.data + index) == 0x0d 
						&& *(param->data_ind.data + index + 1) == 0x0a)) 
					{		
						break;
					}
					*(params_buf + index - command_delimiter) = *(param->data_ind.data + index);
				}
				*(params_buf + index - command_delimiter) = '\0'; //argu EOF 
				index++;
				uart_priv.data = 0;
				for(; index < param->data_ind.len; index++) {
					if((*(param->data_ind.data + index) == ' ')|| (*(param->data_ind.data + index) == 0x0d 
						&& *(param->data_ind.data + index + 1) == 0x0a)) 
					{			
						break;
					}
					uart_priv.data = uart_priv.data * 10;
					uart_priv.data += *(param->data_ind.data + index)-0x30;
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
					else {	
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
					else {	
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
				for(index = 0; ; index++) {
					if(*(command_buf+index) == '\0') {
						spp_data[index] = ' ';
						break;
					}
					else {
						spp_data[index] = *(command_buf + index);
					}
				}
				index++;
				for(tmp = 0; ; tmp++, index++) {
					if(*(params_buf+tmp) == '\0') {
						spp_data[index] = ' ';
						break;
					}
					else {
						spp_data[index] = *(params_buf + tmp);
					}
				}
				index++;	
				for(tmp = 0; tmp < 8; tmp++, index++) {
					spp_data[index] = info[tmp];
				}
				spp_data[index] = '\r';
				index++;
				spp_data[index] = '\n';
				index++;
				esp_spp_write(param->write.handle, index, spp_data);
				free(params_buf);
				free(command_buf);
			}
			break;
		case ESP_SPP_CONG_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
			break;
		case ESP_SPP_WRITE_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
			break;
		case ESP_SPP_SRV_OPEN_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
			break;
		default:
			break;
	}
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	switch (event) {
    		case ESP_BT_GAP_AUTH_CMPL_EVT:{
        		if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            			ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            			esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
       	 		} else {
            			ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        		}
        		break;
    		}
    		case ESP_BT_GAP_PIN_REQ_EVT:{
        		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        		if (param->pin_req.min_16_digit) {
            			ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            			esp_bt_pin_code_t pin_code = {0};
            			esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        		} else {
           			ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            			esp_bt_pin_code_t pin_code;
            			pin_code[0] = '1';
            			pin_code[1] = '2';
            			pin_code[2] = '3';
            			pin_code[3] = '4';
            			esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        		}
        		break;
    		}
    		case ESP_BT_GAP_CFM_REQ_EVT:
        		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        		esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        		break;
    		case ESP_BT_GAP_KEY_NOTIF_EVT:
        		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        		break;
    		case ESP_BT_GAP_KEY_REQ_EVT:
        		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        		break;
    		default: {
        		ESP_LOGI(SPP_TAG, "event: %d", event);
       	 		break;
    		}
    	}
    	return;
}

void bt_ssp_init()
{
    	esp_err_t ret; 
#if 0
    	esp_err_t ret = nvs_flash_init();
    	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        	ESP_ERROR_CHECK(nvs_flash_erase());
        	ret = nvs_flash_init();
    	}
    	ESP_ERROR_CHECK( ret );
#endif
    	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        	return;
    	}

    	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        	return;
    	}

    	if ((ret = esp_bluedroid_init()) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        	return;
    	}

    	if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        	return;
    	}

    	if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        	return;
    	}

    	if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        	return;
    	}
    	if ((ret = esp_bredr_tx_power_set(ESP_PWR_LVL_P6, ESP_PWR_LVL_P9)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s esp bredr tx set power: %s\n", __func__, esp_err_to_name(ret));
        	return;
    	}
	
    	if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        	return;
    	}

    	/* Set default parameters for Secure Simple Pairing */
    	esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    	esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    	/*
     	* Set default parameters for Legacy Pairing
     	* Use variable pin, input pin code when pairing
     	*/
    	esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    	esp_bt_pin_code_t pin_code;
    	esp_bt_gap_set_pin(pin_type, 0, pin_code);
	bt_start_flag = 0;
}

void bt_ssp_reinit()
{
    	esp_err_t ret; 
#if 0
    	esp_err_t ret = nvs_flash_init();
    	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        	ESP_ERROR_CHECK(nvs_flash_erase());
        	ret = nvs_flash_init();
    	}
    	ESP_ERROR_CHECK( ret );
#endif
#if 0 
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    	
#endif
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        	//return;
    	}
    	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        	//return;
    	}

    	if ((ret = esp_bluedroid_init()) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        	//return;
    	}

    	if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        	//return;
    	}
    	if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        	//return;
    	}

    	if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        	//return;
    	}

    	if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        	ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        	//return;
    	}

        ESP_LOGE(SPP_TAG, "%s ready to set paramteres for Secure Simple Pairing\n", __func__);
    	/* Set default parameters for Secure Simple Pairing */
    	esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    	esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    	/*
     	* Set default parameters for Legacy Pairing
     	* Use variable pin, input pin code when pairing
     	*/
    	esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    	esp_bt_pin_code_t pin_code;
    	esp_bt_gap_set_pin(pin_type, 0, pin_code);
	bt_start_flag = 1;
}

