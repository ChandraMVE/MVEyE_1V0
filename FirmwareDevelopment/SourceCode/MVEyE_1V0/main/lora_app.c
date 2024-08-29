//-----------------------------------------------------------------
///
///     \file lora_app.c
///
///     \brief lora application framework driver
///
///
///     \author       Chandrashekhar Venkatesh
///
///     Location:     India
///
///     Project Name: MVEyE_1V0
///
///     \date Created 20AUG2024
///
///      Tools:  EspressifIDE
///      Device:   ESP32WROOM
///		 Operating System: windows 10
/// 	 Java Runtime Version: 17.0.11+9
///      Eclipse Version: 4.30.0.v20231201-0110
///      Eclipse CDT Version: 11.4.0.202309142347
///      IDF Eclipse Plugin Version: 3.0.0.202406051940
///      IDF Version:   5.3
///
/// Copyright © 2024 MicriVision Embedded Pvt Ltd
///
/// Confidential Property of MicroVision Embedded Pvt Ltd
///
//-----------------------------------------------------------------

//==============================================================================
//          __             __   ___  __
//  | |\ | /  ` |    |  | |  \ |__  /__`
//  | | \| \__, |___ \__/ |__/ |___ .__/
//
//==============================================================================
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora_app.h"
#include "lora_llc68.h"
#include <ctype.h>
#include "accelerometer_KXTJ3.h"
#include "esp_err.h"
#include "string.h"
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "LORA_APP"
#define TIMEOUT 100
#define PING 1
#define PONG 0
//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================

//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================
/*******************************************************************************
 * Function name  : task_ping
 *
 * Description    : ping function task
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void task_ping (void *pvParameters)
 {
 	ESP_LOGI(pcTaskGetName(NULL), "Start");
 	uint8_t txData[256]; // Maximum Payload size of SX1261/62/68 is 255
	uint8_t rxData[256]; // Maximum Payload size of SX1261/62/68 is 255

	while(1) {
	
		TickType_t nowTick = xTaskGetTickCount();
		int txLen = sprintf((char *)txData, "Hello World %"PRIu32, nowTick);
		uint8_t len = strlen((char *)txData);

		// Wait for transmission to complete
		if (LoRaSend(txData, txLen, LLCC68_TXMODE_SYNC)) {
			ESP_LOGI(pcTaskGetName(NULL), "Send success");

			bool waiting = true;
			TickType_t startTick = xTaskGetTickCount();
			while(waiting) {
				uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
				TickType_t currentTick = xTaskGetTickCount();
				TickType_t diffTick = currentTick - startTick;
				if ( rxLen > 0 ) {
					ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen, rxData);
					ESP_LOGI(pcTaskGetName(NULL), "Response time is %"PRIu32" millisecond", diffTick * portTICK_PERIOD_MS);
					waiting = false;
				}
				
				ESP_LOGD(pcTaskGetName(NULL), "diffTick=%"PRIu32, diffTick);
				if (diffTick > TIMEOUT) {
					ESP_LOGW(pcTaskGetName(NULL), "No response within %d ticks", TIMEOUT);
					waiting = false;
				}
				vTaskDelay(1); // Avoid WatchDog alerts
			} // end waiting

		} else {
			ESP_LOGE(pcTaskGetName(NULL), "Send fail");
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	} // end while 
 }
 /*******************************************************************************
 * Function name  : task_pong
 *
 * Description    : pong function task
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void task_pong (void *pvParameters)
 {
	 ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t txData[256]; // Maximum Payload size of SX1261/62/68 is 255
	uint8_t rxData[256]; // Maximum Payload size of SX1261/62/68 is 255
	while(1) {
		uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
		if ( rxLen > 0 ) { 
			printf("Receive rxLen:%d\n", rxLen);
			for(int i=0;i< rxLen;i++) {
				printf("%02x ",rxData[i]);
			}
			printf("\n");

			for(int i=0;i< rxLen;i++) {
				if (rxData[i] > 0x19 && rxData[i] < 0x7F) {
					char myChar = rxData[i];
					printf("%c", myChar);
				} else {
					printf("?");
				}
			}
			printf("\n");

			int8_t rssi, snr;
			GetPacketStatus(&rssi, &snr);
			printf("rssi=%d[dBm] snr=%d[dB]\n", rssi, snr);

			for(int i=0;i<rxLen;i++) {
				if (isupper(rxData[i])) {
					txData[i] = tolower(rxData[i]);
				} else {
					txData[i] = toupper(rxData[i]);
				}
			}

			// Wait for transmission to complete
			if (LoRaSend(txData, rxLen, LLCC68_TXMODE_SYNC)) {
				ESP_LOGD(pcTaskGetName(NULL), "Send success");
			} else {
				ESP_LOGE(pcTaskGetName(NULL), "LoRaSend fail");
			}

		}
		vTaskDelay(1); // Avoid WatchDog alerts
	} // end while 
 }
 
/*******************************************************************************
 * Function name  : create_lora_task
 *
 * Description    : function to create Lora application tasks
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void create_lora_task(void)
{
#if PING
	xTaskCreate(&task_ping, "PING", 1024*4, NULL, 5, NULL);
#endif
#if PONG	
	xTaskCreate(&task_pong, "PONG", 1024*4, NULL, 5, NULL);
#endif
}

/*******************************************************************************
 * Function name  : LoRaAppInit
 *
 * Description    : function to initialize the Lora app with all parameters
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void LoRaAppInit(void)
{	
	uint32_t frequencyInHz = 0;
#if CONFIG_169MHZ
	frequencyInHz = 169000000;
	ESP_LOGI(TAG, "Frequency is 169MHz");
#elif CONFIG_433MHZ
	frequencyInHz = 433000000;
	ESP_LOGI(TAG, "Frequency is 433MHz");
#elif CONFIG_470MHZ
	frequencyInHz = 470000000;
	ESP_LOGI(TAG, "Frequency is 470MHz");
#elif CONFIG_866MHZ
	frequencyInHz = 866000000;
	ESP_LOGI(TAG, "Frequency is 866MHz");
#elif CONFIG_868MHZ
	frequencyInHz = 868130000;
	ESP_LOGI(TAG, "Frequency is 868MHz");
#elif CONFIG_915MHZ
	frequencyInHz = 915000000;
	ESP_LOGI(TAG, "Frequency is 915MHz");
#elif CONFIG_OTHER
	ESP_LOGI(TAG, "Frequency is %dMHz", CONFIG_OTHER_FREQUENCY);
	frequencyInHz = CONFIG_OTHER_FREQUENCY * 1000000;
#endif

	// Initialize LoRa
	LoRaInit();
	int8_t txPowerInDbm = 22;
	
#if CONFIG_USE_TCXO
	ESP_LOGW(TAG, "Enable TCXO");
	float tcxoVoltage = 3.3; // use TCXO
	bool useRegulatorLDO = true; // use DCDC + LDO
#else
	ESP_LOGW(TAG, "Disable TCXO");
	float tcxoVoltage = 0.0;  // don't use TCXO
	bool useRegulatorLDO = false;  // use only LDO in all modes
#endif

	//LoRaDebugPrint(true);
	if (LoRaBegin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
		ESP_LOGE(TAG, "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}
	
	uint8_t spreadingFactor = 7;
	uint8_t bandwidth = LLCC68_LORA_BW_125_0;
	uint8_t codingRate = LLCC68_LORA_CR_4_5;
	uint16_t preambleLength = 8;
	uint8_t payloadLen = 0;
	bool crcOn = true;
	bool invertIrq = false;
#if CONFIF_ADVANCED
	spreadingFactor = CONFIG_SF_RATE;
	bandwidth = CONFIG_BANDWIDTH;
	codingRate = CONFIG_CODING_RATE
#endif
	LoRaConfig(spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen, crcOn, invertIrq);
}


