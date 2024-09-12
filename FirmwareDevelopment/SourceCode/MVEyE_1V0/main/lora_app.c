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
<<<<<<< HEAD
=======
#define DEVICE_ID 001
#define DESTINATION_DEVICE_ID 003
#define MAX_HOPS 5
>>>>>>> 8dcd53a713103c96d5b910f1859a983916ca348a
//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
<<<<<<< HEAD
extern char mqtt_message[256];

union acceleration_union
{
	float	acceleration[3];
	uint8_t	acceleration_buff[12]; 
};

union acceleration_union tx,rx;

float acceleration_reading[3];
=======
typedef struct {
	
    uint8_t senderID;
    uint8_t destinationID;
    uint8_t hopCount;
    char    payload[256];
    
} MeshPacket;

extern char mqtt_message[256];
>>>>>>> 8dcd53a713103c96d5b910f1859a983916ca348a
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================

void forward_message(MeshPacket* packet);

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
 	MeshPacket packet;
 	
 	packet.senderID 	 = DEVICE_ID;
    packet.destinationID = DESTINATION_DEVICE_ID;
    packet.hopCount 	 = 0;
 	strcpy((char *)packet.payload, "Hello, this is a test message!");

	uint8_t buffer[sizeof(MeshPacket)];
	memcpy(buffer, &packet, sizeof(MeshPacket));

	for(int i = 0;i < sizeof(MeshPacket); i++)
	{
		ESP_LOGI(TAG, "Buffer:%d",buffer[i]);
	}
	
	while(1) {
		//TickType_t nowTick = xTaskGetTickCount();
<<<<<<< HEAD
		//int txLen = sprintf((char *)txData, "Hello World %"PRIu32, nowTick);
		//uint8_t len = strlen((char *)txData);
=======
>>>>>>> 8dcd53a713103c96d5b910f1859a983916ca348a

		for(int i=0; i < 3; i++){
			
			tx.acceleration[i] = acceleration_reading[i];
		}

		// Wait for transmission to complete
<<<<<<< HEAD
		if (LoRaSend(tx.acceleration_buff, sizeof(acceleration_reading), LLCC68_TXMODE_SYNC)) {
			//ESP_LOGI(pcTaskGetName(NULL), "Send success");

			bool waiting = true;
			TickType_t startTick = xTaskGetTickCount();
			while(waiting) {
				uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
					
			for (int i = 0; i < 12; i++) 
			{
        		rx.acceleration_buff[i] = rxData[i];
    		}
    		
    			//mqtt_message[rxLen] = '\0';
    			ESP_LOGI(pcTaskGetName(NULL), "acceleration data received from LoRa X = %f, Y = %f, Z = %f",
    			rx.acceleration[0], rx.acceleration[1], rx.acceleration[2]);
    			
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
=======
		if (LoRaSend(buffer, sizeof(MeshPacket), LLCC68_TXMODE_SYNC)) 
		{	
			// Print the data to be transmitted
        	ESP_LOGI(TAG, "Transmitting Message:");
        	ESP_LOGI(TAG, "Sender ID: %d", packet.senderID);
        	ESP_LOGI(TAG, "Destination ID: %d", packet.destinationID);
        	ESP_LOGI(TAG, "Hop Count: %d", packet.hopCount);
        	ESP_LOGI(TAG, "Payload: %s", packet.payload);	
        	
         ESP_LOGI(pcTaskGetName(NULL), "Ping message sent");
        } else {
            ESP_LOGE(pcTaskGetName(NULL), "Ping message failed");
        }	
        
                vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

>>>>>>> 8dcd53a713103c96d5b910f1859a983916ca348a
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
	
	MeshPacket receivedPacket;
	
	uint8_t buffer[20];
	
	while(1) {
<<<<<<< HEAD
		uint8_t rxLen = LoRaReceive(rxData, sizeof(rxData));
		
			for (int i = 0; i < 12; i++) 
			{
        		rx.acceleration_buff[i] = rxData[i];
    		}
    		
    	//mqtt_message[rxLen] = '\0';
    	ESP_LOGI(pcTaskGetName(NULL), "acceleration data received from LoRa X = %f, Y = %f, Z = %f",
					rx.acceleration[0], rx.acceleration[1], rx.acceleration[2] );
    			
    	// ESP_LOGI(pcTaskGetName(NULL), "mqtt_message %s",mqtt_message);
    	
=======
		uint8_t rxLen = LoRaReceive(buffer, sizeof(buffer));
		
>>>>>>> 8dcd53a713103c96d5b910f1859a983916ca348a
		if ( rxLen > 0 ) { 
		
        	
			if (receivedPacket.destinationID == DEVICE_ID) {
                ESP_LOGI(TAG, "Message received for me: %s", receivedPacket.payload);
               

<<<<<<< HEAD
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
			
			for(int i=0; i < 3; i++){
			
				tx.acceleration[i] = acceleration_reading[i];
			}
			
			// Wait for transmission to complete
			if (LoRaSend(tx.acceleration_buff, sizeof(acceleration_reading), LLCC68_TXMODE_SYNC)) {
				ESP_LOGD(pcTaskGetName(NULL), "Send success");
			} else {
				ESP_LOGE(pcTaskGetName(NULL), "LoRaSend fail");
			}
		}
		vTaskDelay(1); // Avoid WatchDog alerts
	} // end while 
 }
 
=======
                 // Send the response
                receivedPacket.senderID = DEVICE_ID;
                receivedPacket.destinationID = receivedPacket.senderID;
                receivedPacket.hopCount = 0;
                if (LoRaSend(buffer, sizeof(MeshPacket), LLCC68_TXMODE_SYNC)) {
                    ESP_LOGI(TAG, "Pong response sent");
                } else {
                    ESP_LOGE(TAG, "Pong response failed");
                }
            } else {
                // Forward the message if it's not for this device
                ESP_LOGI(TAG, "Message received for another device, forwarding...");
                forward_message(&receivedPacket); 
            }
        }
        
        vTaskDelay(1); // Avoid WatchDog alerts
    }
    
    			for(int i = 0; i < sizeof( buffer ); i++ )
				{
					ESP_LOGI(TAG, "Received information %d", buffer[i]);
				}	
}               
                              
>>>>>>> 8dcd53a713103c96d5b910f1859a983916ca348a
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
	
	uint8_t spreadingFactor = 11;
	uint8_t bandwidth = LLCC68_LORA_BW_500_0;
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
<<<<<<< HEAD
}
=======
}

/*******************************************************************************
 * Function name  : forward_message
 *
 * Description    : maintains the hoping mechanism.
 * Parameters     : packet pointer
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 06SEP2024
 ******************************************************************************/
 
void forward_message(MeshPacket* packet) {
    if (packet->hopCount >= MAX_HOPS) {
        ESP_LOGW(TAG, "Message dropped, exceeded max hops");
        return;
    }

    packet->hopCount++;
    if (LoRaSend((uint8_t*)packet, sizeof(MeshPacket), LLCC68_TXMODE_SYNC)) {
        ESP_LOGI(TAG, "Message forwarded, hop count: %d", packet->hopCount);
    } else {
        ESP_LOGE(TAG, "Failed to forward message");
    }
}
>>>>>>> 8dcd53a713103c96d5b910f1859a983916ca348a
