//-----------------------------------------------------------------
///
///     \file loraMesh.c
///
///     \brief lora application framework driver
///
///
///     \author       Keerthi Mallesh 
///
///     Location:     India
///
///     Project Name: MVEyE_1V0
///
///     \date Created 10SEP2024
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

#include "loraMesh.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <string.h>
#include "lora_llc68.h"

//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "LORA_APP"	
#define TIMEOUT 100
#define SENDER_ID 3
#define DESTINATION_ID 1
#define DEVICE_1 0
#define DEVICE_2 0
#define DEVICE_3 1
//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
typedef struct {
	
    uint8_t senderID;
    uint8_t destinationID;
    uint8_t hopCount;
    char    payload[30];
    
} MeshPacket;

MeshPacket packet;
MeshPacket receivedPacket;


uint8_t destination_id;

//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================

/***********************************************************************************
 * Function name  : device_primary
 *
 * Description    : primary device.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 11SEP2024
 ***********************************************************************************/
void device_primary( void *pvParameters )
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	
    packet.senderID = SENDER_ID;
    packet.destinationID = DESTINATION_ID;
    packet.hopCount = 0;
    strncpy(packet.payload, "HELLO_device_3", sizeof(packet.payload));
    	
    uint8_t buffer[sizeof(MeshPacket)];
	memcpy(buffer, &packet, sizeof(MeshPacket));
	
	// Wait for transmission to complete
	if ( LoRaSend(buffer, sizeof(MeshPacket), LLCC68_TXMODE_SYNC ) ) 
	{	
		// Print the data to be transmitted
       	ESP_LOGI(TAG, "Transmitting Message:");
       	ESP_LOGI(TAG, "Sender ID: %d", packet.senderID);
       	ESP_LOGI(TAG, "Destination ID: %d", packet.destinationID);
       	ESP_LOGI(TAG, "Hop Count: %d", packet.hopCount);
       	ESP_LOGI(TAG, "Payload: %s", packet.payload);
       	packet.hopCount++;
	}
        
    while(1) {
        	uint8_t rxData[sizeof(MeshPacket)];
        	uint8_t rxlen = LoRaReceive(rxData,sizeof(rxData));
        	      ESP_LOGI(TAG, "Waiting to receive at device 1....");
        	      
        	if( rxlen > 0){				
					receivedPacket.senderID = rxData[0];
        			receivedPacket.destinationID = rxData[1];
       				receivedPacket.hopCount = rxData[2];
       				
				if( receivedPacket.destinationID == 1 ){

					ESP_LOGI(TAG, "SENDER_DEVICE_ID: %d", receivedPacket.senderID );
					ESP_LOGI(TAG, "DESTINATION_DEVICE_ID: %d", receivedPacket.destinationID );
					ESP_LOGI(TAG, "Hop_Count: %d", receivedPacket.hopCount );
					
					ESP_LOGI(TAG, "Acknowledgement_Received" );
				}
		
			}
		vTaskDelay(pdMS_TO_TICKS(1000));
		}
}

/***********************************************************************************
 * Function name  : device_secondary
 *
 * Description    : secondary device.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 11SEP2024
 ***********************************************************************************/
void device_secondary( void *pvParameters )
{	
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t rxData[sizeof(MeshPacket)];
	
	while(1)
	{
		uint8_t rxlen = LoRaReceive(rxData,sizeof(rxData));
		ESP_LOGI(TAG, "Waiting to receive at device 2....");
		
		if(rxlen > 0)
		{
			receivedPacket.senderID = rxData[0];
			ESP_LOGI(TAG, "SENDER_DEVICE_ID: %d", receivedPacket.senderID );
	
			receivedPacket.destinationID = rxData[1];
			ESP_LOGI(TAG, "DESTINATION_DEVICE_ID: %d", receivedPacket.destinationID );   
        
			receivedPacket.hopCount = rxData[2];
			ESP_LOGI( TAG, "Hop_Count: %d", receivedPacket.hopCount );
			receivedPacket.hopCount = rxData[2]++;
			
			LoRaSend(rxData, sizeof(MeshPacket), LLCC68_TXMODE_SYNC );
			ESP_LOGI(TAG, "Sent the message received from device 2....");
			
			rxlen = 0;
			memset( rxData,0,sizeof(rxData));		
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}	
} 

/***********************************************************************************
 * Function name  : device_ternary
 *
 * Description    : primary device.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 11SEP2024
 ***********************************************************************************/
void device_Ternary( void *pvParameters )
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t rxData[sizeof(MeshPacket)];
	bool receive_done = 0;
	
	while(1)
	{
		uint8_t rxlen = LoRaReceive(rxData,sizeof(rxData));
		ESP_LOGI(TAG, "Waiting to receive at device 3....");
		
		if( rxlen > 0 ){
			receivedPacket.senderID = rxData[0];
			receivedPacket.destinationID = rxData[1];
			receivedPacket.hopCount = rxData[2];
			
			if( receivedPacket.destinationID == 3 ){
								
				ESP_LOGI(TAG, "SENDER_DEVICE_ID: %d", receivedPacket.senderID );
	
				ESP_LOGI(TAG, "DESTINATION_DEVICE_ID: %d", receivedPacket.destinationID );   
        	
				ESP_LOGI( TAG, "Hop_Count: %d", receivedPacket.hopCount );
				receivedPacket.hopCount = rxData[2]++;
				
				ESP_LOGI(TAG, "Acknowledgement_Received" );
				
				rxlen = 0;
				memset( rxData,0,sizeof(rxData));
				receive_done = 1;
			}
		}
		
		if( receive_done ) {
    		packet.senderID = SENDER_ID;
    		packet.destinationID = DESTINATION_ID;
    		packet.hopCount = 0;
    		strncpy(packet.payload, "HELLO_device_1", sizeof(packet.payload));
    	
    		uint8_t buffer[sizeof(MeshPacket)];
			memcpy(buffer, &packet, sizeof(MeshPacket));
	
			LoRaSend(buffer, sizeof(MeshPacket), LLCC68_TXMODE_SYNC );	
	}	
	
	vTaskDelay(pdMS_TO_TICKS(1000));
}		
} 

/***********************************************************************************
 * Function name  : create_lora_mesh
 *
 * Description    : create_lora_mesh.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 11SEP2024
 ***********************************************************************************/
void create_lora_mesh( void )
{
	
#if DEVICE_1
	xTaskCreate(&device_primary, "device_primary", 1024*4, NULL, 5, NULL);
#endif

#if DEVICE_2	
	xTaskCreate(&device_secondary, "device_secondary", 1024*4, NULL, 5, NULL);
#endif

#if DEVICE_3	
	xTaskCreate(&device_Ternary, "device_ternary", 1024*4, NULL, 5, NULL);
#endif

}
 