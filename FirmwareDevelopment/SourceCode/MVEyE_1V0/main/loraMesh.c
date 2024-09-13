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

typedef struct MeshNode {
	
    uint8_t deviceID;      
    float rssi;              
    char device_address[16];  
    struct MeshNode *next;
    
} MeshNode;

MeshNode *mesh_head = NULL;

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

/***********************************************************************************
 * Function name  : lora_receive_task
 *
 * Description    : Be in receive mode for 5 seconds.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 12SEP2024
 ***********************************************************************************/
 
 void lora_receive_task() {
	 
	 ESP_LOGI(pcTaskGetName(NULL), "Start");
	 
	 uint8_t buffer[20];
	 
	 while(1)
	 {
		uint8_t rxLen = LoRaReceive(buffer, sizeof(buffer));
		
		if(rxLen > 0)
		{
			for(int i=0;i<sizeof(buffer);i++){
						ESP_LOGI(pcTaskGetName(NULL), "The received message:%d", buffer[i] );
			}
			
		}
 		vTaskDelay(5000);
	 }
 }
 
 /***********************************************************************************
 * Function name  : transmit_task
 *
 * Description    : Be in transmit mode for 5 seconds.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 12SEP2024
 ***********************************************************************************/
 
 void lora_broadcast_task() {
	 
	 ESP_LOGI(pcTaskGetName(NULL), "Start");
	 
	 uint8_t tx_buffer[20];
	 
	 while(1)
	 {
			int8_t rssi, snr;
			GetPacketStatus(&rssi, &snr);	
			
			tx_buffer[0] = rssi;
			tx_buffer[1] = snr;
				
				if (LoRaSend(tx_buffer, sizeof(tx_buffer), LLCC68_TXMODE_SYNC))

 		vTaskDelay(5000);
	 }
 }
 
 /***********************************************************************************
 * Function name  : create_node
 *
 * Description    : creating nodes for lora mesh.
 * Parameters     : deviceID, rssi, device_address pointer.
 * Returns        : newNode
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 13SEP2024
 ***********************************************************************************/ 
 
MeshNode* create_node(uint8_t deviceID, float rssi, const char *device_address) {
	 
    struct MeshNode* newNode = (struct MeshNode*)malloc(sizeof(struct MeshNode));
    
    if ( newNode == NULL ) {
		ESP_LOGI(TAG, "Memory allocation failed\n");
    }
    
    newNode->deviceID = deviceID;
    newNode->rssi = rssi;
    newNode->next = NULL;  
    
    ESP_LOGI(TAG,"Device ID %d inserted with RSSI %.2f\n", deviceID, rssi);
    
    return newNode;
}

 /***********************************************************************************
 * Function name  : create_node
 *
 * Description    : creating nodes for lora mesh.
 * Parameters     : deviceID, rssi, device_address pointer.
 * Returns        : newNode
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 13SEP2024
 ***********************************************************************************/
 
void add_mesh_node(uint8_t device_id, float rssi, const char *device_address) {
    MeshNode *new_node = create_node(device_id, rssi, device_address);
    if (new_node == NULL) return;

    if (mesh_head == NULL) {
        mesh_head = new_node;
    } else {
        MeshNode *temp = mesh_head;
        while (temp->next != NULL) {
            temp = temp->next;
        }
        temp->next = new_node;
    }
}

 /***********************************************************************************
 * Function name  : display_mesh_nodes
 *
 * Description    : display nodes connected.
 * Parameters     : None.
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 13SEP2024
 ***********************************************************************************/
 
 void display_mesh_nodes() {
	 
    MeshNode *current = mesh_head;
    
    ESP_LOGI(TAG, "LoRa Mesh Network Information:\n");
    
    while (current != NULL) {
		ESP_LOGI( TAG, "Device ID: %d, RSSI: %.2f\n",current->deviceID, current->rssi );
        current = current->next;
    }
}

 /***********************************************************************************
 * Function name  : deleteNode
 *
 * Description    : deleting the node from the mesh network.
 * Parameters     : pointer head.
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 13SEP2024
 ***********************************************************************************/
 
void deleteNode(struct MeshNode** head, int id) {
	
    struct MeshNode *temp = *head, *prev = NULL;
    
    if (temp != NULL && temp->deviceID == id) {
        *head = temp->next;   
        free(temp); 
        ESP_LOGI( TAG, "Device ID %d removed from the mesh.\n", id );         
    }
    
    while (temp != NULL && temp->deviceID != id) {
        prev = temp;
        temp = temp->next;
    }
    
    if (temp == NULL) {
		ESP_LOGI( TAG, "Device ID %d not found in the mesh.\n", id );         
    }
    
    prev->next = temp->next;
    free(temp);
    ESP_LOGI( TAG, "Device ID %d removed from the mesh.\n", id );    
}

 /***********************************************************************************
 * Function name  : findDevice
 *
 * Description    : checking the device information of different nodes .
 * Parameters     : destination ID.
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 13SEP2024
 ***********************************************************************************/

MeshNode* findDevice( int destinationID) {
	
     MeshNode* current = mesh_head;
	ESP_LOGI( TAG, "Routing to Device ID %d:\n", destinationID);    

    while (current != NULL) {
		ESP_LOGI( TAG, "Checking Device ID: %d\n", current->deviceID);    
        
        if (current->deviceID == destinationID) {
            ESP_LOGI( TAG, "Destination Device ID %d found with RSSI %.2f\n", current->deviceID, current->rssi);
            return current;
        }
        current = current->next;
    }
    
	ESP_LOGI( TAG, "Device ID %d not found in the network.\n", destinationID);
	return NULL;
}