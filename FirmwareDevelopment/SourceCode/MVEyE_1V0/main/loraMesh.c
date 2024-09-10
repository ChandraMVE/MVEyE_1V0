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
#include <string.h>
#include "lora_llc68.h"

//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "LORA_APP"
#define SENDER_ID 1
#define DESTINATION_ID 3	
//==============================================================================
//   __     	   __   __                          __   __
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

bool mesh_send( void ) {
	
    MeshPacket packet;
    packet.senderID = SENDER_ID;
    packet.destinationID = DESTINATION_ID;
    packet.hopCount = 0;
    strncpy(packet.payload, "HELLO_device_1", sizeof(packet.payload));

    uint8_t buffer[sizeof(MeshPacket)];
    memcpy(buffer, &packet, sizeof(MeshPacket));

    ESP_LOGI(TAG, "Sending message from Node %d to Node %d", packet.senderID, packet.destinationID );
    
    if (LoRaSend(buffer, sizeof(buffer),LLCC68_TXMODE_SYNC)) {
        ESP_LOGI(TAG, "Message sent successfully.");
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to send message.");
        return false;
    }
}

void mesh_receive( void *pvParameters ) {
	
    uint8_t buffer[sizeof(MeshPacket)];
    MeshPacket receivedPacket;

    while (1) {
        int rxLen = LoRaReceive(buffer, sizeof(buffer));
        if (rxLen > 0) {
            memcpy(&receivedPacket, buffer, sizeof(MeshPacket));

            ESP_LOGI(TAG, "Received message from Node %d to Node %d", receivedPacket.senderID, receivedPacket.destinationID);
            ESP_LOGI(TAG, "Payload: %s", receivedPacket.payload);

            if (receivedPacket.destinationID == DESTINATION_ID) {
                ESP_LOGI(TAG, "Message reached the destination: Node %d", DESTINATION_ID);
            } else if (receivedPacket.hopCount < MAX_HOP_COUNT) {
                ESP_LOGI(TAG, "Forwarding message from Node %d to Node %d", receivedPacket.senderID, receivedPacket.destinationID);
                receivedPacket.hopCount++;
                
                // Forward the packet
                uint8_t forwardBuffer[sizeof(MeshPacket)];
                memcpy(forwardBuffer, &receivedPacket, sizeof(MeshPacket));
                LoRaSend(forwardBuffer, sizeof(MeshPacket),LLCC68_TXMODE_SYNC );
            } else {
                ESP_LOGW(TAG, "Max hop count reached. Dropping message.");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Task delay to avoid tight polling loop
    }
}
