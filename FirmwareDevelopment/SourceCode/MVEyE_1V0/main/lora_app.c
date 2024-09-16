//-----------------------------------------------------------------
///
///     \file lora_node1.c
///
///     \brief lora application for Node 1 (source)
///
///
///     \author       Chandrashekhar Venkatesh
///
///     Project Name: MVEyE_1V0
///
///     \date Created 20AUG2024
///
///     Tools:  EspressifIDE
///     Device:   ESP32WROOM
///
//-----------------------------------------------------------------

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora_app.h"
#include "lora_llc68.h"
#include <string.h>

#define TAG "LORA_NODE1"
#define DEVICE_ID 001
#define DESTINATION_DEVICE_ID 003

typedef struct {
    uint8_t senderID;
    uint8_t destinationID;
    uint8_t hopCount;
    char payload[256];
} MeshPacket;

/*******************************************************************************
 * Function name  : task_ping
 *
 * Description    : Node 1 task to send ping message
 * Parameters     : None
 * Returns        : None
 ******************************************************************************/
void task_ping(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start sending ping...");
    MeshPacket packet;

    packet.senderID = DEVICE_ID;
    packet.destinationID = DESTINATION_DEVICE_ID;
    packet.hopCount = 0;
    strcpy((char *)packet.payload, "Hello from Node 1!");

    uint8_t buffer[sizeof(MeshPacket)];
    memcpy(buffer, &packet, sizeof(MeshPacket));

    while (1) {
        if (LoRaSend(buffer, sizeof(MeshPacket), LLCC68_TXMODE_SYNC)) {
            ESP_LOGI(TAG, "Ping Message Sent to Node 3");
            ESP_LOGI(TAG, "Payload: %s", packet.payload);
        } else {
            ESP_LOGE(TAG, "Failed to send ping message");
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Send ping every 5 seconds
    }
}

/*******************************************************************************
 * Function name  : task_receive_pong
 *
 * Description    : Node 1 task to receive pong response
 * Parameters     : None
 * Returns        : None
 ******************************************************************************/
void task_receive_pong(void *pvParameters)
{
    ESP_LOGI(pcTaskGetName(NULL), "Start receiving pong...");

    MeshPacket receivedPacket;
    uint8_t buffer[sizeof(MeshPacket)];

    while (1) {
        uint8_t rxLen = LoRaReceive(buffer, sizeof(buffer));
        if (rxLen > 0) {
            memcpy(&receivedPacket, buffer, sizeof(MeshPacket));

            if (receivedPacket.destinationID == DEVICE_ID) {
                ESP_LOGI(TAG, "Pong response received from Node 3");
                ESP_LOGI(TAG, "Payload: %s", receivedPacket.payload);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Check for response every second
    }
}

/*******************************************************************************
 * Function name  : create_lora_task_node1
 *
 * Description    : Creates tasks for sending ping and receiving pong in Node 1
 * Parameters     : None
 * Returns        : None
 ******************************************************************************/
void create_lora_task_node1(void)
{
    // Create task for sending ping messages
    xTaskCreate(&task_ping, "PING", 1024*4, NULL, 5, NULL);

    // Create task for receiving pong responses
    xTaskCreate(&task_receive_pong, "RECEIVE_PONG", 1024*4, NULL, 5, NULL);
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
}
