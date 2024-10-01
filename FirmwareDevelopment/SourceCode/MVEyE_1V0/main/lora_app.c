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
#include "freertos/semphr.h"
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
#define DEVICE_ID 001
#define DESTINATION_DEVICE_ID 002
#define MAX_HOPS 5
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
    char    payload[256];
    
} MeshPacket;

uint8_t rxBuffer[256];
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================
// Define the semaphores
SemaphoreHandle_t tx_done_semaphore = NULL;
SemaphoreHandle_t rx_done_semaphore = NULL;

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
void task_ping(void *pvParameters) {
    LoRaAppInit();
    SetDioIrqParams(LLCC68_IRQ_TX_DONE | LLCC68_IRQ_TIMEOUT, LLCC68_IRQ_TX_DONE | LLCC68_IRQ_TIMEOUT, LLCC68_IRQ_NONE, LLCC68_IRQ_NONE);
    SetTx(200);
    ESP_LOGI(pcTaskGetName(NULL), "Start");

    MeshPacket packet;
    packet.senderID = DEVICE_ID;
    packet.destinationID = DESTINATION_DEVICE_ID;
    packet.hopCount = 0;
    strcpy((char *)packet.payload, "Hello, this is MVE");

    uint8_t buffer[sizeof(MeshPacket)];
    memcpy(buffer, &packet, sizeof(MeshPacket));

    while (1) {
        // Wait for transmission to complete
        if (LoRaSend(buffer, sizeof(MeshPacket), LLCC68_TXMODE_SYNC)) {
            // Wait for Tx done signal
            if (xSemaphoreTake(tx_done_semaphore, portMAX_DELAY)) {
                ESP_LOGI(TAG, "Transmitting Message-> SendID: %d DestID: %d HopCount: %d PayLoad: %s\r\n",
                          packet.senderID, packet.destinationID, packet.hopCount, packet.payload);
                ESP_LOGI(pcTaskGetName(NULL), "Ping message sent");
            }
        } else {
            ESP_LOGE(pcTaskGetName(NULL), "Ping message failed");
        }
        ClearIrqStatus(LLCC68_IRQ_TX_DONE);
    	configure_lora_for_rx();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust delay as needed
    }
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
void task_pong(void *pvParameters) {
    configure_lora_for_rx();
    ESP_LOGI(pcTaskGetName(NULL), "Start");

    MeshPacket receivedPacket;
    uint8_t buffer[sizeof(MeshPacket)]; // Match the size to MeshPacket
	uint16_t irqStatus = GetIrqStatus();
    
    if (irqStatus & LLCC68_IRQ_RX_DONE) {
        ClearIrqStatus(LLCC68_IRQ_RX_DONE);
        // LoRaReceive(rxBuffer, sizeof(rxBuffer));
        // process_received_packet(rxBuffer, receivedPacketLength);
    } else if (irqStatus & LLCC68_IRQ_TIMEOUT) {
        printf("RxTimeout Interrupt\n");
        ClearIrqStatus(LLCC68_IRQ_TIMEOUT);
        configure_lora_for_rx();  // Re-enter reception mode
    }
    while (1) {
        // Wait for reception signal
        if (xSemaphoreTake(rx_done_semaphore, portMAX_DELAY)) {
            uint8_t rxLen = LoRaReceive(buffer, sizeof(buffer));
            if (rxLen > 0) {
                memcpy(&receivedPacket, buffer, sizeof(receivedPacket)); // Ensure you copy received data
                if (receivedPacket.destinationID == DEVICE_ID) {
                    ESP_LOGI(TAG, "Message received for me: %s", receivedPacket.payload);
                    // Send the response
                    receivedPacket.senderID = DEVICE_ID;
                    receivedPacket.destinationID = receivedPacket.senderID;
                    receivedPacket.hopCount = 0;
                    strcpy((char *)receivedPacket.payload, "Hello, this is MVE2");
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
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Avoid WatchDog alerts
    }
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
	
	uint8_t spreadingFactor = 11;
	uint8_t bandwidth = LLCC68_LORA_BW_500_0;
	uint8_t codingRate = LLCC68_LORA_CR_4_5;
	uint16_t preambleLength = 12;  //12 need
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

/***********************************************************************************
 * Function name  : init_dio1_interrupt
 * Description    : Initializes the GPIO interrupt for DIO1. Configures the pin as input 
 *                  with pull-up enabled, and sets the interrupt to trigger on the rising edge.
 * Parameters     : None
 * Returns        : None
 * Known Issues   : None
 * Note           :
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/

/***********************************************************************************
 * Function name  : enable_dio1_interrupt
 * Description    : Enables the interrupt functionality for the DIO1 pin.
 * Parameters     : None
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void enable_dio1_interrupt_Tx(void)
{
    gpio_intr_enable(LORA_DIO1); // Enable interrupt
    ESP_LOGI(TAG, "DIO1 interrupt enabled on GPIO %d", LORA_DIO1);
}
/***********************************************************************************
 * Function name  : disable_dio1_interrupt
 * Description    : Disables the interrupt functionality for the DIO1 pin.
 * Parameters     : None
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void disable_dio1_interrupt_Tx(void)
{
    gpio_intr_disable(LORA_DIO1); // Disable interrupt
    ESP_LOGI(TAG, "DIO1 interrupt disabled on GPIO %d", LORA_DIO1);
}
void enable_dio1_interrupt_Rx(void)
{
    gpio_intr_enable(LORA_DIO1); // Enable interrupt
    ESP_LOGI(TAG, "DIO1 interrupt enabled on GPIO %d", LORA_DIO2);
}
/***********************************************************************************
 * Function name  : disable_dio1_interrupt
 * Description    : Disables the interrupt functionality for the DIO1 pin.
 * Parameters     : None
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void disable_dio1_interrupt_Rx(void)
{
    gpio_intr_disable(LORA_DIO1); // Disable interrupt
    ESP_LOGI(TAG, "DIO1 interrupt disabled on GPIO %d", LORA_DIO2);
}

void configure_lora_for_rx(void) {
	LoRaAppInit();
	//SetDio2AsRfSwitchCtrl(false);
    SetDioIrqParams(LLCC68_IRQ_RX_DONE | LLCC68_IRQ_TIMEOUT, LLCC68_IRQ_RX_DONE | LLCC68_IRQ_TIMEOUT, LLCC68_IRQ_NONE,LLCC68_IRQ_NONE);
    SetRx(0xFFFFFFFF);  // Continuous mode
}

// Function to initialize GPIO for DIO1 (TxDone) and DIO2 (RxDone)
void gpio_init_for_lora_irq(void) {
    tx_done_semaphore = xSemaphoreCreateBinary();
    rx_done_semaphore = xSemaphoreCreateBinary();
     // Configure GPIO for DIO1 (TxDone)
    gpio_config_t io_conf_tx;
    io_conf_tx.intr_type = GPIO_INTR_POSEDGE;  // Trigger interrupt on rising edge
    io_conf_tx.pin_bit_mask = (1ULL << LORA_DIO1);  // Select GPIO pin
    io_conf_tx.mode = GPIO_MODE_INPUT;  // Set as input mode
    io_conf_tx.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable pull-up
    io_conf_tx.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
    gpio_config(&io_conf_tx);

    // Configure GPIO for DIO2 (RxDone)
    gpio_config_t io_conf_rx;
    io_conf_rx.intr_type = GPIO_INTR_POSEDGE;  // Trigger interrupt on rising edge
    io_conf_rx.pin_bit_mask = (1ULL << LORA_DIO2);  // Select GPIO pin
    io_conf_rx.mode = GPIO_MODE_INPUT;  // Set as input mode
    io_conf_rx.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable pull-up
    io_conf_rx.pull_down_en = GPIO_PULLDOWN_DISABLE;  // Disable pull-down
    gpio_config(&io_conf_rx);

    // Install the ISR service only if not already installed
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        gpio_install_isr_service(0);
        isr_service_installed = true;
    }

    // Attach the interrupt handlers for DIO1 and DIO2
    gpio_isr_handler_add(LORA_DIO1, tx_done_isr_handler, (void*) LORA_DIO1);
    gpio_isr_handler_add(LORA_DIO2, rx_done_isr_handler, (void*) LORA_DIO2);
}


// ISR Handler for TxDone (DIO1)
void IRAM_ATTR tx_done_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
     disable_dio1_interrupt_Tx();
    // Give the semaphore to unblock the waiting task
    xSemaphoreGiveFromISR(tx_done_semaphore, &xHigherPriorityTaskWoken);
    // Yield to higher priority task if necessary
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// ISR Handler for RxDone (DIO2)
void IRAM_ATTR rx_done_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
     disable_dio1_interrupt_Rx();
    // Give the semaphore to unblock the waiting task
    xSemaphoreGiveFromISR(rx_done_semaphore, &xHigherPriorityTaskWoken);
    // Yield to higher priority task if necessary
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}


	
	
	








