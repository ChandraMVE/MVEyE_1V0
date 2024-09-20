//-----------------------------------------------------------------
///
///     \file lora_Mesh_app.c
///
///     \brief LoRa Mesh driver
///
///
///     \author       Keerthi Mallesh 
///
///     Location:     India
///
///     Project Name: MVEyE_1V0
///
///     \date Created 20SEP2024
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

#include "lora_llc68.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"
#include "esp_timer.h"
#include "LoRa_Mesh_app.h"
#include "LoRa_Mesh_Net.h"
#include "Route.h"
#include "esp_timer.h"
#include "lora_app.h"

//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================

#define TAG "LoRaMeshApp"

#define ACK_CONFIG ACK

#define APP_TX(p, queue, timeout) \
	do { \
		p.Header.NetHeader.hop = 0; \
		p.Header.NetHeader.src = Route.getNetAddr(); \
		xQueueSend(queue, &p, 0); \
	} while (0)
		
#define DST						0x1

#define PING_TIMEOUT 3000

#define STAT_PERIOD_MS			10000

#define NET_TX_BUF_NUM 8
#define NET_RX_BUF_NUM 8
#define MAC_TX_BUF_NUM 8
#define MAC_RX_BUF_NUM 8

//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================

typedef struct {
	
	uint8_t upload_node;
	uint8_t ping_dest;
	
} Ping_t;

TaskHandle_t app_gw_handle;
TaskHandle_t app_ping_handle;
QueueHandle_t app_ping_buf;
int8_t ping_dst = -1;
TaskHandle_t app_stat_handle;
QueueHandle_t app_stat_buf;
TaskHandle_t app_recv_handle;
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================

/***********************************************************************************
 * Function name  : app_ping_task
 *
 * Description    : app ping task.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
 * date           : 20SEP2024
 ***********************************************************************************/
void app_ping_task(void * pvParameter)
{
    Ping_t ping_req;
    uint32_t ret;
    int16_t time;
    LoRaPkg p = {
        .Header.type = TYPE_PING,                  
        .Header.NetHeader.ack = ACK_NO,            
    };
    
    LoRaPkg u = {
        .Header.type = TYPE_DATA,                  
        .Header.NetHeader.ack = ACK_CONFIG,        
        .Header.NetHeader.subtype = SUB_CONTROL,   
        .AppData.custom[0] = CMD_UPLOAD_PING,      
    };

    while (1) {
        if (xQueueReceive(app_ping_buf, &ping_req, portMAX_DELAY) == pdTRUE)
        {
            p.Header.NetHeader.dst = ping_dst = ping_req.ping_dest;
            time = (xTaskGetTickCount() * 1000) >> 10;  // Convert tick count to milliseconds
            APP_TX(p, net_tx_buf, 0);
            ret = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PING_TIMEOUT));
            if (ret != 0) {
                time = (uint16_t)(((xTaskGetTickCount() * 1000) >> 10) - time);
            } else {
                time = -1;
            }
            if (Route.getNetAddr() == ping_req.upload_node) {
                if (time < 0) {
                    ESP_LOGI(TAG,"0x%02x -> 0x%02x, timeout!", Route.getNetAddr(), ping_req.ping_dest);
                } else {
                    ESP_LOGI(TAG,"0x%02x -> 0x%02x, %d ms", Route.getNetAddr(), ping_req.ping_dest, time);
                }
            } else {
                u.Header.NetHeader.dst = ping_req.upload_node;
                u.AppData.custom[0] = CMD_UPLOAD_PING;
                u.AppData.custom[1] = ping_req.ping_dest;
                memcpy(u.AppData.custom + 2, &time, sizeof(time));
                APP_TX(u, net_tx_buf, 0);
            }
        }
    }
}

/***********************************************************************************
 * Function name  : print_data
 *
 * Description    : print data.
 * Parameters     : pointer p
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
 * date           : 20SEP2024
 ***********************************************************************************/
void print_data (LoRaPkg *p)
{
	ESP_LOGI(TAG,"===>");
	ESP_LOGI(TAG,"NET: 0x%02x, MAC: 0x%02x(%d), Rssi: %d, Snr: %d", p->Header.NetHeader.src, 
		p->Header.MacHeader.src, p->Header.NetHeader.hop, p->stat.RssiPkt, p->stat.SnrPkt);
	ESP_LOGI(TAG,"<===");
}

/***********************************************************************************
 * Function name  : print_pingret
 *
 * Description    : print pingret.
 * Parameters     : pointer p
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
 * date           : 20SEP2024
 ***********************************************************************************/
 
void print_pingret (LoRaPkg *p)
{
	int16_t ping_ret;
	// Copy from the buffer to ping_ret
	memcpy(&ping_ret, p->AppData.custom + 2, sizeof(ping_ret));
	//memcpy(p->AppData.custom + 2, &ping_ret, sizeof(ping_ret));
	if (ping_ret < 0) {
		ESP_LOGI(TAG,"0x%02x -> 0x%02x, timeout!", 
			p->Header.NetHeader.src, p->AppData.custom[1]);
	} else {
		ESP_LOGI(TAG,"0x%02x -> 0x%02x, %d ms", 
			p->Header.NetHeader.src, p->AppData.custom[1], ping_ret);
	}
}

/***********************************************************************************
 * Function name  : notify_ping_app
 *
 * Description    : notify ping app.
 * Parameters     : pointer p
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh 
 * date           : 20SEP2024
 ***********************************************************************************/
 
void notify_ping_app (LoRaPkg *p)
{
	Ping_t ping_req;
	ping_req.ping_dest = p->AppData.custom[1];
	ping_req.upload_node = p->Header.NetHeader.src;
	xQueueSend(app_ping_buf, &ping_req, 0);
}

/***********************************************************************************
 * Function name  : app_control
 *
 * Description    : app control
 * Parameters     : pointer p
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh 
 * date           : 20SEP2024
 ***********************************************************************************/
 
void app_control (LoRaPkg *p)
{
	uint8_t subtype = (uint8_t)p->Header.NetHeader.subtype;

	switch (subtype) {
		case SUB_DATA:
			print_data(p);
			break;
		case SUB_CONTROL:
			switch (p->AppData.custom[0]) {
				case CMD_PING: 
					notify_ping_app(p);
					break;
				case CMD_UPLOAD_PING: print_pingret(p);
					break;
				case CMD_LED: //led_control(p);
					break;
				case CMD_GET_STAT: /* TODO */
					break;
				case CMD_UPLOAD_STAT: /* TODO */
					break;
				default: break;
			}
			break;
		default:
			break;
	}
}

/***********************************************************************************
 * Function name  : app_gw_task
 *
 * Description    : secondary device.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 20SEP2024
 ***********************************************************************************/
 
void app_gw_task (void * pvParameter)
{
	Ping_t ping_req;
	ping_req.upload_node = *(uint8_t *)pvParameter;
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(5000));
		ping_req.ping_dest = 0x2;
		xQueueSend(app_ping_buf, &ping_req, 0);
		
		vTaskDelay(pdMS_TO_TICKS(5000));
		ping_req.ping_dest = 0x3;
		xQueueSend(app_ping_buf, &ping_req, 0);	
	}
}

/***********************************************************************************
 * Function name  : app_recv_task
 *
 * Description    : app_recv_task.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 20SEP2024
 ***********************************************************************************/
 
void app_recv_task (void * pvParameter)
{
	LoRaPkg p; uint8_t type;

	while (1) {
		if ( xQueueReceive(net_rx_buf, &p, portMAX_DELAY) == pdPASS )
		{
			type = (uint8_t)p.Header.type;
			switch (type) {
				case TYPE_DATA: app_control(&p);
					break;
				case TYPE_PING: /* ping ack, without payload */
					if (ping_dst == p.Header.NetHeader.src)
						xTaskNotifyGive(app_ping_task);
					break;
				default: break;
			}
		}
	}
}
/***********************************************************************************
 * Function name  : app_stat_task
 *
 * Description    : app_stat_task.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 20SEP2024
 ***********************************************************************************/
void app_stat_task ( void * pvParameter)
{
	uint8_t dst; LoRaPkg t = {
		.Header.type = TYPE_DATA,
		.Header.NetHeader.subtype = SUB_CONTROL,
		.Header.NetHeader.ack = ACK_CONFIG,
		.AppData.custom[0] = CMD_UPLOAD_STAT,
	};
	
	while (1) {
		if ( xQueueReceive(app_stat_buf, &dst, pdMS_TO_TICKS(STAT_PERIOD_MS)) == pdTRUE ) {
			ESP_LOGI(TAG,"L7: CMD_UPLOAD_STAT recv!");
			t.Header.NetHeader.dst = dst;
			memcpy(t.AppData.custom + 1 , &mac_tx_done, sizeof(mac_tx_done));
			APP_TX(t, net_tx_buf, 0);
		}
		
		ESP_LOGI(TAG,"===>");
		ESP_LOGI(TAG,"PHY CAD det/done: [%"PRIu32"], [%"PRIu32"]", phy_cad_det, phy_cad_done);
		ESP_LOGI(TAG,"PHY Rx err/timeout/done: [%"PRIu32"],[%"PRIu32"], [%"PRIu32"]", phy_rx_err, phy_rx_timeout, phy_rx_done);
		ESP_LOGI(TAG,"MAC Tx:[%"PRIu32"]; Rx: [%"PRIu32"]", mac_tx_done, mac_rx_done);
		ESP_LOGI(TAG,"NET Rx: [%"PRIu32"]; Tx acked: [%"PRIu32"] (retry: [%"PRIu32"], fail: [%"PRIu32"])", net_rx_done, net_tx_ack_ok, 
			net_tx_ack_retry, net_tx_ack_fail);
		ESP_LOGI(TAG,"<===");
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
 
 void mac_tx_start_hook (void) {
	//nrf_gpio_pin_set(LED_YELLOW);
}

void mac_rx_start_hook (void) {
	//nrf_gpio_pin_set(LED_GREEN);
}

void mac_tx_end_hook (void) {
	//nrf_gpio_pin_clear(LED_YELLOW);
}

void mac_rx_end_hook (void) {
	//nrf_gpio_pin_clear(LED_GREEN);
}

void mac_cad_done_hook (void) {
	//nrf_gpio_pin_toggle(LED_RED);
}

/***********************************************************************************
 * Function name  : mesh_task
 *
 * Description    : mesh_task.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
 * date           : 20SEP2024
 ***********************************************************************************/
void mesh_task( void *pvParameters )
{ 
	ESP_LOGI(TAG,"HI, i am in main");
	vTaskDelay(pdMS_TO_TICKS(1000));
    TimerHandle_t LinkQMap_timer; 
    esp_log_level_set("*", ESP_LOG_INFO); 
    ESP_LOGI(TAG, "Compile Time: %s %s", __DATE__, __TIME__); 
    LoRaAppInit();
    SetCadParams(5, 0x03, 0x0A, 0x00, 1000); // Set CAD parameters: symbol number 5, detection peak 0x03, detection minimum 0x0A, exit mode 0x00, and timeout 1000 ms.
    SetCad(); // Apply the CAD settings.

	//get status

    // Set up MAC layer parameters
    static mac_net_param_t param = {
        .mac_hooks.macCadDone = mac_cad_done_hook, 
        .mac_hooks.macRxStart = mac_rx_start_hook, 
        .mac_hooks.macTxStart = mac_tx_start_hook, 
        .mac_hooks.macRxEnd = mac_rx_end_hook, 
        .mac_hooks.macTxEnd = mac_tx_end_hook, 
    };

    // Create FreeRTOS queues for communication between tasks
    net_tx_buf = xQueueCreate(NET_TX_BUF_NUM, sizeof(LoRaPkg)); 
    net_rx_buf = xQueueCreate(NET_RX_BUF_NUM, sizeof(LoRaPkg)); 
    mac_tx_buf = xQueueCreate(MAC_TX_BUF_NUM, sizeof(LoRaPkg)); 
    mac_rx_buf = xQueueCreate(MAC_RX_BUF_NUM, sizeof(LoRaPkg)); 
    app_ping_buf = xQueueCreate(8, sizeof(Ping_t)); 
    app_stat_buf = xQueueCreate(8, sizeof(uint8_t)); 

    // Create semaphores for synchronization
    m_irq_Semaphore = xSemaphoreCreateBinary(); 
    m_ack_Semaphore = xSemaphoreCreateBinary(); 

    ESP_LOGI(TAG, "LoRaPkg max size: %d", SIZE_PKG_MAX); // Log the maximum size of LoRa package.

   // Addr_t addr; // Declare an address variable (unused in the current code).

    do {
        if (net_tx_buf && net_rx_buf && mac_tx_buf && mac_rx_buf &&
            app_ping_buf && app_stat_buf && m_irq_Semaphore && m_ack_Semaphore) {
            xTaskCreate(lora_mac_task, "lora_mac", configMINIMAL_STACK_SIZE + 200, 
                &param, 3, &lora_mac_handle); // Task for LoRa MAC layer.
            xTaskCreate(lora_net_tx_task, "lora_net_tx", configMINIMAL_STACK_SIZE + 200, 
                &param, 2, &lora_net_tx_handle); // Task for LoRa network transmission.
            xTaskCreate(lora_net_rx_task, "lora_net_rx", configMINIMAL_STACK_SIZE + 200, 
                &param, 2, &lora_net_rx_handle); // Task for LoRa network reception.

            xTaskCreate(app_recv_task, "app_recv", configMINIMAL_STACK_SIZE + 200, 
                NULL, 1, &app_recv_handle); // Task for receiving application data.

            xTaskCreate(app_stat_task, "app_stat", configMINIMAL_STACK_SIZE + 100, 
                NULL, 1, &app_stat_handle); // Task for handling application statistics.

            xTaskCreate(app_ping_task, "app_ping", configMINIMAL_STACK_SIZE + 100, 
                NULL, 1, &app_ping_handle); // Task for handling application pings.

            LinkQMap_timer = xTimerCreate("LQM_timer", pdMS_TO_TICKS(LINKQMAP_CLEAR_PERIOD), 
                pdTRUE, 0, Route.clearLinkQuailtyMapTimer); // Timer to clear the link quality map at intervals defined by LINKQMAP_CLEAR_PERIOD.
            xTimerStart(LinkQMap_timer, 0); // Start the timer.

            ESP_LOGI(TAG," lora mesh starting..."); // Log the start of the LoRa mesh network.
            //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            vTaskStartScheduler(); // Start the FreeRTOS scheduler, which will manage the tasks.
        } else {
            ESP_LOGI(TAG,"ERROR! xQueueCreate() or xSemaphoreCreateBinary() failed!"); // Log an error message if any queue or semaphore creation failed.
            break; // Exit the do-while loop.
        }
    } while (0); // Loop only once.
}
/***********************************************************************************
 * Function name  : create_mesh_task
 *
 * Description    : create_mesh_task.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh 
 * date           : 20SEP2024
 ***********************************************************************************/
void create_mesh_task( void )
{
	 	xTaskCreate(&mesh_task, "LoRa_Mesh", 1024*4, NULL, 5, NULL);
}
