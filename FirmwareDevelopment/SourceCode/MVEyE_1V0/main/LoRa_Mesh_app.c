//-----------------------------------------------------------------
///
///     \file lora_Mesh_app.c
///
///     \brief LoRa Mesh driver
///
///
///     \author       VenkataSuresh 
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
#include "esp_log.h"
#include "string.h"
#include "esp_timer.h"
#include "LoRa_Mesh_app.h"
#include "LoRa_Mesh_Net.h"
#include "LoRa_Mesh_Route.h"
#include "esp_timer.h"
#include "lora_app.h"
#include "esp_task_wdt.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <sys/_types.h>

//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================

#define TAG "LoRaMeshApp"
#define VALID_IDENTIFIER 0xdeadbeef
#define ACK_CONFIG ACK

#define APP_TX(p, queue, timeout) \
	do { \
		p.Header.NetHeader.hop = 0; \
		p.Header.NetHeader.src = Route.getNetAddr(); \
		xQueueSend(queue, &p, 0); \
	} while (0)
		
#define DST						0x2
#define Self				    0x2
#define PING_TIMEOUT 			5000

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
Addr_t addr;
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
static TaskHandle_t app_upload_handle;
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================
/***********************************************************************************
 * Function name  : app_upload_task
 *
 * Description    : app app_upload_task.
 * Parameters     : None
 * Returns        : None
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
 * date           : 20SEP2024
 ***********************************************************************************/
/*static void app_upload_task (void * pvParameter)
{
	//float volatile temp;
	
	LoRaPkg p = {
		.Header.type = TYPE_DATA,
		.Header.NetHeader.src = Self,
		.Header.NetHeader.dst = DST,
		.Header.NetHeader.subtype = SUB_DATA,
		.Header.NetHeader.ack = ACK_CONFIG,
	};

	while (1)
	{    ESP_LOGI(TAG,"I am in upload Task ");
		 p.AppData.temp = 43.4;
		 p.AppData.volt = 35 ;
		 APP_TX(p, net_tx_buf, 0);
		 ESP_LOGI(TAG," Upload complete");
		 vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}*/

static void app_upload_task(void *pvParameter)
{
    LoRaPkg p = {
        .Header.type = TYPE_DATA,
        .Header.NetHeader.src = Self,
        .Header.NetHeader.dst = DST,
        .Header.NetHeader.subtype = SUB_DATA,
        .Header.NetHeader.ack = ACK_CONFIG
    };

    while (1)
    {    
        ESP_LOGI(TAG, "In upload Task");

        // Set sample data
        p.AppData.temp = 43.4; // Example temperature
        p.AppData.volt = 35;   // Example voltage

        ESP_LOGI(TAG, "Size of LoRaPkg: %zu", sizeof(LoRaPkg)); // Log the size of the package

        // Log header before sending
        ESP_LOGI(TAG, "Sending packet - type: %u, src: %u, dst: %u, temp: %.2f, volt: %u", 
                 p.Header.type, p.Header.NetHeader.src, p.Header.NetHeader.dst,
                 p.AppData.temp, p.AppData.volt);

        // Check header type validity
        if (p.Header.type >= TYPE_MAX) {
            ESP_LOGE(TAG, "Error: Invalid header type: %u", p.Header.type);
            continue; // Skip sending this packet
        }

        // Send the packet using the APP_TX macro
        APP_TX(p, net_tx_buf, 0);
        ESP_LOGI(TAG, "Upload complete");

        // Delay before sending the next packet
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


/***********************************************************************************
 * Function name  : app_ping_task
 *
 * Description    : app ping task.
 * Parameters     : None
 * Returns        : None
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
        .Header.NetHeader.dst = DST,
                   
    };
    
    LoRaPkg u = {
        .Header.type = TYPE_DATA,                  
        .Header.NetHeader.ack = ACK_CONFIG,        
        .Header.NetHeader.subtype = SUB_CONTROL,   
        .AppData.custom[0] = CMD_UPLOAD_PING, 
        .Header.NetHeader.dst = DST,     
    };

    while (1) {
		ESP_LOGI(TAG,"I am in Ping Task ");
        if (xQueueReceive(app_ping_buf, &ping_req, portMAX_DELAY) == pdTRUE)
        {
			ping_dst = ping_req.ping_dest;
            p.Header.NetHeader.dst = ping_dst;
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
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/***********************************************************************************
 * Function name  : print_data
 * Description    : print data.
 * Parameters     : pointer p
 * Returns        : None
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
	ESP_LOGI(TAG,"__upd__[%d][%f]%d]\n",p->Header.NetHeader.src,(p->AppData.temp),p->AppData.volt);
	ESP_LOGI(TAG,"<===");
	
}

/***********************************************************************************
 * Function name  : print_pingret
 *
 * Description    : print pingret.
 * Parameters     : pointer p
 * Returns        : None
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
 * Description    : notify ping app.
 * Parameters     : pointer p
 * Returns        : None
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
 * Description    : app control
 * Parameters     : pointer p
 * Returns        : None
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh 
 * date           : 20SEP2024
 ***********************************************************************************/
void app_control (LoRaPkg *p)
{
	uint8_t subtype = (uint8_t)p->Header.NetHeader.subtype;
	ESP_LOGI(TAG,"I am in app control ");

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
 * Description    : secondary device.
 * Parameters     : None
 * Returns        : None
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
 * date           : 20SEP2024
 ***********************************************************************************/
void app_gw_task(void *pvParameter)
{
    if (pvParameter == NULL) {
        ESP_LOGE(TAG, "Invalid parameter passed to app_gw_task");
        vTaskDelete(NULL); // Exit the task if the parameter is invalid
        return;
    }

    Ping_t ping_req;
    ping_req.upload_node = *(uint8_t *)pvParameter; // Safe to dereference now

    while (1) {
        ESP_LOGI(TAG, "I am gateway ");
        vTaskDelay(pdMS_TO_TICKS(5000));

        ping_req.ping_dest = 0x2;
        if (xQueueSend(app_ping_buf, &ping_req, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to send ping to 0x2");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
        ping_req.ping_dest = 0x3;
        if (xQueueSend(app_ping_buf, &ping_req, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to send ping to 0x3");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/***********************************************************************************
 * Function name  : app_recv_task
 * Description    : app_recv_task.
 * Parameters     : None
 * Returns        : None
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
 * date           : 20SEP2024
 ***********************************************************************************/
void app_recv_task (void * pvParameter)
{
	LoRaPkg p; uint8_t type;

	while (1) {
		ESP_LOGI(TAG,"I am in app Receive Task ");
		if ( xQueueReceive(net_rx_buf, &p, portMAX_DELAY) == pdPASS )
		{
			type = (uint8_t)p.Header.type;
			switch (type) {
				case TYPE_DATA: app_control(&p);
					break;
				case TYPE_PING: /* ping ack, without payload */
					if (ping_dst == p.Header.NetHeader.src)
						xTaskNotifyGive(app_ping_handle);
					break;
				default: break;
			}
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
/***********************************************************************************
 * Function name  : app_stat_task
 * Description    : app_stat_task.
 * Parameters     : None
 * Returns        : None
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
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
		ESP_LOGI(TAG,"I am in app Stat Task ");
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
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
/***********************************************************************************
 * Function name  : device_secondary
 * Description    : secondary device.
 * Parameters     : None
 * Returns        : None
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh
 * date           : 11SEP2024
 ***********************************************************************************/
 
 void mac_tx_start_hook (void) {
	//nrf_gpio_pin_set(LED_YELLOW);
	ESP_LOGI(TAG,"TxStart LED!");
}

void mac_rx_start_hook (void) {
	ESP_LOGI(TAG,"RXStart LED!");
}

void mac_tx_end_hook (void) {
	//nrf_gpio_pin_clear(LED_YELLOW);
	ESP_LOGI(TAG,"TXEND LED!");
}

void mac_rx_end_hook (void) {
	ESP_LOGI(TAG,"RXEND LED!");
}

void mac_cad_done_hook (void) {
	//nrf_gpio_pin_toggle(LED_RED);
	ESP_LOGI(TAG,"CadDoneLED!");
}

void mac_cad_detect_hook(void) {
    ESP_LOGI(TAG, "CAD Detection LED !");
}

/***********************************************************************************
 * Function name  : LoRa_Mesh
 * Description    : create_mesh_task.
 * Parameters     : None
 * Returns        : None
 * Known Issues   :
 * Note           :
 * author         : VenkataSuresh 
 * date           : 20SEP2024
 ***********************************************************************************/
void LoRa_Mesh(){
	
	ESP_LOGI(TAG,"HI, i am in main");
	vTaskDelay(pdMS_TO_TICKS(1000));
    TimerHandle_t LinkQMap_timer; 
   
    esp_log_level_set("*", ESP_LOG_INFO); 
    ESP_LOGI(TAG, "Compile Time: %s %s", __DATE__, __TIME__); 
    LoRaAppInit();
    SetCadParams(LLCC68_CAD_ON_16_SYMB, CAD_DET_PEAK, CAD_DET_MIN, LORA_CAD_RX, 0);
    // symbols need to use 16
    static mac_net_param_t param = {
        .mac_hooks.macCadDone = mac_cad_done_hook,
        .mac_hooks.macCadDetect = mac_cad_detect_hook,  
        .mac_hooks.macRxStart = mac_rx_start_hook, 
        .mac_hooks.macTxStart = mac_tx_start_hook, 
        .mac_hooks.macRxEnd = mac_rx_end_hook, 
        .mac_hooks.macTxEnd = mac_tx_end_hook, 
    };
	ESP_LOGI(TAG, "param completed:");
    // Create FreeRTOS queues for communication between tasks
    net_tx_buf = xQueueCreate(NET_TX_BUF_NUM, sizeof(LoRaPkg)); 
    net_rx_buf = xQueueCreate(NET_RX_BUF_NUM, sizeof(LoRaPkg)); 
    mac_tx_buf = xQueueCreate(MAC_TX_BUF_NUM, sizeof(LoRaPkg)); 
    mac_rx_buf = xQueueCreate(MAC_RX_BUF_NUM, sizeof(LoRaPkg)); 
    app_ping_buf = xQueueCreate(8, sizeof(Ping_t)); 
    app_stat_buf = xQueueCreate(8, sizeof(uint8_t)); 
	ESP_LOGI(TAG, "Queue completed:");
    // Create semaphores for synchronization
    m_irq_Semaphore = xSemaphoreCreateBinary(); 
    m_ack_Semaphore = xSemaphoreCreateBinary(); 

    ESP_LOGI(TAG, "LoRaPkg max size: %d", SIZE_PKG_MAX); // Log the maximum size of LoRa package.
    
    uint32_t customer1 = 0;
    
	/*esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    	ESP_ERROR_CHECK(nvs_flash_erase());
    	ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret); 
	nvs_handle_t handle;
	ret = nvs_open("storage", NVS_READWRITE, &handle);
	if (ret != ESP_OK) {
    	ESP_LOGE(TAG, "ERROR! Failed to open NVS handle: %s", esp_err_to_name(ret));
    	return; 
	}

	uint32_t customer0 = 0;
	uint32_t customer1 = 0;

	// Read CUSTOMER0
	ret = nvs_get_u32(handle, "CUSTOMER0", &customer0);
	if (ret == ESP_ERR_NVS_NOT_FOUND) {
    	ESP_LOGW(TAG, "CUSTOMER0 not found, setting to VALID_IDENTIFIER");
    	customer0 = VALID_IDENTIFIER;
    	ret = nvs_set_u32(handle, "CUSTOMER0", customer0);
    	ESP_ERROR_CHECK(ret); 
    	nvs_commit(handle); // Commit the changes
	} else if (ret != ESP_OK) {
    	ESP_LOGE(TAG, "ERROR! Could not read CUSTOMER0 from NVS: %s", esp_err_to_name(ret));
    	nvs_close(handle);
    	return;
	}

	ret = nvs_get_u32(handle, "CUSTOMER1", &customer1);
	if (ret == ESP_ERR_NVS_NOT_FOUND || customer1 != 0x01) {
    	ESP_LOGW(TAG, "CUSTOMER1 not found or not set to 0x01, setting it to 0x01");
    	customer1 = DST; // Set CUSTOMER1 to 0x01
    	ret = nvs_set_u32(handle, "CUSTOMER1", customer1);
    	ESP_ERROR_CHECK(ret);
    	nvs_commit(handle); // Commit the changes
	} else if (ret != ESP_OK) {
    	ESP_LOGE(TAG, "ERROR! Could not read CUSTOMER1 from NVS: %s", esp_err_to_name(ret));
    	nvs_close(handle);
    	return;
	}

	// Check if CUSTOMER0 is valid
	if (customer0 == VALID_IDENTIFIER) {
    	addr.mac = addr.net = customer1; // Set the address from CUSTOMER1
    	Route.initRouteTable(&addr);
    	ESP_LOGI(TAG, "Using NVS as addr: 0x%08x", (unsigned int)customer1);
	} else {
    	ESP_LOGE(TAG, "ERROR! Invalid NVS CUSTOMER0 value: 0x%08x", (unsigned int)customer0);
	}

	nvs_close(handle); // Close the NVS handle*/
   addr.mac =Self;
   addr.net =Self;
   customer1 = Self;
   Route.initRouteTable(&addr);
   ESP_LOGI(TAG, "net.current:%d",addr.net);
   ESP_LOGI(TAG, "mac.current:%d",addr.mac);
   
        if (net_tx_buf && net_rx_buf && mac_tx_buf && mac_rx_buf &&
            app_ping_buf && app_stat_buf && m_irq_Semaphore && m_ack_Semaphore) {
          // xTaskCreate(lora_mac_task, "lora_mac", configMINIMAL_STACK_SIZE + 3500, &param, 3, &lora_mac_handle);
          
          	xTaskCreate(&lora_mac_receive_task, "LoRa_MAC_Receive_Task", 4096, (void*)&param, 3, NULL);
    		xTaskCreate(&lora_mac_transmit_task, "LoRa_MAC_Transmit_Task", 4096, (void*)&param, 3, NULL);
                 // Task for LoRa MAC layer.
            xTaskCreate(lora_net_tx_task, "lora_net_tx", configMINIMAL_STACK_SIZE + 1500, 
                &param, 2, &lora_net_tx_handle); // Task for LoRa network transmission.
            xTaskCreate(lora_net_rx_task, "lora_net_rx", configMINIMAL_STACK_SIZE + 1500, 
                &param, 2, &lora_net_rx_handle); // Task for LoRa network reception.
    		if (customer1 == 0x1) 
    		{
        		/*// Code for pinging to ID 0x2 and 0x3 periodically
        		uint8_t upload_node_id = 0x1; 
				if (xTaskCreate(app_gw_task, "app_gw", configMINIMAL_STACK_SIZE + 3000, (void *)&upload_node_id, 1, &app_gw_handle) != pdPASS) {
    				ESP_LOGE(TAG, "Failed to create app_gw_task");
				} else {
    				ESP_LOGI(TAG, "app_gw_task created successfully");
				}*/
				xTaskCreate(app_upload_task, "app_upload", configMINIMAL_STACK_SIZE + 1000, NULL, 1, &app_upload_handle);
				ESP_LOGI(TAG, "app_upload_task created successfully");
    		} else {
        		// Create the upload task if CUSTOMER1 is not 1
        		/*xTaskCreate(app_upload_task, "app_upload", configMINIMAL_STACK_SIZE + 1000, NULL, 1, &app_upload_handle);
        		ESP_LOGI(TAG, "app_upload_task created successfully");*/
        	}
            xTaskCreate(app_recv_task, "app_recv", configMINIMAL_STACK_SIZE + 1500, 
                NULL, 1, &app_recv_handle); // Task for receiving application data.

            xTaskCreate(app_stat_task, "app_stat", configMINIMAL_STACK_SIZE + 1500, 
                NULL, 1, &app_stat_handle); // Task for handling application statistics.

            xTaskCreate(app_ping_task, "app_ping", configMINIMAL_STACK_SIZE + 1000, 
                NULL, 1, &app_ping_handle); // Task for handling application pings.

           	LinkQMap_timer = xTimerCreate("LQM_timer", pdMS_TO_TICKS(LINKQMAP_CLEAR_PERIOD), 
                pdTRUE, 0, Route.clearLinkQuailtyMapTimer); // Timer to clear the link quality map at intervals defined by LINKQMAP_CLEAR_PERIOD.
            xTimerStart(LinkQMap_timer, 0); // Start the timer.
			
            ESP_LOGI(TAG," lora mesh starting...");
            //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
          // vTaskStartScheduler(); // Start the FreeRTOS scheduler, which will manage the tasks.
           ESP_LOGI(TAG," Scheduler Done");
        } else {
            ESP_LOGI(TAG,"ERROR! xQueueCreate() or xSemaphoreCreateBinary() failed!"); 
        }
}

