//-----------------------------------------------------------------
///
///     \file LoRa_Mesh_Net.c
///
///     \brief LoRa Mesh Network framework driver
///
///     \author    Venkata Suresh
///
///     Location:     India
///
///     Project Name: MVEyE_1V0
///
///     \date Created 20AUG2024
///
///      Tools:  EspressifIDE
///      Device:   ESP32WROOM
///      Operating System: Windows 10
///      Java Runtime Version: 17.0.11+9
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
#include "stdlib.h"
#include "lora_llc68.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "string.h"
#include "esp_timer.h"
#include "LoRa_Mesh_app.h"
#include "esp_log.h" 
#include "LoRa_Mesh_Net.h"
#include "esp_task_wdt.h"
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "LORA_NET"

#define NET_TX(p, queue, timeout, h) \
	do { \
		if (h->netTx != NULL) h->netTx(); \
		xQueueSend(queue, p, timeout); \
		net_tx_done++; \
	} while (0)

#define NET_RX(p, queue, timeout, h) \
	do { \
		if (h->netRecv != NULL) h->netRecv(); \
		xQueueSend(queue, p, timeout); \
		net_rx_done++; \
	} while (0)

#define GEN_Route_Adv(p, dest) \
	do { \
		p.RouteData.hops = 0; \
		p.Header.type = TYPE_RA; \
		p.Header.MacHeader.dst = MAC_BROADCAST_ADDR; \
		p.Header.NetHeader.src = Route.getNetAddr(); \
		p.Header.NetHeader.dst = dest; \
		p.Header.NetHeader.hop = 0; \
		p.Header.NetHeader.subtype = SUB_RA; \
		p.Header.NetHeader.pid = ra_pid; \
		ra_pid++; \
	} while (0)

#define GEN_PING_ACK(q, p) \
	do { \
		q.Header.type = TYPE_PING; \
		q.Header.NetHeader.src = Route.getNetAddr(); \
		q.Header.NetHeader.dst = p.Header.NetHeader.src; \
		q.Header.NetHeader.hop = 0; \
		q.Header.NetHeader.ack = ACK; \
	} while (0)

#define ACK_TIMEOUT 800    // Timeout duration for ACK
#define ACK_MAX 3          // Maximum number of ACK retries
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
/* Network layer counters */
uint32_t net_tx_ack_ok;    
uint32_t net_tx_ack_retry; 
uint32_t net_tx_ack_fail;  
uint32_t net_tx_done;      
uint32_t net_tx_drop;      
uint32_t net_rx_done;      
uint32_t net_rx_drop;      
uint32_t net_fwd_done;     
uint32_t net_fwd_err;      
QueueHandle_t net_tx_buf;  
QueueHandle_t net_rx_buf;  
uint32_t ack_time;         
TaskHandle_t lora_net_tx_handle; 
uint8_t ack_wait_id = 0;    
static uint8_t ra_pid = 0;  
int16_t _last_seen_pid[255]; 
static int16_t _last_ra[255]; 
SemaphoreHandle_t m_ack_Semaphore; 

static void ra_handle(LoRaPkg* p, lora_net_hook *hook); 
/***********************************************************************************
 * Function name  : send_wait_ack
 * Description    : Handles sending a LoRa packet and waits for an ACK within the 
 *                  specified timeout. Retries sending if no ACK is received.
 * Parameters     : p     - Pointer to the LoRa package.
 *                  hook  - Pointer to the network hook for transmission handling.
 * Returns        : 0 on success (ACK received), -1 on failure (ACK not received).
 * Known Issues   : None
 * Note           :
 * Author         : C.venkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static int8_t send_wait_ack(LoRaPkg* p, lora_net_hook* hook)
{
    uint8_t tries = 0; 
    ack_time = RTOS_TIME; 
    while (tries < ACK_MAX) {
        ESP_LOGI(TAG, "L3: ack pid: %d, tries: %d", ack_wait_id, tries); 
        NET_TX(p, mac_tx_buf, portMAX_DELAY, hook); 
        xSemaphoreTake(m_ack_Semaphore, portMAX_DELAY);
       	ack_time = RTOS_TIME; 
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ACK_TIMEOUT)) != 0) 
        {
			net_tx_ack_ok++; 
           	ESP_LOGI(TAG, "L3: ack ok!"); 
         	return 0; 
       	}
        tries++; 
        net_tx_ack_retry++; 
    }
    net_tx_ack_fail++; 
    ESP_LOGI(TAG, "L3: ack failed!"); 
    return -1; 
}
/***********************************************************************************
 * Function name  : lora_net_tx_task
 * Description    : LoRa network transmission task. 
 * Parameters     : pvParameter - Pointer to network parameters (mac_net_param_t).
 * Returns        : None (void).
 * Known Issues   : None
 * Note           :  
 * Author         : C. VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void lora_net_tx_task(void *pvParameter)
{
    mac_net_param_t *param = (mac_net_param_t *)pvParameter; 
    lora_net_hook *hook = &(param->net_hooks); 
    LoRaPkg p, t; 
    int16_t nexthop; 
    
	//esp_task_wdt_add(NULL);
    while (1) {
		ESP_LOGI(TAG,"I am in LoRa Net_TX");
        if (xQueueReceive(net_tx_buf, &p, portMAX_DELAY) == pdPASS) 
        {
			ESP_LOGI(TAG, "Destination:%d",p.Header.NetHeader.dst);
		    ESP_LOGI(TAG, "local:%d",Route.getNetAddr());
            // Skip processing if the destination is local
            if (p.Header.NetHeader.dst == Route.getNetAddr())
            {
            	ESP_LOGI(TAG, "%d",p.Header.NetHeader.dst);
                continue;
            }
            if (p.Header.NetHeader.dst == NET_BROADCAST_ADDR) {
                p.Header.MacHeader.dst = MAC_BROADCAST_ADDR;
                NET_TX(&p, mac_tx_buf, portMAX_DELAY, hook);
                ESP_LOGI(TAG, "L3: Tx broadcast pkg!");
            } else {
                nexthop = Route.getRouteTo(p.Header.NetHeader.dst);
                ESP_LOGI(TAG,"next hop:%d",nexthop);
                if (nexthop < 0) {//no route found -1
                    net_tx_drop++; 
                    GEN_Route_Adv(t, p.Header.NetHeader.dst);
                    ESP_LOGI(TAG, "L3: No route to: 0x%02x, send Routte_Adv, pid: %d", p.Header.NetHeader.dst, t.Header.NetHeader.pid); // Debug log for RA
                    ESP_LOGI(TAG, "network Sending packet - type: %u, src: %u, dst: %u, temp: %.2f, volt: %u", 
                 	t.Header.type, p.Header.NetHeader.src, t.Header.NetHeader.dst,
                 	t.AppData.temp, t.AppData.volt);
                    NET_TX(&t, mac_tx_buf, portMAX_DELAY, hook);
                    ESP_LOGI(TAG, "nexthop NET Tx done!");
                } else {//route found
                    net_fwd_done++;
                    p.Header.MacHeader.dst = nexthop;
                    ESP_LOGI(TAG, "L3: Tx dst: 0x%02x, nh: 0x%02x", p.Header.NetHeader.dst, nexthop);

                    if (p.Header.type == TYPE_DATA && p.Header.NetHeader.ack == ACK) {
                        p.Header.NetHeader.pid = ack_wait_id;
                        if (send_wait_ack(&p, hook) < 0) {
                            Route.delRouteByNexthop(nexthop);
                        }
                        ack_wait_id++;
                    } else {
                        NET_TX(&p, mac_tx_buf, portMAX_DELAY, hook);
                        ESP_LOGI(TAG, "L3: Tx without ack done!"); 
                    }
                }
            }
        }
        ESP_LOGI(TAG, "NET Tx done!");
        vTaskDelay(pdMS_TO_TICKS(CAD_PERIOD_MS));
    }
}

/***********************************************************************************
 * Function name  : lora_net_rx_task
 * Description    : LoRa network reception task. 
 * Parameters     : pvParameter - Pointer to network parameters (mac_net_param_t).
 * Returns        : None (void).
 * Known Issues   : None
 * Note           : 
 * Author         : Venkata Suresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void lora_net_rx_task (void * pvParameter)
{
	mac_net_param_t *param = (mac_net_param_t *)pvParameter; 
	lora_net_hook *hook = &(param->net_hooks); 
	memset(_last_seen_pid, -1, 255); 
	memset(_last_ra, -1, 255); 
	LoRaPkg p, t; 
	//esp_task_wdt_add(NULL);
	
	while (1) {
		ESP_LOGI(TAG,"I am in LoRa Net_RX");
		if ( xQueueReceive(mac_rx_buf, &p, portMAX_DELAY) == pdPASS) // Receive packet from MAC layer queue
		{
			if (p.Header.NetHeader.dst == Route.getNetAddr()
					|| p.Header.NetHeader.dst == NET_BROADCAST_ADDR) {
				switch((uint8_t)p.Header.type) {
					case TYPE_DATA: 
						ESP_LOGI(TAG,"L3: recv TYPE_DATA"); 
						NET_RX(&p, net_rx_buf, 0, hook); 
						break;
					case TYPE_DATA_ACK: 
						break;
					case TYPE_PING: 
						if (p.Header.NetHeader.ack == ACK_NO) 
						{
							ESP_LOGI(TAG,"L3: recv TYPE_PING, send ping ack"); /* send pingack */
							GEN_PING_ACK(t, p); 
							xQueueSend(net_tx_buf, &t, portMAX_DELAY);
						} else {
							ESP_LOGI(TAG,"L3: recv ping ack"); /* It is a pingack */
							NET_RX(&p, net_rx_buf, 0, hook);
						}
						break;
					case TYPE_RA: 
						ESP_LOGI(TAG,"L3: recv TYPE_RA"); 
						ra_handle(&p, hook);
						break;
					default:
						net_rx_drop++; 
						break;
				}
			} else {
				if (p.Header.NetHeader.hop < MAX_HOPS ) {
					if (hook->netForward != NULL)
						hook->netForward();
					ESP_LOGI(TAG,"L3: forward, send to net_tx_buf");
					p.Header.NetHeader.hop++;
					xQueueSend(net_tx_buf, &p, portMAX_DELAY);
				} else {
					ESP_LOGI(TAG,"L3: hop max, drop!");
					net_fwd_err++; net_rx_drop++;
				}
			}
		}
		//esp_task_wdt_reset(); // Reset the WDT for this task
		vTaskDelay(pdMS_TO_TICKS(CAD_PERIOD_MS));
	}
}
/***********************************************************************************
 * Function name  : send_ra_respon
 * Description    : Handles the reception of a Route Advertisement (RA) package and sends 
 *                  a Route Advertisement Response (RA_RESPON) if the RA packet is new.
 * Parameters     : p - Pointer to the LoRa package (LoRaPkg*).
 *                  hook - Pointer to the network hook (lora_net_hook*).
 * Returns        : None (void).
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/	
static void send_ra_respon(LoRaPkg* p, lora_net_hook *hook)
{
	ESP_LOGI(TAG,"L3: recv RA from: 0x%02x, pid: %d, last pid: %d",
		p->Header.NetHeader.src ,p->Header.NetHeader.pid, _last_ra[p->Header.NetHeader.src]);
	
	if (p->Header.NetHeader.pid != _last_ra[p->Header.NetHeader.src])
	{
		ESP_LOGI(TAG,"L3: send RA_RESPON to ndst: 0x%02x!", p->Header.NetHeader.src);
		_last_ra[p->Header.NetHeader.src] = p->Header.NetHeader.pid;
		
		p->Header.NetHeader.subtype = SUB_RA_RESPON;
		p->Header.NetHeader.dst = p->Header.NetHeader.src;
		p->Header.NetHeader.src = Route.getNetAddr();
		p->Header.NetHeader.hop = 0;
		
		//NRF_LOG_HEX_DBG(p, sizeof(LoRaPkg));
		xQueueSend(net_tx_buf, p, portMAX_DELAY);
	} else {
		ESP_LOGI(TAG,"L3: dup RA, drop!");
	}
}
/***********************************************************************************
 * Function name  : ra_handle
 * Description    : Handles Route Advertisement (RA) packets by determining the subtype 
 *                  and calling the appropriate handler function (RA_RESPON, RA_FAIL, etc.).
 * Parameters     : p - Pointer to the LoRa package (LoRaPkg*).
 *                  hook - Pointer to the network hook (lora_net_hook*).
 * Returns        : None (void).
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static void ra_handle(LoRaPkg* p, lora_net_hook *hook)
{
	switch (p->Header.NetHeader.subtype) {
		case SUB_RA: send_ra_respon(p, hook);
			break;
		case SUB_RA_RESPON: break;
		case SUB_RA_FAIL: break; /* TODO */
		default: break;
	}
}


