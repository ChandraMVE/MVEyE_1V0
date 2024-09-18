//-----------------------------------------------------------------
///
///     \file LoRa_Mesh_app.c
///
///     \brief LoRa Mesh application framework driver
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
#include "esp_log.h" // For ESP32 logging functions
#include "LoRa-Mesh_Net.h"

/* Network layer counters */
uint32_t net_tx_ack_ok;    // Number of successful ACKs
uint32_t net_tx_ack_retry; // Number of ACK retries
uint32_t net_tx_ack_fail;  // Number of failed ACKs
uint32_t net_tx_done;      // Number of successfully transmitted packets
uint32_t net_tx_drop;      // Number of dropped packets (No route to host)
uint32_t net_rx_done;      // Number of successfully received packets
uint32_t net_rx_drop;      // Number of dropped received packets
uint32_t net_fwd_done;     // Number of successfully forwarded packets
uint32_t net_fwd_err;      // Number of forwarding errors

QueueHandle_t net_tx_buf;  // Queue for network transmit packets
QueueHandle_t net_rx_buf;  // Queue for network receive packets

uint32_t ack_time;         // Time when the last ACK was sent

static void ra_handle(LoRaPkg* p, lora_net_hook *hook); // Forward declaration for handling route advertisement

TaskHandle_t lora_net_tx_handle; // Task handle for network transmission

uint8_t ack_wait_id = 0;    // ID for the current ACK wait
static uint8_t ra_pid = 0;  // Route advertisement packet ID

int16_t _last_seen_pid[255]; // Array to track the last seen packet ID for each node
static int16_t _last_ra[255]; // Array to track the last route advertisement for each node

#define TAG "LORA_MAC"


// Macro to send a packet through the network layer
#define NET_TX(p, queue, timeout, h) \
	do { \
		if (h->netTx != NULL) h->netTx(); \
		xQueueSend(queue, p, timeout); \
		net_tx_done++; \
	} while (0)

// Macro to receive a packet from the network layer
#define NET_RX(p, queue, timeout, h) \
	do { \
		if (h->netRecv != NULL) h->netRecv(); \
		xQueueSend(queue, p, timeout); \
		net_rx_done++; \
	} while (0)

// Macro to generate a route advertisement (RA) packet
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

// Macro to generate a ping acknowledgment (PINGACK) packet
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

SemaphoreHandle_t m_ack_Semaphore; // Semaphore to synchronize ACK reception

// Function to send a packet and wait for an acknowledgment
static int8_t send_wait_ack(LoRaPkg* p, lora_net_hook* hook)
{
    uint8_t tries = 0; // Number of attempts to receive an ACK
    ack_time = esp_timer_get_time(); // Record the current time using ESP32 timer functions

    while (tries < ACK_MAX) {
        ESP_LOGI(TAG, "L3: ack pid: %d, tries: %d", ack_wait_id, tries); // Debug log for ACK attempt
        NET_TX(p, mac_tx_buf, portMAX_DELAY, hook); // Send the packet

        // Wait for semaphore indicating MAC layer transmission is done
        if (xSemaphoreTake(m_ack_Semaphore, portMAX_DELAY) == pdTRUE) {
            ack_time = esp_timer_get_time(); // Update the current time

            // Wait for task notification indicating ACK received within the timeout period
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ACK_TIMEOUT)) != 0) {
                net_tx_ack_ok++; // Increment ACK success counter
                ESP_LOGI(TAG, "L3: ack ok!"); // Debug log for successful ACK
                return 0; // Return success
            }
        }

        tries++; // Increment retry counter
        net_tx_ack_retry++; // Increment retry counter
    }

    net_tx_ack_fail++; // Increment ACK failure counter
    ESP_LOGI(TAG, "L3: ack failed!"); // Debug log for failed ACK
    return -1; // Return failure
}


// Task for handling network layer transmission
void lora_net_tx_task(void *pvParameter)
{
    mac_net_param_t *param = (mac_net_param_t *)pvParameter; // Get network parameters
    lora_net_hook *hook = &(param->net_hooks); // Get network hooks
    LoRaPkg p, t; // Packet variables
    int8_t nexthop; // Next hop for packet forwarding

    while (1) {
        if (xQueueReceive(net_tx_buf, &p, portMAX_DELAY) == pdPASS) // Receive packet from transmission queue
        {
            // Skip processing if the destination is local
            if (p.Header.NetHeader.dst == Route.getNetAddr())
                continue;

            // Determine the transmission scenario based on the destination address
            if (p.Header.NetHeader.dst == NET_BROADCAST_ADDR) {
                // Broadcast packet: use MAC broadcast address
                p.Header.MacHeader.dst = MAC_BROADCAST_ADDR;
                NET_TX(&p, mac_tx_buf, portMAX_DELAY, hook); // Send the broadcast packet
                ESP_LOGI(TAG, "L3: Tx broadcast pkg!"); // Debug log for broadcast
            } else {
                // Unicast packet: determine next hop
                nexthop = Route.getRouteTo(p.Header.NetHeader.dst); // Get the next hop for the destination
                if (nexthop < 0) {
                    // No route found: send route advertisement (RA)
                    net_tx_drop++; // Increment drop counter
                    GEN_Route_Adv(t, p.Header.NetHeader.dst); // Generate RA packet
                    ESP_LOGI(TAG, "L3: No route to: 0x%02x, send Routte_Adv, pid: %d", p.Header.NetHeader.dst, t.Header.NetHeader.pid); // Debug log for RA
                    NET_TX(&t, mac_tx_buf, portMAX_DELAY, hook); // Send RA packet
                } else {
                    // Forward packet to next hop
                    net_fwd_done++; // Increment forwarding done counter
                    p.Header.MacHeader.dst = nexthop; // Set the MAC address of the next hop
                    ESP_LOGI(TAG, "L3: Tx dst: 0x%02x, nh: 0x%02x", p.Header.NetHeader.dst, nexthop); // Debug log for forwarding

                    if (p.Header.type == TYPE_DATA && p.Header.NetHeader.ack == ACK) {
                        p.Header.NetHeader.pid = ack_wait_id; // Set packet ID for ACK
                        if (send_wait_ack(&p, hook) < 0) {
                            Route.delRouteByNexthop(nexthop); // Delete route if ACK failed
                        }
                        ack_wait_id++; // Increment ACK wait ID
                    } else {
                        NET_TX(&p, mac_tx_buf, portMAX_DELAY, hook); // Send packet without waiting for ACK
                        ESP_LOGI(TAG, "L3: Tx without ack done!"); // Debug log for non-ACK packet
                    }
                }
            }
        }
    }
}

// Task for handling network layer reception
void lora_net_rx_task (void * pvParameter)
{
	mac_net_param_t *param = (mac_net_param_t *)pvParameter; // Get network parameters
	lora_net_hook *hook = &(param->net_hooks); // Get network hooks
	memset(_last_seen_pid, -1, 255); // Initialize last seen packet IDs to -1
	memset(_last_ra, -1, 255); // Initialize last route advertisements to -1
	LoRaPkg p, t; // Packet variables
	
	while (1) {
		if ( xQueueReceive(mac_rx_buf, &p, portMAX_DELAY) == pdPASS) // Receive packet from MAC layer queue
		{
			/* Check if the packet is for this node or broadcast */
			if (p.Header.NetHeader.dst == Route.getNetAddr()
					|| p.Header.NetHeader.dst == NET_BROADCAST_ADDR) {
				switch((uint8_t)p.Header.type) {
					case TYPE_DATA: // Data packet
						ESP_LOGI(TAG,"L3: recv TYPE_DATA"); // Debug log for received data
						NET_RX(&p, net_rx_buf, 0, hook); // Send to network layer receive queue
						break;
					case TYPE_DATA_ACK: // Data acknowledgment
						break;
					case TYPE_RA: // Route advertisement
						if (_last_ra[p.Header.NetHeader.src] == p.Header.NetHeader.pid) // Check if already seen
							break;
						_last_ra[p.Header.NetHeader.src] = p.Header.NetHeader.pid; // Update last route advertisement
						ESP_LOGI(TAG,"L3: recv TYPE_RA from %d, pid %d", p.Header.NetHeader.src, p.Header.NetHeader.pid); // Debug log for RA
						NET_RX(&p, net_rx_buf, 0, hook); // Send RA to network layer receive queue
						break;
					case TYPE_PING: // Ping request
						GEN_PING_ACK(t, p); // Generate ping acknowledgment
						NET_TX(&t, mac_tx_buf, portMAX_DELAY, hook); // Send ping acknowledgment
						ESP_LOGI(TAG,"L3: recv TYPE_PING, sent PINGACK!"); // Debug log for ping acknowledgment
						break;
					default:
						break;
				}
			} else if (p.Header.NetHeader.dst == MAC_BROADCAST_ADDR) {
				NET_RX(&p, net_rx_buf, 0, hook); // Broadcast packet: send to network layer receive queue
			}
		}
	}
}



