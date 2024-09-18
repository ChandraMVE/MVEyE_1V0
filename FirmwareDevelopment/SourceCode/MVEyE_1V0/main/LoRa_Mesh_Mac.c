#include "lora_llc68.h"
#include "stdlib.h"            // Standard library for memory management and utilities
#include "string.h"
#include "freertos/FreeRTOS.h"           // FreeRTOS for task scheduling
#include "freertos/semphr.h"    // Semaphore management in FreeRTOS
#include "esp_timer.h"             // Timer utilities in FreeRTOS
#include "Route.h"         // Routing table management for the mesh network
#include "esp_log.h"
#include "LoRa-Mesh_Net.h"
/* PHY (Physical layer) counters to track radio events */
uint32_t phy_cad_done;          // Number of successful CAD (Channel Activity Detection)
uint32_t phy_cad_det;           // Number of CAD detections
uint32_t phy_rx_done;           // Number of successful RX operations
uint32_t phy_rx_err;            // Number of RX errors (CRC/Header error)
uint32_t phy_rx_timeout;        // Number of RX timeouts
uint32_t phy_tx_done;           // Number of successful TX operations
uint32_t phy_tx_err;            // Number of TX errors

/* MAC (Medium Access Control) layer counters */
uint32_t mac_rx_done;           // Number of successfully received MAC frames
uint32_t mac_rx_drop;           // Number of dropped MAC frames
uint32_t mac_tx_done;           // Number of successfully transmitted MAC frames
uint32_t mac_ack_respon;        // Number of ACK responses generated

QueueHandle_t mac_tx_buf;       // Queue for MAC transmission buffer
QueueHandle_t mac_rx_buf;       // Queue for MAC reception buffer

static uint8_t tx_timer;        // Timer for transmission interval control

SemaphoreHandle_t m_irq_Semaphore; // Semaphore to handle radio IRQs
#define TAG "LORA_MAC"
#define RTOS_TIME ((xTaskGetTickCount() * 1000) >> 9)

// Macro to generate an acknowledgment (ACK) packet
#define GEN_ACK(q, p) \
	do { \
		q.Header.type = TYPE_DATA_ACK; \
		q.Header.MacHeader.dst = p->Header.MacHeader.src; \
		q.Header.NetHeader.src = Route.getNetAddr(); \
		q.Header.NetHeader.dst = p->Header.MacHeader.src; \
		q.Header.NetHeader.hop = 0; \
		q.Header.NetHeader.pid = p->Header.NetHeader.pid; \
	} while (0)
// Function to process incoming MAC layer packet
bool mac_peek_pkg(LoRaPkg* p) {
    uint8_t i;

    // Log packet source and destination information
    ESP_LOGI(TAG,"L2: nsrc: 0x%02x, ndst: 0x%02x, msrc: 0x%02x, mdst: 0x%02x",
        p->Header.NetHeader.src, p->Header.NetHeader.dst, p->Header.MacHeader.src, p->Header.MacHeader.dst);
    
    // Process if the packet is a Route Advertisement (RA) type
    if (p->Header.type == TYPE_RA && p->Header.NetHeader.subtype == SUB_RA) {
        for (i = 0; i < p->RouteData.hops; i++) {
            ESP_LOGI(TAG,"L2: recv RA_List_%d: 0x%02x", i, p->RouteData.RA_List[i]); // Log RA list
        }

        // Ignore RA from local node
        if (p->Header.NetHeader.src == Route.getNetAddr()) {
            ESP_LOGI(TAG,"ignore RA from local");
            return false;
        }

        // Ignore previously received RA
        for (i = 0; i < p->RouteData.hops; i++) {
            if (p->RouteData.RA_List[i] == Route.getNetAddr()) {
                ESP_LOGI(TAG,"ignore RA already process");
                return false;
            }
        }

        // Update route information
        Route.updateRoute(p->Header.NetHeader.src, p->Header.MacHeader.src, p->Header.NetHeader.hop);
        for (i = 0; i < p->RouteData.hops; i++) {
            Route.updateRoute(p->RouteData.RA_List[i], p->Header.MacHeader.src, p->RouteData.hops - i - 1);
        }

        // Rebroadcast if the destination is not this node
        if (p->Header.NetHeader.dst != Route.getNetAddr()) {
            p->RouteData.RA_List[p->RouteData.hops] = Route.getNetAddr();  // Add current node to RA list
            p->Header.NetHeader.hop++; p->RouteData.hops++;  // Increment hop count
            ESP_LOGI(TAG,"L2: RA rebroadcast!");
            for (i = 0; i < p->RouteData.hops; i++) {
                ESP_LOGI(TAG,"L2: send RA_List_%d: 0x%02x", i, p->RouteData.RA_List[i]);
            }
            xQueueSend(mac_tx_buf, p, 0);  // Send packet to MAC TX buffer for rebroadcast
            return false;
        }
    }
    return true;  // Packet is valid and should be processed
}

extern uint8_t ack_wait_id;      // ID of the packet waiting for ACK
extern uint32_t ack_time;        // Time when the ACK request was sent

// Function to handle received MAC layer packets                           
static void mac_rx_handle(LoRaPkg* p)
{
	LoRaPkg t; bool ret;
	uint8_t mac_dst = p->Header.MacHeader.dst;

	Route.updateLinkQualityMap(p->Header.MacHeader.src, p->stat.RssiPkt);
	PRINT_LINKQUALITY_MAP;

	ret = mac_peek_pkg(p);

	if (mac_dst == Route.getMacAddr() || mac_dst == MAC_BROADCAST_ADDR) {
		mac_rx_done++;
		if (p->Header.type == TYPE_DATA 
				&& p->Header.NetHeader.ack == ACK
				&& mac_dst != MAC_BROADCAST_ADDR 
				&& p->Header.NetHeader.dst != NET_BROADCAST_ADDR) {
			mac_ack_respon++;
			//LOG_DBG("L2: send ack! pid: %d", p->Header.NetHeader.pid);
			GEN_ACK(t, p);
			xQueueSend(mac_tx_buf, &t, 0);

			/* check dup data */
			if (p->Header.NetHeader.pid == _last_seen_pid[p->Header.MacHeader.src]) {
				ESP_LOGI(TAG,"L2: dup data, drop!");
				mac_rx_drop++;
				return;
			} else {
				_last_seen_pid[p->Header.MacHeader.src] = p->Header.NetHeader.pid;
				//LOG_DBG("L2: update last pid from 0x%02x: %d", p->Header.MacHeader.src, p->Header.NetHeader.pid);
			}
		} else if (p->Header.type == TYPE_DATA_ACK) {
			//LOG_DBG("L2: recv TYPE_DATA_ACK pid: %d, wait pid: %d", p->Header.NetHeader.pid, ack_wait_id);
			if (p->Header.NetHeader.pid == ack_wait_id) {
				ESP_LOGI(TAG,"L2: ack notify, Delay: %d", RTOS_TIME - ack_time);
				xTaskNotifyGive(lora_net_tx_handle);
			} else {
				ESP_LOGI(TAG,"L2: ack ignore!");
				mac_rx_drop++;
			}
			return;
		} else if (p->Header.type == TYPE_RA && p->Header.NetHeader.subtype == SUB_RA_RESPON) {
			uint8_t i, j;
			for ( i = 0; i < p->RouteData.hops; i++) {
				ESP_LOGI(TAG,"L2: recv RA_RESPON_List_%d: 0x%02x", i , p->RouteData.RA_List[i]);
			}

			Route.updateRoute(p->Header.NetHeader.src,
				p->Header.MacHeader.src, p->Header.NetHeader.hop);
			
			if (p->Header.NetHeader.dst == Route.getNetAddr()) {
				for ( i = 0; i < p->RouteData.hops; i++) {
					Route.updateRoute(p->RouteData.RA_List[i],
						p->Header.MacHeader.src, i);
				}
			} else {
				for ( i = 0; i < p->RouteData.hops; i++) {
					if (p->RouteData.RA_List[i] == Route.getMacAddr())
						break;
				}
				i++;
				for (j = 0 ; i < p->RouteData.hops; i++, j++) {
					Route.updateRoute(p->RouteData.RA_List[i],
						p->Header.MacHeader.src, j);
				}
			}
		}
		if (ret)
			xQueueSend(mac_rx_buf, p, 0);
	} else {
		mac_rx_drop++;
	}
}

#define SET_RADIO(fun,irq) \
	do { BoardEnableIrq(); \
		fun; \
		xSemaphoreTake(m_irq_Semaphore, portMAX_DELAY); \
		irq = GetIrqStatus( ); \
		ClearIrqStatus( IRQ_RADIO_ALL ); \
	} while (0)

#define IS_IRQ(irq,x) \
	(((irq) & (x) ) == x)

TaskHandle_t lora_mac_handle;


void lora_mac_task(void *pvParameter)
{
    uint8_t irqRegs;
    uint32_t timer;
    uint8_t pkgsize;
    uint8_t pkgbuf[255];
    PkgType hdr_type;
    LoRaPkg rxtmp, txtmp;
    mac_net_param_t *param = (mac_net_param_t *)pvParameter;
    lora_mac_hook *hook = &(param->mac_hooks);

    LoRaInit(); // Initialize LoRa hardware

    while (1) {
        // Enable debug print if needed
        LoRaDebugPrint(true); 

        // Enter standby mode
        SetStandby(LLCC68_STANDBY_RC);

        // Configure and start CAD
        LoRaConfig(7, 125, 1, 8, 0, true, false); // Adjust configuration as needed
        SetCadParams(5, 0x03, 0x0A, 0x00, 1000); // Set CAD parameters
        SetCad(); // Start CAD

        // Check IRQ status
        irqRegs = GetIrqStatus();
        if (IS_IRQ(irqRegs, LLCC68_IRQ_CAD_DONE)) {
            phy_cad_done++;
            if (hook->macCadDone != NULL) hook->macCadDone();

            if (IS_IRQ(irqRegs, LLCC68_IRQ_CAD_DETECTED)) {
                phy_cad_det++;
                if (hook->macCadDetect != NULL) hook->macCadDetect();

                // CAD detected, switch to RX mode
                
               RadioSetMaxPayloadLength( LLCC68_PACKET_TYPE_LORA, 0xff); // Set maximum payload length
               
                timer = RTOS_TIME;
                if (hook->macRxStart != NULL) hook->macRxStart();
                SetRx(RX_TIMEOUT); // Start RX
                if (hook->macRxEnd != NULL) hook->macRxEnd();

                // Check IRQ status after RX
                irqRegs = GetIrqStatus();
                if (IS_IRQ(irqRegs, LLCC68_IRQ_CRC_ERR) || IS_IRQ(irqRegs, LLCC68_IRQ_HEADER_ERR)) {
                    phy_rx_err++;
                    ESP_LOGI(TAG,"L1: Rx error!");
                } else if (IS_IRQ(irqRegs, LLCC68_IRQ_TIMEOUT)) {
                    phy_rx_timeout++;
                    ESP_LOGI(TAG,"L1: Rx timeout!");
                } else if (IS_IRQ(irqRegs, LLCC68_IRQ_RX_DONE)) {
                    phy_rx_done++;
                    pkgsize = 0;
                    GetPayload(pkgbuf, &pkgsize, sizeof(pkgbuf));
                    ESP_LOGI(TAG,"L1: Rx done, size: %d, time: %d", pkgsize, RTOS_TIME - timer);

                    hdr_type = (PkgType)(pkgbuf[0]);
                    if (hdr_type < TYPE_MAX && pkgsize == pkgSizeMap[hdr_type][1]) {
                        memcpy(&rxtmp, pkgbuf, pkgsize);
                        int8_t rssi, snr;
                        GetPacketStatus(&rssi, &snr);
                        rxtmp.stat.RssiPkt = rssi;
                        rxtmp.stat.SnrPkt = snr;

                        mac_rx_handle(&rxtmp);
                    }
                } else {
                    phy_rx_err++;
                    ESP_LOGI(TAG,"L1: Rx unknown error!");
                }
            } else {
                // CAD not detected, handle TX
                if (uxQueueMessagesWaiting(mac_tx_buf) != 0) {
                    if (tx_timer == 0 && xQueueReceive(mac_tx_buf, &txtmp, 0) == pdPASS) {
                        mac_tx_done++;
                        tx_timer = (xTaskGetTickCount() & TX_TIMER_MASK);

                        hdr_type = txtmp.Header.type;
                        pkgsize = (hdr_type < TYPE_MAX) ? pkgSizeMap[hdr_type][1] : SIZE_PKG_MAX;

                        txtmp.Header.MacHeader.src = Route.getMacAddr();
                        timer = RTOS_TIME;
                        if (hook->macTxStart != NULL) hook->macTxStart();
                        if (LoRaSend((uint8_t *)&txtmp, pkgsize, TX_TIMEOUT)) {
                            if (hook->macTxEnd != NULL) hook->macTxEnd();
                            if (txtmp.Header.type == TYPE_DATA && txtmp.Header.NetHeader.ack == ACK)
                                xSemaphoreGive(m_ack_Semaphore);
                            ESP_LOGI(TAG,"L1: Tx done, size: %d, time: %d", pkgsize, RTOS_TIME - timer);
                            phy_tx_done++;
                        } else {
                            ESP_LOGI(TAG,"L1: Tx error/timeout!");
                            phy_tx_err++;
                        }
                    } else {
                        --tx_timer;
                    }
                }
            }
        }

        // Ensure LoRa is in standby mode
        SetStandby(LLCC68_STANDBY_RC);
        vTaskDelay(pdMS_TO_TICKS(CAD_PERIOD_MS));
    }
}

