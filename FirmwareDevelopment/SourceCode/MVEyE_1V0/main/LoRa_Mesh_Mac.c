//-----------------------------------------------------------------
///
///     \file LoRa_Mesh_Mac.c
///
///     \brief LoRa_Mesh_Mac framework driver
///
///     \author       Venkata Suresh
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
#include "lora_app.h"
#include "lora_llc68.h"
#include "stdlib.h"            
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"           
#include "freertos/semphr.h"    
#include "esp_timer.h"             
#include "LoRa_Mesh_Route.h"         
#include "esp_log.h"
#include "LoRa_Mesh_Net.h"
#include "esp_task_wdt.h"
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "LORA_MAC"
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
	
#define SET_RADIO(fun, irq) \
    do { \
        uint32_t io_num;            \
        gpio_intr_enable(LORA_DIO1); \
        fun; \
        xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);  \
        irq = GetIrqStatus();  \
        ClearIrqStatus(LLCC68_IRQ_ALL);  \
    } while (0)
#define SET_RADIO2(fun, irq) \
    do { \
        uint32_t io_num; \
        gpio_intr_enable(LORA_DIO3); \
        fun; \
        xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);  \
        irq = GetIrqStatus();  \
        ClearIrqStatus(LLCC68_IRQ_ALL);  \
    } while (0)
#define IS_IRQ(irq,x) \
	(((irq) & (x) ) == x)

//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
extern uint8_t ack_wait_id;      
extern uint32_t ack_time;        
SemaphoreHandle_t m_irq_Semaphore;
SemaphoreHandle_t m_irq2_Semaphore;
TaskHandle_t lora_net_rx_handle;
QueueHandle_t mac_tx_buf;       
QueueHandle_t mac_rx_buf; 
TaskHandle_t lora_mac_handle;     
extern TaskHandle_t app_recv_handle; 
extern uint32_t io_num;
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================
/* PHY (Physical layer) counters to track radio events */
uint32_t phy_cad_done;          
uint32_t phy_cad_det;           
uint32_t phy_rx_done;           
uint32_t phy_rx_err;            
uint32_t phy_rx_timeout;        
uint32_t phy_tx_done;          
uint32_t phy_tx_err;            
const int8_t pkgSizeMap[][2] = {
	{TYPE_DATA,			SIZE_DATA},
	{TYPE_DATA_ACK,		SIZE_DATA_ACK},
	{TYPE_PING,			SIZE_PING},
	{TYPE_RA,			SIZE_RA},
	{TYPE_MAX,			-1},
};
/* MAC (Medium Access Control) layer counters */
uint32_t mac_rx_done;           
uint32_t mac_rx_drop;           
uint32_t mac_tx_done;           
uint32_t mac_ack_respon;        
static uint8_t tx_timer;       
static QueueHandle_t gpio_evt_queue = NULL; 
/*******************************************************************************
 * Function: dio1_isr_handler
 * Description: Interrupt Service Routine (ISR) for DIO1 pin. 
 * Parameters: 
 *   - arg: Pointer to optional argument (DIO1 pin).
 * Returns: None
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ******************************************************************************/
void IRAM_ATTR dio1_isr_handler(void *arg) {
    // Give semaphore from ISR
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    gpio_intr_disable(LORA_DIO1);
    
}
void IRAM_ATTR rx_done_isr_handler(void *arg) {
   	uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    gpio_intr_disable(LORA_DIO3);
   
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
void init_dio1_interrupt(void)
{
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,   // Interrupt on Rising edge
        .mode = GPIO_MODE_INPUT,          // Set pin as input
        .pin_bit_mask = (1ULL << LORA_DIO1), // Pin mask for DIO1
        .pull_down_en = 0,                // Disable pull-down
        .pull_up_en = 1,                  // Enable pull-up
    };
    gpio_config(&io_conf);
    
     // Configure GPIO for DIO2 (RxDone)
    gpio_config_t io_conf_rx;
    io_conf_rx.intr_type = GPIO_INTR_POSEDGE;  // Trigger interrupt on rising edge
    io_conf_rx.pin_bit_mask = (1ULL << LORA_DIO3);  // Select GPIO pin
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

    gpio_isr_handler_add(LORA_DIO1, dio1_isr_handler, (void*)LORA_DIO1);
    gpio_isr_handler_add(LORA_DIO3, rx_done_isr_handler, (void*) LORA_DIO3);

    ESP_LOGI(TAG, "DIO1 interrupt initialized on GPIO %d", LORA_DIO1);
}
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
void enable_dio1_interrupt(void)
{
    gpio_intr_enable(LORA_DIO1); // Enable interrupt
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
void disable_dio1_interrupt(void)
{
    gpio_intr_disable(LORA_DIO1); // Disable interrupt
}

/***********************************************************************************
 * Function name  : mac_peek_pkg
 *
 * Description    : Inspects a LoRa package and processes Route Advertisement (RA) 
 *                  packets. 
 * Parameters     : p - Pointer to the LoRa package to inspect (LoRaPkg).
 * Returns        : true if the packet is valid and should be processed, false otherwise.
 *
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
bool mac_peek_pkg(LoRaPkg* p) {
    uint8_t i;
    ESP_LOGI(TAG,"L2: nsrc: 0x%02x, ndst: 0x%02x, msrc: 0x%02x, mdst: 0x%02x",
        p->Header.NetHeader.src, p->Header.NetHeader.dst, p->Header.MacHeader.src, p->Header.MacHeader.dst);
    
    if (p->Header.type == TYPE_RA && p->Header.NetHeader.subtype == SUB_RA) {
        for (i = 0; i < p->RouteData.hops; i++) {
            ESP_LOGI(TAG,"L2: recv RA_List_%d: 0x%02x", i, p->RouteData.RA_List[i]); // Log RA list
        }
        if (p->Header.NetHeader.src == Route.getNetAddr()) {
            ESP_LOGI(TAG,"ignore RA from local");
            return false;
        }
        for (i = 0; i < p->RouteData.hops; i++) {
            if (p->RouteData.RA_List[i] == Route.getNetAddr()) {
                ESP_LOGI(TAG,"ignore RA already process");
                return false;
            }
        }
        Route.updateRoute(p->Header.NetHeader.src, p->Header.MacHeader.src, p->Header.NetHeader.hop);
        for (i = 0; i < p->RouteData.hops; i++) {
            Route.updateRoute(p->RouteData.RA_List[i], p->Header.MacHeader.src, p->RouteData.hops - i - 1);
        }
        if (p->Header.NetHeader.dst != Route.getNetAddr()) {
            p->RouteData.RA_List[p->RouteData.hops] = Route.getNetAddr();  // Add current node to RA list
            p->Header.NetHeader.hop++; p->RouteData.hops++;  // Increment hop count
            ESP_LOGI(TAG,"L2: RA rebroadcast!");
            for (i = 0; i < p->RouteData.hops; i++) {
                ESP_LOGI(TAG,"L2: send RA_List_%d: 0x%02x", i, p->RouteData.RA_List[i]);
            }
            xQueueSend(mac_tx_buf, p, 0);  
            return false;
        }
    }
    return true;  
}
/***********************************************************************************
 * Function name  : mac_rx_handle
 *
 * Description    : Handles incoming LoRa packets by updating link quality, processing 
 *                  different packet types (data, acknowledgments, route advertisements), 
 *                  and managing retransmission and routing updates.
 * Parameters     : p - Pointer to the received LoRa package (LoRaPkg).
 * Returns        : None (void).
 *
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/      
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
			ESP_LOGI(TAG,"L2: send ack! pid: %d", p->Header.NetHeader.pid);
			GEN_ACK(t, p);
			xQueueSend(mac_tx_buf, &t, 0);
			if (p->Header.NetHeader.pid == _last_seen_pid[p->Header.MacHeader.src]) {
				ESP_LOGI(TAG,"L2: dup data, drop!");
				mac_rx_drop++;
				return;
			} else {
				_last_seen_pid[p->Header.MacHeader.src] = p->Header.NetHeader.pid;
				ESP_LOGI(TAG,"L2: update last pid from 0x%02x: %d", p->Header.MacHeader.src, p->Header.NetHeader.pid);
			}
		} else if (p->Header.type == TYPE_DATA_ACK) {
			ESP_LOGI(TAG,"L2: recv TYPE_DATA_ACK pid: %d, wait pid: %d", p->Header.NetHeader.pid, ack_wait_id);
			if (p->Header.NetHeader.pid == ack_wait_id) {
				ESP_LOGI(TAG,"L2: ack notify, Delay: %lu", (unsigned long)(RTOS_TIME - ack_time));
				xTaskNotifyGive(lora_net_tx_handle);
			} else {
				ESP_LOGI(TAG,"L2: ack ignore!");
				mac_rx_drop++;
			}
			return;
		} else if (p->Header.type == TYPE_RA && p->Header.NetHeader.subtype == SUB_RA_RESPON) {
			uint8_t i, j;
			for ( i = 0; i < p->RouteData.hops; i++) {
				ESP_LOGI(TAG,"L2: recv RA_RESPON_List_%u: 0x%02x", i , p->RouteData.RA_List[i]);
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
/***********************************************************************************
 * Function name  : lora_mac_task
 *
 * Description    : Manages the LoRa MAC layer, handling both transmission and 
 *                  reception of LoRa packets. 
 * Parameters     : pvParameter - Pointer to MAC network parameters (mac_net_param_t).
 * Returns        : None (void).
 *
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
/*void lora_mac_task(void *pvParameter)
{
    uint16_t irqRegs;
    uint32_t timer;
    uint8_t pkgsize;
    uint8_t pkgbuf[255];
    uint8_t buffer[255];
    PkgType hdr_type;
    LoRaPkg rxtmp, txtmp;
    SleepParams_t sleepConfig;
    sleepConfig.sleepStart = LLCC68_SLEEP_START_COLD;
    sleepConfig.rtcStatus = LLCC68_SLEEP_RTC_OFF;
    mac_net_param_t *param = (mac_net_param_t *)pvParameter;
    lora_mac_hook *hook = &(param->mac_hooks); 
    while (1) 
    {
       	 ESP_LOGI(TAG,"I am in LoRa Mack");
	  	 SetStandby(LLCC68_STANDBY_RC);
     	 SET_RADIO(RadioStartCad(), irqRegs );
       if (IS_IRQ(irqRegs, LLCC68_IRQ_CAD_DONE)) 
       {
            phy_cad_done++;
            if (hook->macCadDone != NULL) hook->macCadDone();
			ESP_LOGI(TAG,"macCadDone!");
            if (IS_IRQ(irqRegs, LLCC68_IRQ_CAD_DETECTED)) 
            {
                phy_cad_det++;
                ESP_LOGI(TAG,"CAD_DETECTED!");
                ESP_LOGI(TAG,"cad irqRegs:%d",irqRegs);
                if (hook->macCadDetect != NULL) hook->macCadDetect();
                 
                RadioSetMaxPayloadLength( LLCC68_PACKET_TYPE_LORA, 0xff); // Set maximum payload length
                timer = RTOS_TIME;
                if (hook->macRxStart != NULL) hook->macRxStart();
                SET_RADIO2(RadioRx(200), irqRegs);
                LoRaReceive(buffer, sizeof(buffer));
                ESP_LOGI(TAG,"Receive irqRegs:%d",irqRegs);
                if (hook->macRxEnd != NULL) hook->macRxEnd();	
                if (IS_IRQ(irqRegs, LLCC68_IRQ_CRC_ERR) || IS_IRQ(irqRegs, LLCC68_IRQ_HEADER_ERR)) 
                {
                    phy_rx_err++;
                    ESP_LOGI(TAG,"L1: Rx error!");
                } else if (IS_IRQ(irqRegs, LLCC68_IRQ_TIMEOUT)) 
                {
                    phy_rx_timeout++;
                    ESP_LOGI(TAG,"L1: Rx timeout!");
                } else if (IS_IRQ(irqRegs, LLCC68_IRQ_RX_DONE)) 
                {
                    phy_rx_done++;
                    GetPayload(pkgbuf, &pkgsize, sizeof(pkgbuf));
					ESP_LOGI(TAG, "L1: Rx done, size: %u, time: %lu", pkgsize, (unsigned long)(RTOS_TIME - timer));
					vTaskDelay(pdMS_TO_TICKS(1000));
                    hdr_type = (PkgType)(pkgbuf[0]);
                    if (hdr_type < TYPE_MAX && pkgsize == pkgSizeMap[hdr_type][1]) 
                    {
						vTaskDelay(pdMS_TO_TICKS(1000));
                        memcpy(&rxtmp, pkgbuf, pkgsize);
                        int8_t rssi, snr;
                        GetPacketStatus(&rssi, &snr);
                        rxtmp.stat.RssiPkt = rssi;
                        rxtmp.stat.SnrPkt = snr;
                        mac_rx_handle(&rxtmp);
                        ESP_LOGI(TAG, "L1: Rx done");
                    }
                } else 
                {
                    phy_rx_err++;
                    ESP_LOGI(TAG,"L1: Rx unknown error!");
                }
            } else 
            {
                if (uxQueueMessagesWaiting(mac_tx_buf) != 0) {
                    if (tx_timer == 0 && xQueueReceive(mac_tx_buf, &txtmp, 0) == pdPASS) {
                        mac_tx_done++;
                        tx_timer = (xTaskGetTickCount() & TX_TIMER_MASK);
                        hdr_type = txtmp.Header.type;
                        pkgsize = (hdr_type < TYPE_MAX) ? pkgSizeMap[hdr_type][1] : SIZE_PKG_MAX;
                        txtmp.Header.MacHeader.src = Route.getMacAddr();
                        timer = RTOS_TIME;
                        if (hook->macTxStart != NULL) hook->macTxStart();
                        //SET_RADIO(LoRaSend((uint8_t *)&txtmp, pkgsize, TX_TIMEOUT), irqRegs );
                        SET_RADIO(RadioSend(TX_TIMEOUT), irqRegs );
                        ESP_LOGI(TAG,"transmission irqRegs:%d",irqRegs);
						if (hook->macTxEnd != NULL) hook->macTxEnd();
						if (txtmp.Header.type == TYPE_DATA && txtmp.Header.NetHeader.ack == ACK)
                                xSemaphoreGive(m_ack_Semaphore);
             			if (IS_IRQ(irqRegs, LLCC68_IRQ_TX_DONE)) 
                		{
							ESP_LOGI(TAG, "L1: Tx done, size: %u, time: %lu", pkgsize, (unsigned long)(RTOS_TIME - timer));
							phy_tx_done++;
						} else {
                            ESP_LOGI(TAG,"L1: Tx error/timeout!");
                            phy_tx_err++;
                        }
                    } else {
						//ESP_LOGI(TAG,"L1: wait Tx timer: %d...", tx_timer);
                        --tx_timer;
                    }
                }
            }
        }
       	SetSleep(sleepConfig);
        vTaskDelay(pdMS_TO_TICKS(1000));
	}

}
*/
/***********************************************************************************
 * Function name  : lora_mac_receive_task
 *
 * Description    : Manages the LoRa MAC layer, handling reception of LoRa packets. 
 * Parameters     : pvParameter - Pointer to MAC network parameters (mac_net_param_t).
 * Returns        : None (void).
 *
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void lora_mac_receive_task(void *pvParameter)
{
    uint16_t irqRegs;
    uint32_t timer;
    uint8_t pkgsize;
    uint8_t pkgbuf[255];
    uint8_t buffer[255];
    PkgType hdr_type;
    LoRaPkg rxtmp;
    mac_net_param_t *param = (mac_net_param_t *)pvParameter;
    lora_mac_hook *hook = &(param->mac_hooks);

    while (1) 
    {
        ESP_LOGI(TAG,"LoRa RX Task: Waiting for CAD");
        SetStandby(LLCC68_STANDBY_RC);
        SET_RADIO(RadioStartCad(), irqRegs);
        
        if (IS_IRQ(irqRegs, LLCC68_IRQ_CAD_DONE)) 
        {
            phy_cad_done++;
            if (hook->macCadDone != NULL) hook->macCadDone();

            if (IS_IRQ(irqRegs, LLCC68_IRQ_CAD_DETECTED)) 
            {
                phy_cad_det++;
                ESP_LOGI(TAG,"CAD_DETECTED!");
                RadioSetMaxPayloadLength(LLCC68_PACKET_TYPE_LORA, 0xff); // Set maximum payload length
                timer = RTOS_TIME;
                if (hook->macRxStart != NULL) hook->macRxStart();

                SET_RADIO2(RadioRx(RX_TIMEOUT), irqRegs);
                //LoRaReceive(buffer, sizeof(buffer));
                ESP_LOGI(TAG,"Receive irqRegs:%d", irqRegs);

                if (hook->macRxEnd != NULL) hook->macRxEnd();

                if (IS_IRQ(irqRegs, LLCC68_IRQ_CRC_ERR) || IS_IRQ(irqRegs, LLCC68_IRQ_HEADER_ERR)) 
                {
                    phy_rx_err++;
                    ESP_LOGI(TAG,"RX error!");
                } 
                else if (IS_IRQ(irqRegs, LLCC68_IRQ_TIMEOUT)) 
                {
                    phy_rx_timeout++;
                    ESP_LOGI(TAG,"RX timeout!");
                } 
                else if (IS_IRQ(irqRegs, LLCC68_IRQ_RX_DONE)) 
                {
                    phy_rx_done++;
                    GetPayload(pkgbuf, &pkgsize, sizeof(pkgbuf));
                    ESP_LOGI(TAG, "RX done, size: %u, time: %lu", pkgsize, (unsigned long)(RTOS_TIME - timer));

                   hdr_type = (PkgType)(pkgbuf[0]);

					// Log the received header type and package size
					ESP_LOGI(TAG, "Received packet header: %d", hdr_type);
					ESP_LOGI(TAG, "Package size: %d", pkgsize);
                    mac_rx_handle(&rxtmp);
                    // Check if the header type is valid and the package size matches expected size
					if (hdr_type < TYPE_MAX && pkgsize == pkgSizeMap[hdr_type][1]) 
					{
    					ESP_LOGI(TAG, "Valid header type: %d and valid package size: %d", hdr_type, pkgsize);
    
    					// Copy the package buffer to the structure for further processing
    					memcpy(&rxtmp, pkgbuf, pkgsize);
    
    					// Retrieve RSSI and SNR values from the packet status
    					int8_t rssi, snr;
    					GetPacketStatus(&rssi, &snr);
    
    					// Store the RSSI and SNR values in the package structure
    					rxtmp.stat.RssiPkt = rssi;
    					rxtmp.stat.SnrPkt = snr;
    
    					// Log the RSSI and SNR values
    					ESP_LOGI(TAG, "RSSI: %d, SNR: %d", rssi, snr);
    
    					// Handle the received packet by passing it to the mac_rx_handle function
    					mac_rx_handle(&rxtmp);
    
    					// Log the successful packet processing
    					ESP_LOGI(TAG, "Packet processed successfully.");
					}else 
					{
    					// Log an error if the header type or package size is invalid
    					ESP_LOGE(TAG, "Invalid header type: %d or package size: %d", hdr_type, pkgsize);
					}
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay to avoid tight looping
    }
}





/***********************************************************************************
 * Function name  : lora_mac_transmit_task
 *
 * Description    : Manages the LoRa MAC layer, handling Transmission of LoRa packets. 
 * Parameters     : pvParameter - Pointer to MAC network parameters (mac_net_param_t).
 * Returns        : None (void).
 *
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void lora_mac_transmit_task(void *pvParameter)
{
    uint16_t irqRegs;
    uint32_t timer;
    uint8_t pkgsize;
    PkgType hdr_type;
    LoRaPkg txtmp;
    mac_net_param_t *param = (mac_net_param_t *)pvParameter;
    lora_mac_hook *hook = &(param->mac_hooks);

    while (1) 
    {
        if (uxQueueMessagesWaiting(mac_tx_buf) != 0) 
        {
            if (tx_timer == 0 && xQueueReceive(mac_tx_buf, &txtmp, 0) == pdPASS) 
            {
                mac_tx_done++;
                tx_timer = (xTaskGetTickCount() & TX_TIMER_MASK);
                hdr_type = txtmp.Header.type;
                pkgsize = (hdr_type < TYPE_MAX) ? pkgSizeMap[hdr_type][1] : SIZE_PKG_MAX;
                txtmp.Header.MacHeader.src = Route.getMacAddr();
                timer = RTOS_TIME;

                if (hook->macTxStart != NULL) hook->macTxStart();

                SET_RADIO(RadioSend(TX_TIMEOUT), irqRegs);
                ESP_LOGI(TAG, "Transmission irqRegs: %d", irqRegs);

                if (hook->macTxEnd != NULL) hook->macTxEnd();

                if (txtmp.Header.type == TYPE_DATA && txtmp.Header.NetHeader.ack == ACK) 
                {
                    xSemaphoreGive(m_ack_Semaphore);
                }

                if (IS_IRQ(irqRegs, LLCC68_IRQ_TX_DONE)) 
                {
                    ESP_LOGI(TAG, "TX done, size: %u, time: %lu", pkgsize, (unsigned long)(RTOS_TIME - timer));
                    phy_tx_done++;
                } 
                else 
                {
                    ESP_LOGI(TAG,"TX error/timeout!");
                    phy_tx_err++;
                }
            } 
            else 
            {
                --tx_timer;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay to avoid tight looping
    }
}

