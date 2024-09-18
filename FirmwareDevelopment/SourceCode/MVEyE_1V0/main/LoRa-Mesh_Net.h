/*
 * LoRa-Mesh_Net.h
 *
 *  Created on: Sep 17, 2024
 *      Author: ADMIN
 */

#ifndef MAIN_LORA_MESH_NET_H_
#define MAIN_LORA_MESH_NET_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "stdint.h"
#include "Route.h"
#define MAC_BROADCAST_ADDR 0xff
#define NET_BROADCAST_ADDR MAC_BROADCAST_ADDR

extern TaskHandle_t lora_net_tx_handle;

void lora_net_tx_task (void * pvParameter);

extern TaskHandle_t lora_net_rx_handle;

void lora_net_rx_task (void * pvParameter);

extern TaskHandle_t lora_mac_handle;

void lora_mac_task(void * pvParameter);

extern QueueHandle_t mac_tx_buf;
extern QueueHandle_t mac_rx_buf;
extern QueueHandle_t net_tx_buf;
extern QueueHandle_t net_rx_buf;

extern SemaphoreHandle_t m_irq_Semaphore;
extern SemaphoreHandle_t m_ack_Semaphore;

/* phy layer counter */
extern uint32_t phy_cad_done;
extern uint32_t phy_cad_det;
extern uint32_t phy_rx_done;
extern uint32_t phy_rx_err;
extern uint32_t phy_rx_timeout;
extern uint32_t phy_tx_done;
extern uint32_t phy_tx_err;

/* mac layer counter */
extern uint32_t mac_rx_done;
extern uint32_t mac_rx_drop;
extern uint32_t mac_tx_done;
extern uint32_t mac_ack_respon;

/* net layer counter */
extern uint32_t net_tx_ack_ok; /* net tx ack done */
extern uint32_t net_tx_ack_retry; /* net tx ack retry times */
extern uint32_t net_tx_ack_fail; /* net tx ack fail */
extern uint32_t net_tx_done; /* net tx done */
extern uint32_t net_tx_drop; /* net tx drop (No route to host) */
extern uint32_t net_rx_done; /* local rx done */
extern uint32_t net_rx_drop; /* net rx drop */
extern uint32_t net_fwd_done; /* forward done */
extern uint32_t net_fwd_err; /* forward drop */

typedef struct {
	void (* macRxStart) (void);
	void (* macTxStart) (void);
	void (* macRxEnd) (void);
	void (* macTxEnd) (void);
	void (* macCadDone) (void);
	void (* macCadDetect) (void);
} lora_mac_hook;

typedef struct {
	void (* netRecv) (void);
	void (* netForward) (void);
	void (* netTx) (void);
} lora_net_hook;

typedef struct {
	lora_net_hook net_hooks;
	lora_mac_hook mac_hooks;
} mac_net_param_t;





typedef enum {
	TYPE_DATA = 0x0,
	TYPE_DATA_ACK,
	TYPE_PING,
	TYPE_RA,
	TYPE_MAX,
} PkgType;

typedef enum {
	SUB_DATA = 0x0,
	SUB_CONTROL,
} subtypeData;

typedef enum {
	SUB_RA = 0x0,
	SUB_RA_RESPON,
	SUB_RA_FAIL,
} subtypeRA;

typedef enum {
	CMD_PING = 0x0,
	CMD_UPLOAD_PING,
	CMD_LED,
	CMD_GET_STAT,
	CMD_UPLOAD_STAT,
} cmdtype;

typedef enum {
	ACK_NO = 0x0,
	ACK,
} ackType;

#ifdef __CC_ARM
#pragma anon_unions
#endif

#pragma pack(1)

typedef struct {
	int8_t RssiPkt;
	int8_t SnrPkt;
	int8_t SignalRssiPkt;
} pkg_Status;

typedef struct {
	uint8_t src;
	uint8_t dst;
} LoRaMac;

typedef struct {
	uint8_t src;
	uint8_t dst;
	uint8_t pid;
	struct {
		uint8_t hop : 4;
		uint8_t subtype : 3;
		ackType ack : 1;
	};
} LoRaNet;

typedef struct {
	PkgType type;
	LoRaMac MacHeader;
	LoRaNet NetHeader;
} LoRaHeader;

typedef	union {
	struct {
		float temp;
		uint16_t volt;
	};
	uint8_t custom[6];
} AppPayload;

typedef struct {
	uint8_t hops;
	uint8_t RA_List[MAX_HOPS];
} RoutePayload;

typedef struct {
	LoRaHeader Header;
	union {
		AppPayload AppData;
		RoutePayload RouteData;
	};
	pkg_Status stat; /* only use in Rx */
} *pLoRaPkg, LoRaPkg;

#pragma pack()

#define SIZE_HDR		(sizeof(LoRaHeader))

#define SIZE_PKG_MAX	SIZE_DATA
#define SIZE_DATA		((SIZE_HDR)+sizeof(AppPayload))
#define SIZE_RA			((SIZE_HDR)+sizeof(RoutePayload))
#define SIZE_PING		((SIZE_HDR)+(0))
#define SIZE_DATA_ACK	((SIZE_HDR)+(0))
#define CAD_PERIOD_MS                   30
#define RX_TIMEOUT                      80
#define TX_TIMEOUT                      250
#define TX_TIMER_MASK                   0x3 /* tx timer is between (0 ~ 3) * CAD_PERIOD_MS  */
#define LINKQMAP_CLEAR_PERIOD			30000
extern const int8_t pkgSizeMap[][2];


#endif /* MAIN_LORA_MESH_NET_H_ */
