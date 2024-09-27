//-----------------------------------------------------------------
///
///     \file Route.c
///
///     \brief Route framework driver
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
#ifndef __MESH_ROUTE_
#define __MESH_ROUTE_
//==============================================================================
//          __             __   ___  __
//  | |\ | /  ` |    |  | |  \ |__  /__`
//  | | \| \__, |___ \__/ |__/ |___ .__/
//
//==============================================================================
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "inttypes.h"
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define ROUTING_TABLE_SIZE 16
#define MAX_HOPS 8
#define TTL_MAX MAX_HOPS
#ifdef DEBUG_ROUTE
extern LinkQualityEntry _LinkQuality[];

#define PRINT_ROUTE_TABLE \
	do {  \
		for (uint8_t _x=0; _x<ROUTING_TABLE_SIZE; _x++) { \
			if ( _routes[_x].state == Valid) \
				NRF_LOG_DBG("Route[%d]: 0x%02x->0x%02x, hop: %d", _x, \
			_routes[_x].dest, _routes[_x].next_hop, _routes[_x].hops); \
		} \
	} while (0)

#define PRINT_LINKQUALITY_MAP \
	do { \
		for (uint8_t _y=0; _y<255; _y++) { \
			if (_LinkQuality[_y].valid == true) \
				NRF_LOG_DBG("LinkQMap[0x%02x]: %d", _y, _LinkQuality[_y].quality); \
		} \
	} while (0)
#else
#define PRINT_ROUTE_TABLE
#define PRINT_LINKQUALITY_MAP
#endif
//==============================================================================
//  ___      __   ___  __   ___  ___  __
//   |  \ / |__) |__  |  \ |__  |__  /__`
//   |   |  |    |___ |__/ |___ |    .__/
//
//==============================================================================
typedef enum {
	Invalid = 0,
	Discovering,
	Valid,
} RouteState;

typedef struct {
	uint8_t mac;
	uint8_t net;
} Addr_t;

typedef struct {
	uint8_t dest;
	uint8_t next_hop;
	RouteState state;
	uint8_t hops;
} RouteTableEntry;

typedef struct {
	bool updated;
	int16_t quality;
	bool valid;
} LinkQualityEntry;

typedef struct {
	int8_t (*getRouteTo) (uint8_t dst);
	void (*updateRoute) (uint8_t dest, uint8_t next_hop, uint8_t hops);
	void (*updateLinkQualityMap) (uint8_t addr, int16_t quality);
	void (*clearLinkQuailtyMapTimer) (TimerHandle_t xTimer);
	void (*initRouteTable) (Addr_t *addr);
	void (*clearRoutingTable) (void);
	uint8_t (*getNetAddr) (void);
	uint8_t (*getMacAddr) (void);
	void (*delRouteByNexthop) (uint8_t next_hop);
	void (*delRouteByDest) (uint8_t dest);
} MeshRoute_t;
//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
extern const MeshRoute_t Route;
extern int16_t _last_seen_pid[];
extern Addr_t _addr;

#endif