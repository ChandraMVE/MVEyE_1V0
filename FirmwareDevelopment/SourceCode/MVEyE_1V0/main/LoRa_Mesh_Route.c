//-----------------------------------------------------------------
///
///     \file LoRa_Mesh_Route.c
///
///     \brief LoRa_Mesh_Route framework driver
///
///     \author        Venkata Suresh
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
#include "esp_log.h"
#include <string.h> 
#include "LoRa_Mesh_Route.h"
#include "freertos/portmacro.h"
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "LoRa_Mesh_Route"
#define RSSI_WEAK2_THRESHOLD -110
#define RSSI_DIFF2_THRESHOLD 10
#define RSSI_WEAK1_THRESHOLD -90
#define RSSI_DIFF1_THRESHOLD 15
#define RSSI_WEAK0_THRESHOLD -75
#define RSSI_DIFF0_THRESHOLD 15
#define DELETE_ROUTE(index) \
	do { \
		memcpy(&_routes[index], &_routes[index+1], \
			sizeof(RouteTableEntry) * (ROUTING_TABLE_SIZE - index - 1)); \
		_routes[ROUTING_TABLE_SIZE - 1].state = Invalid; \
	} while (0)
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
Addr_t _addr;  
static RouteTableEntry _routes[ROUTING_TABLE_SIZE];  
LinkQualityEntry _LinkQuality[255];  
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; 
/***********************************************************************************
 * Function name  : isNeedUpdate
 * Description    : Determines if a route update is needed based on the link quality 
 *                  of old and new hops. 
 * Parameters     : old_hop - The identifier of the old hop.
 *                  new_hop - The identifier of the new hop.
 *                  old_hop_count - The hop count associated with the old hop.
 *                  new_hop_count - The hop count associated with the new hop.
 * Returns        : true if an update is needed, false otherwise.
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static bool isNeedUpdate(uint8_t old_hop, uint8_t new_hop, uint8_t old_hop_count, uint8_t new_hop_count) {
    int16_t i, j;
    if (!(_LinkQuality[new_hop].valid && _LinkQuality[old_hop].valid)) {
        ESP_LOGI("ROUTE_UPDATE", "Y0");  
        return true;  
    }
    i = _LinkQuality[old_hop].quality;
    j = _LinkQuality[new_hop].quality;
    ESP_LOGI("ROUTE_UPDATE", "oldhop: 0x%02x(%d), newhop: 0x%02x(%d)", old_hop, i, new_hop, j);  
    if (i < RSSI_WEAK2_THRESHOLD && j - i > RSSI_DIFF2_THRESHOLD) {
        ESP_LOGI("ROUTE_UPDATE", "Y1");  
        return true;
    } 
    else if (i < RSSI_WEAK1_THRESHOLD && j - i > RSSI_DIFF1_THRESHOLD) {
        ESP_LOGI("ROUTE_UPDATE", "Y2");  // Debug log for update condition Y2
        return true;
    } 
    else if (i < RSSI_WEAK0_THRESHOLD && j - i > RSSI_DIFF0_THRESHOLD) {
        ESP_LOGI("ROUTE_UPDATE", "Y3"); 
        return true;
    } 
    else if (j > RSSI_WEAK0_THRESHOLD && new_hop_count < old_hop_count) {
        ESP_LOGI("ROUTE_UPDATE", "Y4");  
        return true;
    }

    ESP_LOGI("ROUTE_UPDATE", "N0");  
    return false;
}
/***********************************************************************************
 * Function name  : _getRouteTo
 * Description    : Searches the routing table for a valid route to the specified 
 *                  destination.
 * Parameters     : dest - The destination address to find a route to.
 * Returns        : The next hop address for the route to the destination, or -1 
 *                  if no valid route is found.
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ********************************************************************************/
static int8_t _getRouteTo(uint8_t dest) {
    uint8_t i;
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].dest == dest && _routes[i].state != Invalid) {
            return _routes[i].next_hop;  
        }
    }
    return -1;  
}
/***********************************************************************************
 * Function name  : _clearLinkQualityMap
 * Description    : Resets the link quality map by marking all entries as invalid. 
 * Parameters     : None
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static void _clearLinkQuailtyMap(void) {
	uint8_t i;
	for (i = 0; i< 255; i++) {
		_LinkQuality[i].valid = false;
	}
}
/***********************************************************************************
 * Function name  : _clearRoutingTable
 * Description    : Clears the routing table by marking all entries as invalid. 
 * Parameters     : None
 * Returns        : None
 * Known Issues   : None
 * Note           :
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static void _clearRoutingTable(void) {
    uint8_t i;
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        _routes[i].state = Invalid;
    }
}
/***********************************************************************************
 * Function name  : _initRouteTable
 * Description    : Initializes the routing table and link quality map. 
 * Parameters     : Addr_t *addr - Pointer to the address structure containing MAC 
 *                               and network addresses.
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static void _initRouteTable(Addr_t *addr) {
    _addr.mac = addr->mac; 
    _addr.net = addr->net; 
    _clearLinkQuailtyMap();  
    _clearRoutingTable(); 
}
/***********************************************************************************
 * Function name  : _updateRoute
 * Description    : Updates the routing table with new route information. 
 * Parameters     : 
 *   - uint8_t dest       : The destination address to update.
 *   - uint8_t next_hop   : The next hop address for the route.
 *   - uint8_t hops       : The number of hops to reach the destination.
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static void _updateRoute(uint8_t dest, uint8_t next_hop, uint8_t hops) {
   portENTER_CRITICAL(&mux);  
    uint8_t i;
    if (dest == Route.getNetAddr())
        goto out;  
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].dest == dest && _routes[i].state == Valid) {
            if (_routes[i].next_hop == next_hop) {
                _routes[i].hops = hops;  
                goto out; 
            }
       
            if (isNeedUpdate(_routes[i].next_hop, next_hop, _routes[i].hops, hops)) {
                ESP_LOGI("ROUTE_UPDATE", "UPDATE! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops);  
                _routes[i].next_hop = next_hop;  
                _routes[i].hops = hops;  
                goto updated;  
            } else {
                goto out;  
            }
        }
    }
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].state == Invalid) {
            ESP_LOGI("ROUTE_UPDATE", "NEW! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops);
            _routes[i].dest = dest;  
            _routes[i].next_hop = next_hop;  
            _routes[i].state = Valid;  
            _routes[i].hops = hops;  
            goto updated;  
        }
    }
    DELETE_ROUTE(0);  
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].state == Invalid) {
            ESP_LOGI("ROUTE_UPDATE", "NEW! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops); 
            _routes[i].dest = dest;
            _routes[i].next_hop = next_hop;  
            _routes[i].state = Valid;  
            _routes[i].hops = hops;  
            goto updated;  
        }
    }
updated:
    PRINT_ROUTE_TABLE;  
out:
    portEXIT_CRITICAL(&mux);  
}
/***********************************************************************************
 * Function name  : _getNetAddr
 * Description    : Retrieves the current network address.
 * Parameters     : None
 * Returns        : - uint8_t : The current network address.
 * Known Issues   : None
 * Note           : This function provides read access to the network address.
 * Author         : C. VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static uint8_t _getNetAddr(void) {
    return _addr.net;
}
/***********************************************************************************
 * Function name  : _getMacAddr
 * Description    : Retrieves the current MAC address.
 * Parameters     : None
 * Returns        : uint8_t : The current MAC address.
 * Known Issues   : None
 * Note           : This function provides read access to the MAC address.
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static uint8_t _getMacAddr(void) {
    return _addr.mac;
}
/***********************************************************************************
 * Function name  : _delRouteByNexthop
 * Description    : Deletes routes from the routing table that have the specified 
 *                  next hop address. 
 * Parameters     : uint8_t next_hop : The next hop address for which routes should be deleted.
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : C. venkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static void _delRouteByNexthop(uint8_t next_hop) {
    portENTER_CRITICAL(&mux);  
    uint8_t i;
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].next_hop == next_hop && _routes[i].state == Valid) {
            _routes[i].state = Invalid;  
        }
    }
    ESP_LOGI(TAG, "DEL by nh: 0x%02x", next_hop);  
    PRINT_ROUTE_TABLE;  
    portEXIT_CRITICAL(&mux);  
}

/***********************************************************************************
 * Function name  : _delRouteByDest
 * Description    : Deletes routes from the routing table that have the specified 
 *                  destination address. 
 * Parameters     : uint8_t dest : The destination address for which routes should be deleted.
 * Returns        : None
 * Known Issues   : None
 * Note           :
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
static void _delRouteByDest(uint8_t dest) {
    portENTER_CRITICAL(&mux);  
    uint8_t i;
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].dest == dest && _routes[i].state == Valid) {
            _routes[i].state = Invalid;  
        }
    }
    ESP_LOGI(TAG, "DEL by dst: 0x%02x", dest);  
    PRINT_ROUTE_TABLE;  
    portEXIT_CRITICAL(&mux);  
}
/***********************************************************************************
 * Function name  : _updateLinkQualityMap
 * Description    : Updates the link quality map for a specific address with the 
 *                  given quality value.
 * Parameters     : 
 *   - uint8_t addr : The address for which the link quality is updated.
 *   - int16_t quality : The quality value to be recorded for the specified address.
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/

static void _updateLinkQualityMap(uint8_t addr, int16_t quality) {
     portENTER_CRITICAL(&mux);  
    _LinkQuality[addr].valid = true;  
    _LinkQuality[addr].updated = true; 
    _LinkQuality[addr].quality = quality;  
    portEXIT_CRITICAL(&mux);  
}
/***********************************************************************************
 * Function name  : _clearLinkQualityMapTimer
 * Description    : Timer callback function that clears invalid entries from the 
 *                  link quality map.
 * Parameters     : 
 *   - TimerHandle_t xTimer : The handle of the timer that triggered this callback.
 * Returns        : None
 * Known Issues   : None
 * Note           : 
 * Author         : 
 * Date           : 20SEP2024
 ***********************************************************************************/
static void _clearLinkQualityMapTimer(TimerHandle_t xTimer) {
    portENTER_CRITICAL(&mux);  // Enter critical section
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 255; i++) {
        if (_LinkQuality[i].valid == true) {
            if (_LinkQuality[i].updated == false) {
                cnt++;
                _LinkQuality[i].valid = false;  
            }
            _LinkQuality[i].updated = false; 
        }
    }
    //ESP_LOGI(TAG,"LinkQualityMap clean: %d", cnt);  
    portEXIT_CRITICAL(&mux);  
}
const MeshRoute_t Route = {
    .getRouteTo = _getRouteTo,
    .initRouteTable = _initRouteTable,
    .clearRoutingTable = _clearRoutingTable,
    .updateRoute = _updateRoute,
    .getNetAddr = _getNetAddr,
    .getMacAddr = _getMacAddr,
    .delRouteByNexthop = _delRouteByNexthop,
    .delRouteByDest = _delRouteByDest,
    .updateLinkQualityMap = _updateLinkQualityMap,
    .clearLinkQuailtyMapTimer = _clearLinkQualityMapTimer,
};
