

#include "esp_log.h"
#include <string.h> 
#include "Route.h"
#include "freertos/portmacro.h"
// Global variables
static Addr_t _addr;  // Stores the network and MAC address of the current node
static RouteTableEntry _routes[ROUTING_TABLE_SIZE];  // Routing table to store route entries
LinkQualityEntry _LinkQuality[255];  // Link quality map for 255 possible addresses
// RSSI threshold definitions for determining link quality
#define TAG "Route"
#define RSSI_WEAK2_THRESHOLD -110
#define RSSI_DIFF2_THRESHOLD 10
#define RSSI_WEAK1_THRESHOLD -90
#define RSSI_DIFF1_THRESHOLD 15
#define RSSI_WEAK0_THRESHOLD -75
#define RSSI_DIFF0_THRESHOLD 15

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // Initialize mutex
// Macro to delete a route from the routing table
#define DELETE_ROUTE(index) \
	do { \
		memcpy(&_routes[index], &_routes[index+1], \
			sizeof(RouteTableEntry) * (ROUTING_TABLE_SIZE - index - 1)); \
		_routes[ROUTING_TABLE_SIZE - 1].state = Invalid; \
	} while (0)
 
// Determines if a route update is needed based on link quality
static bool isNeedUpdate(uint8_t old_hop, uint8_t new_hop, uint8_t old_hop_count, uint8_t new_hop_count) {
    int16_t i, j;

    // Check if link quality for either the old or new hop is not valid
    if (!(_LinkQuality[new_hop].valid && _LinkQuality[old_hop].valid)) {
        ESP_LOGI("ROUTE_UPDATE", "Y0");  // Debug log for link quality validity check
        return true;  // Update needed if link quality is not valid
    }

    // Retrieve the link quality values for the old and new hops
    i = _LinkQuality[old_hop].quality;
    j = _LinkQuality[new_hop].quality;
    ESP_LOGI("ROUTE_UPDATE", "oldhop: 0x%02x(%d), newhop: 0x%02x(%d)", old_hop, i, new_hop, j);  // Debug log for RSSI values

    // Check if the quality of the new hop is significantly better than the old hop
    if (i < RSSI_WEAK2_THRESHOLD && j - i > RSSI_DIFF2_THRESHOLD) {
        ESP_LOGI("ROUTE_UPDATE", "Y1");  // Debug log for update condition Y1
        return true;
    } 
    // Check if the quality difference between new and old hop is substantial under a different condition
    else if (i < RSSI_WEAK1_THRESHOLD && j - i > RSSI_DIFF1_THRESHOLD) {
        ESP_LOGI("ROUTE_UPDATE", "Y2");  // Debug log for update condition Y2
        return true;
    } 
    // Check if the quality difference between new and old hop is sufficient under another condition
    else if (i < RSSI_WEAK0_THRESHOLD && j - i > RSSI_DIFF0_THRESHOLD) {
        ESP_LOGI("ROUTE_UPDATE", "Y3");  // Debug log for update condition Y3
        return true;
    } 
    // Check if the new hop has better quality and fewer hops
    else if (j > RSSI_WEAK0_THRESHOLD && new_hop_count < old_hop_count) {
        ESP_LOGI("ROUTE_UPDATE", "Y4");  // Debug log for update condition Y4
        return true;
    }

    ESP_LOGI("ROUTE_UPDATE", "N0");  // Debug log for no update
    return false;
}

// Retrieves the next hop address for a given destination
static int8_t _getRouteTo(uint8_t dest) {
    uint8_t i;
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].dest == dest && _routes[i].state != Invalid) {
            return _routes[i].next_hop;  // Return the next hop address
        }
    }
    return -1;  // Return -1 if no route to destination
}
static void _clearLinkQuailtyMap(void) {
	uint8_t i;
	for (i = 0; i< 255; i++) {
		_LinkQuality[i].valid = false;
	}
}
// Clears the routing table by marking all entries as invalid
static void _clearRoutingTable(void) {
    uint8_t i;
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        _routes[i].state = Invalid;
    }
}
// Initializes the routing table and link quality map
static void _initRouteTable(Addr_t *addr) {
    _addr.mac = addr->mac;  // Set MAC address
    _addr.net = addr->net;  // Set network address
    _clearLinkQuailtyMap();  // Clear the link quality map
    _clearRoutingTable();  // Clear the routing table
}
// Updates the routing table with a new route or modifies an existing route
static void _updateRoute(uint8_t dest, uint8_t next_hop, uint8_t hops) {
   portENTER_CRITICAL(mux);  // Enter critical section to prevent concurrent access
    uint8_t i;

    // Skip update if destination is the same as the current node's network address
    if (dest == Route.getNetAddr())
        goto out;  // Skip further processing if destination matches local node's address

    // Check if the route already exists
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].dest == dest && _routes[i].state == Valid) {
            // If next hop is the same, just update the hop count
            if (_routes[i].next_hop == next_hop) {
                _routes[i].hops = hops;  // Update hop count for the existing route
                goto out;  // Skip to the end
            }

            // Check if an update is needed based on link quality
            if (isNeedUpdate(_routes[i].next_hop, next_hop, _routes[i].hops, hops)) {
                ESP_LOGI("ROUTE_UPDATE", "UPDATE! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops);  // Debug log for route update
                _routes[i].next_hop = next_hop;  // Update next hop address
                _routes[i].hops = hops;  // Update hop count
                goto updated;  // Skip to the update section
            } else {
                goto out;  // No update needed
            }
        }
    }

    // If route is not found, add a new route
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].state == Invalid) {
            ESP_LOGI("ROUTE_UPDATE", "NEW! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops);  // Debug log for new route
            _routes[i].dest = dest;  // Set destination address
            _routes[i].next_hop = next_hop;  // Set next hop address
            _routes[i].state = Valid;  // Mark route as valid
            _routes[i].hops = hops;  // Set hop count
            goto updated;  // Skip to the update section
        }
    }

    // If no invalid slot found, replace the oldest route
    DELETE_ROUTE(0);  // Delete the first route entry to make space
    // Find an invalid slot for the new route
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].state == Invalid) {
            ESP_LOGI("ROUTE_UPDATE", "NEW! dst: 0x%02x, nh: 0x%02x, hop: %d", dest, next_hop, hops);  // Debug log for new route
            _routes[i].dest = dest;  // Set destination address
            _routes[i].next_hop = next_hop;  // Set next hop address
            _routes[i].state = Valid;  // Mark route as valid
            _routes[i].hops = hops;  // Set hop count
            goto updated;  // Skip to the update section
        }
    }
updated:
    PRINT_ROUTE_TABLE;  // Print the updated routing table for debugging
out:
    portEXIT_CRITICAL(mux);  // Exit critical section to allow other tasks to access the routing table
}

// Retrieves the network address of the current node
static uint8_t _getNetAddr(void) {
    return _addr.net;
}

// Retrieves the MAC address of the current node
static uint8_t _getMacAddr(void) {
    return _addr.mac;
}

// Deletes routes by the next hop address
static void _delRouteByNexthop(uint8_t next_hop) {
    portENTER_CRITICAL(mux);  // Enter critical section to prevent concurrent access
    uint8_t i;
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].next_hop == next_hop && _routes[i].state == Valid) {
            _routes[i].state = Invalid;  // Mark route as invalid
        }
    }
    ESP_LOGI(TAG, "DEL by nh: 0x%02x", next_hop);  // Debug log for route deletion
    PRINT_ROUTE_TABLE();  // Print the updated routing table
    portEXIT_CRITICAL(mux);  // Exit critical section
}

// Deletes routes by the destination address
static void _delRouteByDest(uint8_t dest) {
    portENTER_CRITICAL(mux);  // Enter critical section to prevent concurrent access
    uint8_t i;
    for (i = 0; i < ROUTING_TABLE_SIZE; i++) {
        if (_routes[i].dest == dest && _routes[i].state == Valid) {
            _routes[i].state = Invalid;  // Mark route as invalid
        }
    }
    ESP_LOGI(TAG, "DEL by dst: 0x%02x", dest);  // Debug log for route deletion
    PRINT_ROUTE_TABLE();  // Print the updated routing table
    portEXIT_CRITICAL(mux);  // Exit critical section
}
// Updates the link quality map for a given address
static void _updateLinkQualityMap(uint8_t addr, int16_t quality) {
     portENTER_CRITICAL(mux);  // Enter critical section
    _LinkQuality[addr].valid = true;  // Mark link quality as valid
    _LinkQuality[addr].updated = true;  // Mark link quality as updated
    _LinkQuality[addr].quality = quality;  // Set the link quality value
    portEXIT_CRITICAL(mux);  // Exit critical section
}


// Timer callback function to clean the link quality map
static void _clearLinkQualityMapTimer(TimerHandle_t xTimer) {
    portENTER_CRITICAL(mux);  // Enter critical section
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 255; i++) {
        if (_LinkQuality[i].valid == true) {
            if (_LinkQuality[i].updated == false) {
                cnt++;
                _LinkQuality[i].valid = false;  // Mark as invalid if not updated
            }
            _LinkQuality[i].updated = false;  // Reset update flag
        }
    }
    LOG_DBG(TAG,"LinkQualityMap clean: %d", cnt);  // Debug log for link quality map cleanup
    portEXIT_CRITICAL(mux);  // Exit critical section
}



// Mesh route interface implementation
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
