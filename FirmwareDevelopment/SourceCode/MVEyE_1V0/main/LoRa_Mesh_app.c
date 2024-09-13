#include "lora_llc68.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"
#include "esp_timer.h"
#include "LoRa_Mesh_app.h"

#define TAG "LORA"

// Node-specific configuration
#define NODE_ID 1
#define SENDER_ID 1
#define target_ID 2

// Global routing table
RoutingTableEntry routingTable[10];
int routingTableSize = 0;
bool isMaster = false;  // Variable to track if this node is the master

void initiateMasterElection() {
    sendMessage("Master Election");

    bool masterFound = checkForExistingMaster();

    if (!masterFound) {
        ESP_LOGI(TAG, "No master found. This %d node will become the master.",NODE_ID);
        isMaster = true;
        sendMessage("I am the master");
    } else {
        ESP_LOGI(TAG, "Master found. This node will be a follower%d.",NODE_ID);
        isMaster = false;
    }
}

bool checkForExistingMaster() {
    uint8_t buffer[256];
    int64_t startTime = esp_timer_get_time();
    while ((esp_timer_get_time() - startTime) < MASTER_ELECTION_TIMEOUT_MS) {
        int length = LoRaReceive(buffer, sizeof(buffer));
        if (length > 0) {
            LoRaMessage* loraMessage = (LoRaMessage*)buffer;
            if (strstr(loraMessage->payload, "I am the master")) {
                ESP_LOGI(TAG, "Master found: Node %d", loraMessage->sourceID);
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for receiving any broadcast
    }
    return false;
}

void sendMessage(const char* message) {
    LoRaMessage loraMessage;
    loraMessage.sourceID = SENDER_ID;
    loraMessage.targetID = target_ID;  // Broadcast to all nodes
    loraMessage.hopCount = 0;
    strncpy(loraMessage.payload, message, sizeof(loraMessage.payload) - 1);
    loraMessage.payload[sizeof(loraMessage.payload) - 1] = '\0';  // Ensure null-termination

    LoRaSend((uint8_t*)&loraMessage, sizeof(loraMessage), LLCC68_TXMODE_SYNC);
    ESP_LOGI(TAG, "Message sent to Node %d: %s", target_ID, message);
}

void receiveMessage(void) {
    uint8_t buffer[256];
    int length = LoRaReceive(buffer, sizeof(buffer));

    if (length > 0) {
        LoRaMessage* loraMessage = (LoRaMessage*)buffer;

        ESP_LOGI(TAG, "Received message from Node %d: %s", loraMessage->sourceID, loraMessage->payload);

        // If the message is meant for this node
        if (loraMessage->targetID == NODE_ID) {
            ESP_LOGI(TAG, "This node is the target%d.",NODE_ID);

            // Create a response message with swapped IDs
            LoRaMessage responseMessage;
            responseMessage.sourceID = NODE_ID;  // This node's ID is now the source
            responseMessage.targetID = loraMessage->sourceID; // Original source ID is now the target
            responseMessage.hopCount = loraMessage->hopCount + 1; // Increment hop count
            strncpy(responseMessage.payload, "I am your servant", sizeof(responseMessage.payload) - 1);
            responseMessage.payload[sizeof(responseMessage.payload) - 1] = '\0'; // Ensure null termination

            // Send the response
            LoRaSend((uint8_t*)&responseMessage, sizeof(responseMessage), LLCC68_TXMODE_SYNC);
            ESP_LOGI(TAG, "Response sent to Node %d: %s", responseMessage.targetID, responseMessage.payload);
        } else {
            // If the message is not meant for this node, relay it
            relayMessage(loraMessage);
        }
    }
}

void relayMessage(LoRaMessage* message) {
    if (message->hopCount >= MAX_HOP_COUNT) {
        ESP_LOGW(TAG, "Message dropped: exceeded max hop count.");
        return;
    }

    // Estimate signal quality to improve routing decisions
    estimateSignalQuality(message->sourceID, message->targetID);
    
    // Update the routing table with this node's information
    addNodeToRoutingTable(message->sourceID, NODE_ID, message->hopCount,
        getSignalStrength(NODE_ID, message->sourceID),
        getSignalStrength(NODE_ID, message->targetID));

    message->hopCount++;
    int nextHop = findBestRelayNode(message->targetID, message->sourceID);

    if (nextHop != -1) {
        LoRaSend((uint8_t*)message, sizeof(*message), LLCC68_TXMODE_SYNC);
        ESP_LOGI(TAG, "Relayed message to Node %d", nextHop);
    } else {
        ESP_LOGW(TAG, "No route found to target");
    }
}

void respondToMessage(LoRaMessage* message) {
    if (message->sourceID == NODE_ID) {
        // This node is the source, so no need to respond
        return;
    }

    LoRaMessage responseMessage;
    responseMessage.sourceID = NODE_ID;
    responseMessage.targetID = message->sourceID;  // Respond to the original sender
    responseMessage.hopCount = message->hopCount + 1;  // Increment hop count for response
    strncpy(responseMessage.payload, "I am your servant", sizeof(responseMessage.payload) - 1);
    responseMessage.payload[sizeof(responseMessage.payload) - 1] = '\0';  // Ensure null-termination

    LoRaSend((uint8_t*)&responseMessage, sizeof(responseMessage), LLCC68_TXMODE_SYNC);
    ESP_LOGI(TAG, "Response sent to Node %d with hop count %d", message->sourceID, responseMessage.hopCount);
}

void addNodeToRoutingTable(uint8_t nodeID, uint8_t nextHop, uint8_t hopCount, int8_t signalStrengthToSource, int8_t signalStrengthToTarget) {
    for (int i = 0; i < routingTableSize; i++) {
        if (routingTable[i].nodeID == nodeID) {
            if (hopCount < routingTable[i].hopCount) {
                routingTable[i].nextHop = nextHop;
                routingTable[i].hopCount = hopCount;
                routingTable[i].signalStrengthToSource = signalStrengthToSource;
                routingTable[i].signalStrengthToTarget = signalStrengthToTarget;
            }
            return;
        }
    }

    if (routingTableSize < 10) {
        routingTable[routingTableSize].nodeID = nodeID;
        routingTable[routingTableSize].nextHop = nextHop;
        routingTable[routingTableSize].hopCount = hopCount;
        routingTable[routingTableSize].signalStrengthToSource = signalStrengthToSource;
        routingTable[routingTableSize].signalStrengthToTarget = signalStrengthToTarget;
        routingTableSize++;
    }
}

void updateRoutingTable(uint8_t sourceID, uint8_t targetID, int8_t rssiFromSource, int8_t rssiFromTarget) {
    for (int i = 0; i < routingTableSize; i++) {
        if (routingTable[i].nodeID == sourceID) {
            routingTable[i].signalStrengthToSource = rssiFromSource;
        }
        if (routingTable[i].nodeID == targetID) {
            routingTable[i].signalStrengthToTarget = rssiFromTarget;
        }
    }
}

void estimateSignalQuality(uint8_t sourceID, uint8_t targetID) {
    // Send a test message from the source node
    sendMessage("TEST");

    // Wait for a short period to receive the test message
    vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed

    // Measure RSSI of the test message received from the source node
    int8_t rssiFromSource = getSignalStrength(NODE_ID, sourceID);

    // Send a test message from the target node
    sendMessage("TEST");

    // Wait for a short period to receive the test message
    vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed

    // Measure RSSI of the test message received from the target node
    int8_t rssiFromTarget = getSignalStrength(NODE_ID, target_ID);

    // Update routing table with estimated signal quality
    updateRoutingTable(sourceID, targetID, rssiFromSource, rssiFromTarget);

    // Log the RSSI values for debugging
    ESP_LOGI(TAG, "RSSI from Source Node %d: %d", sourceID, rssiFromSource);
    ESP_LOGI(TAG, "RSSI from Target Node %d: %d", targetID, rssiFromTarget);
}

int findBestRelayNode(uint8_t targetID, uint8_t sourceID) {
    int8_t bestSignalQuality = -128;  // Initialize with lowest possible RSSI value
    int bestNodeID = -1;
    for (int i = 0; i < routingTableSize; i++) {
        if (routingTable[i].nodeID != sourceID && routingTable[i].nodeID != targetID) {
            int8_t signalStrengthToSource = routingTable[i].signalStrengthToSource;
            int8_t signalStrengthToTarget = routingTable[i].signalStrengthToTarget;

            // Calculate the overall signal quality as an average or combined metric
            int8_t signalQuality = (signalStrengthToSource + signalStrengthToTarget) / 2;

            if (signalQuality > bestSignalQuality) {
                bestSignalQuality = signalQuality;
                bestNodeID = routingTable[i].nodeID;
            }
        }
    }
    return bestNodeID;
}

// Function to get RSSI value from LLCC68
int8_t getSignalStrength(uint8_t nodeID, uint8_t targetID) {
    // Replace with actual RSSI retrieval code
    return GetRssiInst();
}
// Task to handle sending and receiving messages
void messageTask(void* pvParameters) {
	if (NODE_ID == SENDER_ID) { // Check if this node is the sender
        sendMessage("I am the boss"); // Send the predefined message
    }
    while (1) {
        receiveMessage();
        vTaskDelay(pdMS_TO_TICKS(MESSAGE_TASK_PERIOD_MS));
    }
}

// Task to handle master election
void masterElectionTask(void* pvParameters) {
    initiateMasterElection();
    
    while(1){
		
		vTaskDelay(pdMS_TO_TICKS(MASTER_ELECTION_TIMEOUT_MS));
	}
}

void create_lora_mesh_Task() {
    // Create the task for master election (optional: you can start this based on a condition)
    xTaskCreate(masterElectionTask, "Master_Election_Task", 1024*4, NULL, 5, NULL);
    ESP_LOGI(TAG, "Master election task created");
    
     xTaskCreate(messageTask, "Message_Task", 1024*4, NULL, 6, NULL);
    ESP_LOGI(TAG, "Message handling task created");
}
