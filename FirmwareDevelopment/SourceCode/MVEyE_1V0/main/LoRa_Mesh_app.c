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
#include "lora_llc68.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"
#include "esp_timer.h"
#include "LoRa_Mesh_app.h"
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "LORA"
// Node-specific configuration
#define NODE_ID 3
#define SENDER_ID 1
#define TARGET_ID 2
#define MASTER_ELECTION_TIMEOUT_MS 3000  // 3 seconds for master election timeout
#define MESSAGE_TASK_PERIOD_MS 1000 // Adjust as needed
//==============================================================================
//  ___      __   ___  __   ___  ___  __
//   |  \ / |__) |__  |  \ |__  |__  /__`
//   |   |  |    |___ |__/ |___ |    .__/
//
//==============================================================================

//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
// Global routing table
RoutingTableEntry routingTable[10];
int routingTableSize = 0;
bool responseReceived = false;  // Global flag to track if the response has been received
bool messageReceivedBySender = false; // Flag to track if the sender node has received a response

//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================

static bool isMaster = false; // Global flag to indicate if this node is the master
static uint8_t masterNodeID = 0; // Global variable to store the master node ID
//==============================================================================
//   __        __          __      ___            __  ___    __        __
//  |__) |  | |__) |    | /  `    |__  |  | |\ | /  `  |  | /  \ |\ | /__`
//  |    \__/ |__) |___ | \__,    |    \__/ | \| \__,  |  | \__/ | \| .__/
//
//==============================================================================

/*****************************************************************************************
 * Function name  : initiateMasterElection
 *
 * Description    : Starts the master election process. If no master is found, this node 
 *                   becomes the master and notifies all nodes.
 * Parameters     : None
 * Returns        : None
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
void initiateMasterElection() {
    ESP_LOGI(TAG, "Node %d is starting master election", NODE_ID);
    sendMessage("Master Election");

    // Check if a master exists within the timeout period
    bool masterFound = checkForExistingMaster();

    if (!masterFound) {
        // If no master is found, this node becomes the master
        ESP_LOGI(TAG, "No master found. This node %d will become the master.", NODE_ID);
        isMaster = true;
        masterNodeID = NODE_ID; // Set the master node ID
        sendMessage("I am the master");

        // Broadcast master node ID to all nodes
        //notifyAllNodesOfMaster(masterNodeID);
    } else {
        // If a master is found, this node becomes a follower
        ESP_LOGI(TAG, "Master found. This node %d will be a follower.", NODE_ID);
        isMaster = false;
    }
}
/*****************************************************************************************
 * Function name  : checkForExistingMaster
 *
 * Description    : Checks for an existing master node by listening for master messages. 
 *                   Returns true if a master is found within the timeout period.
 * Parameters     : None
 * Returns        : bool - true if a master is found, false otherwise.
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
bool checkForExistingMaster() {
    uint8_t buffer[256];
    int64_t startTime = esp_timer_get_time();
    while ((esp_timer_get_time() - startTime) < MASTER_ELECTION_TIMEOUT_MS) {
        int length = LoRaReceive(buffer, sizeof(buffer));
        if (length > 0) {
            LoRaMessage* loraMessage = (LoRaMessage*)buffer;
            if (strstr(loraMessage->payload, "I am the master")) {
                masterNodeID = loraMessage->sourceID; // Set the master node ID
                ESP_LOGI(TAG, "Master found: Node %d", masterNodeID);
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for receiving any broadcast
    }
    return false;
}

/*****************************************************************************************
 * Function name  : notifyAllNodesOfMaster
 *
 * Description    : Sends a broadcast message to all nodes with the ID of the new master.
 * Parameters     : uint8_t masterID - ID of the new master node.
 * Returns        : None
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
void notifyAllNodesOfMaster(uint8_t masterID) {
    char message[50];
    snprintf(message, sizeof(message), "Master ID: %d", masterID);
    sendMessage(message);
}
/*****************************************************************************************
 * Function name  : sendMessage
 *
 * Description    : Sends a message with the specified payload to a target node.
 * Parameters     : const char* message - The message payload.
 * Returns        : None
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
void sendMessage(const char* message) {
    LoRaMessage loraMessage;
    loraMessage.sourceID = SENDER_ID;
    loraMessage.targetID = TARGET_ID;  // Broadcast to all nodes
    loraMessage.hopCount = 0;
    strncpy(loraMessage.payload, message, sizeof(loraMessage.payload) - 1);
    loraMessage.payload[sizeof(loraMessage.payload) - 1] = '\0';  // Ensure null-termination

    LoRaSend((uint8_t*)&loraMessage, sizeof(loraMessage), LLCC68_TXMODE_SYNC);
    ESP_LOGI(TAG, "Message sent to Node %d: %s", TARGET_ID, message);
}

/*****************************************************************************************
 * Function name  : receiveMessage
 *
 * Description    : Receives and processes a message. If it's for this node, it handles
 *                   it appropriately; otherwise, it relays the message.
 * Parameters     : None
 * Returns        : None
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
void receiveMessage(void) {
    static bool messageRelayed = false;  // Flag to track if the message has been relayed
    
    uint8_t buffer[256];
    int length = LoRaReceive(buffer, sizeof(buffer));

    if (length > 0) {
        // Cast the received buffer to a LoRaMessage structure
        LoRaMessage* loraMessage = (LoRaMessage*)buffer;

        // Log the received message details
        ESP_LOGI(TAG, "Received message from Node %d: %s", loraMessage->sourceID, loraMessage->payload);

        // If the message is meant for this node
        if (loraMessage->targetID == NODE_ID) {
            ESP_LOGI(TAG, "This node is the target: Node %d.", NODE_ID);

            // Check if the received message says "I am the boss"
            if (strcmp(loraMessage->payload, "I am the boss") == 0) {
                ESP_LOGI(TAG, "Message received: 'I am the boss'");
                // Create a response message with swapped IDs
                LoRaMessage responseMessage;
                responseMessage.sourceID = NODE_ID;  // This node's ID is now the source
                responseMessage.targetID = loraMessage->sourceID;  // Original source ID is now the target
                responseMessage.hopCount = loraMessage->hopCount + 1;  // Increment hop count
                responseMessage.rssi = getSignalStrength();
                strncpy(responseMessage.payload, "I am your servant", sizeof(responseMessage.payload) - 1);
                responseMessage.payload[sizeof(responseMessage.payload) - 1] = '\0';  // Ensure null termination

                // Send the response
                LoRaSend((uint8_t*)&responseMessage, sizeof(responseMessage), LLCC68_TXMODE_SYNC);
                ESP_LOGI(TAG, "Response sent to Node %d: %s, RSSI: %d (Hop Count: %d)", responseMessage.targetID, responseMessage.payload, responseMessage.rssi,responseMessage.hopCount);
            } else {
                ESP_LOGI(TAG, "Already sender received message: %s  RSSI: %d (Hop Count: %d)", loraMessage->payload,loraMessage->rssi ,loraMessage->hopCount);
            }
        } else if (!messageRelayed) {
            // If the message is not meant for this node and hasn't been relayed yet, relay it
            ESP_LOGI(TAG, "This node is not the target. Relaying the message.");
            relayMessage(loraMessage);

            // Mark that the message has been relayed
            messageRelayed = true;
        }
    } else {
        ESP_LOGW(TAG, "No message received or an error occurred.");
    }
}

/*****************************************************************************************
 * Function name  : relayMessage
 *
 * Description    : Relays a received message to the next node, incrementing the hop count.
 * Parameters     : LoRaMessage* loraMessage - The message to relay.
 * Returns        : None
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
void relayMessage(LoRaMessage* loraMessage) {
    // Increment hop count before relaying the message
    loraMessage->hopCount += 1;
    int8_t rssi = getSignalStrength();
    // Log that the message is being relayed
    ESP_LOGI(TAG, "Relaying message from Node %d to Node %d (Hop Count: %d), RSSI: %d", 
             loraMessage->sourceID, loraMessage->targetID, loraMessage->hopCount, rssi);

    // Send the message to the next node
    LoRaSend((uint8_t*)loraMessage, sizeof(LoRaMessage), LLCC68_TXMODE_SYNC);
}
/*****************************************************************************************
 * Function name  : getSignalStrength
 *
 * Description    : Retrieves the RSSI value from the LoRa module.
 * Parameters     : None
 * Returns        : int8_t - The RSSI value.
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
// Function to get RSSI value from LLCC68
int8_t getSignalStrength() {
    // Replace with actual RSSI retrieval code
    return GetRssiInst();
}
/*****************************************************************************************
 * Function name  : messageTask
 *
 * Description    : Task for sending and receiving messages. If this node is the sender, 
 *                   it sends a predefined message and waits for a response.
 * Parameters     : void* pvParameters - Task parameters (not used).
 * Returns        : None
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
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
/*****************************************************************************************
 * Function name  : masterElectionTask
 *
 * Description    : Task for handling master election. It calls initiateMasterElection 
 *                   and waits for the timeout period.
 * Parameters     : void* pvParameters - Task parameters (not used).
 * Returns        : None
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
// Task to handle master election
void masterElectionTask(void* pvParameters) {
    initiateMasterElection();
    
    while(1){
        vTaskDelay(pdMS_TO_TICKS(MASTER_ELECTION_TIMEOUT_MS));
    }
}
/*****************************************************************************************
 * Function name  : create_lora_mesh_Task
 *
 * Description    : Creates tasks for master election and message handling.
 * Parameters     : None
 * Returns        : None
 * author         : C.VenkataSuresh
 * date           : 16 Sep 2024
 *****************************************************************************************/
void create_lora_mesh_Task() {
    // Create the task for master election (optional: you can start this based on a condition)
    xTaskCreate(masterElectionTask, "Master_Election_Task", 1024*4, NULL, 5, NULL);
    ESP_LOGI(TAG, "Master election task created");
    
    xTaskCreate(messageTask, "Message_Task", 1024*4, NULL, 6, NULL);
    ESP_LOGI(TAG, "Message handling task created");
}
