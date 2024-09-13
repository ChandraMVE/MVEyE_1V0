#ifndef MAIN_LORA_MESH_APP_H_
#define MAIN_LORA_MESH_APP_H_

#include "stdint.h"
#include "stdbool.h"

// Constants
#define BROADCAST_ID 255  // Broadcast to all nodes
#define MAX_HOP_COUNT 10
#define MASTER_ELECTION_TIMEOUT_MS 2000  // Timeout for master election in milliseconds
#define MESSAGE_TASK_PERIOD_MS 1000      // Period for message handling task

// Structures
typedef struct {
    uint8_t sourceID;
    uint8_t targetID;
    uint8_t hopCount;
    char payload[100];
} LoRaMessage;

typedef struct {
    uint8_t nodeID;
    uint8_t nextHop;
    uint8_t hopCount;
    int8_t signalStrengthToSource;  // Signal strength to the master node
    int8_t signalStrengthToTarget;  // Signal strength to the target node
} RoutingTableEntry;

// Function declarations
void respondToMessage(LoRaMessage* message);
void sendMessage(const char* message);
void receiveMessage();
void relayMessage(LoRaMessage* message);
void addNodeToRoutingTable(uint8_t nodeID, uint8_t nextHop, uint8_t hopCount, int8_t signalStrengthToMaster, int8_t signalStrengthToTarget);
int findBestRelayNode(uint8_t targetID, uint8_t sourceID);
int8_t getSignalStrength(uint8_t nodeID, uint8_t targetID);
void initiateMasterElection();
bool checkForExistingMaster();
void messageTask(void* pvParameters);
void masterElectionTask(void* pvParameters);
void create_lora_mesh_Task(void);
void estimateSignalQuality(uint8_t sourceID, uint8_t targetID);
void estimateSignalQuality(uint8_t sourceID, uint8_t targetID);

#endif /* MAIN_LORA_MESH_APP_H_ */
