//-----------------------------------------------------------------
///
///     \file LoRa_Mesh_app.h
///
///     \brief Lora_Mesh_app driver header file
///
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
///		 Operating System: windows 10
/// 	 Java Runtime Version: 17.0.11+9
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

#ifndef MAIN_LORA_MESH_APP_H_
#define MAIN_LORA_MESH_APP_H_

#include "stdint.h"
#include "stdbool.h"

//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
// Constants
#define BROADCAST_ID 255  // Broadcast to all nodes
#define MAX_HOP_COUNT 10
#define MESSAGE_TASK_PERIOD_MS 1000      // Period for message handling task
//==============================================================================
//  ___      __   ___  __   ___  ___  __
//   |  \ / |__) |__  |  \ |__  |__  /__`
//   |   |  |    |___ |__/ |___ |    .__/
//
//==============================================================================
// Structures
typedef struct {
    uint8_t sourceID;
    uint8_t targetID;
    uint8_t hopCount;
    char payload[100];
    int rssi;
} LoRaMessage;

typedef struct {
    uint8_t nodeID;
    uint8_t nextHop;
    uint8_t hopCount;
    int8_t signalStrengthToSource;  // Signal strength to the master node
    int8_t signalStrengthToTarget;  // Signal strength to the target node
} RoutingTableEntry;

//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================


//==============================================================================
//   __        __          __      ___            __  ___    __        __
//  |__) |  | |__) |    | /  `    |__  |  | |\ | /  `  |  | /  \ |\ | /__`
//  |    \__/ |__) |___ | \__,    |    \__/ | \| \__,  |  | \__/ | \| .__/
//
//==============================================================================

// Function declarations
void sendMessage(const char* message);
bool hasMessageBeenRelayed(uint8_t sourceID, uint8_t hopCount);
void addRelayedMessage(uint8_t sourceID, uint8_t hopCount);
void receiveMessage();
void relayMessage(LoRaMessage* message);
int8_t getSignalStrength(void);
void initiateMasterElection();
bool checkForExistingMaster();
void messageTask(void* pvParameters);
void masterElectionTask(void* pvParameters);
void create_lora_mesh_Task(void);
void notifyAllNodesOfMaster(uint8_t masterID);

#endif /* MAIN_LORA_MESH_APP_H_ */
