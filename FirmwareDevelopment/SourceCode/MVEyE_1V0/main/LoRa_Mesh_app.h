/*
 * LoRa_Mesh_app.h
 *
 *  Created on: 20-Sep-2024
 *      Author: User
 */

#ifndef MAIN_LORA_MESH_APP_H_
#define MAIN_LORA_MESH_APP_H_

#include "LoRa_Mesh_Net.h"


void mesh_task( void *pvParameters );
void app_stat_task ( void * pvParameter);
void app_recv_task (void * pvParameter);
void app_gw_task (void * pvParameter);
void app_control (LoRaPkg *p);
void notify_ping_app (LoRaPkg *p);
void print_pingret (LoRaPkg *p);
void print_data (LoRaPkg *p);
void app_ping_task(void * pvParameter);
void create_mesh_task( void );





#endif /* MAIN_LORA_MESH_APP_H_ */
