//-----------------------------------------------------------------
///
///     \file lora_app.h
///
///     \brief lora application framework driver header
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

#ifndef MAIN_LORA_APP_H_
#define MAIN_LORA_APP_H_

//==============================================================================
//          __             __   ___  __
//  | |\ | /  ` |    |  | |  \ |__  /__`
//  | | \| \__, |___ \__/ |__/ |___ .__/
//
//==============================================================================

//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#include <stdint.h>
#define CONFIG_868MHZ	1
#define CONFIG_USE_TCXO	0
#define CONFIF_ADVANCED 0
#define CONFIG_SF_RATE	7
#define CONFIG_BANDWIDTH	4
#define CONFIG_CODING_RATE	1
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
extern uint8_t rxBuffer[256];
//==============================================================================
//   __        __          __      ___            __  ___    __        __
//  |__) |  | |__) |    | /  `    |__  |  | |\ | /  `  |  | /  \ |\ | /__`
//  |    \__/ |__) |___ | \__,    |    \__/ | \| \__,  |  | \__/ | \| .__/
//
//==============================================================================
void create_lora_task(void);
void LoRaAppInit(void);
void configure_lora_for_rx(void);
void init_dio1_interrupt_Rx(void);
void init_dio1_interrupt_Tx(void);
void enable_dio1_interrupt_Rx(void);
void disaable_dio1_interrupt_Rx(void);
void enable_dio1_interrupt_Tx(void);
void disaable_dio1_interrupt_Tx(void);
void gpio_init_for_lora_irq(void);
void tx_done_isr_handler(void* arg);
void rx_done_isr_handler(void* arg);
#endif /* MAIN_LORA_APP_H_ */
