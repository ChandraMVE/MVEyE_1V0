//-----------------------------------------------------------------
///
///     \file leds.c
///
///     \brief led driver file
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#include "string.h"
#include "driver/gpio.h"
#include "stdbool.h"
#include "leds.h"
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================

//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
bool state_led = true;
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================

/*******************************************************************************
 * Function name  : leds_task
 *
 * Description    : function to manage all leds
 * Parameters     : pvParameters
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
static void leds_task(void *pvParameters)
{
	while(1)
	{
		gpio_set_level(GREEN_LED, (uint32_t)state_led);
		gpio_set_level(BLUE_LED, (uint32_t)state_led);
		gpio_set_level(RED_LED, (uint32_t)state_led);
		vTaskDelay(250 / portTICK_PERIOD_MS);
		state_led = !state_led;
	}
	
}

/*******************************************************************************
 * Function name  : create_leds_task
 *
 * Description    : function to create a task manager for leds
 * Parameters     : pvParameters
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void create_leds_task(void)
{
	xTaskCreate(leds_task, "leds_task", 1024, NULL, 10, NULL);
}

/*******************************************************************************
 * Function name  : init_leds
 *
 * Description    : function to initialize the leds as output
 * Parameters     : void
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void init_leds(void)
{
	//Special care for JTAG pins
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GREEN_LED], PIN_FUNC_GPIO);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[BLUE_LED], PIN_FUNC_GPIO);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[RED_LED], PIN_FUNC_GPIO);
	
	//Configure all LEDs as output
	gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(BLUE_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
	
	//Set all LEDs default Off state
	gpio_set_level(GREEN_LED, (uint32_t)state_led);
	gpio_set_level(BLUE_LED, (uint32_t)state_led);
	gpio_set_level(RED_LED,(uint32_t)state_led);
}