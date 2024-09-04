//-----------------------------------------------------------------
///
///     \file Acceleromter_app.c
///
///     \brief Acceleromter_app application framework driver 
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
#include "accelerometer_KXTJ3.h"
#include "driver/i2c_master.h"
#include "hal/i2c_types.h"
#include "hal/i2c_hal.h"
#include "hal/gpio_hal.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include "esp_err.h"
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "Accelerometer_App"
//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
extern float acceleration_reading[3];
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================

/*******************************************************************************
 * Function name  : Accelerometer_Task
 *
 * Description    : FreeRTOS task to read and process accelerometer data.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   : Ensure task does not block indefinitely.
 * Note           : Static; initialize accelerometer before use.
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 static void Accelerometer_Task (void *pvParameters)
 {
 	ESP_LOGI(pcTaskGetName(NULL), "Start");

	uint8_t who_am_i;    
	
    // Initialize KXTJ3-1057
    if (accel7_init(_ACCEL7_DATA_RESP_8bit, _ACCEL7_RANGE_16g) != 0) 
    {
        ESP_LOGE(TAG, "KXTJ3-1057 initialization failed");
        return;
    }
	while(1) 
	{
		who_am_i = accel7_readByte(_ACCEL7_REG_WHO_AM_I);
		// Read Who Am I Reg
		ESP_LOGI("APP", "Who Am I register value: 0x%02X",who_am_i);
		
		// Read accelerometer data
    	acceleration_reading[0] =  accel7_getAxis(_ACCEL7_AXIS_X);
   	 	acceleration_reading[1] =  accel7_getAxis(_ACCEL7_AXIS_Y);
    	acceleration_reading[2] =  accel7_getAxis(_ACCEL7_AXIS_Z);
    	
		// Print accelerometer data
    	ESP_LOGI(TAG, "Accelerometer readings: X=%f, Y=%f, Z=%f", acceleration_reading[0], acceleration_reading[1], acceleration_reading[2]);
    	
		vTaskDelay(pdMS_TO_TICKS(1000));
	} // end while 
 }

/*******************************************************************************
 * Function name  : create_Accelerometer_task
 *
 * Description    : function to create Acceleration data task
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void create_Accelerometer_task(void)
{
    xTaskCreate(Accelerometer_Task, "Accelerometer_Task", 1024*8, NULL, 1, NULL);

}






