//-----------------------------------------------------------------
///
///     \file Accelerometer_app.c
///
///     \brief Accelerometer application framework driver
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
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "accelero_app.h"
#include "accelero_driver.h"

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

//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================
static QueueHandle_t gpio_evt_queue = NULL;


/***********************************************************************************
 * Function name  : gpio_isr_handler
 *
 * Description    : gpio_isr_handler.
 * Parameters     : pointer arguments
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/*****************************************************************************************
 * Function name  : config_accelero_interrupt
 *
 * Description    : Configuring the Accelerometer and creating the task of accelerometer.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 *****************************************************************************************/
void config_accelero_interrupt()
{
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio interrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_34, GPIO_INTR_ANYEDGE);
    
        //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_34, gpio_isr_handler, (void*) GPIO_INPUT_IO_34);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_34);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_34, gpio_isr_handler, (void*) GPIO_INPUT_IO_34);
}

/***********************************************************************************
 * Function name  : accelero_gpio_interrupt_task
 *
 * Description    : accelero_gpio_interrupt_task.
 * Parameters     : pointer arguments
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 12SEP2024
 ***********************************************************************************/
static void Accelerometer_Task(void *pvParameters) {
    ESP_LOGI(TAG, "Accelero Task Created");
    
    config_accelero_interrupt();
	setup_accelero_latched();

    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            //printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            //printf(" Acceleration X, Y, Z = %f %f %f\r\n", axisAccel(X), axisAccel(Y), axisAccel(Z));
        }
    }    
 }

/*******************************************************************************
 * Function name  : create_accelerometer_task
 *
 * Description    : Function to create the Accelerometer data task
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   : Ensure task creation parameters are appropriate.
 * Note           : Task should be created after GPIO configuration.
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void create_Accelerometer_task() 
{
    xTaskCreate(Accelerometer_Task, "Accelerometer_Task", 2048*4, NULL, 10, NULL);
}