//-----------------------------------------------------------------
///
///     \file lora_llc68.c
///
///     \brief Lora chip driver source file
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
#include "arch/sys_arch.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lora_llc68.h"
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
/*define of lora spi bus*/
spi_device_handle_t lora_spi;

spi_bus_config_t lora_buscfg={
        .miso_io_num=LORA_MISO,
		.mosi_io_num=LORA_MOSI,
        .sclk_io_num=LORA_SCK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
};

/* Configuration of LORA spi bus */
spi_device_interface_config_t lora_devcfg={
        .clock_speed_hz=12*1000*1000,           //Clock out at 12 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num= -1,//LORA_SPI_CS,             //CS pin
        .queue_size=7                           //We want to be able to queue 7 transactions at a time
};
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================
static char tag[] = "lora_llc68";

static void vspi_init(void)
{
	esp_err_t ret;
	
	ESP_LOGI(tag, "vspi initialization \n");
	
	ret = spi_bus_initialize(VSPI_HOST, &lora_buscfg, 1);
    ESP_ERROR_CHECK(ret);

}

/*******************************************************************************
 * Function name  : lora_spi_init
 *
 * Description    : function to initialize the lora spi
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void lora_spi_init(void)
{
	esp_err_t ret;
	
	ESP_LOGI(tag, "LORA spi config init\n");
	vspi_init();
	
    ret=spi_bus_add_device(VSPI_HOST, &lora_devcfg, &lora_spi);
		
    ESP_ERROR_CHECK(ret);
}

/*******************************************************************************
 * Function name  : lora_spi_remove
 *
 * Description    : function to remove the lora spi
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void lora_spi_remove(void)
{
	esp_err_t ret;
	
	ESP_LOGI(tag, "lora spi bus removed \n");
	
	ret = spi_bus_remove_device(lora_spi);
	ESP_ERROR_CHECK(ret);
}

/*******************************************************************************
 * Function name  : lora_cs_low
 *
 * Description    : function to drive lora chipselect low
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
static void lora_cs_low(void)
{
	gpio_set_level(LORA_RESETn, DRIVE_LOW);
}

/*******************************************************************************
 * Function name  : lora_cs_high
 *
 * Description    : function to drive lora chipselect high
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
static void lora_cs_high(void)
{
	gpio_set_level(LORA_RESETn, DRIVE_HIGH);
}


/*******************************************************************************
 * Function name  : lora_reset_low
 *
 * Description    : function to drive lora reset low
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
static void lora_reset_low(void)
{
	gpio_set_level(LORA_RESETn, DRIVE_LOW);
}

/*******************************************************************************
 * Function name  : lora_reset_high
 *
 * Description    : function to drive lora reset high
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
static void lora_reset_high(void)
{
	gpio_set_level(LORA_RESETn, DRIVE_HIGH);
}

/*******************************************************************************
 * Function name  : lora_io_init
 *
 * Description    : function to initialize the lora io's
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/

void lora_io_init(void)
{
	ESP_LOGI(tag, "lora ios initialization \n");
	gpio_set_direction(LORA_CSn,GPIO_MODE_OUTPUT);
	gpio_set_direction(LORA_RESETn,GPIO_MODE_OUTPUT);
	gpio_set_direction(LORA_BUSY,GPIO_MODE_INPUT);
	
	lora_cs_high();
	lora_reset_low();	
	sys_delay_ms(100);// we want to delay Reset for 100mSec
	lora_reset_high();
}





