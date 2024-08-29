//-----------------------------------------------------------------
///
///     \file accelerometer_KXTJ3.c
///
///     \brief accelerometer driver code
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
// Sensitivity factors for different full-scale ranges
// These should be defined based on your accelerometer's datasheet
#define SENSITIVITY_FACTOR_2G_8BIT 0.016f
#define SENSITIVITY_FACTOR_4G_8BIT 0.031f
#define SENSITIVITY_FACTOR_8G_8BIT 0.0625f
#define SENSITIVITY_FACTOR_16G_8BIT 0.125f

#define SENSITIVITY_FACTOR_2G_12BIT 0.001f
#define SENSITIVITY_FACTOR_4G_12BIT 0.002f
#define SENSITIVITY_FACTOR_8G_12BIT 0.0039f
#define SENSITIVITY_FACTOR_16G_12BIT 0.0078f

#define SENSITIVITY_FACTOR_8G_14BIT 0.00098f
#define SENSITIVITY_FACTOR_16G_14BIT 0.00195f

// Sensitivity factors array based on the full-scale range and resolution
// Indexed by [range][resolution]
float sensitivityFactors[4][3] = {
    { SENSITIVITY_FACTOR_2G_8BIT, SENSITIVITY_FACTOR_2G_12BIT, 0.0f }, // Assuming full-scale range 2G
    { SENSITIVITY_FACTOR_4G_8BIT, SENSITIVITY_FACTOR_4G_12BIT, 0.0f }, // Assuming full-scale range 4G
    { SENSITIVITY_FACTOR_8G_8BIT, SENSITIVITY_FACTOR_8G_12BIT, SENSITIVITY_FACTOR_8G_14BIT }, // Assuming full-scale range 8G
    { SENSITIVITY_FACTOR_16G_8BIT, SENSITIVITY_FACTOR_16G_12BIT, SENSITIVITY_FACTOR_16G_14BIT } // Assuming full-scale range 16G
};


uint8_t _fullScaleRange = 0; // This should be set according to your sensor configuration
//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
const uint8_t _ACCEL7_AXIS_X  = 0x06;
const uint8_t _ACCEL7_AXIS_Y  = 0x08;
const uint8_t _ACCEL7_AXIS_Z  = 0x0A;

const uint8_t _ACCEL7_DATA_RESP_8bit   = 0x10;
const uint8_t _ACCEL7_DATA_RESP_12bit  = 0x20;
const uint8_t _ACCEL7_DATA_RESP_14bit  = 0x30;

const uint8_t _ACCEL7_RANGE_2g   = 0x01;
const uint8_t _ACCEL7_RANGE_4g   = 0x02;
const uint8_t _ACCEL7_RANGE_8g   = 0x03;
const uint8_t _ACCEL7_RANGE_16g  = 0x04;

/* 22 Registers */
const uint8_t _ACCEL7_REG_AXIS_X_LSB           = 0x06;
const uint8_t _ACCEL7_REG_AXIS_X_MSB           = 0x07;
const uint8_t _ACCEL7_REG_AXIS_Y_LSB           = 0x08;
const uint8_t _ACCEL7_REG_AXIS_Y_MSB           = 0x09;
const uint8_t _ACCEL7_REG_AXIS_Z_LSB           = 0x0A;
const uint8_t _ACCEL7_REG_AXIS_Z_MSB           = 0x0B;
const uint8_t _ACCEL7_REG_DCST_RESP            = 0x0C;
const uint8_t _ACCEL7_REG_WHO_AM_I             = 0x0F;
const uint8_t _ACCEL7_REG_INT_SOURCE1          = 0x16;
const uint8_t _ACCEL7_REG_INT_SOURCE2          = 0x17;
const uint8_t _ACCEL7_REG_STATUS               = 0x18;
const uint8_t _ACCEL7_REG_INT_REL              = 0x1A;
const uint8_t _ACCEL7_REG_CTRL_REG1            = 0x1B;
const uint8_t _ACCEL7_REG_CTRL_REG2            = 0x1D;
const uint8_t _ACCEL7_REG_INT_CTRL_REG1        = 0x1E;
const uint8_t _ACCEL7_REG_INT_CTRL_REG2        = 0x1F;
const uint8_t _ACCEL7_REG_DATA_CTRL_REG        = 0x21;
const uint8_t _ACCEL7_REG_WAKEUP_COUNTER       = 0x29;
const uint8_t _ACCEL7_REG_NA_CAUNTER           = 0x2A;
const uint8_t _ACCEL7_REG_SELF_TEST            = 0x3A;
const uint8_t _ACCEL7_REG_WAKEUP_THRESHOLD_MSB = 0x6A;
const uint8_t _ACCEL7_REG_WAKEUP_THRESHOLD_LSB = 0x6B;

/* Control Regiter 1 masking */
const uint8_t _ACCEL7_CTRL_REG1_MODE_OPERATING      = 0x80;   //8th bit MSB  is Set means Operating Mode 
const uint8_t _ACCEL7_CTRL_REG1_MODE_STANDBY        = 0x00;   //8th bit MSB  is clear means standby Mode
const uint8_t _ACCEL7_CTRL_REG1_RES_LOW_CURRENT     = 0x00;   //7th bit MSB  is clear means Low power and 8-bit Resolution, Only available for ODR ≤ 200 Hz. Bandwidth (Hz) = 800.
const uint8_t _ACCEL7_CTRL_REG1_RES_HIGH_RESOLUTION = 0x40;   //7th bit MSB  is set means High Resolution, 12-bit or 14-bit valid. Bandwidth (Hz) = ODR/2
const uint8_t _ACCEL7_CTRL_REG1_DRDYE_ENABLE        = 0x20;   //6th bit MSB  is set means availability of new acceleration data is reflected as an interrupt
const uint8_t _ACCEL7_CTRL_REG1_DRDYE_DISABLE       = 0x00;   //6th bit MSB  is set means availability of new acceleration data is not reflected as an interrupt
const uint8_t _ACCEL7_CTRL_REG1_RANGE_2g            = 0x00;   //5th, 4th and 3rd bits MSB  are clear means selects the +_ 2g acceleration range of the accelerometer outputs per
const uint8_t _ACCEL7_CTRL_REG1_RANGE_4g            = 0x08;   //5th = 0, 4th = 1 and 3rd = 0 bits MSB  means selects the +_4g acceleration range of the accelerometer outputs per
const uint8_t _ACCEL7_CTRL_REG1_RANGE_8g            = 0x10;   //5th = 1, 4th = 0 and 3rd = 0 bits MSB  means selects the +_8g acceleration range of the accelerometer outputs per
const uint8_t _ACCEL7_CTRL_REG1_RANGE_16g           = 0x04;   //5th = 0, 4th = 0 and 3rd = 1 bits MSB  means selects the +_16g acceleration range of the accelerometer outputs per
const uint8_t _ACCEL7_CTRL_REG1_RANGE_HIGH_RES_8g   = 0x18;   //5th = 1, 4th = 1 and 3rd = 0 bits MSB  means selects the +_8g' acceleration range of the accelerometer outputs per
const uint8_t _ACCEL7_CTRL_REG1_RANGE_HIGH_RES_16g  = 0x1C;   //5th = 1, 4th = 1 and 3rd = 1 bits MSB  means selects the +_16g' acceleration range of the accelerometer outputs per
const uint8_t _ACCEL7_CTRL_REG1_WAKEUP_DISABLE      = 0x00;   //2nd bit is 0 Wake-Up function disabled
const uint8_t _ACCEL7_CTRL_REG1_WAKEUP_ENABLE       = 0x02;   //2nd bit is 1 Wake-Up function enabled

/* Control Register 2 */
const uint8_t _ACCEL7_CTRL_REG2_START_RAM_REBOOT       = 0x80; //8th bit MSB  is Set means start RAM reboot routine 
const uint8_t _ACCEL7_CTRL_REG2_DCST_ENABLE            = 0x10; //5th bit LSB  is Set means sets DCST_RESP register to 0xAA and when DCST_RESP is read,sets this bit to 0 and sets DCST_RESP to 0x55
const uint8_t _ACCEL7_CTRL_REG2_DCST_DISABLE           = 0x00; //5th bit LSB  is clear No Action
const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_0_781Hz  = 0x00; //3rd = 0, 2nd = 0 and 1st = 0 is sets the Output Data Rate for the Wake-Up function (motion detection) as 0.781Hz
const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_1_563Hz  = 0x01; //3rd = 0, 2nd = 0 and 1st = 1 is sets the Output Data Rate for the Wake-Up function (motion detection) as 0.781Hz*2
const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_3_125Hz  = 0x02; //3rd = 0, 2nd = 1 and 1st = 0 is sets the Output Data Rate for the Wake-Up function (motion detection) as 0.781Hz*4
const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_6_25Hz   = 0x03; //3rd = 0, 2nd = 1 and 1st = 1 is sets the Output Data Rate for the Wake-Up function (motion detection) as 0.781Hz*8
const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_12_5Hz   = 0x04; //3rd = 1, 2nd = 0 and 1st = 0 is sets the Output Data Rate for the Wake-Up function (motion detection) as 0.781Hz*16
const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_25Hz     = 0x05; //3rd = 1, 2nd = 0 and 1st = 1 is sets the Output Data Rate for the Wake-Up function (motion detection) as 0.781Hz*32
const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_50Hz     = 0x06; //3rd = 1, 2nd = 1 and 1st = 0 is sets the Output Data Rate for the Wake-Up function (motion detection) as 0.781Hz*64
const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_100Hz    = 0x07; //3rd = 1, 2nd = 1 and 1st = 1 is sets the Output Data Rate for the Wake-Up function (motion detection) as 0.781Hz*128

/* Interrupt control register 1 */
const uint8_t _ACCEL7_INT_CTRL_REG1_IEN_DISABLE  = 0x00; //5th bit = 0, sets the physical interrupt pin (INT) is disabled
const uint8_t _ACCEL7_INT_CTRL_REG1_IEN_ENABLE   = 0x20; //5th bit = 1, sets the physical interrupt pin (INT) is enabled
const uint8_t _ACCEL7_INT_CTRL_REG1_IEA_DISABLE  = 0x00; //4th bit = 0, sets the polarity of the physical interrupt pin (INT) is active LOW
const uint8_t _ACCEL7_INT_CTRL_REG1_IEA_ENABLE   = 0x10; //4th bit = 1, sets the polarity of the physical interrupt pin (INT) is active HIGH
const uint8_t _ACCEL7_INT_CTRL_REG1_IEL_DISABLE  = 0x00; //3rd bit = 0, sets the physical interrupt pin (INT) latches until it is cleared by reading INT_REL
const uint8_t _ACCEL7_INT_CTRL_REG1_IEL_ENABLE   = 0x08; //3rd bit = 1, sets the physical interrupt pin (INT) will transmit one pulse with a period of 0.03 - 0.05ms
const uint8_t _ACCEL7_INT_CTRL_REG1_SELF_TEST_POL_NEGATIVE  = 0x00; //2nd bit = 0, sets the negative polarity (Self-test unsupported)
const uint8_t _ACCEL7_INT_CTRL_REG1_SELF_TEST_POL_POSITIVE  = 0x02; //2nd bit = 1, sets the positive polarity (Self-test supported)

/* Interrupt control register 2 */
const uint8_t _ACCEL7_INT_CTRL_REG2_ULMODE_ENABLE       = 0x80;  // 8th bit = 0, sets the Unlatched mode is disabled
const uint8_t _ACCEL7_INT_CTRL_REG2_ULMODE_DISABLE      = 0x00;  // 8th bit = 1, sets the Unlatched mode is enabled
const uint8_t _ACCEL7_INT_CTRL_REG2_X_NEGATIVE_ENABLE   = 0x20;  // 6th bit = 1, sets the X-Axis Negative enabled
const uint8_t _ACCEL7_INT_CTRL_REG2_X_NEGATIVE_DISABLE  = 0x00;  // 6th bit = 0, sets the X-Axis Negative disabled
const uint8_t _ACCEL7_INT_CTRL_REG2_Y_NEGATIVE_ENABLE   = 0x08;  // 4th bit = 1, sets the Y-Axis Negative enabled
const uint8_t _ACCEL7_INT_CTRL_REG2_Y_NEGATIVE_DISABLE  = 0x00;  // 4th bit = 0, sets the Y-Axis Negative disabled
const uint8_t _ACCEL7_INT_CTRL_REG2_Z_NEGATIVE_ENABLE   = 0x02;  // 2th bit = 1, sets the Z-Axis Negative enabled
const uint8_t _ACCEL7_INT_CTRL_REG2_Z_NEGATIVE_DISABLE  = 0x00;  // 2th bit = 0, sets the Z-Axis Negative disabled
const uint8_t _ACCEL7_INT_CTRL_REG2_X_POSITIVE_ENABLE   = 0x10;  // 5th bit = 1, sets the X-Axis Positive enabled
const uint8_t _ACCEL7_INT_CTRL_REG2_X_POSITIVE_DISABLE  = 0x00;  // 5th bit = 0, sets the X-Axis Positive disabled
const uint8_t _ACCEL7_INT_CTRL_REG2_Y_POSITIVE_ENABLE   = 0x04;  // 3rd bit = 1, sets the Y-Axis Positive enabled
const uint8_t _ACCEL7_INT_CTRL_REG2_Y_POSITIVE_DISABLE  = 0x00;  // 3rd bit = 0, sets the Y-Axis Positive disabled
const uint8_t _ACCEL7_INT_CTRL_REG2_Z_POSITIVE_ENABLE   = 0x01;  // 1st bit = 1, sets the Z-Axis Positive enabled
const uint8_t _ACCEL7_INT_CTRL_REG2_Z_POSITIVE_DISABLE  = 0x00;  // 1st bit = 0, sets the Z-Axis Positive disabled

/* Data contorl register */
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_0_781Hz   = 0x08;  //4th bit = 1, 3rd bit = 0, 2nd bit = 0, 1st bit = 0, sets the 0.781 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_1_563Hz   = 0x09;  //4th bit = 1, 3rd bit = 0, 2nd bit = 0, 1st bit = 1, sets the 1.563 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_3_125Hz   = 0x0A;  //4th bit = 1, 3rd bit = 0, 2nd bit = 1, 1st bit = 0, sets the 3.125 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_6_25Hz    = 0x0B;  //4th bit = 1, 3rd bit = 0, 2nd bit = 1, 1st bit = 1, sets the 6.25 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_12_5Hz    = 0x00;  //4th bit = 0, 3rd bit = 0, 2nd bit = 0, 1st bit = 0, sets the 12.5 output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_25Hz      = 0x01;  //4th bit = 0, 3rd bit = 0, 2nd bit = 0, 1st bit = 1, sets the 25 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_50Hz      = 0x02;  //4th bit = 0, 3rd bit = 0, 2nd bit = 1, 1st bit = 0, sets the 50 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_100Hz     = 0x03;  //4th bit = 0, 3rd bit = 0, 2nd bit = 1, 1st bit = 1, sets the 100 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_200Hz     = 0x04;  //4th bit = 0, 3rd bit = 1, 2nd bit = 0, 1st bit = 0, sets the 200 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_400Hz     = 0x05;  //4th bit = 0, 3rd bit = 1, 2nd bit = 0, 1st bit = 1, sets the 400 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_800Hz     = 0x06;  //4th bit = 0, 3rd bit = 1, 2nd bit = 1, 1st bit = 0, sets the 800 Hz output data rate (ODR) for the low-pass filtered
const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_1600Hz    = 0x07;  //4th bit = 0, 3rd bit = 1, 2nd bit = 1, 1st bit = 1, sets the 1600 Hz output data rate (ODR) for the low-pass filtered

i2c_master_dev_handle_t dev_handle;
//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================
static const uint8_t DEVICE_OK = 0x00;
static const uint8_t DEVICE_ERROR = 0x01;
static uint8_t _dataResolution = 0;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */

static uint8_t _checkStatus();

/*******************************************************************************
 * Function name  : _checkStatus
 *
 * Description    : Checks if a specific interrupt or status flag (bit 4) is set 
 *                   in the accelerometer's `INT_SOURCE1` register.
 *
 * Parameters     : None
 *
 * Returns        : uint8_t
 *                   - `1` if bit 4 is set, indicating the event or condition has 
 *                     occurred.
 *                   - `0` if bit 4 is not set, indicating no event or condition.
 *
 * author         : 
 * date           : 20AUG2024
 ******************************************************************************/

static uint8_t _checkStatus()
{
    uint8_t status;
    
    status = accel7_readByte(_ACCEL7_REG_INT_SOURCE1);
    if((status & 0x10) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/*******************************************************************************
 * Function name  : accel7_i2cDriverInit
 *
 * Description    : Initializes the I2C driver for the accelerometer by 
 *                   configuring the I2C bus with specified parameters and 
 *                   creating an I2C master bus handle for communication.
 *
 * Parameters     : None
 *
 * Returns        : None
 *
 * author         : 
 * date           : 20AUG2024
 ******************************************************************************/

void accel7_i2cDriverInit()
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PortNum,
        .scl_io_num = I2C_SCL,
        .sda_io_num = I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
     // Create and initialize the I2C bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    // Configure I2C device
	i2c_device_config_t dev_cfg = {
    	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
    	.device_address = ADDR_KXTJ3_7BIT,
    	.scl_speed_hz = 100000,
	};

	
	// Add I2C device to the bus
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    
}

/*******************************************************************************
 * Function name  : accel7_writeByte
 *
 * Description    : Writes a byte of data to a specified register of the 
 *                   accelerometer via I2C. The function transmits the register 
 *                   address followed by the data byte using the I2C bus handle.
 *
 * Parameters     : 
 *                   uint8_t reg - The register address to write to.
 *                   uint8_t _data - The data byte to write to the register.
 *
 * Returns        : None
 *
 * author         : 
 * date           : 20AUG2024
 ******************************************************************************/

void accel7_writeByte(uint8_t reg, uint8_t _data)
{
    uint8_t writeReg[2];
    
    writeReg[0] = reg;  // Register address
    writeReg[1] = _data; // Data byte to write
    
    i2c_master_transmit(dev_handle, writeReg, 2, -1); // -1 indicates no timeout
}

/*******************************************************************************
 * Function name  : accel7_readByte
 *
 * Description    : Reads a byte of data from a specified register of the 
 *                   accelerometer via I2C. It transmits the register address 
 *                   and receives the data byte using the I2C master bus handle.
 *
 * Parameters     : uint8_t reg - The register address to read from.
 *
 * Returns        : uint8_t - The byte of data read from the register.
 *
 * author         : 
 * date           : 20AUG2024
 ******************************************************************************/

uint8_t accel7_readByte(uint8_t reg)
{
    uint8_t writeReg[1];
    uint8_t ReadReg[1];
    
    writeReg[0] = reg; // Register address to read from
    
    i2c_master_transmit_receive(dev_handle, writeReg, 1, ReadReg, 1, -1); // Perform the I2C read operation

    return ReadReg[0]; // Return the byte of data read from the register
}
/*******************************************************************************
 * Function name  : accel7_getAxis
 *
 * Description    : Reads and returns the acceleration data for a specified 
 *                   axis from the accelerometer. It waits until the sensor is 
 *                   ready, reads two bytes of data from the selected axis, 
 *                   combines them, and adjusts the result based on the data 
 *                   resolution setting (12-bit, 14-bit, or other).
 *
 * Parameters     : uint8_t _axis - Register address for the axis to read.
 *
 * Returns        : int16_t - The 16-bit signed acceleration value for the axis.
 *
 * author         : 
 * date           : 20AUG2024
 ******************************************************************************/

float accel7_getAxis(uint8_t _axis)
{
    uint8_t writeReg[ 1 ];
    
    int16_t AxisData;
    float Data;
    
    
    writeReg[ 0 ] = _axis;

    while(_checkStatus() != 0)
    {
        //Delay Function need to Add
    };

    AxisData = accel7_readByte(_axis + 1);
    AxisData = AxisData << 8;
    AxisData = AxisData | accel7_readByte( _axis );
        
    if(_dataResolution == _ACCEL7_DATA_RESP_12bit)
    {
        AxisData = (AxisData >> 4);
    }
    else if (_dataResolution == _ACCEL7_DATA_RESP_14bit)
    {
        AxisData = (AxisData >> 2);
    }
    else
    {
        AxisData = (AxisData >> 8);
    }
    
    // Convert AxisData to a floating-point value using sensitivity factor
    // Sensitivity factor is selected based on full-scale range and resolution
    int resolutionIndex;
    switch (_dataResolution)
    {
        case _ACCEL7_DATA_RESP_8bit:
            resolutionIndex = 0;
            break;
        case _ACCEL7_DATA_RESP_12bit:
            resolutionIndex = 1;
            break;
        case _ACCEL7_DATA_RESP_14bit:
            resolutionIndex = 2;
            break;
        default:
            // Handle unexpected resolution
            return 0.0f;
    }

    Data = (float)AxisData * sensitivityFactors[_fullScaleRange][resolutionIndex];

    return Data;
    

    return Data;
}

/*******************************************************************************
 * Function name  : accel7_init
 *
 * Description    : Initializes the accelerometer by configuring its data 
 *                   resolution and measurement range. It first verifies the 
 *                   device ID to ensure the correct sensor is connected. Based 
 *                   on the specified resolution and range, it sets the appropriate 
 *                   parameters in the accelerometer's control register.
 *
 * Parameters     : 
 *                   uint8_t dataRes - The desired data resolution (8, 12, or 14 bits).
 *                   uint8_t range - The measurement range (e.g., ±2g, ±4g, ±8g, ±16g).
 *
 * Returns        : uint8_t
 *                   - `DEVICE_OK` if initialization is successful.
 *                   - `DEVICE_ERROR` if the device ID check fails.
 *
 * author         : 
 * date           : 20AUG2024
 ******************************************************************************/

uint8_t accel7_init(uint8_t dataRes, uint8_t range)
{
    uint8_t I_AM;
    uint8_t setRes;
    uint8_t setRange;
        
    I_AM = accel7_readByte(_ACCEL7_REG_WHO_AM_I);
    
    if(I_AM != 0x35)
    {
        return DEVICE_ERROR;
    }
    
    /*Data resolution 8bit, 12bit and 14bit*/      
    /* Axis range +-2g, +-4g, +-8g and +-16g*/
    _dataResolution = dataRes;
    
    if(_dataResolution == _ACCEL7_DATA_RESP_12bit)
    {
        if(range == _ACCEL7_RANGE_8g)
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_8g;
        }
        else if (range == _ACCEL7_RANGE_16g)
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_16g;
        }
        else
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_4g;
        }
        setRes = _ACCEL7_CTRL_REG1_RES_HIGH_RESOLUTION;
    }
    else if(_dataResolution == _ACCEL7_DATA_RESP_14bit)
    {
        if (range == _ACCEL7_RANGE_16g)
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_16g;
        }
        else
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_8g;
        }
        setRes = _ACCEL7_CTRL_REG1_RES_HIGH_RESOLUTION;
    }
    else
    {
        if (range == _ACCEL7_RANGE_16g)
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_16g;
        }
        else if (range == _ACCEL7_RANGE_8g)
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_8g;
        }
        else if (range == _ACCEL7_RANGE_4g)
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_4g;
        }
        else
        {
            setRange = _ACCEL7_CTRL_REG1_RANGE_2g;
        }
        setRes = _ACCEL7_CTRL_REG1_RES_HIGH_RESOLUTION;
    }
    
    accel7_writeByte(_ACCEL7_REG_CTRL_REG1,
                     _ACCEL7_CTRL_REG1_MODE_OPERATING |
                     _ACCEL7_CTRL_REG1_DRDYE_ENABLE | 
                     setRes |
                     setRange);

    return DEVICE_OK;
}
/*******************************************************************************
 * Function name  : accel7_getInterruptState
 *
 * Description    : Reads the current state of a specified GPIO pin to determine 
 *                   if an interrupt has occurred. This function returns the 
 *                   logical level of the pin, which indicates whether the interrupt 
 *                   signal is high or low.
 *
 * Parameters     : 
 *                   uint8_t pin - The GPIO pin number to check.
 *
 * Returns        : uint8_t
 *                   - The current level of the specified GPIO pin (high or low).
 *
 * author         : 
 * date           : 20AUG2024
 ******************************************************************************/

uint8_t accel7_getInterruptState(uint8_t pin)
{
    return gpio_get_level(pin);
}


/*******************************************************************************
 * Function name  : Acc_data
 *
 * Description    : Acceleration_data function task
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 static void Accelerometer_Task (void *pvParameters)
 {
 	ESP_LOGI(pcTaskGetName(NULL), "Start");

	uint8_t I_AM;    
    float x, y, z;
    // Initialize KXTJ3-1057
    if (accel7_init(_ACCEL7_DATA_RESP_8bit, _ACCEL7_RANGE_16g) != 0) {
        ESP_LOGE(TAG, "KXTJ3-1057 initialization failed");
        return;
    }
	while(1) {
		
		I_AM = accel7_readByte(_ACCEL7_REG_WHO_AM_I);
		ESP_LOGI("APP", "Who Am I register value: 0x%02X",I_AM);
		// Read accelerometer data
    	x =  accel7_getAxis(_ACCEL7_AXIS_X);
   	 	y =  accel7_getAxis(_ACCEL7_AXIS_Y);
    	z =  accel7_getAxis(_ACCEL7_AXIS_Z);

    	ESP_LOGI(TAG, "Accelerometer readings: X=%f, Y=%f, Z=%f", x, y, z);
		
		vTaskDelay(pdMS_TO_TICKS(1000));
	} // end while 
 }

/*******************************************************************************
 * Function name  : create_Accelerometer_task
 *
 * Description    : function to create Acceleration_data tasks
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






