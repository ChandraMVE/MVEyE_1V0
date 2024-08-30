//-----------------------------------------------------------------
///
///     \file accelerometer_KXTJ3.h
///
///     \brief accelerometer driver code support header
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

#ifndef MAIN_ACCELEROMETER_KXTJ3_H_
#define MAIN_ACCELEROMETER_KXTJ3_H_
#include "stdint.h"
#include "driver/i2c_master.h"
//==============================================================================
//          __             __   ___  __
//  | |\ | /  ` |    |  | |  \ |__  /__`
//  | | \| \__, |___ \__/ |__/ |___ .__/
//
//==============================================================================
extern i2c_master_dev_handle_t dev_handle;
//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================

#define PortNum     0
#define I2C_SDA 	GPIO_NUM_0
#define I2C_SCL		GPIO_NUM_4	
#define ACC_INTn	GPIO_NUM_34
#define ADDR_KXTJ3_7BIT	0x0F
                         
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
/* Registers */
extern const uint8_t _ACCEL7_REG_AXIS_X_LSB          ;
extern const uint8_t _ACCEL7_REG_AXIS_X_MSB          ;
extern const uint8_t _ACCEL7_REG_AXIS_Y_LSB          ;
extern const uint8_t _ACCEL7_REG_AXIS_Y_MSB          ;
extern const uint8_t _ACCEL7_REG_AXIS_Z_LSB          ;
extern const uint8_t _ACCEL7_REG_AXIS_Z_MSB          ;
extern const uint8_t _ACCEL7_REG_DCST_RESP           ;
extern const uint8_t _ACCEL7_REG_WHO_AM_I            ;
extern const uint8_t _ACCEL7_REG_INT_SOURCE1         ;
extern const uint8_t _ACCEL7_REG_INT_SOURCE2         ;
extern const uint8_t _ACCEL7_REG_STATUS              ;
extern const uint8_t _ACCEL7_REG_INT_REL             ;
extern const uint8_t _ACCEL7_REG_CTRL_REG1           ;
extern const uint8_t _ACCEL7_REG_CTRL_REG2           ;
extern const uint8_t _ACCEL7_REG_INT_CTRL_REG1       ;
extern const uint8_t _ACCEL7_REG_INT_CTRL_REG2       ;
extern const uint8_t _ACCEL7_REG_DATA_CTRL_REG       ;
extern const uint8_t _ACCEL7_REG_WAKEUP_COUNTER      ;
extern const uint8_t _ACCEL7_REG_NA_CAUNTER          ;
extern const uint8_t _ACCEL7_REG_SELF_TEST           ;
extern const uint8_t _ACCEL7_REG_WAKEUP_THRESHOLD_MSB;
extern const uint8_t _ACCEL7_REG_WAKEUP_THRESHOLD_LSB;

/* Control Regiter 1 */
extern const uint8_t _ACCEL7_CTRL_REG1_MODE_OPERATING     ;
extern const uint8_t _ACCEL7_CTRL_REG1_MODE_STANDBY       ;
extern const uint8_t _ACCEL7_CTRL_REG1_RES_LOW_CURRENT    ;
extern const uint8_t _ACCEL7_CTRL_REG1_RES_HIGH_RESOLUTION;
extern const uint8_t _ACCEL7_CTRL_REG1_DRDYE_ENABLE       ;
extern const uint8_t _ACCEL7_CTRL_REG1_DRDYE_DISABLE      ;
extern const uint8_t _ACCEL7_CTRL_REG1_RANGE_2g           ;
extern const uint8_t _ACCEL7_CTRL_REG1_RANGE_4g           ;
extern const uint8_t _ACCEL7_CTRL_REG1_RANGE_8g           ;
extern const uint8_t _ACCEL7_CTRL_REG1_RANGE_16g          ;
extern const uint8_t _ACCEL7_CTRL_REG1_RANGE_HIGH_RES_8g  ;
extern const uint8_t _ACCEL7_CTRL_REG1_RANGE_HIGH_RES_16g ;
extern const uint8_t _ACCEL7_CTRL_REG1_WAKEUP_DISABLE     ;
extern const uint8_t _ACCEL7_CTRL_REG1_WAKEUP_ENABLE      ;

/* Control Register 2 */
extern const uint8_t _ACCEL7_CTRL_REG2_START_RAM_REBOOT     ;
extern const uint8_t _ACCEL7_CTRL_REG2_DCST_ENABLE          ;
extern const uint8_t _ACCEL7_CTRL_REG2_DCST_DISABLE         ;
extern const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_0_781Hz;
extern const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_1_563Hz;
extern const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_3_125Hz;
extern const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_6_25Hz ;
extern const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_12_5Hz ;
extern const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_25Hz   ;
extern const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_50Hz   ;
extern const uint8_t _ACCEL7_CTRL_REG2_OUT_DATA_RATE_100Hz  ;

/* Interrupt control register 1 */
extern const uint8_t _ACCEL7_INT_CTRL_REG1_IEN_DISABLE;
extern const uint8_t _ACCEL7_INT_CTRL_REG1_IEN_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG1_IEA_DISABLE;
extern const uint8_t _ACCEL7_INT_CTRL_REG1_IEA_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG1_IEL_DISABLE;
extern const uint8_t _ACCEL7_INT_CTRL_REG1_IEL_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG1_SELF_TEST_POL_NEGATIVE;
extern const uint8_t _ACCEL7_INT_CTRL_REG1_SELF_TEST_POL_POSITIVE;

/* Interrupt control register 2 */
extern const uint8_t _ACCEL7_INT_CTRL_REG2_ULMODE_ENABLE     ;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_ULMODE_DISABLE    ;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_X_NEGATIVE_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_X_NEGATIVE_DISABLE;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_Y_NEGATIVE_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_Y_NEGATIVE_DISABLE;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_Z_NEGATIVE_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_Z_NEGATIVE_DISABLE;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_X_POSITIVE_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_X_POSITIVE_DISABLE;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_Y_POSITIVE_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_Y_POSITIVE_DISABLE;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_Z_POSITIVE_ENABLE ;
extern const uint8_t _ACCEL7_INT_CTRL_REG2_Z_POSITIVE_DISABLE;

/* Data contorl register */
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_0_781Hz;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_1_563Hz;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_3_125Hz;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_6_25Hz ;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_12_5Hz ;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_25Hz   ;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_50Hz   ;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_100Hz  ;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_200Hz  ;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_400Hz  ;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_800Hz  ;
extern const uint8_t _ACCEL7_DATA_CTRL_REG_ODR_1600Hz ;

/* Accelerometer Axis*/
extern const uint8_t _ACCEL7_AXIS_X;
extern const uint8_t _ACCEL7_AXIS_Y;
extern const uint8_t _ACCEL7_AXIS_Z;

/* Resolution*/
extern const uint8_t _ACCEL7_DATA_RESP_8bit;
extern const uint8_t _ACCEL7_DATA_RESP_12bit;
extern const uint8_t _ACCEL7_DATA_RESP_14bit;

/* g-Rates */
extern const uint8_t _ACCEL7_RANGE_2g;
extern const uint8_t _ACCEL7_RANGE_4g;
extern const uint8_t _ACCEL7_RANGE_8g;
extern const uint8_t _ACCEL7_RANGE_16g;



//==============================================================================
//   __        __          __      ___            __  ___    __        __
//  |__) |  | |__) |    | /  `    |__  |  | |\ | /  `  |  | /  \ |\ | /__`
//  |    \__/ |__) |___ | \__,    |    \__/ | \| \__,  |  | \__/ | \| .__/
//
//==============================================================================
/**
 * @brief Initializes the I2C driver for the accelerometer.
 *
 * @details Configures the I2C bus and creates a master bus handle.
 *
 * @param[in] None
 *
 * @return None
 */
void accel7_i2cDriverInit(void);                     

/**
 * @brief Functions for write one byte in register
 *
 * @param[in] reg    Register in which the data will be written
 * @param[in] _data  Data which be written in the register
 */
void accel7_writeByte(uint8_t reg, uint8_t _data);

/**
 * @brief Functions for read byte from register
 *
 * @param[in] reg    Register which will be read
 * @retval one byte data which is read from the register
 */
uint8_t accel7_readByte(uint8_t reg);

/**
 * @brief Functions for read axis data
 *
 * @param[ in ] axis    Axis data which will be read
 * @retval Axis data
 */
float accel7_getAxis(uint8_t _axis);

/**
 * @brief Functions for initialize the chip
 *
 * @param[ in ] dataRes      Data resolution (8 bit, 12bit or 14bit)
 * @param[ in ] range        Accelerometer g-range (+-2g, +-4g, +-8g, +-16g)
 *
 * @retval Information whether the chip is successfully initialized or not.
 *
 * Functions initializes accelerometer g-range, Data resolution, 
   operating mode and enable the reporting of the availability of new acceleration data as an interrupt.
 */
uint8_t accel7_init(uint8_t dataRes, uint8_t range);

/**
 * @brief Functions for read INT pin state
 *
 * @retval Interrupt state
 */
uint8_t accel7_getInterruptState(uint8_t pin);


#endif /* MAIN_ACCELEROMETER_KXTJ3_H_ */
