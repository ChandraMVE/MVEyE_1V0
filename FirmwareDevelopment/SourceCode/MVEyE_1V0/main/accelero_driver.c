//-----------------------------------------------------------------
///
///     \file acceler_driver.c
///
///     \brief lora application framework driver
///
///
///     \author       Naveen GS
///
///     Location:     India
///
///     Project Name: MVEyE_1V0
///
///     \date Created 09SEP2024
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
#include "accelero_Driver.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "driver/gpio.h"

//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
static const char *TAG = "accelero_driver";

#define I2C_SDA 					GPIO_NUM_0
#define I2C_SCL						GPIO_NUM_4

#define I2C_MASTER_SCL_IO           I2C_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           I2C_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000       /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0            /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0            /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define ACCELERO_SENSOR_ADDR        0x0F        /*!< Slave address of the accelero sensor */

//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
bool detectedInterrupt = false; 	// Create variable for detection

/***********************************************************************************
 * Function name  : setup_accelero_latched
 *
 * Description    : Initializing the accelerometer latched settings.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh
 * date           : 09SEP2024
 ***********************************************************************************/
void setup_accelero_latched()
{
	float sampleRate = 200;			// HZ - Samples per second - 0.781, 1.563, 3.125, 6.25,
	          						// 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
	          						// Sample rates ≥ 400Hz force High Resolution mode on
	uint8_t accelRange = 2; 		// 14-bit Mode only supports 8g or 16g
	bool highRes = true; 			// Set high resolution mode on
	int16_t threshold = 0; 			// Sets wake-up threshold to default
	uint8_t moveDur = 0; 			// Sets movement duration to default
	uint8_t naDur = 0; 				// Sets non-activity duration to default
	bool polarity = 1; 				// Sets INT pin polarity to active HIGH
	float wuRate = -1; 				// Sets wake-up sample rate to IMU sample rate
	bool latched = true; 			// Enables latched interrupt mode
	bool pulsed = false; 			// Disables pulsed interrupt mode
	bool motion = false; 			// Disables Motion Detection interrupt
	bool dataReady = true; 			// Enables New Data Ready interrupt
	bool intPin = true;				//false; // Disables INT pin operation


  vTaskDelay(1000 / portTICK_PERIOD_MS);

  if (begin(sampleRate, accelRange, highRes, 0) == IMU_SUCCESS)
  {
    printf("IMU initialized.\r\n");
  }
  else
  {
    printf("Failed to initialize IMU.\r\n");
    while(true); // stop running sketch if failed
  }

  uint8_t readData = 0;

  // Get the ID:
  if (readRegister_accelero(&readData, KXTJ3_WHO_AM_I) ==
  IMU_SUCCESS) {
  printf("Who am I? 0x%x\r\n", readData);
  } else {
    printf("Communication error, stopping.\r\n");
    while(true); // stop running sketch if failed
  }

  // Sets all Wake-Up parameters to defaults
  // Disables Wake-Up and INT pin functions
  // Enables Latched mode and Data Ready mode
  if (intConf(threshold, moveDur, naDur, polarity, wuRate,
  latched, pulsed, motion, dataReady, intPin) == IMU_SUCCESS)
  {
    printf("Latched interrupt configured.\r\n");
  } else {
    printf("Communication error, stopping.\r\n");
    while(true); // stop running sketch if failed
  }
}

/***********************************************************************************
 * Function name  : app_main_accelero_latched
 *
 * Description    : Whenever the data ready pin is true the readings will be printed.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
void app_main_accelero_latched(void)
{
	  //ESP_LOGI(TAG, "inside accelero latched app\r\n");
	  printf("******gpio level *********** = %d\r\n", gpio_get_level(GPIO_INPUT_IO_34));
	  // Check to see if new data is ready
	  if (dataReady())
	  {
	    // Since new data is ready, read it
	    printf(" Acceleration X, Y, Z = %f %f %f\r\n", axisAccel(X), axisAccel(Y), axisAccel(Z));
	    // Reading acceleration data resets the Data Ready latch
	    // Motion Detection requires manually resetting the latch
	    //ResetInterrupt();
	  }
	  vTaskDelay(100 / portTICK_PERIOD_MS);
}

#if 0
// Function to print user-friendly interrupt axis names
String printAxis(wu_axis_t axis)
{
  switch (axis)
  {
    case ZPOS:
      return "ZPOS";
      break;
    case ZNEG:
      return "ZNEG";
      break;
    case YPOS:
      return "YPOS";
      break;
    case YNEG:
      return "YNEG";
      break;
    case XPOS:
      return "XPOS";
      break;
    case XNEG:
      return "XNEG";
      break;
    default:
      return "NONE";
      break;
  }
}
#endif

/***********************************************************************************
 * Function name  : setup_accelero_unlatched
 *
 * Description    : Configuring the Accelerometer Reading.
 * Parameters     : pointer arguments
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
void setup_accelero_unlatched()
{

float sampleRate =
    6.25; 					// HZ - Samples per second - 0.781, 1.563, 3.125, 6.25,
          					// 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
          					// Sample rates ≥ 400Hz force High Resolution mode on
uint8_t accelRange = 2; 	// 14-bit Mode only supports 8g or 16g
bool highRes = true; 		// Set high resolution mode on
int16_t threshold = 128; 	// Sets wake-up threshold to 0.5g
uint8_t moveDur = 1; 		// Sets movement duration to 160ms
uint8_t naDur = 10; 		// Sets non-activity duration to 1.6s
bool polarity = 1; 			// Sets INT pin polarity to active HIGH(1); active low(0)

  if (begin(sampleRate, accelRange, highRes, 1) == IMU_SUCCESS)
  {
    printf("IMU initialized.\r\n");
  }
  else
  {
    printf("Failed to initialize IMU.\r\n");
    while(true); // stop running sketch if failed
  }
    
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  uint8_t readData = 0;

  // Get the ID:
  if (readRegister_accelero(&readData, KXTJ3_WHO_AM_I) ==
  IMU_SUCCESS) {
  printf("Who am I? 0x%x\r\n", readData);
  } else {
    printf("Communication error, stopping.\r\n");
    while(true); 			// stop running sketch if failed
  }

  // 128 / 256 = 0.5g change in acceleration threshold
  // 1 / 6.25 = 160ms movement duration
  // 10 / 6.25 = 1.6s non-activity reset
  // INT pin active HIGH
  // Wake-Up Sample Rate == IMU Sample Rate (6.25 Hz)
  
  float wuRate = -1;
  bool latched = false;
  bool pulsed = false;
  bool motion = true;
  bool dataReady = false;
  bool intPin = true;
                         
  
  if (intConf(threshold, moveDur, naDur, polarity,
  wuRate, latched, pulsed, motion, dataReady, intPin) ==
  IMU_SUCCESS) 
  {
    printf("Unlatched interrupt configured.\r\n");
  } else {
    printf("Communication error, stopping.\r\n");
    while(true); 		// stop running sketch if failed
  }

  // Disable the Z-axis for motion detection
  if (intDisableAxis(ZPOS, ZNEG, BLANK, BLANK, BLANK) == IMU_SUCCESS)
  {
    printf("Disabled Z-axis from interrupt.");
  } else {
    printf("Communication error, stopping.");
    while(true); 		// stop running sketch if failed
  }
}

/***********************************************************************************
 * Function name  : app_main_accelero_unlatched
 *
 * Description    : accelero_unlatched.
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
void app_main_accelero_unlatched(void)
{
  // This reads the state of Pin D1 to see if it's HIGH
  // If it is and we haven't already detected interrupt, print
  // Also asks which axis triggered the interrupt
  // If pin goes LOW we reset the sentinel value to try again
  printf("******gpio level *********** = %d\r\n", gpio_get_level(GPIO_INPUT_IO_34));
  printf("******* mot detected = %d\r\n", motionDetected());
  if (gpio_get_level(GPIO_INPUT_IO_34) == 1 && !detectedInterrupt)
  {
    printf(" ***********Interrupt fired!");
    printf(" Motion Direction: ");
    //printf(printAxis(motionDirection()));
    resetInterrupt(); 	// reset direction bit
    detectedInterrupt = true;
  }
  else if (gpio_get_level(GPIO_INPUT_IO_34) == 0 && detectedInterrupt)
  {
    detectedInterrupt = false;
  }
}

/***********************************************************************************
 * Function name  : i2c_accelero_register_read
 *
 * Description    : Accelerometer read register.
 * Parameters     : register address, data pointer, length
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
static esp_err_t i2c_accelero_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, ACCELERO_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/***********************************************************************************
 * Function name  : i2c_accelero_register_write_byte
 *
 * Description    : Accelerometer write register by byte.
 * Parameters     : register address, data.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
static esp_err_t i2c_accelero_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ACCELERO_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/***********************************************************************************
 * Function name  : i2c_master_init
 *
 * Description    : i2c_master_init.
 * Parameters     : None
 * Returns        : returnError
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

bool highRes   = false;
bool debugMode = false;
bool en14Bit   = false;
float accelSampleRate; 		// Sample Rate - 0.781, 1.563, 3.125, 6.25, 12.5, 25,
                       		// 50, 100, 200, 400, 800, 1600Hz
uint8_t accelRange;    		// Accelerometer range = 2, 4, 8, 16g

kxtj3_status_t begin(float sampleRate, uint8_t accRange, bool highResSet,
                            bool debugSet)
{
  kxtj3_status_t returnError = IMU_SUCCESS;
  accelSampleRate            = sampleRate;
  accelRange                 = accRange;
  highRes                    = highResSet;
  en14Bit                    = false;
  debugMode                  = debugSet;

  if (debugMode) {
    printf("Configuring IMU\r\n");
  }

  // Sample rates ≥ 400Hz force High Resolution mode on
  if (accelSampleRate > 200 && !highRes) {
    highRes = true;
  }

  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C initialized successfully");

  // Power-up time is up to 30ms according to DataSheet, so delay 50ms just to
  // play it safe
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Perform software reset to make sure IMU is in good state
  returnError = softwareReset();

  // Check previous returnError to see if we should stop
  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Check the ID register to determine if the operation was a success.
  uint8_t _whoAmI;

  readRegister_accelero(&_whoAmI, KXTJ3_WHO_AM_I);

  if (_whoAmI != 0x35) {
    return IMU_HW_ERROR;
  }

  // Check the self-test register to determine if the IMU is up.
  uint8_t _selfTest;

  readRegister_accelero(&_selfTest, KXTJ3_DCST_RESP);

  if (_selfTest != 0x55) {
    return IMU_HW_ERROR;
  }

  if (debugMode) {
    printf("Apply settings\r\n");
  }

  returnError = applySettings();

  return returnError;
}

/***********************************************************************************
 * Function name  : readRegister_acceleroRegion
 *
 * Description    : readRegister_acceleroRegion.
 * Parameters     : Output pointer, offset, length.
 * Returns        : returnError
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t readRegister_acceleroRegion(uint8_t *outputPointer, uint8_t offset,
                                         uint8_t length)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  ESP_ERROR_CHECK(i2c_accelero_register_read(offset, outputPointer, length));

  return returnError;
}

/***********************************************************************************
 * Function name  : readRegister_accelero
 *
 * Description    : readRegister_accelero.
 * Parameters     : Output pointer, offset.
 * Returns        : returnError
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t readRegister_accelero(uint8_t *outputPointer, uint8_t offset)
{
  // Return value
  kxtj3_status_t returnError = IMU_SUCCESS;

  ESP_ERROR_CHECK(i2c_accelero_register_read(offset, outputPointer, 1));

  if (debugMode) {
    printf("Read register 0x%x\r\n", offset);
    printf("Data = 0x%x\r\n", *outputPointer);
  }

  //*outputPointer = result;
  return returnError;
}

/***********************************************************************************
 * Function name  : readRegister_acceleroInt16
 *
 * Description    : reading hte register of type integer 16 bit.
 * Parameters     : Output pointer, offset.
 * Returns        : returnError
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t readRegister_acceleroInt16(int16_t *outputPointer, uint8_t offset)
{
	  kxtj3_status_t returnError = IMU_SUCCESS;
  // offset |= 0x80; //turn auto-increment bit on
  uint8_t myBuffer[2];
  ESP_ERROR_CHECK(i2c_accelero_register_read(offset, myBuffer, 2));
  int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);

  if (debugMode) {
    printf("16 bits from 0x%x\r\n", offset);
    printf(" = %d\r\n", output);
  }

  *outputPointer = output;
  return returnError;
}

/***********************************************************************************
 * Function name  : writeRegister
 *
 * Description    : Write Register.
 * Parameters     : Offset, dataToWrite.
 * Returns        : returnError
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t writeRegister(uint8_t offset, uint8_t dataToWrite)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  ESP_ERROR_CHECK(i2c_accelero_register_write_byte(offset, dataToWrite));

  return returnError;
}

/***********************************************************************************
 * Function name  : softwareReset
 *
 * Description    : softwareReset
 * Parameters     : None.
 * Returns        : returnError
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t softwareReset(void)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

#if 0
  // Start by copying the current I2C address to a temp variable
  // We must do this because the IMU could boot with a bit-flipped address
  uint8_t tempAddress        = I2CAddress;

  // Write 0x00 to KXTJ3_SOFT_REST to confirm IMU is on the bus at address
  Wire.beginTransmission(I2CAddress);
  Wire.write(KXTJ3_SOFT_REST);
  Wire.write(0x00);

  // If NACK returned, switch I2CAddress to flipped version and try again
  if (Wire.endTransmission() != 0) {
    if (I2CAddress == 0x0F) {
      I2CAddress = 0x0D;
    } else if (I2CAddress == 0x0E) {
      I2CAddress = 0x0C;
    }

    Wire.beginTransmission(I2CAddress);
    Wire.write(KXTJ3_SOFT_REST);
    Wire.write(0x00);

    // If still NACK, give up, need to power cycle IMU to recover
    if (Wire.endTransmission() != 0) {
      // Return I2CAddress to normal before returning
      if (I2CAddress != tempAddress) {
        I2CAddress = tempAddress;
      }
      return IMU_HW_ERROR;
    }
  }

  // Attempt to address CTRL_REG2 and end if NACK returned
  Wire.beginTransmission(I2CAddress);
  Wire.write(KXTJ3_CTRL_REG2);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    // Return I2CAddress to normal before returning
    if (I2CAddress != tempAddress) {
      I2CAddress = tempAddress;
    }
    return IMU_HW_ERROR;
  }

  // Send software reset command to CTRL_REG2 and end if NACK returned
  Wire.beginTransmission(I2CAddress);
  Wire.write(KXTJ3_CTRL_REG2);
  Wire.write(0x80);

  if (Wire.endTransmission() != 0) {
    // Return I2CAddress to normal before returning
    if (I2CAddress != tempAddress) {
      I2CAddress = tempAddress;
    }
    return IMU_HW_ERROR;
  }

  // Set I2CAddress back to normal since we've successfully reset the IMU
  if (I2CAddress != tempAddress) {
    I2CAddress = tempAddress;
  }

  // Delay for software start-up before returning (TN017 Table 1)
  vTaskDelay(1000 / portTICK_PERIOD_MS);2);


#endif
  return returnError;
}

/***********************************************************************************
 * Function name  : axisAccel
 *
 * Description    : Accelerometer reading based on the axis
 * Parameters     : _axis.
 * Returns        : outFloat
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
float axisAccel(axis_t _axis)
{
  int16_t outRAW;
  uint8_t regToRead = 0;
  switch (_axis) {
  case 0:
    // X axis
    regToRead = KXTJ3_XOUT_L;
    break;
  case 1:
    // Y axis
    regToRead = KXTJ3_YOUT_L;
    break;
  case 2:
    // Z axis
    regToRead = KXTJ3_ZOUT_L;
    break;

  default:
    // Not valid axis return NAN
    return NAN;
    break;
  }

  // Don't proceed if the read failed
  if (readRegister_acceleroInt16(&outRAW, regToRead) != IMU_SUCCESS) {
    return NAN;
  }

  // The LSB may contain garbage, so 0 any unused bits
  if (!highRes) {
    outRAW &= 0b1111111100000000; 		// 8-bit mode
  } else if (en14Bit) {
    outRAW &= 0b1111111111111100; 		// 14-bit mode
  } else {
    outRAW &= 0b1111111111110000; 		// 12-bit mode
  }

  float outFloat;

  switch (accelRange) {
  case 2:
    outFloat = (float)outRAW / 16384;
    break;
  case 4:
    outFloat = (float)outRAW / 8192;
    break;
  case 8:
    outFloat = (float)outRAW / 4096;
    break;
  case 16:
    outFloat = (float)outRAW / 2048;
    break;
  default:
    outFloat = 0;
    break;
  }

  return outFloat;
}

/***********************************************************************************
 * Function name  : standby
 *
 * Description    : standby.
 * Parameters     : _en.
 * Returns        : Success/Fail
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t standby(bool _en)
{
  kxtj3_status_t returnError = IMU_SUCCESS;
  uint8_t _ctrl;

  // "Backup" KXTJ3_CTRL_REG1
  returnError = readRegister_accelero(&_ctrl, KXTJ3_CTRL_REG1);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  if (_en)
    _ctrl &= 0x7E;
  else
    _ctrl |= (0x01 << 7); 		// disable standby-mode -> Bit7 = 1 = operating mode

  returnError = writeRegister(KXTJ3_CTRL_REG1, _ctrl);

  // If taking out of standby, follow start-up delay
  if (!_en && returnError == IMU_SUCCESS) {
    startupDelay();
  }

  return returnError;
}

/***********************************************************************************
 * Function name  : startupDelay
 *
 * Description    : startupDelay.
 * Parameters     : None.
 * Returns        : None.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
void startupDelay(void)
{
  if (highRes) {
    if (accelSampleRate < 1)
      vTaskDelay(1300 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 3)
      vTaskDelay(650 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 6)
      vTaskDelay(330 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 12)
      vTaskDelay(170 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 25)
      vTaskDelay(90 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 50)
      vTaskDelay(45 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 100)
      vTaskDelay(25 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 200)
      vTaskDelay(11 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 400)
      vTaskDelay(6 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 800)
      vTaskDelay(4 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 1600)
      vTaskDelay(3 / portTICK_PERIOD_MS);
    else
      vTaskDelay(2 / portTICK_PERIOD_MS);
  } else {
    if (accelSampleRate < 800 && accelSampleRate > 200)
      vTaskDelay(4 / portTICK_PERIOD_MS);
    else if (accelSampleRate < 1600 && accelSampleRate > 400)
      vTaskDelay(3 / portTICK_PERIOD_MS);
    else
      vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

/***********************************************************************************
 * Function name  : applySettings
 *
 * Description    : applySettings.
 * Parameters     : None.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t applySettings(void)
{
  kxtj3_status_t returnError = IMU_SUCCESS;
  uint8_t dataToWrite        = 0; 				// Temporary variable

  // Note that to properly change the value of this register, the PC1 bit in
  // CTRL_REG1 must first be set to “0”.
  returnError = standby(true);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Build DATA_CTRL_REG

  //  Convert ODR
  if (accelSampleRate < 1)
    dataToWrite |= 0x08; // 0.781Hz
  else if (accelSampleRate < 2)
    dataToWrite |= 0x09; // 1.563Hz
  else if (accelSampleRate < 4)
    dataToWrite |= 0x0A; // 3.125Hz
  else if (accelSampleRate < 8)
    dataToWrite |= 0x0B; // 6.25Hz
  else if (accelSampleRate < 16)
    dataToWrite |= 0x00; // 12.5Hz
  else if (accelSampleRate < 30)
    dataToWrite |= 0x01; // 25Hz
  else if (accelSampleRate < 60)
    dataToWrite |= 0x02; // 50Hz
  else if (accelSampleRate < 150)
    dataToWrite |= 0x03; // 100Hz
  else if (accelSampleRate < 250)
    dataToWrite |= 0x04; // 200Hz
  else if (accelSampleRate < 450)
    dataToWrite |= 0x05; // 400Hz
  else if (accelSampleRate < 850)
    dataToWrite |= 0x06; // 800Hz
  else
    dataToWrite |= 0x07; // 1600Hz

  // Now, write the patched together data
  if (debugMode) {
    printf("KXTJ3_DATA_CTRL_REG: 0x%x\r\n", dataToWrite);
  }
  returnError = writeRegister(KXTJ3_DATA_CTRL_REG, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Build CTRL_REG1

  // LOW power, 8-bit mode
  dataToWrite = 0x80;

  if (highRes) {
    if (debugMode) {
      printf("High Resolution set\r\n");
    }
    dataToWrite = 0xC0;
  }

  //  Convert scaling
  switch (accelRange) {
  default:
  case 2:
    dataToWrite |= (0x00 << 2);
    break;
  case 4:
    dataToWrite |= (0x02 << 2);
    break;
  case 8:
    dataToWrite |= (0x04 << 2);
    break;
  case 16:
    dataToWrite |= (0x01 << 2);
    break;
  }

  // Now, write the patched together data
  if (debugMode) {
    printf("KXTJ3_CTRL_REG1: 0x%x\r\n", dataToWrite);
  }
  returnError = writeRegister(KXTJ3_CTRL_REG1, dataToWrite);
  startupDelay();
  return returnError;
}

/***********************************************************************************
 * Function name  : enable14Bit
 *
 * Description    : enable14Bit.
 * Parameters     : accRange.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t enable14Bit(uint8_t accRange)
{
  kxtj3_status_t returnError = IMU_SUCCESS;
  uint8_t dataToWrite        = 0;        		// Temporary variable
  accelRange                 = accRange; 		// Set accelRange to new value
  highRes                    = true;     		// Make sure highRes is set to true
  en14Bit                    = true;     		// Set 14-bit check to true as well

  // Note that to properly change the value of this register, the PC1 bit in
  // CTRL_REG1 must first be set to “0”.
  returnError = standby(true);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  if (debugMode) {
    printf("Switching to 14-bit mode\r\n");
  }

  // Build CTRL_REG1

  switch (accelRange) {
  default:
  case 16:
    dataToWrite = 0xDC;
    break;
  case 8:
    dataToWrite = 0xD8;
    break;
  }

  // Write the new data to CTRL_REG1
  if (debugMode) {
    printf("KXTJ3_CTRL_REG1: 0x%x\r\n", dataToWrite);
  }
  returnError = writeRegister(KXTJ3_CTRL_REG1, dataToWrite);

  if (returnError == IMU_SUCCESS) {
    startupDelay();
  }

  return returnError;
}

/***********************************************************************************
 * Function name  : intConf
 *
 * Description    : intConf.
 * Parameters     : threshold,moveDur,naDur,polarity,wuRate,latched,pulsed,motion,dataReady
 *					intPin.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t intConf(int16_t threshold, uint8_t moveDur, uint8_t naDur,
                              bool polarity, float wuRate, bool latched,
                              bool pulsed, bool motion, bool dataReady,
                              bool intPin)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  // Note that to properly change the value of this register, the PC1 bit in
  // CTRL_REG1 must first be set to “0”.
  returnError = standby(true);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Build INT_CTRL_REG1

  uint8_t dataToWrite = 0x00; // Interrupt pin disabled, active LOW, latched

  // uint8_t dataToWrite = 0x20; // Interrupt enabled, active LOW, latched

  if (pulsed)
    dataToWrite |= (0x01 << 3); // Interrupt pin pulsed

  if (polarity == 1)
    dataToWrite |= (0x01 << 4); // Active HIGH

  if (intPin)
    dataToWrite |= (0x01 << 5); // Interrupt pin enabled

  if (debugMode) {
    printf("KXTJ3_INT_CTRL_REG1: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_INT_CTRL_REG1, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  uint8_t _reg1;

  // First "back up" current settings to a temporary variable
  returnError = readRegister_accelero(&_reg1, KXTJ3_CTRL_REG1);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  if (motion)
    _reg1 |= (0x01 << 1); // Sets WUFE to enabled

  if (dataReady)
    _reg1 |= (0x01 << 5); // Sets DRDY to enabled

  if (debugMode) {
    printf("KXTJ3_CTRL_REG1: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_CTRL_REG1, _reg1);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Set data rate for Wake-Up Function

  if (wuRate < 0) {
    // Sets the Data Rate for the Wake-Up (motion detect) function to match ODR
    // Start by checking DATA_CTRL_REG for the current ODR

    returnError = readRegister_accelero(&_reg1, KXTJ3_DATA_CTRL_REG);

    if (returnError != IMU_SUCCESS) {
      return returnError;
    }

    // Set ODRWU based on ODR
    // Maximum ODRWU is 100 Hz

    switch (_reg1) {
    case 0x09:
      dataToWrite = 0x01; // 1.563 Hz
      break;
    case 0x0A:
      dataToWrite = 0x02; // 3.125 Hz
      break;
    case 0x0B:
      dataToWrite = 0x03; // 6.25 Hz
      break;
    case 0x00:
      dataToWrite = 0x04; // 12.5 Hz
      break;
    case 0x01:
      dataToWrite = 0x05; // 25 Hz
      break;
    case 0x02:
      dataToWrite = 0x06; // 50 Hz
      break;
    case 0x03:
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
      dataToWrite = 0x07; // 100 Hz
      break;
    default:
      dataToWrite = 0x00; // 0.781 Hz
      break;
    }
  } else if (wuRate < 1)
    dataToWrite = 0x00; // 0x781 Hz
  else if (wuRate < 2)
    dataToWrite = 0x01; // 1.563 Hz
  else if (wuRate < 4)
    dataToWrite = 0x02; // 3.125 Hz
  else if (wuRate < 7)
    dataToWrite = 0x03; // 6.25 Hz
  else if (wuRate < 13)
    dataToWrite = 0x04; // 12.5 Hz
  else if (wuRate < 26)
    dataToWrite = 0x05; // 25 Hz
  else if (wuRate < 51)
    dataToWrite = 0x06; // 50 Hz
  else
    dataToWrite = 0x07; // 100 Hz

  if (debugMode) {
    printf("KXTJ3_CTRL_REG2: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_CTRL_REG2, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Build INT_CTRL_REG2

  dataToWrite = 0x3F; // enable interrupt on all axes any direction, latched

  if (!latched)
    dataToWrite |= (0x01 << 7); // enable unlatched mode

  if (debugMode) {
    printf("KXTJ3_INT_CTRL_REG2: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_INT_CTRL_REG2, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Set WAKE-UP (motion detect) Threshold

  dataToWrite = (uint8_t)(threshold >> 4);

  if (debugMode) {
    printf("KXTJ3_WAKEUP_THRESHOLD_H: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_WAKEUP_THRESHOLD_H, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  dataToWrite = (uint8_t)(threshold << 4);

  if (debugMode) {
    printf("KXTJ3_WAKEUP_THRESHOLD_L: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_WAKEUP_THRESHOLD_L, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // WAKEUP_COUNTER -> Sets the time motion must be present before a wake-up
  // interrupt is set WAKEUP_COUNTER (counts) = Wake-Up Delay Time (sec) x
  // Wake-Up Function ODR(Hz)

  dataToWrite = moveDur;

  if (debugMode) {
    printf("KXTJ3_WAKEUP_COUNTER: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_WAKEUP_COUNTER, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Non-Activity register sets the non-activity time required before another
  // wake-up interrupt will be reported. NA_COUNTER (counts) = Non-ActivityTime
  // (sec) x Wake-Up Function ODR(Hz)

  dataToWrite = naDur;

  if (debugMode) {
    printf("KXTJ3_NA_COUNTER: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_NA_COUNTER, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Set IMU to Operational mode
  returnError = standby(false);

  return returnError;
}

/***********************************************************************************
 * Function name  : intDisableAxis
 *
 * Description    : intDisableAxis.
 * Parameters     : first,second,third,fourth,fifth.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t intDisableAxis(wu_axis_t first, wu_axis_t second,
                                     wu_axis_t third, wu_axis_t fourth,
                                     wu_axis_t fifth)
{
  // Create temporary variables
  kxtj3_status_t returnError = IMU_SUCCESS;
  uint8_t temp               = 0x00;
  uint8_t dataToWrite        = 0b00111111;
  uint8_t bitCheck;

  // Note that to properly change the value of this register, the PC1 bit in
  // CTRL_REG1 must first be set to “0”.
  returnError = standby(true);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Check to see if ULMODE bit is set and set if so
  returnError = readRegister_accelero(&bitCheck, KXTJ3_INT_CTRL_REG2);
  if (returnError != IMU_SUCCESS)
    return returnError;
  if (bitCheck & (0x01 << 7))
    dataToWrite |= (0x01 << 7);

  if (first & NONE || second & NONE || third & NONE || fourth & NONE ||
      fifth & NONE) {
    // Rebuild INT_CTRL_REG2 with 0x00 using XOR to enable all axes
    dataToWrite ^= temp;
  } else {
    // combine the requested axes
    temp |= first;
    temp |= second;
    temp |= third;
    temp |= fourth;
    temp |= fifth;

    // Rebuild INT_CTRL_REG2 with new axis data using XOR
    dataToWrite ^= temp;
  }

  // Write the new values to INT_CTRL_REG2
  if (debugMode) {
    printf("KXTJ3_INT_CTRL_REG2: 0x%x\r\n", dataToWrite);
  }

  returnError = writeRegister(KXTJ3_INT_CTRL_REG2, dataToWrite);

  // Set IMU to Operational mode
  returnError = standby(false);

  return returnError;
}

#if 0
kxtj3_status_t intDisableAxis(wu_axis_t first)
{
  return intDisableAxis(first, BLANK, BLANK, BLANK, BLANK);
}

kxtj3_status_t intDisableAxis(wu_axis_t first, wu_axis_t second)
{
  return intDisableAxis(first, second, BLANK, BLANK, BLANK);
}

kxtj3_status_t intDisableAxis(wu_axis_t first, wu_axis_t second,
                                     wu_axis_t third)
{
  return intDisableAxis(first, second, third, BLANK, BLANK);
}

kxtj3_status_t intDisableAxis(wu_axis_t first, wu_axis_t second,
                                     wu_axis_t third, wu_axis_t fourth)
{
  return intDisableAxis(first, second, third, fourth, BLANK);
}
#endif

/***********************************************************************************
 * Function name  : dataReady
 *
 * Description    : dataReady.
 * Parameters     : None.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
bool dataReady(void)
{
  uint8_t _reg1;

  readRegister_accelero(&_reg1, KXTJ3_INT_SOURCE1);

  // Bit 4 is Data Ready Interrupt Bit
  if (_reg1 & (0x01 << 4)) {
    return true;
  } else {
    return false;
  }
}

/***********************************************************************************
 * Function name  : motionDetected
 *
 * Description    : motionDetected.
 * Parameters     : None.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
bool motionDetected(void)
{
  uint8_t _reg1;

  readRegister_accelero(&_reg1, KXTJ3_INT_SOURCE1);

  // Bit 1 is Wake-Up Function Sense Bit
  if (_reg1 & (0x01 << 1)) {
    return true;
  } else {
    return false;
  }
}

/***********************************************************************************
 * Function name  : motionDirection
 *
 * Description    : motionDirection.
 * Parameters     : None.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
wu_axis_t motionDirection(void)
{
  uint8_t _reg1;

  readRegister_accelero(&_reg1, KXTJ3_INT_SOURCE2);

  if (_reg1 & (0x01 << 0))
    return ZPOS;
  else if (_reg1 & (0x01 << 1))
    return ZNEG;
  else if (_reg1 & (0x01 << 2))
    return YPOS;
  else if (_reg1 & (0x01 << 3))
    return YNEG;
  else if (_reg1 & (0x01 << 4))
    return XPOS;
  else if (_reg1 & (0x01 << 5))
    return XNEG;
  else
    return NONE;
}

/***********************************************************************************
 * Function name  : resetInterrupt
 *
 * Description    : resetInterrupt.
 * Parameters     : None.
 * Returns        : Success/Fail.
 *
 * Known Issues   :
 * Note           :
 * author         : Naveen GS
 * date           : 09SEP2024
 ***********************************************************************************/
kxtj3_status_t resetInterrupt(void)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  uint8_t _reg1;

  // Reading the INT_REL register releases the latch
  returnError = readRegister_accelero(&_reg1, KXTJ3_INT_REL);

  return returnError;
}
	