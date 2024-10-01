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
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "arch/sys_arch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"
#include "lora_llc68.h"

//==============================================================================
//   __   ___  ___         ___  __
//  |  \ |__  |__  | |\ | |__  /__`
//  |__/ |___ |    | | \| |___ .__/
//
//==============================================================================
#define TAG "LORA_LLCC68"
//==============================================================================
//   __        __   __                          __   __
//  / _` |    /  \ |__)  /\  |       \  /  /\  |__) /__`
//  \__> |___ \__/ |__) /~~\ |___     \/  /~~\ |  \ .__/
//
//==============================================================================
static uint8_t PacketParams[6];
static bool    txActive;
static int     txLost = 0;
static bool    debugPrint;

//==============================================================================
//   __  ___      ___    __                __   __
//  /__`  |   /\   |  | /  `    \  /  /\  |__) /__`
//  .__/  |  /~~\  |  | \__,     \/  /~~\ |  \ .__/
//
//==============================================================================
/*define of lora spi bus*/
static const int SPI_Frequency = 2000000;
static spi_device_handle_t lora_spi;

/*******************************************************************************
 * Function name  : LoRaErrorDefault
 *
 * Description    : function to Lora error default wait
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void LoRaErrorDefault(int error)
{
	if (debugPrint) {
		ESP_LOGE(TAG, "LoRaErrorDefault=%d", error);
	}
	while (true) {
		vTaskDelay(1);
	}
}

__attribute__ ((weak, alias ("LoRaErrorDefault"))) void LoRaError(int error);

/*******************************************************************************
 * Function name  : LoRaInit
 *
 * Description    : function to Lora initialize all io's & chip
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void LoRaInit(void)
{
#if 0
	ESP_LOGI(TAG, "CONFIG_MISO_GPIO=%d", CONFIG_MISO_GPIO);
	ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d", CONFIG_MOSI_GPIO);
	ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d", CONFIG_SCLK_GPIO);
	ESP_LOGI(TAG, "CONFIG_NSS_GPIO=%d", CONFIG_NSS_GPIO);
	ESP_LOGI(TAG, "CONFIG_RST_GPIO=%d", CONFIG_RST_GPIO);
	ESP_LOGI(TAG, "CONFIG_BUSY_GPIO=%d", CONFIG_BUSY_GPIO);
	ESP_LOGI(TAG, "CONFIG_TXEN_GPIO=%d", CONFIG_TXEN_GPIO);
	ESP_LOGI(TAG, "CONFIG_RXEN_GPIO=%d", CONFIG_RXEN_GPIO);
	ESP_LOGI(TAG, "LORA_DIO1=%d",LORA_DIO1);
	ESP_LOGI(TAG, "LORA_DIO2=%d",LORA_DIO2);
	ESP_LOGI(TAG, "LORA_DIO3=%d",LORA_DIO3);
#endif

	txActive = false;
	debugPrint = false;

	gpio_set_direction(CONFIG_NSS_GPIO,GPIO_MODE_OUTPUT);
	gpio_set_direction(CONFIG_RST_GPIO,GPIO_MODE_OUTPUT);

	gpio_set_direction(CONFIG_TXEN_GPIO,GPIO_MODE_OUTPUT);
	gpio_set_direction(CONFIG_RXEN_GPIO,GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(CONFIG_TXEN_GPIO, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(CONFIG_RXEN_GPIO, GPIO_PULLUP_ONLY);

	gpio_set_direction(CONFIG_BUSY_GPIO,GPIO_MODE_INPUT);
	gpio_set_pull_mode(CONFIG_BUSY_GPIO, GPIO_PULLUP_ONLY);
	
	gpio_set_direction(LORA_DIO1,GPIO_MODE_INPUT);
	gpio_set_direction(LORA_DIO2,GPIO_MODE_INPUT);
	gpio_set_direction(LORA_DIO3,GPIO_MODE_INPUT);
	
	spi_bus_config_t spi_bus_config = {
		.sclk_io_num = CONFIG_SCLK_GPIO,
		.mosi_io_num = CONFIG_MOSI_GPIO,
		.miso_io_num = CONFIG_MISO_GPIO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	esp_err_t ret;
	ret = spi_bus_initialize( VSPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO );
	ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t lora_devcfg;
	memset( &lora_devcfg, 0, sizeof( spi_device_interface_config_t ) );
	lora_devcfg.clock_speed_hz = SPI_Frequency;
	// It does not work with hardware CS control.
	//devcfg.spics_io_num = LLCC68_SPI_SELECT;
	// It does work with software CS control.
	lora_devcfg.spics_io_num = -1;
	lora_devcfg.queue_size = 7;
	lora_devcfg.mode = 0;
	lora_devcfg.flags = SPI_DEVICE_NO_DUMMY;

	//spi_device_handle_t handle;
	ret = spi_bus_add_device( VSPI_HOST, &lora_devcfg, &lora_spi);
	ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
}

/*******************************************************************************
 * Function name  : spi_write_byte
 *
 * Description    : function to write to lora spi register
 * Parameters     : Data pointer , Data length
 * Returns        : success/fail
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
bool spi_write_byte(uint8_t* Dataout, size_t DataLength )
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Dataout;
		SPITransaction.rx_buffer = NULL;
		spi_device_transmit( lora_spi, &SPITransaction );
	}

	return true;
}

/*******************************************************************************
 * Function name  : spi_read_byte
 *
 * Description    : function to write to lora spi register
 * Parameters     : Data input, Data pointer , Data length
 * Returns        : success/fail
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
bool spi_read_byte(uint8_t* Datain, uint8_t* Dataout, size_t DataLength )
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Dataout;
		SPITransaction.rx_buffer = Datain;
		spi_device_transmit( lora_spi, &SPITransaction );
	}

	return true;
}

/*******************************************************************************
 * Function name  : spi_transfer
 *
 * Description    : function to transmit the address
 * Parameters     : address
 * Returns        : datain
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
uint8_t spi_transfer(uint8_t address)
{
	uint8_t datain[1];
	uint8_t dataout[1];
	dataout[0] = address;
	spi_read_byte(datain, dataout, 1 );
	return datain[0];
}

/*******************************************************************************
 * Function name  : LoRaBegin
 *
 * Description    : function to Lora transmit begin
 * Parameters     : uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage, 
 					bool useRegulatorLDO
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 int16_t LoRaBegin(uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage, bool useRegulatorLDO) 
{
	if ( txPowerInDbm > 22 )
		txPowerInDbm = 22;
	if ( txPowerInDbm < -3 )
		txPowerInDbm = -3;
	
	Reset();
	ESP_LOGI(TAG, "Reset");
	
	uint8_t wk[2];
	ReadRegister(LLCC68_REG_LORA_SYNC_WORD_MSB, wk, 2); // 0x0740
	uint16_t syncWord = (wk[0] << 8) + wk[1];
	ESP_LOGI(TAG, "syncWord=0x%x", syncWord);
	if (syncWord != LLCC68_SYNC_WORD_PUBLIC && syncWord != LLCC68_SYNC_WORD_PRIVATE) {
		ESP_LOGE(TAG, "LLCC68 error, maybe no SPI connection");
		return ERR_INVALID_MODE;
	}

	ESP_LOGI(TAG, "LLCC68 installed");
	SetStandby(LLCC68_STANDBY_RC);
	SetDio2AsRfSwitchCtrl(true);
	ESP_LOGI(TAG, "tcxoVoltage=%f", tcxoVoltage);
	// set TCXO control, if requested
	if(tcxoVoltage > 0.0) {
		SetDio3AsTcxoCtrl(tcxoVoltage, RADIO_TCXO_SETUP_TIME); // Configure the radio to use a TCXO controlled by DIO3
	}

	Calibrate(	LLCC68_CALIBRATE_IMAGE_ON
				| LLCC68_CALIBRATE_ADC_BULK_P_ON
				| LLCC68_CALIBRATE_ADC_BULK_N_ON
				| LLCC68_CALIBRATE_ADC_PULSE_ON
				| LLCC68_CALIBRATE_PLL_ON
				| LLCC68_CALIBRATE_RC13M_ON
				| LLCC68_CALIBRATE_RC64K_ON
				);

	ESP_LOGI(TAG, "useRegulatorLDO=%d", useRegulatorLDO);
	if (useRegulatorLDO) {
		SetRegulatorMode(LLCC68_REGULATOR_LDO); // set regulator mode: LDO
	} else {
		SetRegulatorMode(LLCC68_REGULATOR_DC_DC); // set regulator mode: DC-DC
	}

	SetBufferBaseAddress(0, 0);
	SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
	SetOvercurrentProtection(60.0);  // current max 60mA for the whole device
	SetPowerConfig(txPowerInDbm, LLCC68_PA_RAMP_200U); //0 fuer Empfaenger
	SetRfFrequency(frequencyInHz);
	return ERR_NONE;
}

/*******************************************************************************
 * Function name  : FixInvertedIQ
 *
 * Description    : function to fixes IQ configuration for inverted IQ
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void FixInvertedIQ(uint8_t iqConfig)
{
	// fixes IQ configuration for inverted IQ
	// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details
	// When exchanging LoRa packets with inverted IQ polarity, some packet losses may be observed for longer packets.
	// Workaround: Bit 2 at address 0x0736 must be set to:
	// “0” when using inverted IQ polarity (see the SetPacketParam(...) command)
	// “1” when using standard IQ polarity

	// read current IQ configuration
	uint8_t iqConfigCurrent = 0;
	ReadRegister(LLCC68_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736

	// set correct IQ configuration
	//if(iqConfig == LLCC68_LORA_IQ_STANDARD) {
	if(iqConfig == LLCC68_LORA_IQ_INVERTED) {
		iqConfigCurrent &= 0xFB; // using inverted IQ polarity
	} else {
		iqConfigCurrent |= 0x04; // using standard IQ polarity
	}

	// update with the new value
	WriteRegister(LLCC68_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736
}

/*******************************************************************************
 * Function name  : LoRaConfig
 *
 * Description    : function to configure the Lora chip
 * Parameters     : spreadingFactor, bandwidth, codingRate, preambleLength, 
 					payloadLen, crcOn, invertIrq
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq) 
{
	SetStopRxTimerOnPreambleDetect(false);
	SetLoRaSymbNumTimeout(0); 
	SetPacketType(LLCC68_PACKET_TYPE_LORA); // LLCC68.ModulationParams.PacketType : MODEM_LORA
	uint8_t ldro = LLCC68_LORA_LOW_DATA_RATE_OPTIMIZE_OFF; // LowDataRateOptimize OFF
	SetModulationParams(spreadingFactor, bandwidth, codingRate, ldro);
	
	PacketParams[0] = (preambleLength >> 8) & 0xFF;
	PacketParams[1] = preambleLength;
	if ( payloadLen )
	{
		PacketParams[2] = 0x01; // Fixed length packet (implicit header)
		PacketParams[3] = payloadLen;
	}
	else
	{
		PacketParams[2] = 0x00; // Variable length packet (explicit header)
		PacketParams[3] = 0xFF;
	}

	if ( crcOn )
		PacketParams[4] = LLCC68_LORA_IQ_INVERTED;
	else
		PacketParams[4] = LLCC68_LORA_IQ_STANDARD;

	if ( invertIrq )
		PacketParams[5] = 0x01; // Inverted LoRa I and Q signals setup
	else
		PacketParams[5] = 0x00; // Standard LoRa I and Q signals setup

	// fixes IQ configuration for inverted IQ
	FixInvertedIQ(PacketParams[5]);

	WriteCommand(LLCC68_CMD_SET_PACKET_PARAMS, PacketParams, 6); // 0x8C

	// Do not use DIO interruptst
	/*SetDioIrqParams(LLCC68_IRQ_ALL,   //all interrupts enabled
					LLCC68_IRQ_NONE,  //interrupts on DIO1
					LLCC68_IRQ_NONE,  //interrupts on DIO2
					LLCC68_IRQ_NONE); //interrupts on DIO3

	// Receive state no receive timeoout
	SetRx(0xFFFFFF);*/
}

/*******************************************************************************
 * Function name  : LoRaDebugPrint
 *
 * Description    : function toenable Lora Debug Prints
 * Parameters     : enable/Disable
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void LoRaDebugPrint(bool enable) 
{
	debugPrint = enable;
}

/*******************************************************************************
 * Function name  : LoRaReceive
 *
 * Description    : function to receive the Lora data
 * Parameters     : enable/Disable
 * Returns        : datalength
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 uint8_t LoRaReceive(uint8_t *pData, int16_t len) 
{
	uint8_t rxLen = 0;
	uint16_t irqRegs = GetIrqStatus();
	//uint8_t status = GetStatus();
	
	if( irqRegs & LLCC68_IRQ_RX_DONE )
	{
		//ClearIrqStatus(LLCC68_IRQ_RX_DONE);
		ESP_LOGI(TAG, "irqStatus=0x%x", irqRegs);
		ClearIrqStatus(LLCC68_IRQ_ALL);
		rxLen = ReadBuffer(pData, len);
	}
	
	return rxLen;
}

/*******************************************************************************
 * Function name  : LoRaSend
 *
 * Description    : function to transmit the Lora data
 * Parameters     : enable/Disable
 * Returns        : datalength
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
bool LoRaSend(uint8_t *pData, int16_t len, uint8_t mode)
{
	uint16_t irqStatus;
	bool rv = false;
	
	if ( txActive == false )
	{
		txActive = true;
		PacketParams[2] = 0x00; //Variable length packet (explicit header)
		PacketParams[3] = len;
		WriteCommand(LLCC68_CMD_SET_PACKET_PARAMS, PacketParams, 6); // 0x8C
		
		//ClearIrqStatus(LLCC68_IRQ_TX_DONE | LLCC68_IRQ_TIMEOUT);
		ClearIrqStatus(LLCC68_IRQ_ALL);
		
		WriteBuffer(pData, len);
		SetTx(500);

		if ( mode & LLCC68_TXMODE_SYNC )
		{
			irqStatus = GetIrqStatus();
			while ( (!(irqStatus & LLCC68_IRQ_TX_DONE)) && (!(irqStatus & LLCC68_IRQ_TIMEOUT)) )
			{
				sys_delay_ms(1);
				irqStatus = GetIrqStatus();
			}
			if (debugPrint) {
				ESP_LOGI(TAG, "irqStatus=0x%x", irqStatus);
				if (irqStatus & LLCC68_IRQ_TX_DONE) {
					ESP_LOGI(TAG, "LLCC68_IRQ_TX_DONE");
				}
				if (irqStatus & LLCC68_IRQ_TIMEOUT) {
					ESP_LOGI(TAG, "LLCC68_IRQ_TIMEOUT");
				}
			}
			txActive = false;
	
			SetRx(0xFFFFFF);
	
			if ( irqStatus & LLCC68_IRQ_TX_DONE) {
				rv = true;
			}
		}
		else
		{
			rv = true;
		}
	}
	if (debugPrint) {
		ESP_LOGI(TAG, "Send rv=0x%x", rv);
	}
	if (rv == false) txLost++;
	return rv;
}

/*******************************************************************************
 * Function name  : ReceiveMode
 *
 * Description    : function to check for Receivemode status
 * Parameters     : enable/Disable
 * Returns        : true/false
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
bool ReceiveMode(void)
{
	uint16_t irq;
	bool rv = false;

	if ( txActive == false )
	{
		rv = true;
	}
	else
	{
		irq = GetIrqStatus();
		if ( irq & (LLCC68_IRQ_TX_DONE | LLCC68_IRQ_TIMEOUT) )
		{ 
			SetRx(0xFFFFFF);
			txActive = false;
			rv = true;
		}
	}

	return rv;
}

/*******************************************************************************
 * Function name  : GetPacketStatus
 *
 * Description    : function to check for packet status
 * Parameters     : *rssiPacket, *snrPacket
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void GetPacketStatus(int8_t *rssiPacket, int8_t *snrPacket)
{
	uint8_t buf[4];
	ReadCommand( LLCC68_CMD_GET_PACKET_STATUS, buf, 4 ); // 0x14
	*rssiPacket = (buf[3] >> 1) * -1;
	( buf[2] < 128 ) ? ( *snrPacket = buf[2] >> 2 ) : ( *snrPacket = ( ( buf[2] - 256 ) >> 2 ) );
}


/*******************************************************************************
 * Function name  : GetPacketStatus
 *
 * Description    : function to set the transmit power
 * Parameters     : Transmit power in DBm
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void SetTxPower(int8_t txPowerInDbm)
{
	SetPowerConfig(txPowerInDbm, LLCC68_PA_RAMP_200U);
}

/*******************************************************************************
 * Function name  : Reset
 *
 * Description    : function to Reset the Lora chip
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void Reset(void)
{
	sys_delay_ms(10);
	gpio_set_level(LLCC68_RESET, LOW);
	sys_delay_ms(20);
	gpio_set_level(LLCC68_RESET,HIGH);
	sys_delay_ms(10);
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);
}

/*******************************************************************************
 * Function name  : Wakeup
 *
 * Description    : function to wakeup Lora
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void Wakeup(void)
{
	GetStatus();
}

/*******************************************************************************
 * Function name  : SetStandby
 *
 * Description    : function to Set Lora to standby mode
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetStandby(uint8_t mode)
{
	uint8_t data = mode;
	WriteCommand(LLCC68_CMD_SET_STANDBY, &data, 1); // 0x80
}

/*******************************************************************************
 * Function name  : GetStatus
 *
 * Description    : function to get Lora chip Status
 * Parameters     : None
 * Returns        : idle/sleep
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
uint8_t GetStatus(void)
{
	uint8_t ret;
	ReadCommand(LLCC68_CMD_GET_STATUS, &ret, 1); // 0xC0
	return ret;
}

/*******************************************************************************
 * Function name  : SetDio3AsTcxoCtrl
 *
 * Description    : function to set DIO3 io as tcxo control
 * Parameters     : voltage, delay
 * Returns        : idle/sleep
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetDio3AsTcxoCtrl(float voltage, uint32_t delay)
{
	uint8_t buf[4];

	//buf[0] = tcxoVoltage & 0x07;
	if(fabs(voltage - 1.6) <= 0.001) {
		buf[0] = LLCC68_DIO3_OUTPUT_1_6;
	} else if(fabs(voltage - 1.7) <= 0.001) {
		buf[0] = LLCC68_DIO3_OUTPUT_1_7;
	} else if(fabs(voltage - 1.8) <= 0.001) {
		buf[0] = LLCC68_DIO3_OUTPUT_1_8;
	} else if(fabs(voltage - 2.2) <= 0.001) {
		buf[0] = LLCC68_DIO3_OUTPUT_2_2;
	} else if(fabs(voltage - 2.4) <= 0.001) {
		buf[0] = LLCC68_DIO3_OUTPUT_2_4;
	} else if(fabs(voltage - 2.7) <= 0.001) {
		buf[0] = LLCC68_DIO3_OUTPUT_2_7;
	} else if(fabs(voltage - 3.0) <= 0.001) {
		buf[0] = LLCC68_DIO3_OUTPUT_3_0;
	} else {
		buf[0] = LLCC68_DIO3_OUTPUT_3_3;
	}

	uint32_t delayValue = (float)delay / 15.625;
	buf[1] = ( uint8_t )( ( delayValue >> 16 ) & 0xFF );
	buf[2] = ( uint8_t )( ( delayValue >> 8 ) & 0xFF );
	buf[3] = ( uint8_t )( delayValue & 0xFF );

	WriteCommand(LLCC68_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4); // 0x97
}

/*******************************************************************************
 * Function name  : Calibrate
 *
 * Description    : function to calibrate the Lora chip
 * Parameters     : voltage, delay
 * Returns        : idle/sleep
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void Calibrate(uint8_t calibParam)
{
	uint8_t data = calibParam;
	WriteCommand(LLCC68_CMD_CALIBRATE, &data, 1); // 0x89
}

/*******************************************************************************
 * Function name  : SetDio2AsRfSwitchCtrl
 *
 * Description    : function to set DIO2 as RF switch Control
 * Parameters     : enable/disable
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetDio2AsRfSwitchCtrl(uint8_t enable)
{
	uint8_t data = enable;
	WriteCommand(LLCC68_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1); // 0x9D
}

/*******************************************************************************
 * Function name  : SetRfFrequency
 *
 * Description    : function to set RF Frequency
 * Parameters     : Frequency
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetRfFrequency(uint32_t frequency)
{
	uint8_t buf[4];
	uint32_t freq = 0;

	CalibrateImage(frequency);

	freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
	buf[0] = (uint8_t)((freq >> 24) & 0xFF);
	buf[1] = (uint8_t)((freq >> 16) & 0xFF);
	buf[2] = (uint8_t)((freq >> 8) & 0xFF);
	buf[3] = (uint8_t)(freq & 0xFF);
	WriteCommand(LLCC68_CMD_SET_RF_FREQUENCY, buf, 4); // 0x86
}

/*******************************************************************************
 * Function name  : CalibrateImage
 *
 * Description    : function to calibrate the image
 * Parameters     : Frequency
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void CalibrateImage(uint32_t frequency)
{
	uint8_t calFreq[2];

	if( frequency> 900000000 )
	{
			calFreq[0] = 0xE1;
			calFreq[1] = 0xE9;
	}
	else if( frequency > 850000000 )
	{
			calFreq[0] = 0xD7;
			calFreq[1] = 0xD8;
	}
	else if( frequency > 770000000 )
	{
			calFreq[0] = 0xC1;
			calFreq[1] = 0xC5;
	}
	else if( frequency > 460000000 )
	{
			calFreq[0] = 0x75;
			calFreq[1] = 0x81;
	}
	else if( frequency > 425000000 )
	{
			calFreq[0] = 0x6B;
			calFreq[1] = 0x6F;
	}
	WriteCommand(LLCC68_CMD_CALIBRATE_IMAGE, calFreq, 2); // 0x98
}

/*******************************************************************************
 * Function name  : SetRegulatorMode
 *
 * Description    : function to set the regulator mode of Lora chip
 * Parameters     : mode
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetRegulatorMode(uint8_t mode)
{
	uint8_t data = mode;
	WriteCommand(LLCC68_CMD_SET_REGULATOR_MODE, &data, 1); // 0x96
}

/*******************************************************************************
 * Function name  : SetBufferBaseAddress
 *
 * Description    : function to set the buffer base address of Lora chip
 * Parameters     : txBaseAddress, rxBaseAddress
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
	uint8_t buf[2];

	buf[0] = txBaseAddress;
	buf[1] = rxBaseAddress;
	WriteCommand(LLCC68_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2); // 0x8F
}

/*******************************************************************************
 * Function name  : SetPowerConfig
 *
 * Description    : function to set the power config
 * Parameters     : power, ramptime
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetPowerConfig(int8_t power, uint8_t rampTime)
{
	uint8_t buf[2];

	if( power > 22 )
	{
			power = 22;
	}
	else if( power < -3 )
	{
			power = -3;
	}
		
	buf[0] = power;
	buf[1] = ( uint8_t )rampTime;
	WriteCommand(LLCC68_CMD_SET_TX_PARAMS, buf, 2); // 0x8E
}

/*******************************************************************************
 * Function name  : SetPaConfig
 *
 * Description    : function to set the power amplifier config
 * Parameters     : paDutyCycle, hpMax, deviceSel, paLut
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
	uint8_t buf[4];

	buf[0] = paDutyCycle;
	buf[1] = hpMax;
	buf[2] = deviceSel;
	buf[3] = paLut;
	WriteCommand(LLCC68_CMD_SET_PA_CONFIG, buf, 4); // 0x95
}

/*******************************************************************************
 * Function name  : SetOvercurrentProtection
 *
 * Description    : function to set the over current protection for Lora chip
 * Parameters     : current limit
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetOvercurrentProtection(float currentLimit)
{
	if((currentLimit >= 0.0) && (currentLimit <= 140.0)) {
		uint8_t buf[1];
		buf[0] = (uint8_t)(currentLimit / 2.5);
		WriteRegister(LLCC68_REG_OCP_CONFIGURATION, buf, 1); // 0x08E7
	}
}

/*******************************************************************************
 * Function name  : SetSyncWord
 *
 * Description    : function to set the over current protection for Lora chip
 * Parameters     : sync
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void SetSyncWord(int16_t sync) {
	uint8_t buf[2];

	buf[0] = (uint8_t)((sync >> 8) & 0x00FF);
	buf[1] = (uint8_t)(sync & 0x00FF);
	WriteRegister(LLCC68_REG_LORA_SYNC_WORD_MSB, buf, 2); // 0x0740
}

/*******************************************************************************
 * Function name  : SetDioIrqParams
 *
 * Description    : function to set the DIO pins IRQ parameters
 * Parameters     : irqMask, dio1Mask, dio2Mask, dio3Mask
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetDioIrqParams (uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
	uint8_t buf[8];

	buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
	buf[1] = (uint8_t)(irqMask & 0x00FF);
	buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
	buf[3] = (uint8_t)(dio1Mask & 0x00FF);
	buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
	buf[5] = (uint8_t)(dio2Mask & 0x00FF);
	buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
	buf[7] = (uint8_t)(dio3Mask & 0x00FF);
	WriteCommand(LLCC68_CMD_SET_DIO_IRQ_PARAMS, buf, 8); // 0x08
}

/*******************************************************************************
 * Function name  : SetStopRxTimerOnPreambleDetect
 *
 * Description    : function to set stop receive timer on Preamable Detect
 * Parameters     : enable/disable
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void SetStopRxTimerOnPreambleDetect(bool enable)
{
	ESP_LOGI(TAG, "SetStopRxTimerOnPreambleDetect enable=%d", enable);
	//uint8_t data = (uint8_t)enable;
	uint8_t data = 0;
	if (enable) data = 1;
	WriteCommand(LLCC68_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1); // 0x9F
}

/*******************************************************************************
 * Function name  : SetLoRaSymbNumTimeout
 *
 * Description    : function command sets the number of symbols used by the modem
 * Parameters     : SymbNum
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetLoRaSymbNumTimeout(uint8_t SymbNum)
{
	uint8_t data = SymbNum;
	WriteCommand(LLCC68_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1); // 0xA0
}

/*******************************************************************************
 * Function name  : SetPacketType
 *
 * Description    : function set Lora packet type
 * Parameters     : packettype
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetPacketType(uint8_t packetType)
{
	uint8_t data = packetType;
	WriteCommand(LLCC68_CMD_SET_PACKET_TYPE, &data, 1); // 0x01
}

/*******************************************************************************
 * Function name  : SetModulationParams
 *
 * Description    : function set Lora modulation parameters
 * Parameters     : spreadingFactor, bandwidth, codingRate, lowDataRateOptimize
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize)
{
	uint8_t data[4];
	//currently only LoRa supported
	data[0] = spreadingFactor;
	data[1] = bandwidth;
	data[2] = codingRate;
	data[3] = lowDataRateOptimize;
	WriteCommand(LLCC68_CMD_SET_MODULATION_PARAMS, data, 4); // 0x8B
}

/*******************************************************************************
 * Function name  : SetCadParams
 *
 * Description    : function set Lora cad parameters
 * Parameters     : cadSymbolNum, cadDetPeak, cadDetMin, cadExitMode, cadTimeout
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void SetCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout)
{
	uint8_t data[7];
	data[0] = cadSymbolNum;
	data[1] = cadDetPeak;
	data[2] = cadDetMin;
	data[3] = cadExitMode;
	data[4] = (uint8_t)((cadTimeout >> 16) & 0xFF);
	data[5] = (uint8_t)((cadTimeout >> 8) & 0xFF);
	data[6] = (uint8_t)(cadTimeout & 0xFF);
	WriteCommand(LLCC68_CMD_SET_CAD_PARAMS, data, 7); // 0x88
}

/*******************************************************************************
 * Function name  : SetCad
 *
 * Description    : function set Lora cad
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetCad(void)
{
	uint8_t data = 0;
	WriteCommand(LLCC68_CMD_SET_CAD, &data, 0); // 0xC5
}

/*******************************************************************************
 * Function name  : GetIrqStatus
 *
 * Description    : function get IRQ status of Lora
 * Parameters     : None
 * Returns        : IRQ status
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 uint16_t GetIrqStatus( void )
{
	uint8_t data[3];
	ReadCommand(LLCC68_CMD_GET_IRQ_STATUS, data, 3); // 0x12
	return (data[1] << 8) | data[2];
}

/*******************************************************************************
 * Function name  : ClearIrqStatus
 *
 * Description    : function clear IRQ status of Lora
 * Parameters     : None
 * Returns        : IRQ status
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void ClearIrqStatus(uint16_t irq)
{
	uint8_t buf[2];

	buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
	buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
	WriteCommand(LLCC68_CMD_CLEAR_IRQ_STATUS, buf, 2); // 0x02
}

/*******************************************************************************
 * Function name  : SetRx
 *
 * Description    : function set Receiver timeout
 * Parameters     : Timeout
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void SetRx(uint32_t timeout)
{
	if (debugPrint) {
		ESP_LOGI(TAG, "----- SetRx timeout=%"PRIu32, timeout);
	}
	SetStandby(LLCC68_STANDBY_RC);
	SetRxEnable();
	uint8_t buf[3];
	buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
	buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
	buf[2] = (uint8_t)(timeout & 0xFF);
	WriteCommand(LLCC68_CMD_SET_RX, buf, 3); // 0x82
	for(int retry=0;retry<10;retry++) {
		if ((GetStatus() & 0x70) == 0x50) break;
		sys_delay_ms(1);
	}
	if ((GetStatus() & 0x70) != 0x50) {
		ESP_LOGE(TAG, "SetRx Illegal Status = 0x%x", GetStatus());
		LoRaError(ERR_INVALID_SETRX_STATE);
	}
	WaitOnBusy();
}

/*******************************************************************************
 * Function name  : SetRxEnable
 *
 * Description    : function set Receiver enable
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void SetRxEnable(void)
{
	if (debugPrint) {
		ESP_LOGI(TAG, "SetRxEnable:SX126x_TXEN=%d SX126x_RXEN=%d", LLCC68_TXEN, LLCC68_RXEN);
	}
	if ((LLCC68_TXEN != -1) && (LLCC68_RXEN != -1)) {
		gpio_set_level(LLCC68_RXEN, LOW); // Will turn on Blue LED
		gpio_set_level(LLCC68_TXEN, HIGH); // Will turn off Green LED
	}
}

/*******************************************************************************
 * Function name  : SetTx
 *
 * Description    : function set Transmit timeout
 * Parameters     : Timeout
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void SetTx(uint32_t timeoutInMs)
{
	if (debugPrint) {
		ESP_LOGI(TAG, "----- SetTx timeoutInMs=%"PRIu32, timeoutInMs);
	}
	SetStandby(LLCC68_STANDBY_RC);
	SetTxEnable();
	uint8_t buf[3];
	uint32_t tout = timeoutInMs;
	if (timeoutInMs != 0) {
		uint32_t timeoutInUs = timeoutInMs * 1000;
		tout = (uint32_t)(timeoutInUs / 0.015625);
	}
	if (debugPrint) {
		ESP_LOGI(TAG, "SetTx timeoutInMs=%"PRIu32" tout=%"PRIu32, timeoutInMs, tout);
	}
	buf[0] = (uint8_t)((tout >> 16) & 0xFF);
	buf[1] = (uint8_t)((tout >> 8) & 0xFF);
	buf[2] = (uint8_t )(tout & 0xFF);
	WriteCommand(LLCC68_CMD_SET_TX, buf, 3); // 0x83
	
	for(int retry=0;retry<10;retry++) {
		if ((GetStatus() & 0x70) == 0x60) break;
		vTaskDelay(1);
	}
	if ((GetStatus() & 0x70) != 0x60) {
		ESP_LOGE(TAG, "SetTx Illegal Status");
		LoRaError(ERR_INVALID_SETTX_STATE);
	}
	WaitOnBusy();
}

/*******************************************************************************
 * Function name  : SetTxEnable
 *
 * Description    : function set Transmit enable
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
 void SetTxEnable(void)
{
	if (debugPrint) {
		ESP_LOGI(TAG, "SetTxEnable:SX126x_TXEN=%d SX126x_RXEN=%d", LLCC68_TXEN, LLCC68_RXEN);
	}
	if ((LLCC68_TXEN != -1) && (LLCC68_RXEN != -1)){
		gpio_set_level(LLCC68_RXEN, HIGH); // Will turn off Blue LED
		gpio_set_level(LLCC68_TXEN, LOW); // Will turn on Green LED
	}
}

/*******************************************************************************
 * Function name  : GetPacketLost
 *
 * Description    : function set Transmit enable
 * Parameters     : None
 * Returns        : txlost status
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
int GetPacketLost(void)
{
	return txLost;
}

/*******************************************************************************
 * Function name  : GetRssiInst
 *
 * Description    : function get the RSSI valve
 * Parameters     : None
 * Returns        : RSSI value
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
uint8_t GetRssiInst(void)
{
	uint8_t buf[2];
	ReadCommand( LLCC68_CMD_GET_RSSI_INST, buf, 2 ); // 0x15
	return buf[1];
}

/*******************************************************************************
 * Function name  : GetRxBufferStatus
 *
 * Description    : function get the Rx buffer status
 * Parameters     : payloadLength, *rxStartBufferPointer
 * Returns        : RSSI value
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer)
{
	uint8_t buf[3];
	ReadCommand( LLCC68_CMD_GET_RX_BUFFER_STATUS, buf, 3 ); // 0x13
	*payloadLength = buf[1];
	*rxStartBufferPointer = buf[2];
	
}

/*******************************************************************************
 * Function name  : WaitForIdle
 *
 * Description    : function to wait for Lora to get into Idle mode
 * Parameters     : timeout
 * Returns        : void
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void WaitForIdle(unsigned long timeout)
{
    //unsigned long start = millis();
    TickType_t start = xTaskGetTickCount();
    sys_delay_ms(1);
    while(xTaskGetTickCount() - start < (timeout/portTICK_PERIOD_MS)) {
        if (gpio_get_level(LLCC68_BUSY) == 0) break;
        sys_delay_ms(1);
    }
    if (gpio_get_level(LLCC68_BUSY)) {
        ESP_LOGE(TAG, "WaitForIdle Timeout timeout=%lu", timeout);
        LoRaError(ERR_IDLE_TIMEOUT);
    }
}

/*******************************************************************************
 * Function name  : ReadBuffer
 *
 * Description    : function to read the Lora buffer
 * Parameters     : *rxData, rxDataLen
 * Returns        : void
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
uint8_t ReadBuffer(uint8_t *rxData, int16_t rxDataLen)
{
	uint8_t offset = 0;
	uint8_t payloadLength = 0;
	GetRxBufferStatus(&payloadLength, &offset);
	if( payloadLength > rxDataLen )
	{
		ESP_LOGW(TAG, "ReadBuffer rxDataLen too small. payloadLength=%d rxDataLen=%d", payloadLength, rxDataLen);
		return 0;
	}

	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	// start transfer
	gpio_set_level(LLCC68_SPI_SELECT, LOW);

	spi_transfer(LLCC68_CMD_READ_BUFFER); // 0x1E
	spi_transfer(offset);
	spi_transfer(LLCC68_CMD_NOP);
	for( int i = 0; i < payloadLength; i++ )
	{
		rxData[i] = spi_transfer(LLCC68_CMD_NOP);  
	}

	// stop transfer
	gpio_set_level(LLCC68_SPI_SELECT, HIGH);
    WaitOnBusy();
	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);

	return payloadLength;
}

/*******************************************************************************
 * Function name  : WriteBuffer
 *
 * Description    : function to write the Lora buffer
 * Parameters     : *txData, txDataLen
 * Returns        : void
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void WriteBuffer(uint8_t *txData, int16_t txDataLen)
{
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	// start transfer
	gpio_set_level(LLCC68_SPI_SELECT, LOW);

	spi_transfer(LLCC68_CMD_WRITE_BUFFER); // 0x0E
	spi_transfer(0); //offset in tx fifo
	for( int i = 0; i < txDataLen; i++ )
	{ 
		 spi_transfer( txData[i]);	
	}

	// stop transfer
	gpio_set_level(LLCC68_SPI_SELECT, HIGH);
	WaitOnBusy();
	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
}

/*******************************************************************************
 * Function name  : WriteRegister
 *
 * Description    : function to write the Lora register
 * Parameters     : reg, * data, numBytes
 * Returns        : void
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void WriteRegister(uint16_t reg, uint8_t* data, uint8_t numBytes) {
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	if(debugPrint) {
		ESP_LOGI(TAG, "WriteRegister: REG=0x%02x", reg);
	}
	// start transfer
	gpio_set_level(LLCC68_SPI_SELECT, LOW);

	// send command byte
	spi_transfer(LLCC68_CMD_WRITE_REGISTER); // 0x0D
	spi_transfer((reg & 0xFF00) >> 8);
	spi_transfer(reg & 0xff);
	
	for(uint8_t n = 0; n < numBytes; n++) {
		uint8_t in = spi_transfer(data[n]);
		(void)in;
		if(debugPrint) {
			ESP_LOGI(TAG, "%02x --> %02x", data[n], in);
			//ESP_LOGI(TAG, "DataOut:%02x ", data[n]);
		}
	}

	// stop transfer
	gpio_set_level(LLCC68_SPI_SELECT, HIGH);
	WaitOnBusy();
	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
}

/*******************************************************************************
 * Function name  : ReadRegister
 *
 * Description    : function to read the Lora register
 * Parameters     : reg, * data, numBytes
 * Returns        : void
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void ReadRegister(uint16_t reg, uint8_t* data, uint8_t numBytes) {
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	if(debugPrint) {
		ESP_LOGI(TAG, "ReadRegister: REG=0x%02x", reg);
	}

	// start transfer
	gpio_set_level(LLCC68_SPI_SELECT, LOW);

	// send command byte
	spi_transfer(LLCC68_CMD_READ_REGISTER); // 0x1D
	spi_transfer((reg & 0xFF00) >> 8);
	spi_transfer(reg & 0xff);
	spi_transfer(LLCC68_CMD_NOP);

	for(uint8_t n = 0; n < numBytes; n++) {
		data[n] = spi_transfer(LLCC68_CMD_NOP);
		if(debugPrint) {
			ESP_LOGI(TAG, "DataIn:%02x ", data[n]);
		}
	}

	// stop transfer
	gpio_set_level(LLCC68_SPI_SELECT, HIGH);
	WaitOnBusy();
	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
}

/*******************************************************************************
 * Function name  : WriteCommand
 *
 * Description    : function to write the Lora register with retry option
 * Parameters     : cmd, * data, numBytes
 * Returns        : void
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void WriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes) {
	uint8_t status;
	for (int retry=1; retry<10; retry++) {
		status = WriteCommandBusyWait(cmd, data, numBytes);
		ESP_LOGD(TAG, "status=%02x", status);
		if (status == 0) break;
		ESP_LOGW(TAG, "WriteCommandBusyWait status=%02x retry=%d", status, retry);
	}
	if (status != 0) {
		ESP_LOGE(TAG, "SPI Transaction error:0x%02x", status);
		LoRaError(ERR_SPI_TRANSACTION);
	}
	WaitOnBusy();
	
}

/*******************************************************************************
 * Function name  : WriteCommandBusyWait
 *
 * Description    : function to write the Lora register with busy wait
 * Parameters     : cmd, * data, numBytes
 * Returns        : void
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
uint8_t WriteCommandBusyWait(uint8_t cmd, uint8_t* data, uint8_t numBytes) {
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	// start transfer
	gpio_set_level(LLCC68_SPI_SELECT, LOW);

	// send command byte
	if(debugPrint) {
		ESP_LOGI(TAG, "WriteCommand: CMD=0x%02x", cmd);
	}
	spi_transfer(cmd);

	// variable to save error during SPI transfer
	uint8_t status = 0;

	// send/receive all bytes
	for(uint8_t n = 0; n < numBytes; n++) {
		uint8_t in = spi_transfer(data[n]);
		if(debugPrint) {
			ESP_LOGI(TAG, "%02x --> %02x", data[n], in);
		}

		// check status
		if(((in & 0b00001110) == LLCC68_STATUS_CMD_TIMEOUT) ||
		 ((in & 0b00001110) == LLCC68_STATUS_CMD_INVALID) ||
		 ((in & 0b00001110) == LLCC68_STATUS_CMD_FAILED)) {
			status = in & 0b00001110;
			break;
		} else if(in == 0x00 || in == 0xFF) {
			status = LLCC68_STATUS_SPI_FAILED;
			break;
		}
	} 

	// stop transfer
	gpio_set_level(LLCC68_SPI_SELECT, HIGH);
	WaitOnBusy()
	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
	return status;
}

/*******************************************************************************
 * Function name  : ReadCommand
 *
 * Description    : function to read the Lora register with busy wait
 * Parameters     : cmd, * data, numBytes
 * Returns        : void
 *
 * Known Issues   :
 * Note           :
 * author         : Chandrashekhar Venkatesh
 * date           : 20AUG2024
 ******************************************************************************/
void ReadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes) {
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	// start transfer
	gpio_set_level(LLCC68_SPI_SELECT, LOW);

	// send command byte
	if(debugPrint) {
		ESP_LOGI(TAG, "ReadCommand: CMD=0x%02x", cmd);
	}
	spi_transfer(cmd);

	// send/receive all bytes
	for(uint8_t n = 0; n < numBytes; n++) {
		data[n] = spi_transfer(LLCC68_CMD_NOP);
		if(debugPrint) {
			ESP_LOGI(TAG, "DataIn:%02x", data[n]);
		}
	}

	// stop transfer
	gpio_set_level(LLCC68_SPI_SELECT, HIGH);
	WaitOnBusy();
	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
}

/*******************************************************************************
 * Function name  : init_spi
 *
 * Description    : spi_init function
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh 
 * date           : 12SEP2024
 ******************************************************************************/
void init_spi() {
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_MISO_GPIO,
        .mosi_io_num = CONFIG_MOSI_GPIO,
        .sclk_io_num = CONFIG_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,   
        .mode = 0,                           
        .spics_io_num = LORA_CSn,            
        .queue_size = 1,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);
    ESP_ERROR_CHECK(ret);
}

/*******************************************************************************
 * Function name  : llcc68_reset
 *
 * Description    : llcc68_reset function 
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh 
 * date           : 12SEP2024
 ******************************************************************************/
void llcc68_reset() {
	
    gpio_set_direction(LLCC68_RESET_SPI, GPIO_MODE_OUTPUT);
    gpio_set_level(LLCC68_RESET_SPI, 0);  
    vTaskDelay(100 / portTICK_PERIOD_MS);  
    gpio_set_level(LLCC68_RESET_SPI, 1);  
    vTaskDelay(100 / portTICK_PERIOD_MS);  
}

/*******************************************************************************
 * Function name  : get_llcc68_version
 *
 * Description    : version of the chip llcc68. 
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh 
 * date           : 12SEP2024
 ******************************************************************************/
void get_llcc68_version() {
    esp_err_t ret;
    uint8_t tx_data[2] = {0xC0, 0x00};  
    uint8_t rx_data[1] = {0};           

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;           
    t.tx_buffer = tx_data;   
    t.rx_buffer = NULL;      
    ret = spi_device_transmit(lora_spi, &t);  
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI command transmit failed: %s", esp_err_to_name(ret));
        return;
    }

    memset(&t, 0, sizeof(t));
    t.length = 8;            
    t.tx_buffer = NULL;      
    t.rx_buffer = rx_data;   
    ret = spi_device_transmit(lora_spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read transmit failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "LLCC68 Version: 0x%02X", rx_data[0]);
}

/*******************************************************************************
 * Function name  : get_llcc68_deveui
 *
 * Description    : get the llcc68 device unique ID. 
 * Parameters     : None
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Keerthi Mallesh 
 * date           : 12SEP2024
 ******************************************************************************/
void get_llcc68_deveui() {
    esp_err_t ret;
    uint8_t tx_data[2] = {0x89, 0x00}; 
    uint8_t rx_data[8] = {0};          
 
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));           
    t.length = 16;                      
    t.tx_buffer = tx_data;              

    ret = spi_device_transmit(lora_spi, &t);
    ESP_ERROR_CHECK(ret);

    memset(&t, 0, sizeof(t));
    t.length = 8 * 8;                   
    t.rx_buffer = rx_data;              

    ret = spi_device_transmit(lora_spi, &t);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "LLCC68 DevEUI: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
             rx_data[0], rx_data[1], rx_data[2], rx_data[3],
             rx_data[4], rx_data[5], rx_data[6], rx_data[7]);
}

/*******************************************************************************
 * Function name  : RadioSetMaxPayloadLength
 *
 * Description    : Set the Max Payload Length. 
 * Parameters     : packetType, maxPayloadLength.
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh 
 * date           : 20SEP2024
 ******************************************************************************/
 
void RadioSetMaxPayloadLength(uint8_t packetType, uint8_t maxPayloadLength)  //12 preable length
{
    if (packetType == LLCC68_PACKET_TYPE_LORA) {
        // Set the maximum payload length for LoRa packets
        LoRaConfig(11, LLCC68_LORA_BW_500_0, LLCC68_LORA_CR_4_5, 12, maxPayloadLength, true, false);
    } else if (packetType == LLCC68_PACKET_TYPE_GFSK) {
        // Set the maximum payload length for FSK packets (if needed)
        LoRaConfig(11, LLCC68_LORA_BW_500_0, LLCC68_LORA_CR_4_5, 12, maxPayloadLength, true, false);
    } 
    ESP_LOGI(TAG,"Setting Max payload length!");
}
/*******************************************************************************
 * Function name  : GetPayload
 *
 * Description    : GetPayload. 
 * Parameters     : buffer, size, ,maxsize.
 * Returns        : None
 *
 * Known Issues   :
 * Note           :
 * author         : Venkata Suresh 
 * date           : 20SEP2024
 ******************************************************************************/
uint8_t GetPayload(uint8_t *buffer, uint8_t *size, uint8_t maxSize)
{
    uint8_t offset = 0; // Buffer offset

    // Get the payload length and starting offset from the LoRa receive buffer
    GetRxBufferStatus(size, &offset);
    
    // Check if the payload size exceeds the provided buffer size
    if (*size > maxSize)
    {
        // Payload is too large for the buffer
        return 1;
    }

    // Read the payload from the LoRa module into the buffer
    ReadBuffer(buffer, *size);

    // Return success
    return 0;
}
/*******************************************************************************
 * Function: LLCC68SetSleep
 * Description: Configures the LLCC68 LoRa chip to enter sleep mode.
 * Parameters: 
 *   - sleepConfig: Sleep configuration parameters (start type, RTC status).
 * Returns: None
 * Author: Venkata Suresh
 * Date: 20SEP2024
 ******************************************************************************/
void SetSleep(SleepParams_t sleepConfig) {
    uint8_t command = sleepConfig.sleepStart | sleepConfig.rtcStatus;
    WriteRegister(LLCC68_SLEEP_START_COLD, &command, 1);
}
/***********************************************************************************
 * Function name  : RadioStartCad
 * Description    : This function initiates the Channel Activity Detection (CAD)
 *                  process for LoRa communication. 
 * Parameters     : None.
 * Returns        : None (void).
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void RadioStartCad( void )
{
    SetDioIrqParams( LLCC68_IRQ_CAD_DONE|LLCC68_IRQ_CAD_DETECTED,
                           LLCC68_IRQ_CAD_DONE|LLCC68_IRQ_CAD_DETECTED,
                           LLCC68_IRQ_NONE, LLCC68_IRQ_NONE );
    SetCad();
}
/***********************************************************************************
 * Function name  : RadioRx
 * Description    : This function initiates the RadioRx for LoRa communication. 
 * Parameters     : uint32_t timeout
 * Returns        : None (void).
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void RadioRx( uint32_t timeout )
{
	bool RxContinuous = false;
    /*SetDioIrqParams( LLCC68_IRQ_RX_DONE | LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_HEADER_ERR | LLCC68_IRQ_CRC_ERR, 
                           LLCC68_IRQ_RX_DONE | LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_HEADER_ERR | LLCC68_IRQ_CRC_ERR, 
                           LLCC68_IRQ_NONE,
                           LLCC68_IRQ_NONE );*/
	SetDioIrqParams( LLCC68_IRQ_ALL, 
                           LLCC68_IRQ_ALL, 
                           LLCC68_IRQ_NONE,
                           LLCC68_IRQ_NONE );
    /*if( RxContinuous == true )
    {
        
    }
    else
    {
        SetRx( timeout << 6 );
    }*/
    SetRx( 0xFFFFFF ); // Rx Continuous
  
}


/*void RadioRx(uint32_t timeout_ms) 
{
    // Log the start of RX mode
    ESP_LOGI(TAG, "RadioRx: Starting RX mode with timeout: %u ms", (unsigned int) timeout_ms);

    // Wait for the radio to be ready before setting the mode
   if (!IsRadioReady()) {
        ESP_LOGE(TAG, "Radio not ready!");
        return;
    }

    // Set the IRQ masks for the desired interrupts (RxDone, Timeout, CrcError, HeaderError)
    uint16_t irqMask = LLCC68_IRQ_RX_DONE | LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_CRC_ERR | LLCC68_IRQ_HEADER_ERR;

    // Enable the interrupts
    WriteCommand(LLCC68_CMD_SET_DIO_IRQ_PARAMS, (uint8_t*)&irqMask, sizeof(irqMask));

    // Set the radio to RX mode with the specified timeout
    uint8_t timeoutBytes[3];
    timeoutBytes[0] = (timeout_ms >> 16) & 0xFF;
    timeoutBytes[1] = (timeout_ms >> 8) & 0xFF;
    timeoutBytes[2] = timeout_ms & 0xFF;

    // Send the command to put the radio into RX mode with the timeout
    WriteCommand(LLCC68_CMD_SET_RX, timeoutBytes, sizeof(timeoutBytes));

    // The rest will be handled by the SET_RADIO macro, which waits for an interrupt and processes it
}
*/
/***********************************************************************************
 * Function name  : RadioSend
 * Description    : This function initiates the RadioSend for LoRa communication. 
 * Parameters     : uint32_t timeout
 * Returns        : None (void).
 * Known Issues   : None
 * Note           : 
 * Author         : C.VenkataSuresh
 * Date           : 20SEP2024
 ***********************************************************************************/
void RadioSend( uint32_t timeout )
{
    SetDioIrqParams( LLCC68_IRQ_TX_DONE | LLCC68_IRQ_TIMEOUT,
                           LLCC68_IRQ_TX_DONE | LLCC68_IRQ_TIMEOUT,
                           LLCC68_IRQ_NONE,
                           LLCC68_IRQ_NONE );

    SetTx( timeout);
}
