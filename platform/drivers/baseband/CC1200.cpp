#include <peripherals/gpio.h>
// #include <zephyr/drivers/gpio.h>                                                                                                                                                     
#include <zephyr/drivers/spi.h>
#include <interfaces/delays.h
#include <hwconfig.h>
#include "CC120x.h"

/*

https://wiki.m17project.org/cc1200

The key is reading/writing to the CFM_TX_DATA_IN / CFM_TX_DATA_OUT registers at a rate of 48kHz.

*/

#define STACK_SIZE 512
#define FRAME_SIZE (16)
#define COMMAND_SIZE 3
#define CONFIGURATION_SIZE 51*COMMAND_SIZE
#define SPI_OP  SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE
#define SPIBB_NODE	DT_NODELABEL(spibb0) // spi baseband controller 0
//const struct device *const dev = DEVICE_DT_GET(SPIBB_NODE);
const struct spi_dt_spec CC1200_dev = SPI_DT_SPEC_GET(SPIBB_NODE, SPI_OP(FRAME_SIZE), 0);

static __aligned(32) uint8_t spi_buffer[size] __used __NOCACHE;

/**
 * For TX mode, deviation control registers (DEVIATION_M and MODCFG_DEV_E) are set for about 5kHz.
 * For RX they are set to about 3kHz. Symbol rate is set to 24k for TX and 1.2k for RX.
 * Nobody knows why these settings work :) RX BW is about 9.5kHz. CFM is enabled in both modes.
 * AFC can be enabled in RX mode by writing to register 0x2F01.
 * Via M17-Project
 */
const uint8_t cc1200_rx_settings[CONFIGURATION_SIZE] =
{
	0x00, CC1200Register::IOCFG2, 0x08,
	0x00, CC1200Register::IOCFG0, 0x09,
	0x00, CC1200Register::SYNC_CFG1, 0x1F,
	0x00, CC1200Register::DEVIATION_M, 0x9F, //deviation - about 3kHz full scale
	0x00, CC1200Register::MODCFG_DEV_E, 0x00, //deviation
	0x00, CC1200Register::DCFILT_CFG, 0x5D,
	0x00, CC1200Register::PREAMBLE_CFG1, 0x00,
	0x00, CC1200Register::PREAMBLE_CFG0, 0x8A,
	0x00, CC1200Register::IQIC, 0xCB,
	0x00, CC1200Register::CHAN_BW, 0xAC, //RX filter BW - 9.5kHz
	0x00, CC1200Register::MDMCFG1, 0x00,
	0x00, CC1200Register::MDMCFG0, 0x45,
	0x00, CC1200Register::SYMBOL_RATE2, 0x3F, //symbol rate 2 - 1.2k sym/s
	0x00, CC1200Register::SYMBOL_RATE1, 0x75, //symbol rate 1
	0x00, CC1200Register::SYMBOL_RATE0, 0x10, //symbol rate 0
	0x00, CC1200Register::AGC_REF, 0x37,
	0x00, CC1200Register::AGC_CS_THR, 0xEC,
	0x00, CC1200Register::AGC_CFG3, 0x11,
	0x00, CC1200Register::AGC_CFG1, 0x51,
	0x00, CC1200Register::AGC_CFG0, 0x87,
	0x00, CC1200Register::FIFO_CFG, 0x00,
	0x00, CC1200Register::FS_CFG, 0x14,
	0x00, CC1200Register::PKT_CFG2, 0x03,
	0x00, CC1200Register::PKT_CFG1, 0x00,
	0x00, CC1200Register::PKT_CFG0, 0x20,
	0x00, CC1200Register::PA_CFG1, 0x03, //output power - 0x03..0x3F (doesn't matter for RX)
	0x00, CC1200Register::PKT_LEN, 0xFF,
	0x2F, CC1200Register::IF_MIX_CFG, 0x1C,
	0x2F, CC1200Register::FREQOFF_CFG, 0x02, //AFC, 0x22 - on, 0x02 - off
	0x2F, CC1200Register::ECG_CFG, 0x0C, //external oscillator's frequency is 40 MHz
	0x2F, CC1200Register::MDMCFG2, 0x09, //16x upsampler, CFM enable
	0x2F, CC1200Register::FREQ2, 0x57, //frequency - round((float)435000000/5000000*(1<<16))=0x570000
	0x2F, CC1200Register::FREQ1, 0x00, //frequency
	0x2F, CC1200Register::FREQ0, 0x00, //frequency
	0x2F, CC1200ExtendedRegister::IF_ADC1, 0xEE,
	0x2F, CC1200ExtendedRegister::IF_ADC0, 0x10,
	0x2F, CC1200ExtendedRegister::FS_DIG1, 0x07,
	0x2F, CC1200ExtendedRegister::FS_DIG0, 0xAF,
	0x2F, CC1200ExtendedRegister::FS_CAL1, 0x40,
	0x2F, CC1200ExtendedRegister::FS_CAL0, 0x0E,
	0x2F, CC1200ExtendedRegister::FS_DIVTWO, 0x03,
	0x2F, CC1200ExtendedRegister::FS_DSM0, 0x33,
	0x2F, CC1200ExtendedRegister::FS_DVC0, 0x17,
	0x2F, CC1200ExtendedRegister::FS_PFD, 0x00,
	0x2F, CC1200ExtendedRegister::FS_PRE, 0x6E,
	0x2F, CC1200ExtendedRegister::FS_REG_DIV_CML, 0x1C,
	0x2F, CC1200ExtendedRegister::FS_SPARE, 0xAC,
	0x2F, CC1200ExtendedRegister::FS_VCO0, 0xB5,
	0x2F, CC1200ExtendedRegister::XOSC5, 0x0E,
	0x2F, CC1200ExtendedRegister::XOSC1, 0x03,
	0x2F, CC1200ExtendedRegister::SERIAL_STATUS, 0x08
};

const uint8_t cc1200_tx_settings[CONFIGURATION_SIZE] =
{
	0x00, CC1200Register::IOCFG2, 0x08,
	0x00, CC1200Register::IOCFG0, 0x09,
	0x00, CC1200Register::SYNC_CFG1, 0x1F,
	0x00, CC1200Register::DEVIATION_M, 0x06, //deviation - 5kHz full scale
	0x00, CC1200Register::MODCFG_DEV_E, 0x01, //deviation
	0x00, CC1200Register::DCFILT_CFG, 0x5D,
	0x00, CC1200Register::PREAMBLE_CFG1, 0x00,
	0x00, CC1200Register::PREAMBLE_CFG0, 0x8A,
	0x00, CC1200Register::IQIC, 0xCB,
	0x00, CC1200Register::CHAN_BW, 0xAC, //RX filter BW - 9.5kHz (doesn't matter for TX)
	0x00, CC1200Register::MDMCFG1, 0x00,
	0x00, CC1200Register::MDMCFG0, 0x45,
	0x00, CC1200Register::SYMBOL_RATE2, 0x83, //symbol rate 2 - 24k symb/s
	0x00, CC1200Register::SYMBOL_RATE1, 0xA9, //symbol rate 1
	0x00, CC1200Register::SYMBOL_RATE0, 0x2A, //symbol rate 0
	0x00, CC1200Register::AGC_REF, 0x37,
	0x00, CC1200Register::AGC_CS_THR, 0xEC,
	0x00, CC1200Register::AGC_CFG3, 0x11,
	0x00, CC1200Register::AGC_CFG1, 0x51,
	0x00, CC1200Register::AGC_CFG0, 0x87,
	0x00, CC1200Register::FIFO_CFG, 0x00,
	0x00, CC1200Register::FS_CFG, 0x14,
	0x00, CC1200Register::PKT_CFG2, 0x03,
	0x00, CC1200Register::PKT_CFG1, 0x00,
	0x00, CC1200Register::PKT_CFG0, 0x20,
	0x00, CC1200Register::PA_CFG1, 0x03, //output power - 0x03..0x3F
	0x00, CC1200Register::PKT_LEN, 0xFF,
	0x2F, CC1200ExtendedRegister::IF_MIX_CFG, 0x1C,
	0x2F, CC1200ExtendedRegister::FREQOFF_CFG, 0x22,
	0x2F, CC1200ExtendedRegister::ECG_CFG, 0x0C, //external oscillator's frequency is 40 MHz
	0x2F, CC1200ExtendedRegister::MDMCFG2, 0x09, //16x upsampler, CFM enable
	0x2F, CC1200ExtendedRegister::FREQ2, 0x57, //frequency - round((float)435000000/5000000*(1<<16))=0x570000
	0x2F, CC1200ExtendedRegister::FREQ1, 0x00, //frequency
	0x2F, CC1200ExtendedRegister::FREQ0, 0x00, //frequency
	0x2F, CC1200ExtendedRegister::IF_ADC1, 0xEE,
	0x2F, CC1200ExtendedRegister::IF_ADC0, 0x10,
	0x2F, CC1200ExtendedRegister::FS_DIG1, 0x07,
	0x2F, CC1200ExtendedRegister::FS_DIG0, 0xAF,
	0x2F, CC1200ExtendedRegister::FS_CAL1, 0x40,
	0x2F, CC1200ExtendedRegister::FS_CAL0, 0x0E,
	0x2F, CC1200ExtendedRegister::FS_DIVTWO, 0x03,
	0x2F, CC1200ExtendedRegister::FS_DSM0, 0x33,
	0x2F, CC1200ExtendedRegister::FS_DVC0, 0x17,
	0x2F, CC1200ExtendedRegister::FS_PFD, 0x00,
	0x2F, CC1200ExtendedRegister::FS_PRE, 0x6E,
	0x2F, CC1200ExtendedRegister::FS_REG_DIV_CML, 0x1C,
	0x2F, CC1200ExtendedRegister::FS_SPARE, 0xAC,
	0x2F, CC1200ExtendedRegister::FS_VCO0, 0xB5,
	0x2F, CC1200ExtendedRegister::XOSC5, 0x0E,
	0x2F, CC1200ExtendedRegister::XOSC1, 0x03,
	0x2F, CC1200ExtendedRegister::SERIAL_STATUS, 0x08
};

void CC1200::init()
{
	// TODO: Utilize a ready signal from User Guide Table 10, two XOSC periods long.
	// Step 1. Give it a few milliseconds to power up, then send a reset command - 0x30 (1-byte write)
	delayMs(5); // power-up delay
	// define an array of byte pairs describing which commands to strobe
	const uint8_t cc1200_reset[COMMAND_SIZE] = {
		0x30|0xC0, // triggers an event determined by the address (0x30 is the reset command, and 0xC0 sets burst read mode for safety.)
		CC1200ExtendedRegister::CFM_TX_DATA_IN,  // triggers an event determined by the address lower bits (register address) set to 0x7E
		0x00                                     // no data byte is expected.
	};
	writeBufferedData(cc1200_reset, COMMAND_SIZE);
	// Step 2. Wait 100ms - probably the delay can be a lot less than that
	delayMs(100);
	// Step 3. Use the CC1200 RX settings to set up CC1200
	writeBufferedData(cc1200_rx_settings, CONFIGURATION_SIZE);
	// Step 4. Send either 0x34 or 0x35 command (1-byte) to enable RX or TX mode
	setFuncMode(CC1200::RX);
	// TODO: Step 5. If necessary (eg. using a simple XO instead of proper TCXO, caesium reference or hydrogen maser) - apply frequency compensation by writing int16_t value to register 0x2F0A. See the code at the end of this paragraph.

	// Step 6. Disable auto address increment for baseband SPI data transfer - write 0 to register 0x2F06
	const uint8_t cc1200_burst_consecutive_write[COMMAND_SIZE] = {
		0x2F,
		CC1200ExtendedRegister::EXT_CTRL,  // triggers an event determined by the address lower bits (register address) set to 0x06
		0x00                               // burst address increment disabled (i.e. consecutive writes to the same address location in burst mode)
	};
	writeBufferedData(cc1200_burst_consecutive_write, COMMAND_SIZE);
	// Step 7. Start Sending Baseband Samples
	// First, send a 3-byte “start”:
	// define an array of byte pairs describing which commands to strobe
	const uint8_t cc1200_start[COMMAND_SIZE-1] = {
		0x2F|0x40, // triggers an event determined by the address (burst-write mode on extended registers)
		CC1200ExtendedRegister::CFM_TX_DATA_IN,  // triggers an event determined by the address lower bits (register address) set to 0x7E
	};
	// Chip Select Low
	setOpMode(CC1200::CFM);
	
	
	writeBufferedData(cc1200_start, COMMAND_SIZE-1); // 2-byte SPI transfer
	
	// Don’t pull SPI_CS line high, do it after all bytes are sent.
}

void CC1200::setFuncMode(const enum mode)
{
	switch(mode)
	{
		case IDLE:
			uint8_t data[1] = {CC1200Register::SIDLE};
			writeBufferedData(data,1);
			break;
		case RX:
			uint8_t data[1] = {CC1200Register::SRX};
			writeBufferedData(data,1);
			break;
		case TX:
			uint8_t data[1] = {CC1200Register::STX};
			writeBufferedData(data,1);
			break;
		default:
			break;
	}
}

void CC1200::setOpMode(const enum opmode mode)
{
	switch(mode)
	{
		case CFM:
			break; 
		case _2_FSK:
			break; 
		case _2_GFSK:
			break; 
		case _4_FSK:
			break; 
		case _4_GFSK:
			break;
		default:
			break;
	}
}

int16_t CC1200::readBufferedData(uint8_t address, int size)
{
	const struct spi_buf spi_buffers[] = {
	{
		.buf = spi_buffer,
		.len = CONFIGURATION_SIZE
	},
	};
	const struct spi_buf_set spi_buffer_set = {
		.buffers = buffer_set,
		.count = ARRAY_SIZE(buffer_set)
	};

	ret = spi_read_dt(&CC1200_dev, &spi_buffer_set);

	memset(buffer, 0, sizeof(buffer));
	memcpy(buffer, data, sizeof(data));
}

void CC1200::writeBufferedData(uint8_t data[], int size)
{
	const struct spi_buf spi_buffers[] = {
		{
			.buf = spi_buffer,
			.len = CONFIGURATION_SIZE
		},
	};
	const struct spi_buf_set spi_buffer_set = {
		.buffers = buffer_set,
		.count = ARRAY_SIZE(buffer_set)
	};

	ret = spi_write_dt(&CC1200_dev, &spi_buffer_set);

	memset(buffer, 0, sizeof(buffer));
	memcpy(buffer, data, sizeof(data));
}