#include <CC1200.h>

/**
 * See Section 3 Microcontroller Interface.
 * @param read sets R/W bit in header byte.
 * @param addr sets register address in SPI interface buffer.
 * @param extended sets register address space to 0x2F in header byte. 
 * @param burst sets transaction access type as burst in header byte.
 * @returns SPI transaction success
*/
bool z_cc1200_access_reg(const struct device *dev, bool read, uint8_t addr,
			 void *data, size_t length, bool extended, bool burst)
{
	
	const struct cc1200_config *config = dev->config;
	// SPI transactions start with a header buffer
	// register access occurs with 6-bit address and one-byte data. 
	uint8_t header_buf[2];
	const struct spi_buf buf[2] = {
		{
			.buf = header_buf,
			// Due to little endianness, a non-extended address has a
			// header length of one byte, extended length is two bytes.
			.len = extended ? 2 : 1 // 0x2F+addr else 0x00+addr
		},
		{
			.buf = data,
			.len = length
		}
	};
	struct spi_buf_set tx = { .buffers = buf };

	header_buf[0] = 0U;
	// Buffer-in whether the access type is Burst Mode in the header byte,
	// next transaction will be addr+1 if EXT_CTRL.BURST_ADDR_INCR_EN = 1.
	if (burst) {
		header_buf[0] |= CC1200_ACCESS_BURST;
	}
	// Buffer-in the register space being used and the register address.
	if (extended) {
		header_buf[0] |= CC1200_REG_EXTENDED_ADDRESS;
		header_buf[1] = addr;
	} else {
		header_buf[0] |= addr;
	}
	
	if (read) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		header_buf[0] |= CC1200_ACCESS_RD;

		tx.count = 1;

		return (spi_transceive_dt(&config->bus, &tx, &rx) == 0);
	}

	/* CC1200_ACCESS_WR is 0 so no need to play with it */
	tx.count =  data ? 2 : 1;

	return (spi_write_dt(&config->bus, &tx) == 0);
}

static uint8_t get_status(const struct device *dev)
{
	uint8_t val;

	if (z_cc1200_access_reg(dev, true, CC1200_INS_SNOP,
				&val, 1, false, false)) {
		/* See Section 3.1.2 */
		return val & CC1200_STATUS_MASK;
	}

	/* We cannot get the status, so let's assume about readiness */
	return CC1200_STATUS_CHIP_NOT_READY;
}

static uint8_t enable_cfm() {
	/** 
     * 8.9.2 Transparent Serial Mode Configuration (Contd.)
     * IOCFGx is set to reflect GPIO0 as CFM, GPIO2 as the clock.
    */ 
    write_reg_iocfg3(0x00);                       // 0x00 IOCFG3
    write_reg_iocfg2(CC1200_GPIO_SIG_SERIAL_CLK); // 0x01 IOCFG2.GPIO2_CFG = SERIAL_CLK
    write_reg_iocfg1(0x00);                       // 0x02 IOCFG1
    write_reg_iocfg0(CC1200_GPIO_SIG_SERIAL_RX);  // 0x03 IOCFG0.GPIO0_CFG = SERIAL_RX

	// runs at 16x the programmed symbol rate. 
	// CLKEN_CFM signal should be output on GPIO2.
	// CFM_RX_DATA_OUT
	write_reg_mdmcfg2(0x00);       // 0x03 MDMCFG.CFM_DATA_EN = 1
	write_reg_ext_ctrl(0x00);      // 0x03 EXT_CTRL.BURST_ADDR_INCR_EN = 0
    // IOC_SYNC_PINS_EN: Enable synchronizer for IO pins.
    // Required for transparent TX and for reading GPIO_STATUS.GPIO_STATE
    write_reg_serial_status(0x08); // 0x91 SERIAL_STATUS.IOC_SYNC_PINS_EN = 1
}

/**
 * CC120X User Guide (SWRU346B) Page 105 of 114
 *
 * RSSI1 - Received Signal Strength Indicator Reg. 1
 * | Bit # | Name      | Reset   | Description                    |
 * | 7:0   | RSSI_11_4 | 0x80    | 8 MSB of RSSI[11:0].           |
 *
 * RSSI0 - Received Signal Strength Indicator Reg. 0
 * | Bit # | Name                | Description                    |
 * | 7     | RSSI0_NOT_USED      |                                |
 * | 6:3   | RSSI_3_0            | 4 LSB of RSSI[11:0]. See RSSI1 |
 * | 2     | CARRIER_SENSE       | 0 No Carrier, 1 Carrier        |
 * | 1     | CARRIER_SENSE_VALID | 0 Not Valid, 1 Valid           |
 * | 0     | RSSI_VALID          | 0 Not Valid, 1, Valid          |
 */
// RSSI[11:0] is a two's complement number with 0.0625 dB resolution hence
// ranging from -128 to 127 dBm.
float CC1200::getRSSI()
{
    uint8_t RSSI0 = read_reg_rssi0(dev);
    // (-0x80 & 1UL ) == 0
    // A value of -128 dBm indicates that the RSSI is invalid.
    if ((RSSI0 & RSSI_VALID) != 0)
    {
        // Received signal strength indicator. 8 MSB of RSSI[11:0].
        uint8_t RSSI_11_4 = read_reg_rssi1(dev);
        // 4 MSB of RSSI[11:0].
        // Register value bits 6:3, (RSSI0 & 0x78) >> 3
        uint8_t RSSI_3_0  = RSSI(RSSI0);
        // AGC gain adjustment. This register is used to adjust RSSI to
        // the actual carrier input signal level to compensate for interpolation
        // gains (two's complement with 1 dB resolution)
        int8_t offset = read_reg_agc_gain_adjust(dev);
        uint16_t rssi_raw = (uint16_t)(RSSI_11_4 << 4) | RSSI_3_0;
        // To get a correct RSSI value a calibrated RSSI offset value should be
        // subtracted from the value given by RSSI[11:0].
        float rssi = (float)(rssi_raw - offset);
        // Put RSSI in units of dBm with a resolution of 0.0625 dB (1 tick)
        rssi *= 0.0625f;
        return rssi;
    }
    return -128.0f;  // invalid RSSI (dBm)
}