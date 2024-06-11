#include <hwconfig.h>
#include <interfaces/delays.h>
#include "CC1200.h"

const struct device* const dev = DEVICE_DT_GET(cc1200);

/**
 * https://wiki.m17project.org/cc1200
 * The key is reading/writing to the CFM_TX_DATA_IN / CFM_TX_DATA_OUT registers at
 * a rate of 48kHz. CFM is enabled in both modes. AFC can be enabled in RX mode by
 * writing to register FREQOFF_CFG. (0x2F01).
 *
 * SP5WWP M17 RX Configuration
 * R[0x01] = 0x08, R[0x03] = 0x09,
 * R[0x08] = 0x1F,
 * R[0x0A] = 0xAD, //deviation - a little bit under 3.3kHz full scale
 * R[0x0B] = 0x00, //deviation
 * R[0x0C] = 0x5D,
 * R[0x0D] = 0x00, R[0x0E] = 0x8A,
 * R[0x0F] = 0xCB,
 * R[0x10] = 0xAC, //RX filter BW - 9.5kHz
 * R[0x11] = 0x00, R[0x12] = 0x45,
 * R[0x13] = 0x43, 0x14 = 0xA9, 0x15 = 0x2A, //symbol rate 2 - 1.5k sym/s, symbol rate 1, symbol rate 0
 * R[0x16] = 0x37, R[0x17] = 0xEC, R[0x19] = 0x11, R[0x1B] = 0x51, R[0x1C] = 0x87,
 * R[0x1D] = 0x00,
 * R[0x20] = 0x14,
 * R[0x26] = 0x03, R[0x27] = 0x00, R[0x28] = 0x20,
 * R[0x2B] = 0x03, //output power - 0x03..0x3F (doesn't matter for RX)
 * R[0x2E] = 0xFF,
 * E[0x00] = 0x1C,
 * E[0x01] = 0x02, //AFC, 0x22 - on, 0x02 - off
 * E[0x04] = 0x0C, //external oscillator's frequency is 40 MHz
 * E[0x05] = 0x09, //16x upsampler, CFM enable
 * E[0x0C] = 0x57, E[0x0D] = 0x00, E[0x0E] = 0x00, //frequency - round((float)435000000/5000000*(1<<16))=0x570000
 * E[0x10] = 0xEE, E[0x11] = 0x10,
 * E[0x12] = 0x07, E[0x13] = 0xAF,
 * E[0x16] = 0x40, E[0x17] = 0x0E, E[0x19] = 0x03, E[0x1B] = 0x33, E[0x1D] = 0x17, E[0x1F] = 0x00, E[0x20] = 0x6E, E[0x21] = 0x1C, E[0x22] = 0xAC, E[0x27] = 0xB5,
 * E[0x32] = 0x0E, E[0x36] = 0x03,
 * E[0x91] = 0x08
 *
 * First 42 entries are for the 42 first registers from
 * address 0x04 to 0x2D included.
 * Next, the last 58 entries are for the 58 registers from
 * extended address 0x00 to 0x39 included
 *
 * These can be generated through TI's SmartRF application.
 */
const struct cc1200_rf_registers_set cc1200_rf_settings = {
    .chan_center_freq0 = 445,
    // .channel_limit = 38, might be important?
    .channel_spacing = 95, /* 200 KHz */
    .registers = { // uint8_t registers[100]; SYNC3 -> CC1200_RF_NON_EXT_SPACE_REGS
        // Sync Word Configuration
        /**
         * all samples (noise or data) received after RX mode is entered will
         * either be put in the RX FIFO or output on a GPIO configured as SERIAL_RX.
         * Note that when 4'ary modulation is used the sync word uses 2'ary modulation
         * (the symbol rate is kept the same)
        */
        0x00, 0x00, 0x00, 0x00 // 0x04 SYNC3, 0x05 SYNC2, 0x06 SYNC1, 0x07 SYNC0
        0x1F, // 0x08 SYNC_CFG1.SYNC_MODE = No Sync Word, SYNC_CFG1.SYNC_THR = 0x1F
        // Symbol Rate <= CHAN_BW/2 = f_xosc/(CHAN_BW.ADC_CIC_DECFACT*CHAN_BW.BB_CIC_DECFACT*4)
        0x00, // 0x09 SYNC_CFG0
        //
        // For TX mode, deviation control registers (DEVIATION_M and MODCFG_DEV_E) are set to about 5kHz.
        0xAD, // 0x0A DEVIATION_M = 0xAD, deviation - a little bit under 3.3kHz full scale
        0x00, // 0x0B MODCFG_DEV_E = 0x00, deviation 
        // Digital DC Removal Configuration
        // DCFILT_FREEZE_COEFF = Manual DC compensation through registers DCFILTOFFSET_Ix, DCFILTOFFSET_Qx
        // DCFILT_BW_SETTLE = 64 samples
        // DCFILT_BW = 5 kHz
        // f_Cut-Off DC Filter ~= f_XOSC/(CHAN_BW.ADC_CIC_DECFACT*2^(2*DCFILT_BW))
        0x5D, // 0x0C DCFILT_CFG.DCFILT_FREEZE_COEFF = 1, DCFILT_CFG.DCFILT_BW_SETTLE = 011, DCFILT_CFG.DCFILT_BW = 101
        //
        // Preamble Configuration
        // PREAMBLE_CFG1: No preamble
        0x00, // 0x0D PREAMBLE_CFG1 = 0x00
        0x8A, // 0x0E PREAMBLE_CFG0 = 0x8A
        // Digital Image Channel Compensation
        // IQIC: IQ image compensation enabled, IQIC update coefficients enabled, 128 samples, > 2048
        0xCB, // 0x0F IQIC: IQIC_EN = 1, IQIC_UPDATE_COEFF_EN = 1, IQIC_BLEN_SETTLE = 00 IQIC_BLEN = 10, IQIC_IMGCH_LEVEL_THR = 11
        //
        // Channel Filter Configuration
        // RX_FILTER_BW = f_xosc/(CHAN_BW.ADC_CIC_DEFACT*CHAN_BW.BB_CIC_DEFACT*2) Hz = 9.5 kHz
        0xAC, // 0x10 CHAN_BW.ADC_CIC_DECFACT = 48, CHAN_BW.BB_CIC_DECFACT = 44
        // 8.9.2 Transparent Serial Mode Configuration
        0x00, // 0x11 MDMCFG1.FIFO_EN = 0 
        0x45, // 0x12 MDMCFG0.TRANSPARENT_MODE_EN = 1, MDMCFG0.VITERBI_EN = 1
        // Symbol rate is set to 24k for TX and 1.2k for RX. symbol rate 2 - 1.5k sym/s
        0x43, // 0x13 SYMBOL_RATE2.SRATE_E = 4 , SYMBOL_RATE2.SRATE_M_19_16 = 
        0xA9, // 0x14 SYMBOL_RATE1 = 0xA9, symbol rate 1
        0x2A, // 0x15 SYMBOL_RATE0 = 0x2A, symbol rate 0
        //
        // Automatic Gain Control Configuration
        // AGC_REFERENCE = 10*log10(RX_FILTER_BW)-92-RSSI_OFFSET = 55 dB
        // AGC_CS_TH = 236 dB (1dB res)
        // GAIN_ADJUSTMENT = 0 dB
        0x37, // 0x16 AGC_REF.AGC_REFERENCE = 0x37
        0xEC, // 0x17 AGC_CS_THR.AGC_CS_TH = 0xEC 
        0x00, // 0x18 AGC_GAIN_ADJUST.GAIN_ADJUSTMENT = 0x00
        // AGC_CFG3: No AGC gain freeze. Keep computing/updating RSSI
        // AGC_CFG2: Receiver starts with maximum gain value, Optimized linearity mode, Max gain is 0 dB to 17 dB.
        // AGC_CFG1: RSSI step is 6 dB during sync search, 16 dB during packet reception
        // Sample Rate = f_{xosc} * SYNC_CFG0.RX_CONFIG_LIMITATION/(CHAN_BW.ADC_CIC_DECFACT)^2
        // SYNC_CFG0.RX_CONFIG_LIMITATION: 0 is 2x, 1 is 4x
        // AGC_CFG0: 7 dB hysteresis level, 60 dB slew rate limit, 2 new samples required per RSSI, 9500 Hz ASK sample rate
        0x11, // 0x19 AGC_CFG3.AGC_SYNC_BEHAVIOUR = 0b000, AGC_MIN_GAIN = 0b10001 
        0x00, // 0x1A AGC_CFG2.AGC_MAX_GAIN = 0 dB to 17 dB
        0x51, // 0x1B AGC_CFG1.RSSI_STEP_THR = 1, AGC_WIN_SIZE = 010, AGC_SETTLE_WAIT = 001
        0x87, // 0x1C AGC_CFG0.AGC_HYST_LEVEL = 10, AGC_SLEWRATE_LIMIT = 00, RSSI_VALID_CNT = 01, AGC_ASK_DECAY = 11
        //
        0x00, // 0x1D FIFO_CFG.FIFO_THR = 127 bytes
        0x1F, // 0x1E DEV_ADDR.DEVICE_ADDR for packet filtering RX.
        // FS Calibration and Settling Configuration
        // SETTLING_CFG: No auto calibration, TX/RX 50/20 μs lock state settle time, 30 μs regulator settling time
        // FS_CFG: Out of lock detector enabled, 410.0 - 480.0 MHz band (LO divider = 8)
        0x00, // 0x1F SETTLING_CFG
        0x14, // 0x20 FS_CFG.FS_LOCK_EN = 1, FS_CFG.FSD_BANDSELECT = 0100
        //
        // Wake On Radio eWOR Configuration
        // WOR_CFG1: High Resolution, Event0 mask mode, WOR_EVENT1 = 16
        // WOR_CFG0: RXDCM1, Clock division enabled, WOR_EVENT2 = 15, RCOSC calibration enabled, RCOSC is running
        // t_Event1 = 0 sec
        // t_Event1 = 1/f_RCOSC*WOR_EVENT1 sec
        // t_Event2 = 2^WOR_EVENT2/f_RCOSC sec
        0x1C,                   // 0x21 WOR_CFG1.WOR_RES = 0 WOR_CFG1.WOR_MODE = 3, WOR_CFG1.EVENT1 = 4
        0xAC,                   // 0x22 RX_DUTY_CYCLE_MODE = 2, DIV_256HZ_EN = 1, EVENT2_CFG = 01, RC_MODE = 10, RC_PD = 0
        0x00,                   // 0x23 WOR_EVENT0_MSB
        0x00,                   // 0x24 WOR_EVENT0_LSB
        // t_RXDCM = 2^(WOR_CFG1.WOR_RES) μs
        0x00,                   // 0x25 RXDCM_TIME
        // Packet Configuration
        // PKT_CFG2: Data byte swap disabled, Standard packet mode enabled, Always give a clear channel indication, Transparent serial mode
        // PKT_CFG1: FEC disabled, Data whitening disabled, No address check, CRC disabled for TX and RX, Status byte not appended
        // PKT_CFG0: Variable packet length mode. Packet length configured by the first byte received after sync word, UART mode disabled, Swap disabled. Start/stop bits values are '1'/'0'
        0x03,                   // 0x26 PKT_CFG2.PKT_FORMAT = 0b11
        0x00,                   // 0x27 PKT_CFG1 = 0x00
        0x20,                   // 0x28 LENGTH_CONFIG = 01, PKT_BIT_LEN = 000 UART_MODE_EN = 0 UART_SWAP_EN = 0 
        0x2A,                   // 0x29 RFEND_CFG1, 0x2A RFEND_CFG0
        0x03,                   // 0x2B PA_CFG1 = 0x03 output power - 0x03..0x3F (doesn't matter for RX)
        0x00, 0x00,             // 0x2C PA_CFG0, 0x2D ASK_CFG
        0xFF,                   // 0x2E PKT_LEN = 0xFF
        // 
        // 0x2F EXTENDED ADDRESS
        // f_if = f_xosc/(CHAN_BW.ADC_CIC_DEFACT*6) kHz
        0x1C,                   // 0x00 IF_MIX_CFG = 0x1C, 110 11 
        // FREQOFF_CFG: Frequency offset correction disabled, Loop gain factor = 1/64
        // Automatic Frequency Compensation: 0x22 - on, 0x02 - off
        0x02,                   // 0x01 FOC_EN = 0, FOC_KI_FACTOR = 01
        0x03,                   // 0x02 TOC_CFG
        0x00,                   // 0x03 MARC_SPARE
        0x0C,                   // 0x04 ECG_CFG = 0x0C, //external oscillator's frequency is 40 MHz
        // General Modem Parameter Configuration
        // Use 16x TX upsampling factor, CFM enable
        0x09,                   // 0x05 MDMCFG.UPSAMPLER_P = 100, MDMCFG2.CFM_DATA_EN = 1
        0x00, 0x00, 0x00, 0x00, // 0x06 EXT_CTRL, 0x07 RCCAL_FINE, 0x08 RCCAL_COARSE, 0x09 RCCAL_OFFSET
        0x00, 0x00              // 0x0A FREQOFF1, 0x0B FREQOFF0
        // Frequency Configuration
        // f_rf = f_vco/FS_CFG.FSD_BANDSELECT
        // f_vco = f_xosc*(FREQ/2^16 + FREQOFF/2^18)
        // Frequency = round((float)435MHz/5MHz*(1<<16)) = 0x570000
        0x57,                   // 0x0C FREQ2 = 0x57, Frequency[23:16]
        0x00,                   // 0x0D FREQ1 = 0x00, Frequency[15:8]
        0x00,                   // 0x0E FREQ0 = 0x00, Frequency[7:0]
        // IF_RESERVED for TI SmartRF Studio
        0x00, 0xEE, 0x10,       // 0x0F IF_ADC2, 0x10 IF_ADC1, 0x11 IF_ADC0
        // Frequency Synthesizer Digital Configuration
        // FS_DIG0: Loop BW in RX and TX is 500 kHz
        // FS_CAL0.LOCK_CFG: Infinite out of lock detector average time
        0x07,                   // 0x12 FS_DIG1 = 0x07, reserved for TI SmartRF Studio
        0xAF,                   // 0x13 FS_DIG0.RX_LPF_BW = 0x11, FS_DIG0.TX_LPF_BW = 0x11
        0x00, 0x00,             // 0x14 FS_CAL3, 0x15 FS_CAL2
        0x40,                   // 0x16 FS_CAL1 = 0x40, reserved for TI SmartRF Studio
        0x0E,                   // 0x17 FS_CAL0.LOCK_CFG = 0x11
        0x00,                   // 0x18 FS_CHP
        // FS_RESERVED for TI SmartRF Studio
        0x03,                   // 0x19 FS_DIVTWO = 0x03, Divider
        0x00,                   // 0x1A FS_DSM1
        0x33,                   // 0x1B FS_DSM0 = 0x33, Digital Synthesizer Module
        0x00,                   // 0x1C FS_DVC1
        0x17,                   // 0x1D FS_DVC0 = 0x17, Divider Chain
        0x00,                   // 0x1E FS_LBI
        0x00,                   // 0x1F FS_PFD = 0x00, Phase Frequency Detector
        0x6E,                   // 0x20 FS_PRE = 0x6E, Prescaler
        0x1C,                   // 0x21 FS_REG_DIV_CML = 0x1C, Divider Regulator
        0xAC,                   // 0x22 FS_SPARE = 0xAC
        0x00, 0x00, 0x00, 0x00, // 0x23-0x26 FS_VCO
        0xB5,                   // 0x27 FS_VCO0 = 0xB5
        0x00, 0x00, 0x00, 0x00, // 0x28-0x2E GBIAS
        0x00, 0x00, 0x00,       // 
        // Intermediate Frequency Amplifier Configuration
        // Single-Side BW > f_if + RX_FILTER_BW/2= 300 kHz
        0x00,                   // 0x2F IFAMP.IFAMP_BW = 00
        0x00, 0x00,             // 0x30 LNA, 0x31 RXMIX
        // Crystal Oscillator Configuration
        // XOSC_BUF_SEL: Select low phase noise, differential buffer (low power buffer still used for digital clock)
        // XOSC_STABLE: Set XOSC is stable (has finished setting)
        0x0E,                   // 0x32 XOSC5 = 0x0E, reserved for TI SmartRF Studio
        0x00, 0x00, 0x00,       // 0x33-0x35 XOSC
        0x03,                   // 0x36 XOSC1.XOSC_BUF_SEL = 0b1, XOSC1.XOSC_STABLE = 0b1
        0x00,                   // 0x37 XOSC0
        0x00, 0x00              // 0x38 ANALOG_SPARE, 0x39 PA_CFG3
	}
};
// fifo 128 BYTES, 9600 Baud:

// 24k baud / 16 = 1.5kbaud

void CC1200::init()
{
    // TODO: Utilize a ready signal from User Guide Table 10, two (2.7?) XOSC periods
    // long, or get_status().
    // power_on_and_setup(dev);
    // for power on and setup, if these command strobes cannot be sent.
    // the device has likely entered a state where it cannot be initialized.
    if (!instruct_sidle(dev) ||
	    !instruct_sftx(dev) ||
	    !instruct_sfrx(dev) ||
	    rf_calibrate(dev)) {
		LOG_ERR("Could not proceed");
		return -EIO;
	}

    /** 
     * 8.9.2 Transparent Serial Mode Configuration (Contd.)
     * The Zephyr 802.15.4 driver arbitrated on IOCFG values and serial status.
     * IOCFGx is manually set again to reflect GPIO0 as CFM GPIO2 as the clock.
    */ 
    write_reg_iocfg3(0x00);        // 0x00 IOCFG3
    write_reg_iocfg2(0x08);        // 0x01 IOCFG2.GPIO2_CFG = SERIAL_CLK
    write_reg_iocfg1(0x00);        // 0x02 IOCFG1
    write_reg_iocfg0(0x09);        // 0x03 IOCFG0.GPIO0_CFG = SERIAL_RX
    // IOC_SYNC_PINS_EN: Enable synchronizer for IO pins.
    // Required for transparent TX and for reading GPIO_STATUS.GPIO_STATE
    write_reg_serial_status(0x08); // 0x91 SERIAL_STATUS.IOC_SYNC_PINS_EN = 1

    cc1200_init(dev);
    setFuncMode(CC1200::RX);
    setOpMode(CC1200::CFM);
}

void CC1200::setFuncMode(const enum mode)
{
    // Send the     
    switch (mode)
    {
        case IDLE:
            instruct_sidle(dev);
            break;
        case RX:
            instruct_sfrx(dev);
            break;
        case TX:
            instruct_sftx(dev);
            break;
        default:
            break;
    }
}

void CC1200::setOpMode(const enum opmode mode)
{
    switch (mode)
    {
        case CFM:
            write_reg_ext_ctrl(BURST_ADDR_INCR_EN)
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


