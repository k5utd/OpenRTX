#ifndef CC1200_H
#define CC1200_H

#include <stdint.h>
#include <stdbool.h>
#include <datatypes.h>
#include <zephyr/drivers/spi.h> // https://docs.zephyrproject.org/latest/samples/drivers/spi_bitbang/README.html#spi-bitbang
#ifdef __cplusplus
extern "C" {
#endif

/**
 * Note: This driver does not include Legacy mode functionality.
 * 
 * CC1200 Datasheet: https://www.ti.com/lit/ds/symlink/cc1200.pdf
 * CC1200 User's Guide: https://www.ti.com/lit/ug/swru346b/swru346b.pdf
 * 
 * Here are the key parameters programmed over the SPI interface:
 *     Power-down/power-up mode (SLEEP/IDLE)
 *     Crystal oscillator power-up/power-down (IDLE/XOFF)
 *     Receive/transmit mode (RX/TX)
 *     Carrier frequency
 *     Symbol rate
 *     Modulation format
 *     RX channel filter bandwidth
 *     RF output power
 *     Data buffering with separate 128-byte receive and transmit FIFOs
 *     Packet radio hardware support
 *     Data whitening
 *     Enhanced Wake-On-Radio (eWOR)
 */

/**
 * Enumeration type defining the bandwidth settings supported by the CC1200 chip.
 * ISM/SRD Bands: 169, 433, 868, 915, and 920 MHz
 * Possible Support for Additional Frequency Bands:
 * 137-158.3 MHz, 205-237.5 MHz, and 274-316.6 MHz
 * See Table 34 for the supported Band Selections.
 * The frequency programming should only be updated when the radio is in the IDLE state.
 */
enum class CC1200_BW : uint8_t
{
    _9P5  = 0,    ///< 9.5kHz bandwidth. Supposedly used in M17-Project experiments with EMK.
    //FS_CFG.FSD_BANDSELECT, LO Divider, RF Band (RF Programming Resolution)
    _140P0 = 1, // 0010,  4,   820-960 Mhz (38.1 Hz)
    _070P0 = 2, // 0100,  8,   410-480 Mhz (19.1 Hz)
    _046P7 = 3, // 0110, 12, 273.3-320 Mhz (12.7 Hz)
    _035P0 = 4, // 1000, 16,   205-240 Mhz (09.5 Hz)
    _028P0 = 5, // 1010, 20,   164-192 Mhz (07.6 Hz)
    _023P3 = 6, // 1011, 24, 136.7-160 Mhz (06.4 Hz)
};

/**
 * Enumeration type defining the possible operating mode configurations for the
 * CC1200 chip.
 */
enum class CC1200_OpMode : uint8_t
{
    CFM     = 0,      ///< N-FSK, Custom Frequency Modulation(CFM)/Analog FM.
    _2_FSK  = 1,      ///< 2-FSK
    _2_GFSK = 2,      ///< 2-GFSK
    _4_FSK  = 3,      ///< 4-FSK
    _4_GFSK = 4       ///< 4-GFSK
};

/**
 * Enumeration type defining the CC1200 functional modes. RX, TX, SLEEP, IDLE
 */
enum class CC1200_FuncMode : uint8_t
{
    IDLE = 0,      ///< Default state, both TX and RX off, not in SLEEP.
    RX  = 1,       ///< RX enabled.
    TX  = 2,       ///< TX enabled.
};

// Table 4: SPI Address Space
enum class CC1200Register : uint8_t
{
    // R/W configuration registers, burst access possible
    // #        Write        #         Read        #
    // # Single Byte # Burst # Single Byte # Burst #
    // #       +0x00 # +0x40 #       +0x80 # +0xC0 #
    IOCFG3 = 0x00,
    IOCFG2 = 0x01,
    IOCFG1 = 0x02,
    IOCFG0 = 0x03,
    SYNC3 = 0x04,
    SYNC2 = 0x05,
    SYNC1 = 0x06,
    SYNC0 = 0x07,
    SYNC_CFG1 = 0x08,
    SYNC_CFG0 = 0x09,
    DEVIATION_M = 0x0A,
    MODCFG_DEV_E = 0x0B,
    DCFILT_CFG = 0x0C,
    PREAMBLE_CFG1 = 0x0D,
    PREAMBLE_CFG0 = 0x0E,
    IQIC = 0x0F,
    CHAN_BW = 0x10,
    MDMCFG1 = 0x11,
    MDMCFG0 = 0x12,
    SYMBOL_RATE2 = 0x13,
    SYMBOL_RATE1 = 0x14,
    SYMBOL_RATE0 = 0x15,
    AGC_REF = 0x16,
    AGC_CS_THR = 0x17,
    AGC_GAIN_ADJUST = 0x18,
    AGC_CFG3 = 0x19,
    AGC_CFG2 = 0x1A,
    AGC_CFG1 = 0x1B,
    AGC_CFG0 = 0x1C,
    FIFO_CFG = 0x1D,
    DEV_ADDR = 0x1E,
    SETTLING_CFG = 0x1F,
    FS_CFG = 0x20,
    WOR_CFG1 = 0x21,
    WOR_CFG0 = 0x22,
    WOR_EVENT0_MSB = 0x23,
    WOR_EVENT0_LSB = 0x24,
    RXDCM_TIME = 0x25,
    PKT_CFG2 = 0x26,
    PKT_CFG1 = 0x27,
    PKT_CFG0 = 0x28,
    RFEND_CFG1 = 0x29,
    RFEND_CFG0 = 0x2A,
    PA_CFG1 = 0x2B,
    PA_CFG0 = 0x2C,
    ASK_CFG = 0x2D,
    PKT_LEN = 0x2E,
    EAC = 0x2F, // Extended Register Space Command
    // Command Strobes registers
    SRES = 0x30,
    SFSTXON = 0x31,
    SXOFF = 0x32,
    SCAL = 0x33,
    SRX = 0x34,
    STX = 0x35,
    SIDLE = 0x36,
    SAFC = 0x37,
    SWOR = 0x38,
    SPWD = 0x39,
    SFRX = 0x3A,
    SFTX = 0x3B,
    SWORRST = 0x3C,
    SNOP = 0x3D,
    DMA = 0x3E, // Direct Memory Access Command
    TX_FIFO = 0x3F,  // TX FIFO
    RX_FIFO = 0x3F   // RX FIFO
};
// Table 5: Extended Register Space Mapping
enum class CC1200ExtendedRegister : uint8_t
{
    IF_MIX_CFG = 0x00,
    FREQOFF_CFG = 0x01,
    TOC_CFG = 0x02,
    MARC_SPARE = 0x03,
    ECG_CFG = 0x04,
    MDMCFG2 = 0x05,
    EXT_CTRL = 0x06,
    RCCAL_FINE = 0x07,
    RCCAL_COARSE = 0x08,
    RCCAL_OFFSET = 0x09,
    FREQOFF1 = 0x0A,
    FREQOFF0 = 0x0B,
    FREQ2 = 0x0C,
    FREQ1 = 0x0D,
    FREQ0 = 0x0E,
    IF_ADC2 = 0x0F,
    IF_ADC1 = 0x10,
    IF_ADC0 = 0x11,
    FS_DIG1 = 0x12,
    FS_DIG0 = 0x13,
    FS_CAL3 = 0x14,
    FS_CAL2 = 0x15,
    FS_CAL1 = 0x16,
    FS_CAL0 = 0x17,
    FS_CHP = 0x18,
    FS_DIVTWO = 0x19,
    FS_DSM1 = 0x1A,
    FS_DSM0 = 0x1B,
    FS_DVC1 = 0x1C,
    FS_DVC0 = 0x1D,
    FS_LBI = 0x1E,
    FS_PFD = 0x1F,
    FS_PRE = 0x20,
    FS_REG_DIV_CML = 0x21,
    FS_SPARE = 0x22,
    FS_VCO4 = 0x23,
    FS_VCO3 = 0x24,
    FS_VCO2 = 0x25,
    FS_VCO1 = 0x26,
    FS_VCO0 = 0x27,
    GBIAS6 = 0x28,
    GBIAS5 = 0x29,
    GBIAS4 = 0x2A,
    GBIAS3 = 0x2B,
    GBIAS2 = 0x2C,
    GBIAS1 = 0x2D,
    GBIAS0 = 0x2E,
    IFAMP = 0x2F,
    LNA = 0x30,
    RXMIX = 0x31,
    XOSC5 = 0x32,
    XOSC4 = 0x33,
    XOSC3 = 0x34,
    XOSC2 = 0x35,
    XOSC1 = 0x36,
    XOSC0 = 0x37,
    ANALOG_SPARE = 0x38,
    PA_CFG3 = 0x39,
    // Not Used: 0x3A - 0x3E
    // Reserved: 0x3F - 0x40
    // Not Used: 0x41 - 0x63
    WOR_TIME1 = 0x64,
    WOR_TIME0 = 0x65,
    WOR_CAPTURE1 = 0x66,
    WOR_CAPTURE0 = 0x67,
    BIST = 0x68,
    DCFILTOFFSET_I1 = 0x69,
    DCFILTOFFSET_I0 = 0x6A,
    DCFILTOFFSET_Q1 = 0x6B,
    DCFILTOFFSET_Q0 = 0x6C,
    IQIE_I1 = 0x6D,
    IQIE_I0 = 0x6E,
    IQIE_Q1 = 0x6F,
    IQIE_Q0 = 0x70,
    RSSI1 = 0x71,
    RSSI0 = 0x72,
    MARCSTATE = 0x73,
    LQI_VAL = 0x74,
    PQT_SYNC_ERR = 0x75,
    DEM_STATUS = 0x76,
    FREQOFF_EST1 = 0x77,
    FREQOFF_EST0 = 0x78,
    AGC_GAIN3 = 0x79,
    AGC_GAIN2 = 0x7A,
    AGC_GAIN1 = 0x7B,
    AGC_GAIN0 = 0x7C,
    CFM_RX_DATA_OUT = 0x7D,
    CFM_TX_DATA_IN = 0x7E,
    ASK_SOFT_RX_DATA = 0x7F,
    RNDGEN = 0x80,
    MAGN2 = 0x81,
    MAGN1 = 0x82,
    MAGN0 = 0x83,
    ANG1 = 0x84,
    ANG0 = 0x85,
    CHFILT_I2 = 0x86,
    CHFILT_I1 = 0x87,
    CHFILT_I0 = 0x88,
    CHFILT_Q2 = 0x89,
    CHFILT_Q1 = 0x8A,
    CHFILT_Q0 = 0x8B,
    GPIO_STATUS = 0x8C,
    FSCAL_CTRL = 0x8D,
    PHASE_ADJUST = 0x8E,
    PARTNUMBER = 0x8F,
    PARTVERSION = 0x90,
    SERIAL_STATUS = 0x91,
    MODEM_STATUS1 = 0x92,
    MODEM_STATUS0 = 0x93,
    MARC_STATUS1 = 0x94,
    MARC_STATUS0 = 0x95,
    PA_IFAMP_TEST = 0x96,
    FSRF_TEST = 0x97,
    PRE_TEST = 0x98,
    PRE_OVR = 0x99,
    ADC_TEST = 0x9A,
    DVC_TEST = 0x9B,
    ATEST = 0x9C,
    ATEST_LVDS = 0x9D,
    ATEST_MODE = 0x9E,
    XOSC_TEST1 = 0x9F,
    XOSC_TEST0 = 0xA0,
    AES = 0xA1,
    MDM_TEST = 0xA2,
    // Not Used: 0xA3 - 0xD1
    RXFIRST = 0xD2,
    TXFIRST = 0xD3,
    RXLAST = 0xD4,
    TXLAST = 0xD5,
    NUM_TXBYTES = 0xD6,
    NUM_RXBYTES = 0xD7,
    FIFO_NUM_TXBYTES = 0xD8,
    FIFO_NUM_RXBYTES = 0xD9,
    RXFIFO_PRE_BUF = 0xDA,
    // Not Used: 0xDB - 0xDF
    // Not Used: 0xE0 - 0xFF (AES Workspace)
    // Registers from 0xE0 to 0xEF are AES_KEY
    // Registers from 0xF0 to 0xFF are AES_BUFFER
};

class CC1200
{
public:
    /**
     * \return a reference to the instance of the CC1200 class (singleton).
     */
    static CC1200& instance()
    {
        static CC1200 instance;
        return instance;
    }

    /**
     * Configure the CC120x.
     * 
     * The configuration registers on the CC120x
     * are located on SPI addresses from 0x00 to 0x2E.
     */
    void init();

    // Regarding the sample config. On the RX filter bandwidth, 9.5kHz was used. 10101100 -> ADC_CIC_DECFACT=10 (Decimation factor 48), and BB_CIC_DECFACT=101100 (44)
    /**
     * Program CC120x into different modes (RX, TX, SLEEP, IDLE, etc)
     *
     * @param mode: name of the mode to switch to.
     * 
     * It should be noted that the chip status byte is sent on the SO pin. 
     */
    inline void setFuncMode();
    /**
     * Program CC120x into different modes (RX, TX, SLEEP, IDLE, etc)
     *
     * @param mode: name of the mode to switch to.
     * 
     * It should be noted that the chip status byte is sent on the SO pin. 
     */
    inline void setOpMode();


    /**
     * Read and write buffered data (RX FIFO and TX FIFO)
     *
     */
    inline int8_t readBufferedData();

    inline void writeBufferedData();


    int readStatus()
    {

    }

    /**
     * Get current RSSI value.
     *
     * @return current RSSI in dBm.
     *
     * The RSSI can be found by reading RSSI1.RSSI_11_4 and RSSI0.RSSI_3_0.
     * It should beted that for most applications using the 8 MSB bits
     * of the RSSI, with 1 dB resolution, is good enough. 
     * Example:
     *   Assume a -65 dBm signal into the antenna and RSSI[11:0] = 0x220 (34) when
     *   AGC_GAIN_ADJUST.GAIN_ADJUSTMENT = 0x00.
     *   This means that the offset is −99 dB as 34 dBm + (–99) dB = –65 dBm.
     *   When the offset is known it can be written to the AGC_GAIN_ADJUST.GAIN_ADJUSTMENT
     *   register field (GAIN_ADJUSTMENT = 0x9D (−99)). When the same signal is input to the antenna,
     *   the RSSI[11:0] register will be 0xBF0 (–65)
     */
    float readRSSI()
    {
        // 0x71-0x72 RSSI1, RSSI0
        //int16_t RSSI1 = SPI_readReg16(CC1200ExtendedRegister::RSSI1);



        //int ret = spi_transceive(dev);


        return 0.0f;
    }



private:

};


#ifdef __cplusplus
}
#endif


#endif // CC1200_H