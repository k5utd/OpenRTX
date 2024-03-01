#ifndef CC1200_H
#define CC1200_H

#include <datatypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/ieee802154/cc1200.h>
#include <zephyr/drivers/ieee802154/ieee802154_cc1200_regs.h>
#ifdef __cplusplus
extern "C" {
#endif

DEFINE_REG_READ(rssi1, CC1200_REG_RSSI1, true)
DEFINE_REG_READ(agc_gain_adjust, CC1200_REG_AGC_GAIN_ADJUST, true)

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
 * Enumeration type defining the bandwidth settings supported by the CC1200
 * chip. ISM/SRD Bands: 169, 433, 868, 915, and 920 MHz Possible Support for
 * Additional Frequency Bands: 137-158.3 MHz, 205-237.5 MHz, and 274-316.6 MHz
 * See Table 34 for the supported Band Selections.
 * The frequency programming should only be updated when the radio is in the
 * IDLE state.
 */
enum class CC1200_BW : uint8_t
{
    _9P5 = 0,  ///< 9.5kHz bandwidth. Supposedly used in M17-Project experiments
               ///< with EMK.
    // FS_CFG.FSD_BANDSELECT, LO Divider, RF Band (RF Programming Resolution)
    _140P0 = 1,  // 0010,  4,   820-960 Mhz (38.1 Hz)
    _070P0 = 2,  // 0100,  8,   410-480 Mhz (19.1 Hz)
    _046P7 = 3,  // 0110, 12, 273.3-320 Mhz (12.7 Hz)
    _035P0 = 4,  // 1000, 16,   205-240 Mhz (09.5 Hz)
    _028P0 = 5,  // 1010, 20,   164-192 Mhz (07.6 Hz)
    _023P3 = 6,  // 1011, 24, 136.7-160 Mhz (06.4 Hz)
};

/**
 * Enumeration type defining the possible operating mode configurations for the
 * CC1200 chip.
 * CFM -- MDMCFG2.CFM_DATA_EN = 1
 */
enum class CC1200_OpMode : uint8_t
{
    // CFM     = 0b0,  ///< N-FSK, Custom Frequency Modulation(CFM)/Analog FM.
    _2_FSK  = 0b000,                 ///< 000 2-FSK
    _2_GFSK = 0b001,                 ///< 001 2-GFSK
    ASK_OOK = 0b011 _4_FSK = 0b100,  ///< 100 4-FSK
    _4_GFSK                = 0b101   ///< 101 4-GFSK
};

/**
 * Enumeration type defining the CC1200 functional modes. RX, TX, SLEEP, IDLE
 */
enum class CC1200_FuncMode : uint8_t
{
    IDLE = 0,  ///< Default state, both TX and RX off, not in SLEEP.
    RX   = 1,  ///< RX enabled.
    TX   = 2,  ///< TX enabled.
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

    // Regarding the sample config. On the RX filter bandwidth, 9.5kHz was used.
    // 10101100 -> ADC_CIC_DECFACT=10 (Decimation factor 48), and
    // BB_CIC_DECFACT=101100 (44)
    /**
     * Program CC120x into different modes (RX, TX, SLEEP, IDLE, etc)
     *
     * @param mode: name of the mode to switch to.
     *
     * It should be noted that the chip status byte is sent on the SO pin.
     */
    inline void setFuncMode();
    /**
     * Program CC120x into different operational modes (N-FSK, 2-FSK, 2-GFSK,
     * 4-FSK, 4-GFSK)
     *
     * @param mode: name of the mode to switch to.
     *
     * It should be noted that the chip status byte is sent on the SO pin.
     */
    inline void setOpMode();

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
     *   Assume a -65 dBm signal into the antenna and RSSI[11:0] = 0x220 (34)
     * when AGC_GAIN_ADJUST.GAIN_ADJUSTMENT = 0x00. This means that the offset
     * is −99 dB as 34 dBm + (–99) dB = –65 dBm. When the offset is known it can
     * be written to the AGC_GAIN_ADJUST.GAIN_ADJUSTMENT register field
     * (GAIN_ADJUSTMENT = 0x9D (−99)). When the same signal is input to the
     * antenna, the RSSI[11:0] register will be 0xBF0 (–65)
     */
    float readRSSI()
    {
        // 0x71-0x72 RSSI1, RSSI0
        // int16_t RSSI1 = SPI_readReg16(CC1200ExtendedRegister::RSSI1);

        // int ret = spi_transceive(dev);

        return 0.0f;
    }

   private:
};

#ifdef __cplusplus
}
#endif

#endif  // CC1200_H
