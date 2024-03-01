#include <hwconfig.h>
#include <interfaces/delays.h>

#include "CC120x.h"

/*

https://wiki.m17project.org/cc1200

The key is reading/writing to the CFM_TX_DATA_IN / CFM_TX_DATA_OUT registers at
a rate of 48kHz.

*/

const struct device* const dev = DEVICE_DT_GET(cc1200);

#define STACK_SIZE 512
#define FRAME_SIZE (16)
#define COMMAND_SIZE 3
#define CONFIGURATION_SIZE 51 * COMMAND_SIZE

/**
 * For TX mode, deviation control registers (DEVIATION_M and MODCFG_DEV_E) are
 * set for about 5kHz. For RX they are set to about 3kHz. Symbol rate is set to
 * 24k for TX and 1.2k for RX. Nobody knows why these settings work :) RX BW is
 * about 9.5kHz. CFM is enabled in both modes. AFC can be enabled in RX mode by
 * writing to register 0x2F01. Via M17-Project
 */

const struct cc1200_rf_registers_set cc1200_rf_settings = {
    .chan_center_freq0 = 445,
    // .channel_limit = 38, might be important?
    .channel_spacing = 95, /* 200 KHz */
    .registers       = {
        0x1F,
        0x9F,  // deviation - about 3kHz full scale
        0x00,  // deviation
        0x5D, 0x00, 0x8A, 0xCB,
        0xAC,  // RX filter BW - 9.5kHz
        0x09, 0x00, 0x45,
        0x3F,  // symbol rate 2 - 1.2k sym/s
        0x75,  // symbol rate 1
        0x10,  // symbol rate 0
        0x37, 0xEC, 0x11, 0x51, 0x87, 0x08, 0x00, 0x14, 0x03, 0x00, 0x20,
        0x03,  // output power - 0x03..0x3F (doesn't matter for RX)
        0xFF, 0x1C,
        0x02,  // AFC, 0x22 - on, 0x02 - off
        0x0C,  // external oscillator's frequency is 40 MHz
        0x09,  // 16x upsampler, CFM enable
        0x57,  // frequency - round((float)435000000/5000000*(1<<16))=0x570000
        0x00,  // frequency
        0x00,  // frequency
        0xEE, 0x10, 0x07, 0xAF, 0x40, 0x0E, 0x03, 0x33, 0x17, 0x00, 0x6E,
        0x1C, 0xAC, 0xB5, 0x0E, 0x03, 0x08}};

void CC1200::init()
{
    // TODO: Utilize a ready signal from User Guide Table 10, two XOSC periods
    // long, or get_status().
    power_on_and_setup(dev);
    cc1200_init(dev);
    setFuncMode(CC1200::RX);
    setOpMode(CC1200::CFM);
}

void CC1200::setFuncMode(const enum mode)
{
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
        uint16_t rssi_raw = (uint16_t)(RSSI_11_4 << 4) | RSSI_3_0;
        // AGC gain adjustment. This register is used to adjust RSSI[11:0] to
        // the actual carrier input signal level to compensate for interpolation
        // gains (two's complement with 1 dB resolution)
        int16_t offset = read_reg_agc_gain_adjust(dev);
        // To get a correct RSSI value a calibrated RSSI offset value should be
        // subtracted from the value given by RSSI[11:0].
        float rssi = (float)(rssi_raw - offset);
        // Put RSSI in units of dBm with a resolution of 0.0625 dB (1 tick)
        rssi *= 0.0625f;
        return rssi;
    }
    return -128.0f;  // invalid RSSI (dBm)
}
