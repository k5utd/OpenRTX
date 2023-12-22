/***************************************************************************
 *   Copyright (C) 2023 by Federico Amedeo Izzo IU2NUO,                    *
 *                         Niccolò Izzo IU2KIN                             *
 *                         Frederik Saraci IU2NRO                          *
 *                         Silvano Seva IU2KWO                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#include <interfaces/radio.h>
#include "CC120x.h"

CC1200& cc1200 = CC1200::instance();

void radio_init(const rtxStatus_t *rtxState)
{
    (void) rtxState;
    cc1200.init();
}

void radio_terminate()
{
    radio_disableRtx();
    cc1200.terminate();
}

void radio_tuneVcxo(const int16_t vhfOffset, const int16_t uhfOffset)
{
    (void) vhfOffset;
    (void) uhfOffset;
}

void radio_setOpmode(const enum opmode mode)
{
    (void) mode;
    switch(mode)
    {
        case OPMODE_CFM:
            cc1200.setOpMode(CC1200_OpMode::CFM);  // CC1200 in CFM mode
            break;

        // case OPMODE_DMR:
        //     cc1200.setOpMode(CC1200_OpMode::DMR);
        //     cc1200.setBandwidth(CC1200_BW::_12P5);
        //     break;

        case OPMODE_M17:
            cc1200.setOpMode(CC1200_OpMode::CFM); 
            cc1200.setBandwidth(CC1200_BW::_9P5);  // Set bandwidth to 9.5kHz for proper deviation
            break;

        default:
            break;
    }
}

bool radio_checkRxDigitalSquelch()
{
    return false;
}

void radio_enableRx()
{

}

void radio_enableTx()
{

}

void radio_disableRtx()
{

}

void radio_updateConfiguration()
{

}

// Per 6.9 RSSI,
// The RSSI is a 12 bits two's complement number with 0.0625 dB
// resolution hence ranging from –128 to 127 dBm (-128 is invalid)
float radio_getRssi()
{
    return -65.0f;  // S1 level: -65dBm
}

enum opstatus radio_getStatus()
{
    return OFF;
}
