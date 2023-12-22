#include <peripherals/gpio.h>
#include <interfaces/delays.h>
#include <zephyr/drivers/spi.h>
#include <hwconfig.h>
#include "CC120x.h"


/*

https://wiki.m17project.org/cc1200

The key is reading/writing to the CFM_TX_DATA_IN / CFM_TX_DATA_OUT registers at a rate of 48kHz.

*/

#define SPIBB_NODE	DT_NODELABEL(spibb0) // spi baseband controller 0
const struct device *const dev = DEVICE_DT_GET(SPIBB_NODE);


void CC1200::init()
{
    // Give it a few milliseconds to power up, then send a reset command - 0x30 (1-byte write)

    // TODO: Utilize a ready signal from User Guide Table 10, two XOSC periods long.
    delayMs(5); // power-up delay
    
    // tx_data[0] = 0x30|0xC0;  // 0x30 is the reset command, and 0xC0 sets burst read mode for safety
    // tx_data[1] = 0x7E;       // lower bits (register address) set to 0x7E
    // tx_data[2] = 0x00;       // no specific register within the extended space


    // tx_data[0] = 0x2F|0x40; // burst-write mode on extended registers
    // tx_data[1] = 0x7E;      // lower bits indicating register address
    // tx_data[2] = *((uint8_t*)&baseband); // raw audio in rate/2 = 24k int8_t (see wiki sample: `fprintf(fout, "int8_t baseband[24000]=\n{\n");`)
    
    //CS low
    
    // 2-byte SPI transfer

    // Wait 100ms - probably the delay can be a lot less than that
    delayMs(100);
    // Use the config below to set up CC1200
    
    // Send either 0x34 or 0x35 command (1-byte) to enable RX or TX mode
    
    // If necessary (eg. using a simple XO instead of proper TCXO, caesium reference or hydrogen maser) - apply frequency compensation by writing int16_t value to register 0x2F0A. See the code at the end of this paragraph.
    
    // Disable auto address increment for baseband SPI data transfer - write 0 to register 0x2F06
    
    // Start sending baseband samples in int8_t format to the 0x2F7E register - don’t pull SPI_CS line high after the first baseband byte, do it after all bytes are sent. First, send a 3-byte “start”:

}

void CC1200::setFuncMode()
{
    
}

void CC1200::setOpMode()
{

}

int16_t CC1200::readBufferedData()
{

}

void CC1200::writeBufferedData()
{

}