#ifndef HWCONFIG_H
#define HWCONFIG_H

#include <zephyr/device.h>

/*
 * Display properties are encoded in the devicetree
 */
#define DISPLAY       DT_CHOSEN(zephyr_display)
#define SCREEN_WIDTH  DT_PROP(DISPLAY, width)
#define SCREEN_HEIGHT DT_PROP(DISPLAY, height)

/* Screen pixel format */
#define PIX_FMT_BW

/* Device has no battery */
#define BAT_NONE
// TODO: DMR and HT Operation
// #define BAT_LIPO_1S

// TODO: Add GPS Module and Functionality
// #define GPS_PRESENT

#endif /* HWCONFIG_H */