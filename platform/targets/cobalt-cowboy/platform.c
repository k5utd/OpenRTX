#include <interfaces/platform.h>
#include <interfaces/delays.h>
#include <hwconfig.h>

#include <zephyr/drivers/gpio.h>
#include <CC1200.h>

/**
 * Standard interface for device-specific hardware.
 * This interface handles:
 * - LEDs
 * - Battery voltage read
 * - Microphone level
 * - Volume and channel selectors
 * - Screen backlight control
 */
#define BUTTON_PTT_NODE DT_NODELABEL(button_ptt)

static const struct gpio_dt_spec button_ptt = GPIO_DT_SPEC_GET_OR(BUTTON_PTT_NODE, gpios, {0});
static const struct device *const qdec_dev = DEVICE_DT_GET(DT_ALIAS(qdec0));

static hwInfo_t hwInfo =
{
    .name        = "cobalt-cowboy",
    .hw_version  = 0,
    .uhf_band    = 0,
    .vhf_band    = 0,
    .uhf_maxFreq = 470,
    .uhf_minFreq = 420,
    .vhf_maxFreq = 0,
    .vhf_minFreq = 0,
};

/**
 * This function handles device hardware initialization.
 * Usually called at power-on
 */
void platform_init()
{
    int ret = gpio_pin_configure_dt(&button_ptt, GPIO_INPUT);
    //dtsi defines SPI pins as MISO GPIO2, SCLK GPIO6, CSEL, GPIO10, MOSI GPIO7
    gpio_pin_configure_dt(&, GPIO_INPUT);
    // TODO: pmu.init(); 
    //For the time being make sure to return the CC1200 to IDLE state before powering off

    cc1200.init();
}

/**
 * This function handles device hardware de-initialization.
 * Usually called at power-down
 */
void platform_terminate()
{
    //return the CC1200 to idle state
    //terminate pmu
}

/**
 * This function reads and returns the current battery voltage in millivolt.
 */
// uint16_t platform_getVbat();

/**
 * This function reads and returns the current microphone input level as a
 * normalised value between 0 and 255.
 */
uint8_t platform_getMicLevel()
{
    return 0;
}

/**
 * This function reads and returns the current volume selector level as a
 * normalised value between 0 and 255.
 */
uint8_t platform_getVolumeLevel()
{
    return volume_level;
}

/**
 * This function reads and returns the current channel selector level.
 */
int8_t platform_getChSelector()
{
    int rc = sensor_sample_fetch(qdec_dev);
    if (rc != 0)
    {
        printk("Failed to fetch sample (%d)\n", rc);
        return 0;
    }

    struct sensor_value val;
    rc = sensor_channel_get(qdec_dev, SENSOR_CHAN_ROTATION, &val);
    if (rc != 0)
    {
        printk("Failed to get data (%d)\n", rc);
        return 0;
    }

    // The esp32-pcnt sensor returns a signed 16-bit value: we remap it into a
    // signed 8-bit one.
    return (int8_t) val.val1;
}

/**
 * This function reads and returns the current PTT status.
 */
bool platform_getPttStatus()
{
    return gpio_pin_get_dt(&button_ptt);
}

/**
 * This function reads and returns the current status of the power on/power off
 * button or switch.
 * @return true if power is enabled, false otherwise.
 */
bool platform_pwrButtonStatus()
{
    // Long press of the power on button triggers a shutdown
    uint8_t btnStatus = 0;
    if(btnStatus == 2)
        return false;

    return true;
}

/**
 * This function turns on the selected led.
 * @param led: which led to control
 */
void platform_ledOn(led_t led)
{
    ;
}

/**
 * This function turns off the selected led.
 * @param led: which led to control
 */
void platform_ledOff(led_t led);

/**
 * This function emits a tone of the specified frequency from the speaker.
 * @param freq: desired frequency
 */
void platform_beepStart(uint16_t freq);
{
    (void) freq;
}
/**
 * This function stops emitting a tone.
 */
void platform_beepStop()
{
    ;
}

/**
 * This function returns a pointer to a data structure containing all the
 * hardware information.
 * WARNING: calling code must ensure that free() is never called on the returned
 * pointer!
 * @return pointer to device's hardware information.
 */
const hwInfo_t *platform_getHwInfo()
{
    return &hwInfo;
}