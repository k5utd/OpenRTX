# SPDX-License-Identifier: GPL-3.0-or-later

#
# CMake configuration file for the T-TWR Plus OpenRTX target
#
cmake_minimum_required(VERSION 3.20.0)

target_sources(app
  PRIVATE
    ${OPENRTX_ROOT}/platform/mcu/ESP32C3/drivers/delays.c

    ${OPENRTX_ROOT}/platform/targets/Cobalt-Cowboy/platform.c
    # ${OPENRTX_ROOT}/platform/targets/Cobalt-Cowboy/pmu.cpp

    # ${OPENRTX_ROOT}/platform/drivers/display/SH1106_Cobalt-Cowboy.c
    # ${OPENRTX_ROOT}/platform/drivers/keyboard/keyboard_Cobalt-Cowboy.c
    ${OPENRTX_ROOT}/platform/drivers/baseband/radio_Cobalt-Cowboy.cpp
    ${OPENRTX_ROOT}/platform/drivers/baseband/CC120x/CC1200.cpp
    ${OPENRTX_ROOT}/platform/drivers/baseband/CC120x/main.cpp
    # ${OPENRTX_ROOT}/platform/drivers/audio/audio_Cobalt-Cowboy.c
    ${OPENRTX_ROOT}/platform/drivers/NVM/flash_zephyr.c
    # ${OPENRTX_ROOT}/platform/drivers/NVM/nvmem_Cobalt-Cowboy.c

    ${OPENRTX_ROOT}/platform/drivers/stubs/cps_io_stub.c
)

target_include_directories(app
  PRIVATE
    ${OPENRTX_ROOT}/platform/targets/ttwrplus

)

target_compile_definitions(app PRIVATE PLATFORM_COBALT_COWBOY CONFIG_SCREEN_BRIGHTNESS CONFIG_UI_NO_KEYBOARD)
