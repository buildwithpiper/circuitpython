/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Keith Evans
 * Based on PewPew
 * Copyright (c) 2019 Radomir Dopieralski
 *
 * Port for RP2040 multicore
 * Copyright (c) 2021 Matthew Matz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdbool.h>

#if defined(PICO_BOARD) || defined(__RP2040__)

#define HAS_GPIO_COPROCESSOR        0   // resolve warnings when building for the RP2040
#define PICO_USE_GPIO_COPROCESSOR   0
#define PICO_SECURE                 0

#include "ports/raspberrypi/sdk/src/rp2_common/hardware_spi/include/hardware/spi.h"
#include "ports/raspberrypi/sdk/src/rp2_common/hardware_gpio/include/hardware/gpio.h"

#else
#include "shared-bindings/digitalio/DigitalInOut.h"

#endif

#include "py/mpstate.h"
#include "__init__.h"
#include "LIGHTSHOW.h"


// Array
#define ROW_SIZE        8
#define COL_SIZE        8

// 2-bits per pixel
#define RED_MASK        0x30
#define GREEN_MASK      0x0C
#define BLUE_MASK       0x03

#define RED_SHIFT       4
#define GREEN_SHIFT     2
#define BLUE_SHIFT      0

// SPI data packet ordering
#define RED_OFFSET      0   // Active-low
#define BLUE_OFFSET     1   // Active-low
#define GREEN_OFFSET    2   // Active-low
#define COL_OFFSET      3   // Active-high

// We'll handle brightness based on the output of cie1931.py
// (see https://jared.geek.nz/2013/feb/linear-led-pwm)
// with INPUT_SIZE = 3 and OUTPUT_SIZE = 8.

// Go with short PWM sequence. Was usingthe output of cie1931.py
// (see https://jared.geek.nz/2013/feb/linear-led-pwm)
// with INPUT_SIZE = 3 and OUTPUT_SIZE = 16, but after changing
// to a single pixel scan this resulted in too much flicker.
//
// Level 3 : 8 frames on, 0 frames off
// Level 2 : 3 frames on, 5 frames off
// Level 1 : 1 frames on, 7 frames off
// Level 0 : 0 frames on, 8 frames off
//
// Note that this is somewhat complicated by the fact that we
// don't want to completely consume the processor. So we'll have
// some idle time where the LEDs will be off. We can adjust this
// based on the needs of the user code and background processing.
// Also will colors with multiple components active appear brighter?
// Need to revisit this.
//
#define FRAME_MASK      0x7
#define COLOR_MASK      0x3

#if defined(PICO_BOARD) || defined(__RP2040__)
void write_four_bytes_to_spi(uint _cs, spi_inst_t *_spi, uint8_t *bytes_out) {
    gpio_put(_cs, 0);
    spi_write_blocking(_spi, bytes_out, 4);
    gpio_put(_cs, 1);
}
#else
void write_four_bytes_to_spi(digitalio_digitalinout_obj_t *_cs, busio_spi_obj_t *_spi, uint8_t *bytes_out) {
    common_hal_digitalio_digitalinout_set_value(_cs, false);
    common_hal_busio_spi_write(_spi, bytes_out, 4);
    common_hal_digitalio_digitalinout_set_value(_cs, true);
}
#endif

void lightshow_tick(void) {

    lightshow_obj_t *lightshow = MP_STATE_VM(lightshow_singleton);
    if (!lightshow) {
        return;
    }

    uint8_t spi_packet[4];
    static uint8_t frame_level[] = {1, 2, 2, 3, 3, 3, 3, 3};
    static uint8_t frame_cnt = 0;

    #if defined(PICO_BOARD) || defined(__RP2040__)
    // Pins
    const uint spi_cs = 17;
    const uint sck_pin = 18;
    const uint mosi_pin = 19;
    // const uint miso_pin = 16;

    // Ports
    spi_inst_t *spi = spi0;

    // Initialize CS pin high
    gpio_init(spi_cs);
    gpio_set_dir(spi_cs, GPIO_OUT);
    gpio_put(spi_cs, 1);

    // Initialize SPI port at 8 MHz
    spi_init(spi, 8000 * 1000);

    // Set SPI format
    spi_set_format(spi0,    // SPI instance
        8,                  // Number of bits per transfer
        1,                  // Polarity (CPOL)
        1,                  // Phase (CPHA)
        SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    // gpio_set_function(miso_pin, GPIO_FUNC_SPI);

    #else
    busio_spi_obj_t *spi;
    digitalio_digitalinout_obj_t *spi_cs;

    spi = MP_OBJ_TO_PTR(lightshow->spi);
    spi_cs = MP_OBJ_TO_PTR(lightshow->chip_select);

    #endif

    #if defined(PICO_BOARD) || defined(__RP2040__)
    while (1) {
    #endif

    for (uint8_t col = 0 ; col < COL_SIZE ; col++) {
        spi_packet[COL_OFFSET] = (1 << col);

        for (uint8_t row = 0 ; row < ROW_SIZE ; row++) {
            uint8_t red_value = 0xff;
            uint8_t green_value = 0xff;
            uint8_t blue_value = 0xff;

            uint8_t color = lightshow->buffer[row * ROW_SIZE + col];

            // We're using upper left-hand corner as the origin, change to
            // ~(0x01 << row) for lower-left origin
            //
            uint8_t on_value = ~(0x80 >> row);

            if (((color >> RED_SHIFT) & COLOR_MASK) >= frame_level[frame_cnt]) {
                red_value = on_value;
            }
            if (((color >> GREEN_SHIFT) & COLOR_MASK) >= frame_level[frame_cnt]) {
                green_value = on_value;
            }
            if (((color >> BLUE_SHIFT) & COLOR_MASK) >= frame_level[frame_cnt]) {
                blue_value = on_value;
            }

            // Lighting a single LED at a time - when we illuminated an entire
            // column there seemed to be some current sharing which caused
            // artifacts for dynamic displays

            // Red
            spi_packet[RED_OFFSET] = red_value;
            spi_packet[GREEN_OFFSET] = 0xff;
            spi_packet[BLUE_OFFSET] = 0xff;
            write_four_bytes_to_spi(spi_cs, spi, spi_packet);

            // Green
            spi_packet[RED_OFFSET] = 0xff;
            spi_packet[GREEN_OFFSET] = green_value;
            write_four_bytes_to_spi(spi_cs, spi, spi_packet);

            // Blue
            spi_packet[GREEN_OFFSET] = 0xff;
            spi_packet[BLUE_OFFSET] = blue_value;
            write_four_bytes_to_spi(spi_cs, spi, spi_packet);
        }
    }

    // Turn off (otherwise last pixel will be brighter for one color)
    spi_packet[BLUE_OFFSET] = 0xff;
    spi_packet[COL_OFFSET] = 0x00;
    write_four_bytes_to_spi(spi_cs, spi, spi_packet);

    frame_cnt = (frame_cnt + 1) & FRAME_MASK;

    #if defined(PICO_BOARD) || defined(__RP2040__)
}      // END while loop
    #endif
}
