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

#include "py/mpstate.h"
#include "py/runtime.h"
#include "__init__.h"
#include "LIGHTSHOW.h"

#include "shared-bindings/digitalio/Pull.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "shared-bindings/util.h"

#if defined(SAMD21) || defined(SAMD51)
#include "samd/timers.h"
#include "timer_handler.h"
#endif

#if defined(PICO_BOARD) || defined(__RP2040__)
#include "pico/multicore.h"
#endif

#if defined(SAMD21) || defined(SAMD51)
static uint8_t lightshow_tc_index = 0xff;
#endif

void lightshow_interrupt_handler(uint8_t index) {
    #if defined(SAMD21) || defined(SAMD51)
    if (index != lightshow_tc_index) {
        return;
    }
    Tc *tc = tc_insts[index];
    if (!tc->COUNT16.INTFLAG.bit.MC0) {
        return;
    }

    lightshow_tick();

    // Clear the interrupt bit.
    tc->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    #endif
}

void lightshow_init() {
    #if defined(SAMD21) || defined(SAMD51)

    if (lightshow_tc_index == 0xff) {
        // Find a spare timer.
        uint8_t index = find_free_timer();
        if (index == 0xff) {
            mp_raise_RuntimeError(MP_ERROR_TEXT("All timers in use"));
        }
        Tc *tc = tc_insts[index];

        lightshow_tc_index = index;
        set_timer_handler(true, index, TC_HANDLER_LIGHTSHOW);

        // We use GCLK0 for SAMD21 and GCLK1 for SAMD51 because they both run
        // at 48mhz making our math the same across the boards.
        //
        // We'll divide this by (1024*94) to get 500 Hz
        // The time spent drawing the screen can be adjusted via the SPI clock.
        // With SCK at 8 MHz it's taking 1.51 ms to draw, which means we could
        // achieve 660 Hz max. So this will leave CircuitPython with 25% of the
        // time to do other things.
        //
        #ifdef SAMD21
        turn_on_clocks(true, index, 0);
        #endif
        #ifdef SAMD51
        turn_on_clocks(true, index, 1);
        #endif


        #ifdef SAMD21
        tc->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
            TC_CTRLA_PRESCALER_DIV1024 |
            TC_CTRLA_WAVEGEN_MFRQ;
        #endif
        #ifdef SAMD51
        tc_reset(tc);
        tc_set_enable(tc, false);
        tc->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16
            | TC_CTRLA_PRESCALER_DIV1024;
        tc->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
        #endif

        tc_set_enable(tc, true);
        tc->COUNT16.CC[0].reg = 93;

        // Clear our interrupt in case it was set earlier
        tc->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
        tc->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
        tc_enable_interrupts(lightshow_tc_index);
    }
    #endif

    // TO DO: PUT A *PROPER* RP2040 CHECK HERE
    #if defined(PICO_BOARD) || defined(__RP2040__)
    multicore_reset_core1();
    multicore_launch_core1(lightshow_tick);
    #endif
}

void lightshow_reset(void) {
    #if defined(SAMD21) || defined(SAMD51)
    if (lightshow_tc_index != 0xff) {
        tc_reset(tc_insts[lightshow_tc_index]);
        lightshow_tc_index = 0xff;
    }
    #endif

    // TO DO: PUT A *PROPER* RP2040 CHECK HERE
    #if defined(PICO_BOARD) || defined(__RP2040__)
    multicore_reset_core1();
    #endif

    MP_STATE_VM(lightshow_singleton) = NULL;
}
