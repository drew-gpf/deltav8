//! clock.zig: Clock configuration for power saving
//! Copyright (C) 2021 Drew P.

//! This program is free software; you can redistribute it and/or modify
//! it under the terms of the GNU General Public License as published by
//! the Free Software Foundation; either version 2 of the License, or
//! (at your option) any later version.

//! This program is distributed in the hope that it will be useful,
//! but WITHOUT ANY WARRANTY; without even the implied warranty of
//! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//! GNU General Public License for more details.

//! You should have received a copy of the GNU General Public License along
//! with this program; if not, write to the Free Software Foundation, Inc.,
//! 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

const c = @cImport({
    @cInclude("pico/stdlib.h");
    @cInclude("hardware/clocks.h");
    @cInclude("hardware/pll.h");
});

const logger = @import("logger.zig");
const uart = @import("uart.zig");

/// zig cant into typecast macros. the _u macro also just doesnt work
/// #define clocks_hw ((clocks_hw_t *const)CLOCKS_BASE)
/// #define CLOCKS_BASE _u(0x40008000)
const clocks_hw = @intToPtr(*c.clocks_hw_t, 0x40008000);

/// ...
/// #define pll_usb pll_usb_hw
/// #define pll_usb_hw ((pll_hw_t *const)PLL_USB_BASE)
/// #define PLL_USB_BASE _u(0x4002c000)
const pll_usb = @intToPtr(*c.pll_hw_t, 0x4002c000);

/// Configure (disable) some clocks we won't be using.
/// This will always turn off the ADC clock.
/// If stdio is disabled, it will turn off the USB clock and most clocks when the device is sleeping.
pub fn configureClocks() void {
    // Turn off ADC clocks always to save power
    c.clock_stop(c.clk_rtc);
    c.clock_stop(c.clk_adc);

    if (!logger.stdio_enabled) {
        // We don't need stdio (which is only over USB), so disable the USB clock as well.
        c.clock_stop(c.clk_usb);
        c.pll_deinit(pll_usb);

        // todo: this breaks things, when everything wired find a bitmask that works.
        // Currently we only use UART so as that subsystem for clock bits to enable when sleeping
        //const sleep_en_bitmask = uart.getUartSleepEn();

        // Configure clocks based on returned constant. We don't use deep sleep mode due to latency concerns.
        //clocks_hw.sleep_en0 = @truncate(u32, sleep_en_bitmask);
        //clocks_hw.sleep_en1 = @truncate(u32, sleep_en_bitmask >> 32);
    }
}