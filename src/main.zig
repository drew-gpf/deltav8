//! main.zig: deltav8 entry point
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

const std = @import("std");
const logger = @import("logger.zig");
const uart = @import("uart.zig");
const intrin = @import("intrin.zig");
const clock = @import("clock.zig");

const c = @cImport({
    @cInclude("pico/stdlib.h");
    @cInclude("hardware/watchdog.h");
});

// Uncomment or change to enable logs for any build mode
//pub const log_level: std.log.Level = .debug;

/// stdlib log handler; no logging is done if stdio is disabled.
pub fn log(comptime level: std.log.Level, comptime scope: @TypeOf(.EnumLiteral), comptime format: []const u8, args: anytype) void {
    _ = scope;

    const prefix = "[" ++ @tagName(level) ++ "]: ";
    logger.log(prefix ++ format ++ "\n", args) catch {};
}

export fn main() void {
    // Configure a generous watchdog timer of 100ms in case something goes wrong.
    // This should be enough time for about 10 data packets while transmitting. During init it should be more than
    // enough time to init everything; although it only exists if the sensor and microcontroller are powered on at the same time
    // as the sensor needs to wait a bit before initializing.
    // During testing it seems the sensor needs ~450ms to initialize (minus the 50ms delay for a valid data packet)
    // so this will restart about 4 times. A reset is required as we will need to also reset the UART device itself.
    // If stdio is enabled we instead want a longer timeout like 1 second to prevent conflicts with print delays.
    // This is probably too long.
    c.watchdog_enable(if (logger.stdio_enabled) 1000 else 100, true);

    // Configure system clocks to save power. This must be updated if the RTC or ADC are used,
    // or if USB is used outside of stdio.
    // I should probably "figure out" (try and find out) which clocks need to be enabled
    //clock.configureClocks();
    logger.initLogger();
    
    // Init the sensor and the accompanying UART
    uart.init();

    // Magic sleep value of 50ms because the sensor's data is unreliable before this period,
    // where dist seems to be some value smaller than it actually should be.
    // This seems to only be needed after the sensor is initialized, so its value can remain fixed.
    c.watchdog_update();
    c.sleep_ms(50);
    c.watchdog_update();

    // Only for demonstration purposes
    c.gpio_init(c.PICO_DEFAULT_LED_PIN);
    c.gpio_set_dir(c.PICO_DEFAULT_LED_PIN, true);
    c.gpio_put(c.PICO_DEFAULT_LED_PIN, false);

    var led_on = false;

    while (true) {
        // Prevent race conditions by masking IRQs; assumes that IRQs are unmasked at this point
        intrin.cpsidi();

        // Try to get the TF Luna packet, this is not guaranteed to return data.
        const next_luna_opt = uart.getNextLuna();

        if (next_luna_opt) |luna| {
            if (luna.getValidDist()) |dist| {
                if (dist < 35) {
                    if (!led_on) c.gpio_put(c.PICO_DEFAULT_LED_PIN, true);
                    led_on = true;
                } else {
                    if (led_on) c.gpio_put(c.PICO_DEFAULT_LED_PIN, false);
                    led_on = false;
                }
            }
        }

        // Wait for next IRQ with IRQs masked to prevent the RX IRQ from getting ignored
        intrin.wfi();
        intrin.cpsiei();

        c.watchdog_update();
    }
}