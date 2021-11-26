//! main.zig: deltav8 entry point
//! Copyright (C) 2021 Drew P.
//!
//! This program is free software; you can redistribute it and/or modify
//! it under the terms of the GNU General Public License as published by
//! the Free Software Foundation; either version 2 of the License, or
//! (at your option) any later version.
//!
//! This program is distributed in the hope that it will be useful,
//! but WITHOUT ANY WARRANTY; without even the implied warranty of
//! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//! GNU General Public License for more details.
//!
//! You should have received a copy of the GNU General Public License along
//! with this program; if not, write to the Free Software Foundation, Inc.,
//! 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

const std = @import("std");
const logger = @import("logger.zig");
const uart = @import("uart.zig");
const intrin = @import("intrin.zig");
const clock = @import("clock.zig");
const adc = @import("adc.zig");

const c = @cImport({
    @cInclude("pico/stdlib.h");
    @cInclude("hardware/watchdog.h");
});

// Uncomment or change to enable logs for any build mode
pub const log_level: std.log.Level = .debug;

/// stdlib log handler; no logging is done if stdio is disabled.
pub fn log(comptime level: std.log.Level, comptime scope: @TypeOf(.EnumLiteral), comptime format: []const u8, args: anytype) void {
    _ = scope;

    const prefix = "[" ++ @tagName(level) ++ "]: ";
    logger.log(prefix ++ format ++ "\n", args) catch {};
}

export fn main() void {
    // Configure a generous watchdog timer of 1 second in case something goes wrong.
    // This should be more than enough time for the sensor to init everything;
    // during testing it seems the sensor needs ~450ms to initialize (minus the 50ms delay for a valid data packet).
    // If stdio is enabled we use a 10 second timeout instead as UART failures will wait five seconds for someone
    // to connect the USB cable.
    c.watchdog_enable(if (logger.stdio_enabled) 10000 else 1000, true);

    // Configure system clocks to save power.
    clock.configureClocks();
    logger.initLogger();

    // Init the ADC peripherals first. The UART is initialized last as it could take up to 500ms.
    adc.init();

    // Init the sensor and the accompanying UART
    uart.init();

    // Magic sleep value of 50ms because the sensor's data is unreliable before this period,
    // where dist seems to be some value smaller than it actually should be.
    // This seems to only be needed after the sensor is initialized, so its value can remain fixed.
    c.watchdog_update();
    c.sleep_ms(50);
    c.watchdog_update();

    // Tell the ADC that we're ready to receive samples
    adc.enableIrqs();

    // Set a new, lower watchdog of 50ms which is enough time to transmit 5 data packets.
    // Because we reset independent of the sensor there won't be much time between resets if something randomly goes wrong.
    c.watchdog_enable(if (logger.stdio_enabled) 500 else 50, true);

    // Main event loop. We have two peripherals, the sensor and the throttle, continuously trying to give us some data;
    // the sensor through the UART RX IRQ and the throttle through the ADC's FIFO IRQ.
    // The sensor runs every ~10ms and the throttle (ADC) runs every ~1ms. To optimize power
    // we tell the M0+ to sleep until either one of these events occur; we then do something if either
    // are reporting new information. Following we then just go back to sleep.
    // n.b.: we'll also get something to report velocity later
    while (true) {
        // Prevent race conditions by masking IRQs; assumes that IRQs are unmasked at this point
        intrin.cpsidi();

        if (uart.getNextLuna()) |luna| {
            _ = luna;
        }

        const throttle_speed_opt = adc.getThrottleSpeed();

        // Wait for next IRQ with IRQs masked to prevent the RX IRQ from getting ignored
        intrin.wfi();
        intrin.cpsiei();

        if (throttle_speed_opt) |speed| {
            std.log.debug("Throttle val: {}", .{speed});
        }

        c.watchdog_update();
    }
}
