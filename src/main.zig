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
const pwm = @import("pwm.zig");

const c = @cImport({
    @cInclude("pico/stdlib.h");
    @cInclude("hardware/watchdog.h");
    @cInclude("hardware/adc.h");
});

// Uncomment or change to enable logs for any build mode
pub const log_level: std.log.Level = .debug;

/// stdlib log handler; no logging is done if stdio is disabled.
pub fn log(comptime level: std.log.Level, comptime scope: @TypeOf(.EnumLiteral), comptime format: []const u8, args: anytype) void {
    _ = scope;

    const prefix = "[" ++ @tagName(level) ++ "]: ";
    logger.log(prefix ++ format ++ "\n", args) catch {};
}

fn mainWrap() !void {
    logger.initLogger();
    try uart.init();
    adc.init();
    adc.enableIrqs();

    while (true) {
        intrin.cpsidi();
        const voltage = adc.getThrottleVoltage();
        intrin.wfi();
        intrin.cpsiei();

        if (voltage) |throttle| {
            uart.controlSpeed(throttle, .clockwise, .left);
            std.log.debug("throttle voltage: {} motor speed: {}", .{ throttle, (std.math.shl(usize, throttle, std.meta.bitCount(u6))) / adc.max_throttle_voltage });
        }
    }
}

export fn main() void {
    mainWrap() catch |e| {
        // Flash the LED
        c.gpio_init(c.PICO_DEFAULT_LED_PIN);
        c.gpio_set_dir(c.PICO_DEFAULT_LED_PIN, true);
        c.gpio_put(c.PICO_DEFAULT_LED_PIN, true);

        // Send message infinitely and wait to get killed by the watchdog
        while (true) {
            std.log.warn("Error: {}", .{e});
            c.sleep_ms(50);

            intrin.loopHint();
        }
    };
}
