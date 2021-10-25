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

const c = @cImport({
    @cInclude("pico/stdlib.h");
});

// Uncomment or change to enable logs for any build mode
//pub const log_level: std.log.Level = .debug;

// Generic log handler. All logs must go to this routine; no logging is done if the binary was not built with stdio enabled.
pub fn log(comptime level: std.log.Level, comptime scope: @TypeOf(.EnumLiteral), comptime format: []const u8, args: anytype) void {
    _ = scope;

    const prefix = "[" ++ @tagName(level) ++ "]: ";
    logger.log(prefix ++ format ++ "\n", args) catch {};
}

export fn main() void {
    logger.initLogger();

    // Init UART0 and associated IRQs so we can get data from the TF Luna
    uart.init();

    // Main event loop; wait for next data packet
    while (true) {
        // Prevent race conditions by masking IRQs; assumes that IRQs are unmasked at this point
        intrin.cpsidi();

        // Try to get the TF Luna packet. This might not be valid if:
        // - This is the first iteration
        // - We received an unrelated IRQ
        // - The UART FIFO queue triggered an IRQ but the level is set such that we still have more bytes to read
        const next_luna_opt = uart.getNextLuna();

        // Wait for next IRQ with IRQs masked to prevent the RX IRQ from getting ignored.
        intrin.wfi();
        intrin.cpsiei();

        // Note that it's entirely possible this log is so slow that we end up skipping packets
        if (next_luna_opt) |luna| {
            std.log.debug("Got UART. Is valid: {}, dist: {}, temp: {} Celcius, strength: {}", .{
                luna.isHeaderValid(), luna.getValidDist(), luna.getTemp(), luna.fields.strength
            }); 
        }
    }
}