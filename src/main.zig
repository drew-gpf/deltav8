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
const uart_hw = @import("hardware/uart.zig");
const intrin = @import("intrin.zig");
const clock = @import("clock.zig");

const c = @cImport({
    @cInclude("pico/stdlib.h");
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

    // Configure system clocks to save power. This must be updated if the RTC or ADC are used,
    // or if USB is used outside of stdio.
    //clock.configureClocks();

    logger.initLogger();
    //c.sleep_ms(5000);

    c.gpio_init(25);
    c.gpio_set_dir(25, true);
    c.gpio_put(25, true);

    uart.init();

    var i: usize = 0;

    while (true) : (i += 1) {
        //var bytes: [9]u8 align(@sizeOf(u16)) = undefined;
        //_ = uart_hw.uart0.getReader().read(bytes[0..]) catch unreachable;

        //std.log.debug("0x{X}", .{ @ptrCast(*u16, &bytes[0]).* });
        std.log.debug("0x{X}", .{c.uart_getc(@ptrCast(*c.uart_inst_t, uart_hw.uart0.getInst()))});

        // Prevent race conditions by masking IRQs; assumes that IRQs are unmasked at this point
        //intrin.cpsidi();

        // Try to get the TF Luna packet, this is not guaranteed to return data.
        //const next_luna_opt = uart.getNextLuna();

        // Wait for next IRQ with IRQs masked to prevent the RX IRQ from getting ignored
        //intrin.wfi();
        //intrin.cpsiei();

        //if (next_luna_opt) |luna| {
            //std.log.debug("Got UART. Is valid: {}, dist: {}, temp: {} Celcius, strength: {}", .{
                //luna.isHeaderValid(), luna.getValidDist(), luna.getTemp(), luna.fields.strength
            //}); 
        //}

        if (i % 1000 == 0) {
            c.gpio_xor_mask(1 << 25);
        }
    }
}