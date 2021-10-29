//! uart.zig: utils for easily querying data from devices connected to UART hardware
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
const intrin = @import("intrin.zig");

const c = @cImport({
    @cInclude("hardware/irq.h");
    @cInclude("hardware/uart.h");
    @cInclude("hardware/clocks.h");
    @cInclude("pico/stdlib.h");
    @cInclude("pico/sync.h");
});

pub const TfLunaStandard9Cm = extern union {
    /// Magic number for header field
    pub const header_magic: u16 = 0x5959;

    /// Maximum distance, in centimeters, that the sensor is able to report.
    pub const max_dist: u16 = 1200;

    fields: packed struct {
        /// Header, must be equal to header_magic.
        header: u16,

        /// Distance, in centimeters, to nearest target. Must be between 0-1200. If outside that range,
        /// the signal strength is probably less than 100 and the value is -1.
        dist: u16,
        
        /// Signal strength, higher is better.
        strength: u16,

        /// Ranged temperature. Degrees C = (this / 8) - 256.
        temperature: u16,

        /// Cumulative sum of all fields before it.
        checksum: u8
    },
    bytes: [9]u8,

    /// Whether or not the struct's header is valid i.e. whether or not the data should be interpreted.
    pub fn isHeaderValid(this: *const TfLunaStandard9Cm) bool {
        if (this.fields.header != header_magic)
            return false;

        // Compute a simple wrapping checksum of the first 8 bytes i.e. before the checksum byte
        var checksum: u8 = 0;

        for (this.bytes[0..8]) |byte| {
            checksum +%= byte;
        }

        return checksum == this.fields.checksum;
    }

    /// Get the distance value, or null if it's invalid.
    pub fn getValidDist(this: *const TfLunaStandard9Cm) ?u16 {
        return if (this.fields.dist <= max_dist) this.fields.dist else null;
    }

    /// Get the temperature of the device, in degrees celcius.
    pub fn getTemp(this: *const TfLunaStandard9Cm) u16 {
        return (this.fields.temperature / 8) - 256;
    }
};

comptime {
    const fields = std.meta.fields(TfLunaStandard9Cm);

    const bytes_type = fields[std.meta.fieldIndex(TfLunaStandard9Cm, "bytes") orelse unreachable].field_type;
    const fields_type = fields[std.meta.fieldIndex(TfLunaStandard9Cm, "fields") orelse unreachable].field_type;

    if (@sizeOf(fields_type) != @sizeOf(bytes_type) or @sizeOf(fields_type) != 9)
        @compileLog("Invalid size of fields, bytes; must be 9 bytes", @sizeOf(fields_type), @sizeOf(bytes_type));
}

/// uart0 pointer from the SDK. This must be done here as the relevant macro can't be translated.
pub const uart0 = @intToPtr(*c.uart_inst_t, 0x40034000);

/// The current data package read from the previous IRQ.
var current_luna_data: TfLunaStandard9Cm = undefined;

/// The current byte read within current_luna_data. When equal to @sizeOf(TfLunaStandard9Cm), all data within the buffer is valid;
/// otherwise, the data is incomplete or stale.
/// This must not be greater than @sizeOf(TfLunaStandard9Cm) i.e. 9 bytes
var current_luna_byte: usize = 0;

/// Init required UART devices.
/// UART0: TF Luna, 115200 baud rate, 8 data bits, 1 stop bit, no parity check, GPIO0 is TX, GPIO1 is RX
/// UART1: uninitialized
/// Additionally this will install an IRQ handler for UART0 on the current core.
pub fn init() void {
    // Init TF Luna UART
    _ = c.uart_init(uart0, 115200);

    // Use GPIO 0, 1 for UART0
    c.gpio_set_function(0, c.GPIO_FUNC_UART);
    c.gpio_set_function(1, c.GPIO_FUNC_UART);

    // Configure UART interface
    uart_set_format(uart0, 8, 1, c.UART_PARITY_NONE);
    uart_set_hw_flow(uart0, false, false);

    // Install IRQ handler
    c.irq_set_exclusive_handler(c.UART0_IRQ, uart0RxIrq);
    c.irq_set_enabled(c.UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
}

/// Get SLEEP_ENX bits required for UART functionality when sleeping.
pub fn getUartSleepEn() u64 {
    // THE PICO SDK USES MACROS FOR NO REASON THAT BREAK TRANSLATE-C
    const CLOCKS_SLEEP_EN1_CLK_SYS_UART0_BITS = 0x00000080;
    const CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS = 0x00000040;

    return (0) | ((CLOCKS_SLEEP_EN1_CLK_SYS_UART0_BITS | CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS) << 32);
}

/// Get most recent data packet from the TF Luna, or null if none available.
/// Interrupts must be disabled when entering with function to avoid race conditions.
pub fn getNextLuna() ?TfLunaStandard9Cm {
    if (current_luna_byte == @sizeOf(TfLunaStandard9Cm)) {
        current_luna_byte = 0;
        return current_luna_data;
    }

    std.debug.assert(current_luna_byte < @sizeOf(TfLunaStandard9Cm));
    return null;
}

/// RX IRQ for the UART0; fired when some amount of data has been filled in the UART's FIFO queue.
fn uart0RxIrq() callconv(.C) void {
    // Get all bytes in the FIFO queue.
    while (c.uart_is_readable(uart0)) {
        // Throw out the previous data packet if we've accomplished the nigh-impossible task
        // of having more bytes in the FIFO queue than we expected
        if (current_luna_byte >= @sizeOf(TfLunaStandard9Cm)) {
            current_luna_byte = 0;
        }

        // Perform a "blocking" read. todo: read from uart manually
        c.uart_read_blocking(uart0, &current_luna_data.bytes[current_luna_byte], @sizeOf(u8));
        current_luna_byte += 1;
    }
}

// The following are macros translated into Zig with small fixes which translate-c couldn't handle
// likely due to them abusing UB.
// Note that these macros may or may not break if the board configuration changes,
// where the only fix would be to change zig translate-c or to export these from C.

fn uart_set_hw_flow(arg_uart: *c.uart_inst_t, arg_cts: bool, arg_rts: bool) callconv(.C) void {
    var uart = arg_uart;
    var cts = arg_cts;
    var rts = arg_rts;
    c.hw_write_masked(&c.uart_get_hw(uart).*.cr, (@as(c_uint, @boolToInt(!!cts)) << @intCast(@import("std").math.Log2Int(c_uint), 15)) | (@as(c_uint, @boolToInt(!!rts)) << @intCast(@import("std").math.Log2Int(c_uint), 14)), @as(c_uint, 16384) | @as(c_uint, 32768));
}

fn uart_set_format(arg_uart: *c.uart_inst_t, arg_data_bits: c_uint, arg_stop_bits: c_uint, arg_parity: c.uart_parity_t) callconv(.C) void {
    var uart = arg_uart;
    var data_bits = arg_data_bits;
    var stop_bits = arg_stop_bits;
    var parity = arg_parity;
    std.debug.assert(data_bits >= 5 and data_bits <= 8);
    std.debug.assert(stop_bits == 1 or stop_bits == 2);
    std.debug.assert(parity == c.UART_PARITY_NONE or parity == c.UART_PARITY_ODD or parity == c.UART_PARITY_EVEN);
    c.hw_write_masked(&c.uart_get_hw(uart).*.lcr_h, ((((data_bits -% @as(c_uint, 5)) << @intCast(@import("std").math.Log2Int(c_uint), 5)) | ((stop_bits -% @as(c_uint, 1)) << @intCast(@import("std").math.Log2Int(c_uint), 3))) | (@as(c_uint, @boolToInt(!!(parity != @bitCast(c_uint, c.UART_PARITY_NONE)))) << @intCast(@import("std").math.Log2Int(c_uint), 1))) | (@as(c_uint, @boolToInt(!!(parity == @bitCast(c_uint, c.UART_PARITY_EVEN)))) << @intCast(@import("std").math.Log2Int(c_uint), 2)), ((@as(c_uint, 96) | @as(c_uint, 8)) | @as(c_uint, 2)) | @as(c_uint, 4));
}

fn uart_set_irq_enables(arg_uart: *c.uart_inst_t, arg_rx_has_data: bool, arg_tx_needs_data: bool) callconv(.C) void {
    var uart = arg_uart;
    var rx_has_data = arg_rx_has_data;
    var tx_needs_data = arg_tx_needs_data;
    c.uart_get_hw(uart).*.imsc = (@as(c_uint, @boolToInt(!!tx_needs_data)) << @intCast(@import("std").math.Log2Int(c_uint), 5)) | (@as(c_uint, @boolToInt(!!rx_has_data)) << @intCast(@import("std").math.Log2Int(c_uint), 4));
    if (rx_has_data) {
        c.hw_write_masked(&c.uart_get_hw(uart).*.ifls, @bitCast(u32, @as(c_int, 0) << @intCast(@import("std").math.Log2Int(c_int), 3)), @as(c_uint, 56));
    }
    if (tx_needs_data) {
        c.hw_write_masked(&c.uart_get_hw(uart).*.ifls, @bitCast(u32, @as(c_int, 0) << @intCast(@import("std").math.Log2Int(c_int), 0)), @as(c_uint, 7));
    }
}