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
/// We use an IRQ for the TF Luna as its refresh rate (up to 250Hz) is horrifyingly slow
/// and thus we stand to save some power by waiting for the next IRQ instead of constantly blocking.
pub fn init() void {
    // Init the TF Luna UART
    _ = c.uart_init(uart0(), 115200);

    // Designate GPIO pins. Practically this enforces how the system is wired.
    c.gpio_set_function(0, c.GPIO_FUNC_UART);
    c.gpio_set_function(1, c.GPIO_FUNC_UART);

    // Configure the TF Luna UART
    uart_set_format(uart0(), 8, 1, c.UART_PARITY_NONE);
    uart_set_hw_flow(uart0(), false, false);

    // Install and register UART0 IRQ handlers for received data.
    c.irq_set_exclusive_handler(c.UART0_IRQ, uart0RxIrq);
    c.irq_set_enabled(c.UART0_IRQ, true);
    uart_set_irq_enables(uart0(), true, false);
}

/// Get most recent data packet from the TF Luna, or null if none available.
/// Interrupts must be disabled when entering with function to avoid race conditions.
/// In the resulting idle loop, it is wise to WFI with IRQs disabled to prevent accidental FIFO overruns
/// where entire packets are thrown out due to being unable to respond to their resulting IRQs.
/// After WFI resumes, enable interrupts to recognize the IRQ.
pub fn getNextLuna() ?TfLunaStandard9Cm {
    // If data is valid, return it; also make sure (if needed) that we haven't overwritten bytes.
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
    // If we are about to overwrite bytes, throw out the previous header to avoid
    // an overrun in the FIFO queue.
    // Note that this is also compatible without the FIFO queue at the cost of potential data loss.
    // With a FIFO queue we still run into a potential latency issue of ~<=.4ms which probably isn't as bad
    // compared to the delay of not having a valid header for ~10ms (according to refresh rate.)
    // So, we just use a FIFO queue and let ourselves at the mercy of however the UART is configured
    // in terms of its level.
    while (c.uart_is_readable(uart0())) {
        // Throw out the previous data packet if we've accomplished the nigh-impossible task
        // of having more bytes in the FIFO queue than we expected
        if (current_luna_byte >= @sizeOf(TfLunaStandard9Cm)) {
            current_luna_byte = 0;
        }

        // Perform a blocking read even though we know data is present.
        // Note that getchar has the same behavior.
        c.uart_read_blocking(uart0(), &current_luna_data.bytes[current_luna_byte], @sizeOf(u8));
        current_luna_byte += 1;
    }
}

/// Get uart0 as would be done with the C macro which can't be translated.
fn uart0() callconv(.Inline) *c.uart_inst_t {
    // Where uart0 eventually translates to this address. May break if a different board is used.
    // The only fix here is to either change zig translate-c or to make a C function that returns the address.
    return @intToPtr(*c.uart_inst_t, 0x40034000);
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