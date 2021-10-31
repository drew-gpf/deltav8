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
const uart = @import("hardware/uart.zig");

const uart0 = uart.uart0;

const c = @cImport({
    @cInclude("hardware/irq.h");
    @cInclude("pico/stdlib.h");
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
pub fn init() void {
    const baud_rate = 115200;

    // Init TF Luna UART
    std.debug.assert(uart0.init(baud_rate) == baud_rate);

    // Use GPIO 0, 1 for UART0
    c.gpio_set_function(0, c.GPIO_FUNC_UART);
    c.gpio_set_function(1, c.GPIO_FUNC_UART);

    // Configure UART interface
    uart0.setFormat(8, 1, .none);
    uart0.setHwFlow(false, false);

    // Install IRQ handler. We trigger an IRQ when the RX FIFO has 12 bytes.
    // In practice it will wait for all 9 bytes to be received and then wait the amount of time
    // taken to receive another 4 bytes.
    c.irq_set_exclusive_handler(c.UART0_IRQ, uart0RxIrq);
    c.irq_set_enabled(c.UART0_IRQ, true);
    uart0.enableIrqFor(.three_quarters, null);
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

/// RX IRQ for the UART0. The level is set to 12 bytes,
/// but in most or all cases this should fire when there are only 9 bytes in the header.
fn uart0RxIrq() callconv(.C) void {
    // Get all bytes in the FIFO queue.
    while (uart0.isReadable()) {
        // Throw out the previous data packet if we've accomplished the nigh-impossible task
        // of having more bytes in the FIFO queue than we expected
        if (current_luna_byte >= @sizeOf(TfLunaStandard9Cm)) {
            current_luna_byte = 0;
        }

        current_luna_data.bytes[current_luna_byte] = uart0.readByte();
        current_luna_byte += 1;
    }
}