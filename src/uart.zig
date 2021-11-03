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
const logger = @import("logger.zig");

const c = @cImport({
    @cInclude("hardware/irq.h");
    @cInclude("pico/stdlib.h");
});

const luna_uart = uart.uart0;
const luna_irq = c.UART0_IRQ;

pub const TfLunaPacket = extern union {
    /// Magic number for header field
    pub const header_magic: u8 = 0x5A;

    fields: packed struct {
        /// Header, must be equal to header_magic.
        header: u8,

        /// Length of packet; must be equal to the size of this packet.
        len: u8,

        /// A byte always equal to zero.
        zero: u8,

        /// Distance, in centimeters, to nearest target.
        dist: u16,
        
        /// Signal strength, higher is better. Distance is invalid if equal to 65535 or is less than 100.
        strength: u16,

        /// Current time in milliseconds
        timestamp: u32,

        /// Cumulative sum of all fields before it.
        checksum: u8
    },
    bytes: [12]u8,

    /// Whether or not the struct's header is valid i.e. whether or not the data should be interpreted.
    pub fn isHeaderValid(this: *const TfLunaPacket) bool {
        if (this.fields.header != TfLunaPacket.header_magic or this.fields.len != @sizeOf(TfLunaPacket) or this.fields.zero != 0)
            return false;

        // Compute a simple wrapping checksum of the first 11 bytes i.e. before the checksum byte
        var checksum: u8 = 0;

        for (this.bytes[0..this.bytes.len-1]) |byte| {
            checksum +%= byte;
        }

        return checksum == this.fields.checksum;
    }

    /// Get the distance value, or null if it's invalid.
    pub fn getValidDist(this: *const TfLunaPacket) ?u16 {
        return if (this.fields.strength >= 100 and this.fields.strength != 65535) this.fields.dist else null;
    }
};

comptime {
    const fields = std.meta.fields(TfLunaPacket);

    const bytes_type = fields[std.meta.fieldIndex(TfLunaPacket, "bytes") orelse unreachable].field_type;
    const fields_type = fields[std.meta.fieldIndex(TfLunaPacket, "fields") orelse unreachable].field_type;

    if (@sizeOf(fields_type) != @sizeOf(bytes_type) or @sizeOf(fields_type) != 12)
        @compileLog("Invalid size of fields, bytes; must be 11 bytes", @sizeOf(fields_type), @sizeOf(bytes_type));
}

/// The current data package read from the previous IRQ.
var current_luna_data: TfLunaPacket = undefined;

/// The index of the first invalid byte of current_luna_data.
var current_luna_byte: usize = 0;

/// Init required UART devices.
/// UART0: TF Luna, 115200 baud rate, 8 data bits, 1 stop bit, no parity check, GPIO0 is TX, GPIO1 is RX
/// UART1: uninitialized
/// Additionally this will install an IRQ handler for UART0 on the current core.
pub fn init() void {
    const baud_rate = 115200;

    // Init TF Luna UART
    std.debug.assert(luna_uart.init(baud_rate) == baud_rate);

    // Use GPIO 0, 1 for UART0
    c.gpio_set_function(0, c.GPIO_FUNC_UART);
    c.gpio_set_function(1, c.GPIO_FUNC_UART);

    // Configure UART interface
    luna_uart.setFormat(8, 1, .none);
    luna_uart.setHwFlow(false, false);

    // Disable data output temporarily to avoid collision with response checking
    // We don't check the checksum for this, reset, and save as those will probably collide with data packets.
    _ = setOutputFreq(0) catch {};

    // Set the sensor to a known state. We configure it to use a 12 byte header which allows each packet to
    // trigger an RX IRQ without waiting.
    enableChecksum() catch |e| fatalError("Failed to enable checksum mode: {}", .{ e });
    setOutputFmt(.id_0) catch |e| fatalError("Failed to set output mode: {}", .{ e });

    const freq = setOutputFreq(100) catch |e| blk: {
        fatalError("Failed to set output frequency: {}", .{ e });
        break :blk 0;
    };

    if (freq != 100) {
        fatalError("Invalid set freq {}", .{ freq });
    }

    // Install IRQ handler. We trigger an IRQ when the RX FIFO has 12 bytes, which is the size
    // of the header we will receive.
    c.irq_set_exclusive_handler(luna_irq, lunaUartRxIrq);
    c.irq_set_enabled(luna_irq, true);
    luna_uart.enableIrqFor(.three_quarters, null);
}

/// Get SLEEP_ENX bits required for UART functionality when sleeping.
pub fn getUartSleepEn() u64 {
    // THE PICO SDK USES MACROS FOR NO REASON THAT BREAK TRANSLATE-C
    const CLOCKS_SLEEP_EN1_CLK_SYS_UART0_BITS = 0x00000080;
    const CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS = 0x00000040;

    // Also enable watchdog tick (bit 12 of sleep_en1) for the timer to work
    return (0) | ((CLOCKS_SLEEP_EN1_CLK_SYS_UART0_BITS | CLOCKS_SLEEP_EN1_CLK_PERI_UART0_BITS | (1 << 12)) << 32);
}

/// Get most recent data packet from the TF Luna, or null if none available.
/// Interrupts must be disabled when entering with function to avoid race conditions.
pub fn getNextLuna() callconv(.Inline) ?TfLunaPacket {
    if (current_luna_byte == @sizeOf(TfLunaPacket)) {
        current_luna_byte = 0;
        return current_luna_data;
    }

    std.debug.assert(current_luna_byte < @sizeOf(TfLunaPacket));
    return null;
}

/// Enable checksum verification of transmitted data
fn enableChecksum() callconv(.Inline) !void {
    var cmd_out: [5]u8 = undefined;
    try sendCmd(8, &[_]u8{ 1 }, cmd_out[0..]);
}

/// Change output format
fn setOutputFmt(fmt: OutputFmt) callconv(.Inline) !void {
    var cmd_out: [5]u8 = undefined;
    try sendCmd(5, &[_]u8{ @enumToInt(fmt) }, cmd_out[0..]);
}

const OutputFmt = enum(u8) {
    /// Standard 9-byte header with cm measurement
    nine_byte_cm = 1,

    /// String-formatted dist val
    pix,

    /// Standard 9-byte header with mm measurement
    nine_byte_mm = 6,

    /// 11-byte header with 4-byte timestamp
    eleven_byte_timestamp,

    /// 12-byte header with extra len byte
    id_0,

    /// 8-byte header with cm measurement
    eight_byte_cm
};

/// Reset saved settings
fn reset() callconv(.Inline) !void {
    var cmd_out: [5]u8 = undefined;
    try sendCmd(0x10, null, cmd_out[0..]);

    if (cmd_out[3] != 0) return error.ResetFailed;
}

/// Save current settings
fn save() callconv(.Inline) !void {
    var cmd_out: [5]u8 = undefined;
    try sendCmd(0x11, null, cmd_out[0..]);

    if (cmd_out[3] != 0) return error.SaveFailed;
}

/// Set a new output frequency; returns actual set frequency
fn setOutputFreq(freq: u16) callconv(.Inline) !u16 {
    var cmd_out: [7]u8 align(@alignOf(u16)) = undefined;
    try sendCmd(3, @ptrCast(*const [2]u8, &freq), cmd_out[1..]);

    // Offset cmd_out by 1 byte so we can read an aligned u16
    return @ptrCast(*u16, &cmd_out[3+1]).*;
}

/// Toggle data output.
fn toggleOutput(enable: bool) callconv(.Inline) !void {
    var cmd_out: [5]u8 = undefined;
    try sendCmd(7, &[_]u8{ @boolToInt(enable) }, cmd_out[0..]);
}

/// Send a command and wait for a response.
/// cmd_out dictates the length of the response. cmd_data should not contain the checksum byte.
/// This function returns an error if cmd_out's checksum is invalid.
/// Note that an invalid length for cmd_out will result in a hang or for the next read to return garbage data.
/// This function automatically builds the command length byte. which_cmd selects the command,
/// and cmd_data contains the data to send. If cmd_data is null, then no data is sent.
fn sendCmd(which_cmd: u8, cmd_data: ?[]const u8, cmd_out: []u8) callconv(.Inline) !void {
    // Flush RX FIFO queue
    while (luna_uart.isReadable()) _ = luna_uart.readByte();

    const writer = luna_uart.getWriter();
    const reader = luna_uart.getReader();

    const header: u8 = 0x5A;
    const len: u8 = if (cmd_data) |data| blk: {
        break :blk @intCast(u8, data.len) + 4;
    } else 4;

    const checksum = blk: {
        var temp: u8 = header +% len +% which_cmd;

        if (cmd_data) |cmd| {
            for (cmd) |byte| {
                temp +%= byte;
            }
        }

        break :blk temp;
    };

    // Write command then checksum. This will unroll to an equivalent sequence of a full command.
    // Because we always at least send 4 bytes we configure len to be cmd_data.len + 4 or just 4.
    _ = writer.writeByte(header) catch unreachable;
    _ = writer.writeByte(len) catch unreachable;
    _ = writer.writeByte(which_cmd) catch unreachable;
    if (cmd_data) |cmd| _ = writer.write(cmd[0..]) catch unreachable;
    _ = writer.writeByte(checksum) catch unreachable;

    // Wait for data to be sent
    luna_uart.waitTx();

    const S = struct {
        var read_first_byte: bool = false;
    };

    if (!S.read_first_byte) {
        S.read_first_byte = true;
        const first = reader.readByte() catch unreachable;

        if (first != 0xFF) {
            _ = reader.read(cmd_out[1..]) catch unreachable;
            cmd_out[0] = first;
        } else {
            _ = reader.read(cmd_out[0..]) catch unreachable;
        }
    } else {
        _ = reader.read(cmd_out[0..]) catch unreachable;
    }

    var out_checksum: u8 = 0;

    for (cmd_out[0..cmd_out.len-1]) |byte| {
        out_checksum +%= byte;
    }

    if (out_checksum != cmd_out[cmd_out.len-1]) {
        std.log.warn("invalid checksum {} vs reported {}\nout bytes:", .{ out_checksum, cmd_out[cmd_out.len-1] });

        for (cmd_out) |byte| {
            std.log.warn("0x{X}", .{byte});
        }

        return error.InvalidChecksum;
    }
}

/// RX IRQ for the luna UART. Because the level is set to 12 bytes and we will always
/// receive a 12 byte packet, this should only fire when the header is complete or needs to be overwritten.
fn lunaUartRxIrq() callconv(.C) void {
    // todo see if this works and is unaffected by timers being disabled
    const S = struct {
        var last_time_opt: ?u32 = null;
    };

    // If more than 5ms have passed between IRQs and we are writing after the start of the header,
    // assume that we missed a byte somewhere.
    if (current_luna_byte != 0 and current_luna_byte < @sizeOf(TfLunaPacket)) {
        if (S.last_time_opt) |last_time| {
            const current_time = c.time_us_32();
            const diff = current_time -% last_time;

            if (diff >= 5000) current_luna_byte = 0;
        }
    }

    while (luna_uart.isReadable()) : (current_luna_byte += 1) {
        if (current_luna_byte == @sizeOf(TfLunaPacket)) {
            current_luna_byte = 0;
        }

        current_luna_data.bytes[current_luna_byte] = luna_uart.readByte();
    }

    S.last_time_opt = c.time_us_32();
}

fn fatalError(comptime reason: []const u8, args: anytype) callconv(.Inline) void {
    // Flash the LED
    c.gpio_init(c.PICO_DEFAULT_LED_PIN);
    c.gpio_set_dir(c.PICO_DEFAULT_LED_PIN, true);
    c.gpio_put(c.PICO_DEFAULT_LED_PIN, true);

    while (true) {
        std.log.warn(reason, args);
        c.sleep_ms(1000);

        asm volatile ("" ::: "memory");
    }
}