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
const uart = @import("rp2040/uart.zig");
const logger = @import("logger.zig");

const c = @cImport({
    @cInclude("hardware/irq.h");
    @cInclude("pico/stdlib.h");
});

const luna_uart = uart.uart0;
const luna_uart_irq = c.UART0_IRQ;

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
    // Use GPIO 0, 1 for UART0
    c.gpio_set_function(0, c.GPIO_FUNC_UART);
    c.gpio_set_function(1, c.GPIO_FUNC_UART);

    // Reset the UART
    luna_uart.init() catch |e| {
        fatalError("Failed to init UART0: {}", .{e});
    };

    // Enable it, setting the required format for the TF Luna
    luna_uart.enable(115200, .none, .one, .eight, true) catch |e| {
        fatalError("Failed to enable UART0: {}", .{e});
    };

    // Enable RX and TX without IRQs.
    luna_uart.setTxRx(true, true);

    // Tell the sensor to stop sending us data packets
    // todo: enable watchdog in case data transmission fails so we can still reset if something goes wrong
    resetSensor();

    // Set the sensor to a known state. We configure it to use a 12 byte header which allows each packet to
    // trigger an RX IRQ without waiting.
    enableChecksum() catch |e| fatalError("Failed to enable checksum mode: {}", .{ e });
    setOutputFmt(.id_0) catch |e| fatalError("Failed to set output mode: {}", .{ e });

    // Disable output for when we re-enable sensor output
    toggleOutput(false) catch |e| fatalError("Failed to disable output: {}", .{ e });

    const freq = setOutputFreq(100) catch |e| blk: {
        fatalError("Failed to set output frequency: {}", .{ e });
        break :blk 0;
    };

    if (freq != 100) {
        fatalError("Invalid set freq {}", .{ freq });
    }

    // Re-enable output and hope that we don't collide with a data packet
    toggleOutput(true) catch |e| fatalError("Failed to enable output: {}", .{ e });

    // Stop using TX as we don't need it anymore.
    luna_uart.disableTx();

    // Install IRQ handler. We trigger an IRQ when the RX FIFO has 12 bytes, which is the size
    // of the header we will receive. We also specify a timeout for incomplete data transfers.
    // todo: change irq to account for timeout. We can do it the other way.
    c.irq_set_exclusive_handler(luna_uart_irq, lunaUartIrq);
    c.irq_set_enabled(luna_uart_irq, true);
    luna_uart.setRxIrq(.three_quarters, true);
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

/// Reset the sensor so we can query its values in a predictible way. If transmission fails or the sensor is misconfigured
/// this will hang indefinitely.
fn resetSensor() void {
    // The TF Luna is extremely sporadic; there's a chance that it's holding out a data packet that it wants to send us,
    // and there's a chance that the first byte we receive is 0xFF.
    // There's a silver lining in that we can cause a soft reset, which should make it stop sending commands.
    // Furthermore we know that the response must come in the form:
    // 0x5A 0x05 0x02 <status> <checksum>
    // Whereas a data packet either starts with 0x59 or has different values for the other bytes
    // (the exception is the 8 byte packet which has no header).
    // So we forego the nice functions that expect everything to work well here and just send the reset command.
    const writer = luna_uart.getWriter();
    const reader = luna_uart.getReader();

    _ = writer.write(&[_]u8{ 0x5A, 0x04, 0x02, 0x60 }) catch unreachable;
    luna_uart.waitTx();

    const first_byte = blk: {
        const byte = reader.readByte() catch unreachable;
        if (byte == 0xFF) break :blk reader.readByte() catch unreachable;

        break :blk byte;
    };

    const handleDataPacket = struct {
        pub fn handleDataPacket(reader_inner: uart.Uart.Reader) void {
            // Regular data packet. Read until expected sequence.
            const model_header = [5]?u8{ 0x5A, 0x05, 0x02, null, null };

            var potential_header: [5]u8 = undefined;
            var current_idx: usize = 0;

            while (true) {
                const current_byte = reader_inner.readByte() catch unreachable;

                if (model_header[current_idx]) |expected_byte| {
                    // current_byte is expected to be equal to expected_byte based on the given sequence.
                    // If it is, we can increment current_idx and set data in potential_header.
                    // Otherwise we need to check if we need to restart the sequence.
                    if (current_byte == expected_byte) {
                        potential_header[current_idx] = current_byte;
                        current_idx += 1;
                    } else {
                        current_idx = 0;

                        if (current_byte == model_header[0].?) {
                            potential_header[current_idx] = current_byte;
                            current_idx += 1;
                        }
                    }
                } else {
                    // Set data in potential_header until current_idx is equal to its length.
                    // Then, verify checksum.
                    potential_header[current_idx] = current_byte;
                    current_idx += 1;

                    if (current_idx == potential_header.len) {
                        const checksum = potential_header[potential_header.len-1];
                        const calculated_checksum = blk: {
                            var temp: u8 = 0;

                            for (potential_header[0..potential_header.len-1]) |byte| {
                                temp +%= byte;
                            }

                            break :blk temp;
                        };

                        // If checksums match, this was most likely our header.
                        if (checksum == calculated_checksum)
                            break;

                        // Always reset the current index.
                        current_idx = 0;

                        // Otherwise, we need to look inside of potential_header to find start bytes.
                        // This can be made simple as we know we do not need to calculate a checksum.
                        // This assumes that null bytes in the header are not followed by non-null bytes.
                        for (model_header) |byte_opt, i| {
                            if (byte_opt != null) continue;
                            if (model_header[current_idx] == null) break;

                            if (potential_header[i] == model_header[current_idx].?) {
                                potential_header[current_idx] = model_header[current_idx].?;
                                current_idx += 1;
                            }
                        }

                        // Restart iteration to get the next byte of the header.
                    }
                }
            }

            // We exited the loop, make sure the reset was successful
            if (potential_header[3] != 0) fatalError("Failed to reset TF-Luna (status 0x{X})", .{ potential_header[3] });
        }
    }.handleDataPacket;

    switch (first_byte) {
        0x5A => {
            // These types of packets are nice because they indicate length.
            // The only exception are 8 byte packets, although we can also just assume that those are impossible.
            var len = reader.readByte() catch unreachable;
            var id = reader.readByte() catch unreachable;

            const handleId2 = struct {
                pub fn handleId2(reader_inner: uart.Uart.Reader) void {
                    const status = reader_inner.readByte() catch unreachable;
                    const checksum = reader_inner.readByte() catch unreachable;
                    const checksum_compare = 0x5A +% 5 +% 2 +% status;

                    if (checksum != checksum_compare) fatalError("Reset response checksum fail: 0x{X} 0x{X} 0x{X} 0x{X} 0x{X}", .{
                        0x5A, 5, 2, status, checksum
                    });

                    if (status != 0) fatalError("Failed to reset TF-Luna (status 0x{X})", .{ status });
                }
            }.handleId2;

            switch (id) {
                // ID 2 is the reset command response; we want this
                2 => handleId2(reader),

                // Some other packet: read all bytes until we find a packet with ID 2
                else => {
                    while (true) loop: {
                        var current_byte: u8 = 3;

                        while (current_byte < len) : (current_byte += 1) {
                            _ = reader.readByte() catch unreachable;
                        }

                        const new_header = reader.readByte() catch unreachable;

                        switch (new_header) {
                            0x5A => {},
                            0x59 => {
                                handleDataPacket(reader);
                                break :loop;
                            },
                            else => fatalError("Unknown header 0x{X}", .{ new_header })
                        }

                        len = reader.readByte() catch unreachable;
                        id = reader.readByte() catch unreachable;

                        switch (id) {
                            2 => {
                                handleId2(reader);
                                break;
                            },
                            else => {}
                        }
                    }
                }
            }
        },

        0x59 => handleDataPacket(reader),
        else => fatalError("Unknown header 0x{X}", .{ first_byte })
    }
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

    // Wait for data to be sent, then read the response
    luna_uart.waitTx();
    _ = reader.read(cmd_out[0..]) catch unreachable;

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

/// Combined UART IRQ. This is called when either the RX FIFO reaches 12 bytes
/// or timed out trying to wait for it to reach 12 bytes after reading some data.
fn lunaUartIrq() callconv(.C) void {
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

    while (!luna_uart.isRxFifoEmpty()) : (current_luna_byte += 1) {
        if (current_luna_byte == @sizeOf(TfLunaPacket)) {
            current_luna_byte = 0;
        }

        current_luna_data.bytes[current_luna_byte] = luna_uart.readByte();
    }

    S.last_time_opt = c.time_us_32();
}

fn fatalError(comptime reason: []const u8, args: anytype) callconv(.Inline) noreturn {
    // Flash the LED
    c.gpio_init(c.PICO_DEFAULT_LED_PIN);
    c.gpio_set_dir(c.PICO_DEFAULT_LED_PIN, true);
    c.gpio_put(c.PICO_DEFAULT_LED_PIN, true);

    c.sleep_ms(5000);
    std.log.warn(reason, args);

    while (true) {}
}