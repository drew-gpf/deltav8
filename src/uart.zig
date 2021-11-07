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
    @cInclude("hardware/watchdog.h");
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
        @compileLog("Invalid size of fields, bytes; must be 12 bytes", @sizeOf(fields_type), @sizeOf(bytes_type));
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

    // Before enabling RX we set the RX communication IRQ which is used to avoid timing issues 
    c.irq_set_exclusive_handler(luna_uart_irq, lunaUartCommIrq);
    c.irq_set_enabled(luna_uart_irq, true);
    luna_uart.setRxIrq(.eighth, true);

    luna_uart.setTxRx(true, true);
    std.log.debug("Initialized UART0", .{});

    // Set the sensor to a known state. We configure it to use a 12 byte header which allows each packet to
    // trigger an RX IRQ without waiting.
    enableChecksum() catch |e| fatalError("Failed to enable checksum mode: {}", .{ e });
    std.log.debug("Enabled checksum verification", .{});

    setOutputFmt(.id_0) catch |e| fatalError("Failed to set output mode: {}", .{ e });
    std.log.debug("Set output mode", .{});

    // Default 100Hz
    setOutputFreq(100) catch |e| fatalError("Failed to set output frequency: {}", .{ e });
    std.log.debug("Set output frequency", .{});

    // Disable RX and TX temporarily
    luna_uart.setRxIrq(null, false);
    luna_uart.setTxRx(false, false);

    // Clear the RX IRQ
    c.irq_set_enabled(luna_uart_irq, false);
    c.irq_remove_handler(luna_uart_irq, lunaUartCommIrq);

    // Set a new handler
    c.irq_set_exclusive_handler(luna_uart_irq, lunaUartIrq);
    c.irq_set_enabled(luna_uart_irq, true);

    // Set the level to 12 bytes, which is also the size of our data packet.
    // todo: change this IRQ to account for the FIFO now being enabled properly
    luna_uart.setRxIrq(.three_quarters, true);
    luna_uart.enableRx();

    std.log.debug("UART ready to use", .{});
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


/// Reset the sensor. Note that using this may cause the sensor to endlessly emit data packets.
fn resetSensor() callconv(.Inline) !void {
    if (getLunaResponse(void, {}, u8, 2) != 0) return error.FailedToReset;
}

/// Enable checksum verification of transmitted data.
fn enableChecksum() callconv(.Inline) !void {
    if (getLunaResponse(u8, 1, u8, 8) != 1) return error.FailedToEnableChecksum;
}

/// Change output format.
fn setOutputFmt(fmt: OutputFmt) callconv(.Inline) !void {
    if (getLunaResponse(u8, @enumToInt(fmt), u8, 5) != @enumToInt(fmt)) return error.FailedToSetOutputFmt;
}

/// Reset saved settings.
fn restoreDefault() callconv(.Inline) !void {
    if (getLunaResponse(void, {}, u8, 0x10) != 0) return error.FailedToRestoreDefaults;
}

/// Save current settings.
fn save() callconv(.Inline) !void {
    if (getLunaResponse(void, {}, u8, 0x11) != 0) return error.FailedToSaveSettings;
}

/// Set a new output frequency, and verify the frequency was actually set.
fn setOutputFreq(freq: u16) callconv(.Inline) !void {
    if (getLunaResponse(u16, freq, u16, 3) != freq) return error.FailedToSetFreq;
}

/// Toggle data output.
fn toggleOutput(enable: bool) callconv(.Inline) !void {
    if (getLunaResponse(u8, @boolToInt(enable), u8, 7) != @boolToInt(enable)) return error.FailedToToggleOutput;
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

/// Wait for a standard response from the TF-Luna.
/// This will send a header byte, computed len, id, data_out (if not void), and computed checksum.
/// It will then wait for a header byte, computed len, id, new DataIn (if not void), and computed checksum.
/// If the required packet is sent back but has a checksum error, this function may hang indefinitely.
fn getLunaResponse(comptime DataOut: type, data_out: DataOut, comptime DataIn: type, id: u8) DataIn {
    const send_data_out = @sizeOf(DataOut) > 0;

    const header_byte = 0x5A;
    const base_len = @sizeOf(u8) * 4;

    const len_out = base_len + @sizeOf(DataOut);
    const len_in = base_len + @sizeOf(DataIn);

    const out_checksum = blk: {
        var temp: u8 = header_byte +% len_out +% id;

        if (comptime send_data_out) {
            for (std.mem.asBytes(&data_out)) |byte| {
                temp +%= byte;
            }
        }

        break :blk temp;
    };

    const writer = luna_uart.getWriter();

    // Before writing our command we want to get our header data to prevent race conditions.
    // Disable IRQs while initializing data to prevent data races.
    intrin.cpsidi();

    var potential_header: [len_in]u8 = undefined;

    CommState.potential_header = potential_header[0..];
    CommState.current_idx = 0;
    CommState.id = id;

    intrin.cpsiei();

    // Continually wait for a proper response.
    var i: usize = 0;

    while (true) : (i += 1) {
        intrin.cpsidi();

        // The IRQ will set the potential header slice to null if all data has been transmitted.
        if (CommState.potential_header == null) {
            intrin.cpsiei();
            break;
        }

        // Don't wait for the IRQ as our command may not have been sent.
        // If we spend 200ms waiting for something but nothing comes, resend the command.
        intrin.cpsiei();

        // Send command to begin with, as well as every 200ms (100us * 2000).
        if (i % 2000 == 0) {
            // Write command then checksum. This will unroll to an equivalent sequence of a full command.
            // Because we always at least send 4 bytes our base len is 4 bytes.
            _ = writer.writeByte(header_byte) catch unreachable;
            _ = writer.writeByte(len_out) catch unreachable;
            _ = writer.writeByte(id) catch unreachable;
            if (comptime send_data_out) _ = writer.write(std.mem.asBytes(&data_out)) catch unreachable;
            _ = writer.writeByte(out_checksum) catch unreachable;
        }

        c.sleep_us(100);
    }

    // Read data.
    if (@sizeOf(DataIn) > 0) {
        var ret: DataIn = undefined;
        std.mem.copy(u8, std.mem.asBytes(&ret), potential_header[3..potential_header.len-1]);

        return ret;
    }
}

const CommState = struct {
    /// Buffer to be filled. Must point to valid memory until invalidated (null).
    /// When all data has been read in an IRQ, the IRQ will set the buffer to null.
    var potential_header: ?[]u8 = null;

    /// Current data index, must be <= potential_header.len
    var current_idx: usize = 0;

    /// Packet ID to look for. Length is determined by the potential header's len.
    var id: u8 = 0;
};

/// Combined UART IRQ used during regular communication. There is no TX IRQ and the RX IRQ
/// is configured to fire after receiving 2 bytes, or timing out.
fn lunaUartCommIrq() callconv(.C) void {
    if (CommState.potential_header) |potential_header| blk: {
        // Read until expected sequence
        const header_byte = 0x5A;
        const model_header = [_]u8{ header_byte, @intCast(u8, potential_header.len), CommState.id };

        while (!luna_uart.isRxFifoEmpty()) {
            const current_byte = luna_uart.readByte();

            if (CommState.current_idx < model_header.len) {
                const expected_byte = model_header[CommState.current_idx];

                // current_byte is expected to be equal to expected_byte based on the given sequence.
                // If it is, we can increment current_idx and set data in potential_header.
                // Otherwise we need to check if we need to restart the sequence.
                if (current_byte == expected_byte) {
                    potential_header[CommState.current_idx] = current_byte;
                    CommState.current_idx += 1;
                } else {
                    CommState.current_idx = 0;

                    if (current_byte == model_header[0]) {
                        potential_header[CommState.current_idx] = current_byte;
                        CommState.current_idx += 1;
                    }
                }
            } else {
                // Set data in potential_header until current_idx is equal to its length.
                // Then, verify checksum.
                potential_header[CommState.current_idx] = current_byte;
                CommState.current_idx += 1;

                if (CommState.current_idx == potential_header.len) {
                    const checksum = potential_header[potential_header.len-1];
                    const calculated_checksum = check: {
                        var temp: u8 = 0;

                        for (potential_header[0..potential_header.len-1]) |byte| {
                            temp +%= byte;
                        }

                        break :check temp;
                    };

                    // If checksums match, this was most likely our header.
                    if (checksum == calculated_checksum) {
                        // Reset state to tell the comm routine that we have its header.
                        CommState.potential_header = null;
                        CommState.id = 0;
                        CommState.current_idx = 0;

                        // Exit the loop from the if statement above so we can flush the FIFO queue.
                        break :blk;
                    }

                    // Always reset the current index.
                    CommState.current_idx = 0;

                    // Otherwise, we need to look inside of potential_header to find start bytes.
                    // This can be made simple as we know we do not need to calculate a checksum.
                    // This assumes that null bytes in the header are not followed by non-null bytes.
                    for (potential_header) |byte, i| {
                        // Only look at variable bytes in the potential header
                        if (i < model_header.len) continue;

                        // Stop once we hit variable bytes in the model header
                        if (CommState.current_idx >= model_header.len) break;

                        // If this byte is present at the sequence specified by the model header,
                        // start using it instead.
                        if (byte == model_header[CommState.current_idx]) {
                            potential_header[CommState.current_idx] = model_header[CommState.current_idx];
                            CommState.current_idx += 1;
                        } else {
                            // Reset the current idx if the byte sequence breaks
                            CommState.current_idx = 0;

                            // Perform the same logic as above; if this is the first model header byte
                            // we still want to set that.
                            if (current_byte == model_header[0]) {
                                potential_header[CommState.current_idx] = current_byte;
                                CommState.current_idx += 1;
                            }
                        }
                    }
                }
            }
        }

        // If the loop exits here then we had read all the bytes from the FIFO queue but still need
        // to read more bytes from the sensor.
        return;
    }

    // Flush FIFO queue to prevent overrun
    while (!luna_uart.isRxFifoEmpty()) _ = luna_uart.readByte();
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