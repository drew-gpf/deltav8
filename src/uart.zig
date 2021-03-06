//! uart.zig: utils for easily querying data from devices connected to UART hardware
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
const intrin = @import("intrin.zig");
const uart = @import("rp2040/uart.zig");
const logger = @import("logger.zig");
const adc = @import("adc.zig");

const c = @cImport({
    @cInclude("hardware/irq.h");
    @cInclude("pico/stdlib.h");
});

const luna_uart = uart.uart0;
const luna_uart_irq = c.UART0_IRQ;

const motor_uart = uart.uart1;

const uart0_tx_gpio = 0;
const uart0_rx_gpio = 1;

const uart1_tx_gpio = 4;

pub const TfLunaPacket = extern union {
    fields: packed struct {
        /// Distance, in centimeters, to nearest target.
        dist: u16,

        /// Signal strength, higher is better. Distance is invalid if equal to 65535 or is less than 100.
        strength: u16,

        /// Current time in milliseconds
        timestamp: u32,
    },
    bytes: [8]u8,

    /// Get the distance value, or null if it's invalid.
    pub inline fn getValidDist(this: *const TfLunaPacket) ?u16 {
        return if (this.fields.strength >= 100 and this.fields.strength != 65535) this.fields.dist else null;
    }
};

pub const MotorPacket = extern union {
    bits: packed struct {
        speed: MotorSpeed,
        dir: MotorDir,
        channel: MotorChannel,
    },
    full: u8,
};

pub const MotorSpeed = u6;
pub const MotorDir = enum(u1) { clockwise, counter_clockwise };
pub const MotorChannel = enum(u1) { left, right };

comptime {
    const fields = std.meta.fields(TfLunaPacket);

    const bytes_type = fields[std.meta.fieldIndex(TfLunaPacket, "bytes") orelse unreachable].field_type;
    const fields_type = fields[std.meta.fieldIndex(TfLunaPacket, "fields") orelse unreachable].field_type;

    if (@sizeOf(fields_type) != @sizeOf(bytes_type) or @sizeOf(fields_type) != 8)
        @compileLog("Invalid size of fields, bytes; must be 8 bytes", @sizeOf(fields_type), @sizeOf(bytes_type));

    if (@sizeOf(MotorPacket) != @sizeOf(u8))
        @compileLog("Invalid size of motor packet; must be 1 byte", @sizeOf(MotorPacket));
}

/// The current data packet read from the previous IRQ.
var current_luna_data: TfLunaPacket = undefined;

/// Whether or not the luna data is valid.
var luna_data_valid = false;

/// Init required UART devices.
/// UART0: TF Luna, 115200 baud rate, 8 data bits, 1 stop bit, no parity check, GPIO0 is TX, GPIO1 is RX
/// UART1: Motor controller, 115200 baud rate, 8 data bits, 1 stop bit, no parity check, GPIO4 is TX (no RX)
/// Additionally this will install an IRQ handler for UART0 on the current core.
pub fn init() !void {
    luna_data_valid = false;

    try initLunaUart();
    try initMotorUart();
}

/// Get most recent data packet from the TF Luna, or null if none available.
/// Interrupts must be disabled when entering this function to avoid race conditions.
pub inline fn getNextLuna() ?TfLunaPacket {
    if (luna_data_valid) {
        luna_data_valid = false;
        return current_luna_data;
    }

    return null;
}

/// Send a new speed value to the motor. This should be called with IRQs enabled;
/// this function will wait for the last speed to be transmitted.
/// throttle_voltage is the relative voltage reported by the throttle ADC and must be below
/// max_throttle_voltage.
pub inline fn controlSpeed(throttle_voltage: u12, dir: MotorDir, channel: MotorChannel) void {
    // Compress to a MotorSpeed value by mapping to a MotorSpeed's range of possible values;
    // out = rel_voltage / (2^(log2(max_rel_voltage) - numBits(MotorSpeed)))
    // However this operation (without converting to ints and shifting to account for inaccuracies) will introduce floating point operations,
    // so it is better to simplify it to:
    // out = rel_voltage / (max_rel_voltage / (maxInt(MotorSpeed) + 1))
    // out = (rel_voltage * (maxInt(MotorSpeed) + 1)) / max_rel_voltage
    // out = (rel_voltage << numBits(MotorSpeed)) / max_rel_voltage
    // This allows us to compute the exact integer value without using costly floating-point math.
    // Note that voltage must be [0, max_rel_voltage) to prevent overflow with @intCast().
    const speed = @intCast(MotorSpeed, (std.math.shl(usize, throttle_voltage, std.meta.bitCount(MotorSpeed))) / adc.max_throttle_voltage);
    const packet = MotorPacket{ .bits = .{ .speed = speed, .dir = dir, .channel = channel } };

    // Wait for the holding register to be transmitted, then send the given packet.
    // For some reason, the following loop will hang forever and any attempt to send a packet otherwise
    // is ignored by the motor controller. Note that this was found to work, and with the throttle input
    // driving it, without UART0 being connected.
    // This.. doesn't really matter as it makes much more sense to drive the motor directly from the throttle,
    // and to just turn off the motor via a relay if one needs to brake.
    while (motor_uart.isTxFifoFull()) intrin.loopHint();
    motor_uart.writeByte(packet.full);
}

fn initLunaUart() !void {
    c.gpio_set_function(uart0_tx_gpio, c.GPIO_FUNC_UART);
    c.gpio_set_function(uart0_rx_gpio, c.GPIO_FUNC_UART);

    try luna_uart.init();
    try luna_uart.enable(115200, .none, .one, .eight, true);

    // Before enabling RX we set the RX communication IRQ which is used to avoid timing issues
    c.irq_set_exclusive_handler(luna_uart_irq, lunaUartCommIrq);
    c.irq_set_enabled(luna_uart_irq, true);
    luna_uart.setRxIrq(.eighth, true);

    luna_uart.setTxRx(true, true);
    std.log.debug("Initialized UART0", .{});

    // Set the sensor to a known state. We configure it to use an 8 byte header which allows each packet to
    // trigger an RX IRQ without waiting.
    try enableChecksum();
    std.log.debug("Enabled checksum verification", .{});

    try setOutputFmt(.eight_byte_cm);
    std.log.debug("Set output mode", .{});

    try setOutputFreq(100);
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

    // Set the level to 8 bytes, which is also the size of our data packet.
    luna_uart.setRxIrq(.quarter, true);
    luna_uart.enableRx();

    std.log.debug("UART0 ready to use", .{});
}

fn initMotorUart() !void {
    c.gpio_set_function(uart1_tx_gpio, c.GPIO_FUNC_UART);

    try motor_uart.init();
    try motor_uart.enable(115200, .none, .one, .eight, false);
    motor_uart.enableTx();

    std.log.debug("UART1 ready to use", .{});
}

/// Reset the sensor.
inline fn resetSensor() !void {
    if (getLunaResponse(void, {}, u8, 2) != 0) return error.FailedToReset;
}

/// Enable checksum verification of transmitted data.
inline fn enableChecksum() !void {
    if (getLunaResponse(u8, 1, u8, 8) != 1) return error.FailedToEnableChecksum;
}

/// Change output format.
inline fn setOutputFmt(fmt: OutputFmt) !void {
    if (getLunaResponse(u8, @enumToInt(fmt), u8, 5) != @enumToInt(fmt)) return error.FailedToSetOutputFmt;
}

/// Reset saved settings.
inline fn restoreDefault() !void {
    if (getLunaResponse(void, {}, u8, 0x10) != 0) return error.FailedToRestoreDefaults;
}

/// Save current settings.
inline fn save() !void {
    if (getLunaResponse(void, {}, u8, 0x11) != 0) return error.FailedToSaveSettings;
}

/// Set a new output frequency, and verify the frequency was actually set.
inline fn setOutputFreq(freq: u16) !void {
    if (getLunaResponse(u16, freq, u16, 3) != freq) return error.FailedToSetFreq;
}

/// Toggle data output.
inline fn toggleOutput(enable: bool) !void {
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
    eight_byte_cm,
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

    while (true) : (i +%= 1) {
        intrin.cpsidi();

        // The IRQ will set the potential header slice to null if all data has been transmitted.
        if (CommState.potential_header == null) {
            intrin.cpsiei();
            break;
        }

        // Don't wait for the IRQ as our command may not have been sent.
        // If we spend 10ms waiting for something but nothing comes, resend the command.
        intrin.cpsiei();

        // Send command to begin with, as well as every 10ms (100us * 100).
        if (i % 100 == 0) {
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
        std.mem.copy(u8, std.mem.asBytes(&ret), potential_header[3 .. potential_header.len - 1]);

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
/// is configured to fire after receiving 4 bytes, or timing out.
/// Note that this IRQ is designed to be extremely reliable when the goal is to simply get the one and only
/// type of header being sent its way. Use of this with the ID-0 data packet is a bad idea since there are latency
/// and reliability concerns.
fn lunaUartCommIrq() callconv(.C) void {
    if (CommState.potential_header) |potential_header| blk: {
        // Read until expected sequence
        const header_byte = 0x5A;
        const model_header = [_]u8{ header_byte, @intCast(u8, potential_header.len), CommState.id };

        while (!luna_uart.isRxFifoEmpty()) {
            const current_byte = luna_uart.readByte() orelse {
                CommState.current_idx = 0;
                continue;
            };

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
                    const checksum = potential_header[potential_header.len - 1];
                    const calculated_checksum = check: {
                        var temp: u8 = 0;

                        for (potential_header[0 .. potential_header.len - 1]) |byte| {
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
                    for (potential_header[model_header.len..]) |byte| {
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

/// Combined UART IRQ. This is called when either the RX FIFO reaches 8 bytes
/// or timed out trying to wait for it to reach 8 bytes after reading some data.
fn lunaUartIrq() callconv(.C) void {
    // Generally we should expect the entire packet to be sent after an IRQ.
    // If the RTIM bit of the masked IRQ status is 1, then we had received fewer than 8 bytes;
    // clear the bit and prevent overruns by reading every byte in the FIFO queue.
    // Otherwise we can just read all the data packet bytes, clearing excess bytes that may have been
    // received.
    const status = luna_uart.getIrqStatus();

    if (status.rtim == 0) blk: {
        for (current_luna_data.bytes) |*byte| {
            // If we were unable to receive data, drain the RX FIFO.
            byte.* = luna_uart.readByte() orelse break :blk;
        }

        luna_data_valid = true;
    }

    while (!luna_uart.isRxFifoEmpty()) _ = luna_uart.readByte();
}
