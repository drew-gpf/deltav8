//! uart.zig: ARM PL011 UART interface for the RP2040, primarily wrappers around SDK functions
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

const c = @cImport({
    @cInclude("pico/stdlib.h");
    @cInclude("hardware/uart.h");
});

const std = @import("std");
const mmio = @import("mmio.zig");

/// Uart in-memory representation with helper functions provided.
/// Note that pointers to the uart struct must be volatile. Use uart0 or uart1
/// to interface with a uart device.
pub const Uart = extern struct {
    hw: c.uart_hw_t,

    const Self = @This();
    const SelfPtr = *volatile Self;

    pub fn getInst(this: SelfPtr) callconv(.Inline) *c.uart_inst_t {
        return @ptrCast(*c.uart_inst_t, &this.hw);
    }

    pub const WriteError = error{};
    pub const Writer = std.io.Writer(SelfPtr, WriteError, writeFn);

    fn writeFn(this: SelfPtr, bytes: []const u8) WriteError!usize {
        for (bytes) |byte| {
            while (!this.isWritable()) asm volatile ("" ::: "memory");
            this.writeByte(byte);
        }

        return bytes.len;
    }

    pub const ReadError = error{};
    pub const Reader = std.io.Reader(SelfPtr, ReadError, readFn);

    fn readFn(this: SelfPtr, bytes: []u8) ReadError!usize {
        for (bytes) |*byte| {
            while (!this.isReadable()) asm volatile ("" ::: "memory");
            byte.* = this.readByte();
        }

        return bytes.len;
    }

    /// Initialize a UART with the given baud rate. Must be called before other functions.
    /// This function returns the actual set baud rate; not all baud rates are supported,
    /// so the nearest baud rate is chosen.
    pub fn init(this: SelfPtr, baud_rate: usize) callconv(.Inline) usize {
        return c.uart_init(this.getInst(), baud_rate);
    }

    /// Deinitialize a UART. Call init() if the UART is to be used again.
    pub fn deinit(this: SelfPtr) callconv(.Inline) void {
        return c.uart_deinit(this.getInst());
    }

    /// Set a UART's baud rate, and return the actual set baud rate.
    /// Not all baud rates are supported, so the returned baud rate is
    /// the nearest available baud rate.
    pub fn setBaudRate(this: SelfPtr, baud_rate: usize) callconv(.Inline) usize {
        return c.uart_set_baudrate(this.getInst(), baud_rate);
    }

    /// Enable/disable UART CTS/RTS
    pub fn setHwFlow(this: SelfPtr, cts: bool, rts: bool) callconv(.Inline) void {
        // Inlined this to zig code as the sdk code won't get translated properly
        mmio.setRegBits(&this.hw.cr, (@as(usize, @boolToInt(cts)) << 15) | (@as(usize, @boolToInt(rts)) << 14),
            0x00004000 | 0x00008000);
    }

    /// Set the uart data format.
    /// data_bits must be [5, 8],
    /// stop_bits must be 1 or 2.
    pub fn setFormat(this: SelfPtr, data_bits: usize, stop_bits: usize, parity: Parity) callconv(.Inline) void {
        // Inlined this to zig code as the sdk code won't get translated properly
        std.debug.assert(data_bits >= 5 and data_bits <= 8);
        std.debug.assert(stop_bits == 1 or stop_bits == 2);

        mmio.setRegBits(
            &this.hw.lcr_h,

            ((data_bits - 5) << 5) |
            ((stop_bits - 1) << 3) |
            (@as(usize, @boolToInt(parity != .none)) << 1) |
            (@as(usize, @boolToInt(parity == .even)) << 2),

            0x00000060 |
            0x00000008 |
            0x00000002 |
            0x00000004
        );
    }

    /// Enable a UART's IRQ. The corresponding IRQ handler must be installed before
    /// calling this function.
    /// rx_level: if not null, enable IRQ when RX FIFO level is reached. If FIFO is disabled, specify .eighth.
    /// tx_level: if not null, enable IRQ when TX FIFO level is reached. If FIFO is disabled, specify .eighth.
    pub fn enableIrqFor(this: SelfPtr, rx_level: ?FifoLevel, tx_level: ?FifoLevel) callconv(.Inline) void {
        // We have to inline this from translated c regardless,
        // but we add crucial functionality in allowing the caller to specify
        // the level for TX and RX which is curiously missing from the SDK.
        this.hw.imsc = (@as(usize, @boolToInt(tx_level != null)) << 5) |
            (@as(usize, @boolToInt(rx_level != null)) << 4);

        if (rx_level) |level| {
            mmio.setRegBits(
                &this.hw.ifls,
                @as(usize, @enumToInt(level)) << 3,
                0x00000038
            );
        }

        if (tx_level) |level| {
            mmio.setRegBits(
                &this.hw.ifls,
                @as(usize, @enumToInt(level)),
                0x00000007
            );
        }
    }

    /// Get whether or not a uart is enabled.
    pub fn isEnabled(this: SelfPtr) callconv(.Inline) bool {
        return (this.hw.cr & 0x00000001) != 0;
    }

    /// Enable or disable the FIFO for a uart. By default the FIFO will be enabled.
    pub fn toggleFifo(this: SelfPtr, enable_fifo: bool) callconv(.Inline) void {
        mmio.setRegBits(
            &this.hw.lcr_h,
            @as(usize, @boolToInt(enable_fifo)) << 4,
            0x00000010
        );
    }

    /// Get whether or not there is space in the TX FIFO for a uart
    pub fn isWritable(this: SelfPtr) callconv(.Inline) bool {
        return (this.hw.fr & 0x00000020) == 0;
    }

    /// Block until the TX FIFO queue is empty
    pub fn waitTx(this: SelfPtr) callconv(.Inline) void {
        while ((this.hw.fr & 0x00000008) != 0) asm volatile ("" ::: "memory");
    }

    /// Get whether or not there is data in the RX FIFO queue
    pub fn isReadable(this: SelfPtr) callconv(.Inline) bool {
        return (this.hw.fr & 0x00000010) == 0;
    }

    /// Get a blocking writer for this UART
    pub fn getWriter(this: SelfPtr) callconv(.Inline) Writer {
        return Writer{ .context = this };
    }

    /// Write a single byte to the UART TX without blocking.
    /// This is unsafe if not called after knowing that the FIFO queue is writable.
    pub fn writeByte(this: SelfPtr, val: u8) callconv(.Inline) void {
        this.hw.dr = val;
    }

    /// Get a blocking reader for this UART
    pub fn getReader(this: SelfPtr) callconv(.Inline) Reader {
        return Reader{ .context = this };
    }

    /// Read a single byte from the UART RX without blocking.
    /// This is unsafe if not called after knowing that there is data in the FIFO queue.
    pub fn readByte(this: SelfPtr) callconv(.Inline) u8 {
        return @truncate(u8, this.hw.dr);
    }

    /// If enable is true, assert a break condition (TX held low). Clear it otherwise.
    pub fn toggleBreak(this: SelfPtr, enable: bool) callconv(.Inline) void {
        if (enable) {
            mmio.orReg(&this.hw.lcr_h, 0x00000001);
        } else {
            mmio.andNotReg(&this.hw.lcr_h, 0x00000001);
        }
    }

    /// Wait up to the given time, in microseconds, for the UART to be readable.
    pub fn readableWithinUs(this: SelfPtr, time: u32) callconv(.Inline) bool {
        return c.uart_is_readable_within_us(this.getInst(), time);
    }
};

pub const uart0 = @intToPtr(*volatile Uart, 0x40034000);
pub const uart1 = @intToPtr(*volatile Uart, 0x40038000);

/// see uart_parity_t
pub const Parity = enum {
    none,
    even,
    odd
};

pub const FifoLevel = enum {
    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 1/8 full (2 bytes)
    eighth,

    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 1/4 full (4 bytes)
    quarter,

    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 1/2 full (8 bytes)
    half,

    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 3/4 full (12 bytes)
    three_quarters,

    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 7/8 full (14 bytes)
    seven_eighths
};