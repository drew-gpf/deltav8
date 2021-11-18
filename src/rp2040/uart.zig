//! uart.zig: ARM PL011 UART interface for the RP2040
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
const uart_struct = @import("uart_struct.zig");

const c = @cImport({
    @cInclude("hardware/resets.h");
    @cInclude("hardware/clocks.h");
});

const Reg = @import("reg.zig").Reg;

const resets_hw = @intToPtr(*volatile c.resets_hw_t, 0x4000c000);
const reset_reg = @intToPtr(*Reg(u32, u32, .read_write), 0x4000c000 + @offsetOf(c.resets_hw_t, "reset"));

const uart0_reset_bit = @as(usize, 1) << 22;
const uart1_reset_bit = @as(usize, 1) << 23;

const UartRegs = uart_struct.UartRegs;
const FifoLevel = uart_struct.FifoLevel;
const WLen = uart_struct.WLen;
const Imsc = uart_struct.Imsc;
const Fr = uart_struct.Fr;

/// Uart in-memory representation with helper functions provided.
/// Use uart0 or uart1 to interface with a uart device.
pub const Uart = extern struct {
    regs: UartRegs,

    /// Initialize this UART. This must be called before other functions.
    /// Note that UARTs depend on the peripheral and system clocks.
    pub inline fn init(this: *Uart) !void {
        // Make sure the peripheral clock is enabled as the UARTs depend on it.
        if (c.clock_get_hz(c.clk_peri) == 0) return error.PeriClockDisabled;
        this.reset();
    }

    /// Reset this UART. It must be re-enabled before being usable again.
    pub inline fn reset(this: *Uart) void {
        const reset_bit = if (this == uart0) uart0_reset_bit else if (this == uart1) uart1_reset_bit else unreachable;

        // Assert this UART's reset pins using the RP2040's interface
        reset_reg.orFull(reset_bit);
        reset_reg.clearFull(reset_bit);
        while (resets_hw.reset_done & reset_bit == 0) asm volatile ("" ::: "memory");
    }

    // zig fmt: off

    /// Enable this UART for general use. This function must be called before calling other functions, but must be called after init() or reset().
    /// This function returns an error if an accurate baud rate could not be set.
    /// Parity checking applies for both TX and RX.
    /// FIFO enable applies for both TX and RX.
    /// This assumes the UART has not already been enabled.
    pub fn enable(
        this: *Uart,
        baud_rate: usize,
        parity: ParityCheck,
        stop_bits: StopBits,
        data_bits: WLen,
        fifo_enable: bool
    ) callconv(.Inline) error{Overflow}!void {
        const uartclk = c.clock_get_hz(c.clk_peri);

        // Baud rate divisor = uartclk/(16 * baud rate) (float) = BrdI + BrdF
        // This consists of a 16-bit integer part (BrdI) and a 6-bit fractional part (BrdF).
        // An integral representation of the fractional part is m = (int)((BrdF * 64) + 0.5) (from ARM datasheet).
        // This can be simplified to integer calculations by multiplying terms by two:
        // 2m = 2 * (int)(64BrdF + 0.5)
        // 2m ~= ((int)128BrdF + 1)
        // m ~= ((int)128BrdF + 1) / 2
        // Since BrdF is multiplied by 128, we can derive instead
        // divisor = 128 * uartclk/(16 * baud rate)
        // divisor = 8 * uartclk/(baud rate)
        // Here it must be noted that the integer value of the divisor, because it's multiplied by 128,
        // contains the integer value of the fractional component (also multiplied by 128):
        // (int)(128 * BrdF) = (int)divisor % 128
        // This allows a reasonably accurate fractional component to be calculated:
        // m ~= ((divisor % 128) + 1) / 2
        const divisor = (uartclk * 8) / baud_rate;

        // Because the divisor is an integer we can derive its integer component by dividing the divisor by 128.
        // This results in the value (int)(uartclk/(16 * baud rate)) = BrdI.
        const brdi = switch (try std.math.cast(u16, divisor / 128)) {
            0 => 1,
            else => |val| val,
        };

        // See above for brdf. Note that truncating the divisor to a u7 is the same as taking its
        // mod 128 value. Because we divide by 2 we guarantee that the value fits in a u6.
        const brdf = @intCast(u6, (@truncate(u7, divisor) +% 1) >> 1);

        // Note that brdi has a max value of 0xFFFF because it's a 16 bit int. If this happens, brdf must be 0.
        // Note also brdi must not be 0. If brdi is 1, brdf is ignored.
        this.regs.ibrd.writeAllBits(brdi);

        switch (brdi) {
            0xFFFF => this.regs.fbrd.writeAllBits(0),
            else => this.regs.fbrd.writeAllBits(brdf)
        }

        // Program LCR to set baud rate divisor. This must be done before enabling the UART.
        // Of course the LCR also contains parity bits, break condition, stop bits, word length, and FIFO enable.
        const parity_as_int = @enumToInt(parity);

        this.regs.lcr_h.setBits(.{
            .pen = @truncate(u1, parity_as_int),
            .eps = @truncate(u1, parity_as_int >> 1),
            .sps = @truncate(u1, parity_as_int >> 2),
            .stp2 = @enumToInt(stop_bits),
            .wlen = data_bits,
            .fen = @boolToInt(fifo_enable)
        });

        // Enable the UART.
        this.regs.cr.clearBits(.{ .rxe = 0, .txe = 0 });
        this.regs.cr.orBits(.{ .uart_en = 1 });
    }

    // zig fmt: on

    /// Enable or disable RX or TX. By default both are disabled.
    pub inline fn setTxRx(this: *Uart, tx_en: bool, rx_en: bool) void {
        this.regs.cr.setBits(.{ .rxe = @boolToInt(rx_en), .txe = @boolToInt(tx_en) });
    }

    /// Enable RX capabilities. This must be called before reading.
    pub inline fn enableRx(this: *Uart) void {
        this.regs.cr.orBits(.{ .rxe = 1 });
    }

    /// Enable TX capabilities. This must be called before writing.
    pub inline fn enableTx(this: *Uart) void {
        this.regs.cr.orBits(.{ .txe = 1 });
    }

    /// Disable RX capabilities. Do not perform reads after this function call.
    pub inline fn disableRx(this: *Uart) void {
        this.regs.cr.clearBits(.{ .rxe = 0 });
    }

    /// Disable TX capabilities. Do not perform writes after this function call.
    pub inline fn disableTx(this: *Uart) void {
        this.regs.cr.clearBits(.{ .txe = 0 });
    }

    /// Enable the UART's RX IRQ for the specified level, or null for no irq.
    /// If timeout_irq is true, an irq will also be fired if the FIFO queue has not received enough characters
    /// to reach the programmed level; if enough time has elapsed for four characters to be transmitted after
    /// initially receiving a transmission, it will trigger the IRQ.
    /// If the FIFO is disabled, any level is acceptable for the fifo level.
    pub inline fn setRxIrq(this: *Uart, level_opt: ?FifoLevel, timeout_irq: bool) void {
        if (level_opt) |level| {
            this.regs.ifls.setBits(.{ .rx_ifl_sel = level });
            this.regs.imsc.setBits(.{ .rxim = 1, .rtim = @boolToInt(timeout_irq) });
        } else {
            this.regs.imsc.clearBits(.{ .rxim = 0, .rtim = 0 });
        }
    }

    /// Enable the UART's TX IRQ for the specified level, or null for no irq.
    /// Note the IRQ is only triggered due to the TX FIFO being popped, or the hold register being transmitted.
    /// If the FIFO is disabled, any level is acceptable for the fifo level.
    pub inline fn setTxIrq(this: *Uart, level_opt: ?FifoLevel) void {
        if (level_opt) |level| {
            this.regs.ifls.setBits(.{ .tx_ifl_sel = level });
            this.regs.imsc.setBits(.{ .txim = 1 });
        } else {
            this.regs.imsc.clearBits(.{ .txim = 0 });
        }
    }

    /// Get masked IRQ status (UARTMIS). Each bit set indicates that the IRQ had just been triggered.
    pub inline fn getIrqStatus(this: *Uart) Imsc {
        return this.regs.mis.readBits();
    }

    /// Get whether or not the UART is transmitting data, or is waiting to transmit data from the TX FIFO queue.
    pub inline fn isBusy(this: *Uart) bool {
        return this.regs.fr.readBits().busy != 0;
    }

    /// Get whether or not the TX FIFO is full
    pub inline fn isTxFifoFull(this: *Uart) bool {
        return this.regs.fr.readBits().txff != 0;
    }

    /// Get whether or not the TX FIFO is empty. Note that it still might be transmitting data; see isBusy.
    pub inline fn isTxFifoEmpty(this: *Uart) bool {
        return this.regs.fr.readBits().txfe != 0;
    }

    /// Get whether or not the RX FIFO is full.
    pub inline fn isRxFifoFull(this: *Uart) bool {
        return this.regs.fr.readBits().rxff != 0;
    }

    /// Get whether or not the RX FIFO is empty. Note that the UART may still be receiving characters.
    pub inline fn isRxFifoEmpty(this: *Uart) bool {
        return this.regs.fr.readBits().rxfe != 0;
    }

    /// Get the frame register (fr). This is recommended if multiple flags must be checked at once,
    /// as multiple reads of the frame register otherwise have to occur.
    pub inline fn getFr(this: *Uart) Fr {
        return this.regs.fr.readBits();
    }

    /// Read a character from the data register. This will read a byte, the maximum size of a character,
    /// although its value will be smaller for smaller word lens.
    /// If the RX FIFO is empty or nothing is in the receive holding register, the return value of this function will not have
    /// any guaranteed value. Only read from the UART if data is known to be present beforehand.
    /// This will not read error bits.
    /// To perform a blocking read (usually outside of an IRQ) see getReader().
    /// Note however that the reader will work on byte-sized slices.
    pub inline fn readByte(this: *Uart) u8 {
        return this.regs.dr.readBits().data;
    }

    /// Write a character to the TX FIFO or holding register.
    /// If the TX FIFO or holding register are full (see isRxFifoFull), a write will cause an overrun.
    /// Do not write in that case. Do not write data values beyond the specified word length.
    /// To perform a blocking write (usually outside of an IRQ) see getWriter().
    /// Note however that the writer will write byte-sized values, so it is generally unrecommended
    /// if the word length is less than 8 bytes.
    pub inline fn writeByte(this: *Uart, val: u8) void {
        this.regs.dr.writeFull(val);
    }

    /// Get a blocking byte writer for this UART
    pub inline fn getWriter(this: *Uart) Writer {
        return Writer{ .context = this };
    }

    /// Get a blocking byte reader for this UART
    pub inline fn getReader(this: *Uart) Reader {
        return Reader{ .context = this };
    }

    pub const WriteError = error{};
    pub const Writer = std.io.Writer(*Uart, WriteError, writeFn);

    pub const ReadError = error{};
    pub const Reader = std.io.Reader(*Uart, ReadError, readFn);

    fn writeFn(this: *Uart, bytes: []const u8) WriteError!usize {
        for (bytes) |byte| {
            while (this.isTxFifoFull()) asm volatile ("" ::: "memory");
            this.writeByte(byte);
        }

        return bytes.len;
    }

    fn readFn(this: *Uart, bytes: []u8) ReadError!usize {
        for (bytes) |*byte| {
            while (this.isRxFifoEmpty()) asm volatile ("" ::: "memory");
            byte.* = this.readByte();
        }

        return bytes.len;
    }
};

pub const uart0 = @intToPtr(*Uart, 0x40034000);
pub const uart1 = @intToPtr(*Uart, 0x40038000);

// zig fmt: off

pub const StopBits = enum(u1) {
    one,
    two
};

/// bit 0 is pen, bit 1 is eps, bit 2 is sps
pub const ParityCheck = enum(u3) {
    /// No parity check
    none = 0b000,
    
    /// Odd parity
    odd = 0b001,

    /// Even parity
    even = 0b011,

    /// Parity bit always 1 (one)
    one = 0b101,

    /// Parity bit always 0 (zero)
    zero = 0b111,
};

// zig fmt: on
