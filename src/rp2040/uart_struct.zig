// zig fmt: off

//! uart_struct.zig: ARM PL011 UART register layout and definitions
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
const reg = @import("reg.zig");

const Reg = reg.Reg;

pub const UartRegs = extern struct {
    /// Data register; used to receive data (and error bits) or to transmit data.
    dr: Reg(u32, Dr, .read_write),

    /// Status register or error clear register; indicates error status of last TX or RX operation.
    /// Note that for RX (which reports error bits anyways), a read to DR is required to update status bits.
    sr_ecr: Reg(u32, TxRxStatus, .{ .read_write_to_clear = 0 }),

    rsvd0: [4]u32,

    /// Frame register
    fr: Reg(u32, Fr, .read_only),

    rsvd1: u32,

    ilpr: Reg(u32, u8, .read_write),

    /// Integer baud rate divisor
    ibrd: Reg(u32, u16, .read_write),

    /// Fractional baud rate register
    fbrd: Reg(u32, u6, .read_write),

    /// Bits 29 to 22 of the UART bit rate and line control register. Note this register must be written to in order to update ibrd or fbrd.
    /// Note that all LCR registers must not be updated when the UART is enabled or when completing a TX/RX after disabling the UART;
    /// In order to program any LCR register, follow the sequence:
    /// 1. disable the UART (see cr)
    /// 2. wait for the end of tx or rx for the current character (see fr)
    /// 3. flush the TX FIFO by disabling the fen bit (see lcr_h)
    /// 4. reprogram the LCR
    /// 5. enable the UART (see cr)
    /// FIFO integrity is not guaranteed if the brk bit is initiated or if the UART is disabled and re-enabled in the middle of a transmission.
    lcr_h: Reg(u32, LcrH, .read_write),

    /// UART control register. Contains the uart enable bit (uart_en) which is required for TX (txe) and RX (rxe).
    cr: Reg(u32, Cr, .read_write),

    /// Interrupt level select for TX and RX
    ifls: Reg(u32, Ifls, .read_write),

    /// Interrupt mask set/clear register
    /// Note that for the RP2040 only the combined interrupt is connected, so all IRQs go to the same handler.
    imsc: Reg(u32, Imsc, .read_write),

    /// Raw interrupt status register; has same format of the Imsc except that
    /// each bit indicates the reason(s) why the IRQ had occurred.
    ris: Reg(u32, Imsc, .read_only),

    /// Masked interrupt status register
    mis: Reg(u32, Imsc, .read_only),

    /// Interrupt clear register; when a bit is written with a value of 1, the corresponding interrupt is acknowledged.
    icr: Reg(u32, Imsc, .write_only),
    dmacr: Reg(u32, u3, .read_write),

    rsvd2: [997]u32,

    periph_id0: Reg(u32, u8, .read_only),
    periph_id1: Reg(u32, u8, .read_only),
    periph_id2: Reg(u32, u8, .read_only),
    periph_id3: Reg(u32, u8, .read_only),

    pcell_id0: Reg(u32, u8, .read_only),
    pcell_id1: Reg(u32, u8, .read_only),
    pcell_id2: Reg(u32, u8, .read_only),
    pcell_id3: Reg(u32, u8, .read_only),
};

pub const Dr = packed struct {
    data: u8,

    // Ignored on writes
    err: TxRxStatus
};

pub const Fr = packed struct {
    cts: u1,
    dsr: u1,
    dcd: u1,

    /// If 1, UART is busy transmitting data. Set when TX FIFO is non-empty; unset when shift register data has been transmitted.
    busy: u1,

    /// RX FIFO or receive holding register empty.
    rxfe: u1,

    /// TX FIFO or transmit holding register full.
    txff: u1,

    /// RX FIFO or receive holding register full.
    rxff: u1,

    /// TX FIFO or transmit holding register empty. Does not indicate if there is data in the TX shift register.
    txfe: u1,
    ri: u1
};

pub const LcrH = packed struct {
    /// If 1, a low level is output on TXD after completing the transmission of the current character. Software must set this bit to 1 for at least
    /// two complete frames to send a normal break command. Generally, keep this bit set to 0.
    brk: u1,

    /// If 1, parity checking and generation is enabled. See the eps and sps bits to select parity mode.
    pen: u1,

    /// If 1, even parity select for transmission and reception; this enforces an even number of 1s in both data and parity bits.
    /// If 0 and parity is enabled then an odd number of 1s are checked. Has no effect if parity checking is disabled.
    eps: u1,

    /// If 1, two stop bits are transmitted at the end of a frame. Note that RX logic won't check for 2 stop bits being received.
    stp2: u1,

    /// FIFO enable for TX and RX.
    fen: u1,

    /// Word length i.e. amount of data transmitted or received per frame.
    wlen: WLen,

    /// Stick parity select. Has no effect if pen = 0. If eps = 1, the parity is transmitted and checked as 0. Otherwise, it is transmitted and checked as 1.
    sps: u1
};

pub const Cr = packed struct {
    /// If 1, UART is enabled. If the UART is disabled in the middle of TX or RX, the current character is completed before stopping.
    uart_en: u1,
    sir_en: u1,
    sir_lp: u1,

    rsvd: u4,

    lbe: u1,

    /// TX enable; if 1, transmission is enabled.
    txe: u1,

    /// RX enable; if 1, receiving is enabled.
    rxe: u1,
    dtr: u1,
    rts: u1,
    out1: u1,
    out2: u1,

    /// If set, RTS hardware control flow is enabled, and data is only requested when there is space in the RX FIFO
    /// for it to be received.
    rts_en: u1,

    /// If set, CTS hardware control flow is enabled, and data is only transmitted when the nUARTCTS signal is asserted.
    cts_en: u1
};

pub const Ifls = packed struct {
    /// TX FIFO interrupt level select
    tx_ifl_sel: FifoLevel,
    
    /// RX FIFO interrupt level select
    rx_ifl_sel: FifoLevel,
};

pub const Imsc = packed struct {
    rimim: u1,
    ctsmim: u1,
    dcdmim: u1,
    dsrmim: u1,

    /// Toggle RX IRQ
    rxim: u1,

    /// Toggle TX IRQ
    txim: u1,

    /// Toggle RX timeout IRQ i.e. if data has been sitting in the RX FIFO for the amount of time taken to receive 4 chars,
    /// but it is not enough data to trigger an IRQ, an IRQ will be sent.
    rtim: u1,

    /// Toggle framing error IRQ
    feim: u1,

    /// Toggle parity error IRQ
    peim: u1,

    /// Toggle break error IRQ
    beim: u1,

    /// Toggle overrun error IRQ
    oeim: u1
};

pub const FifoLevel = enum(u3) {
    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 1/8 full (4 bytes)
    eighth,

    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 1/4 full (8 bytes)
    quarter,

    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 1/2 full (16 bytes)
    half,

    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 3/4 full (24 bytes)
    three_quarters,

    /// Trigger IRQ when FIFO is >= (RX) <= (TX) 7/8 full (28 bytes)
    seven_eighths
};

pub const WLen = enum(u2) {
    five,
    six,
    seven,
    eight
};

pub const TxRxStatus = packed struct {
    /// Frame error
    fe: u1,

    /// Parity error
    pe: u1,

    /// Break error
    be: u1,

    /// Overrun error
    oe: u1
};