//! mmio.zig: MMIO atomic accessors (address_mapped.h)
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

/// Atomically XOR a peripheral register by a given value (XOR bits).
/// reg must point to the register.
pub fn xorReg(reg: anytype, val: @TypeOf(reg.*)) callconv(.Inline) void {
    @intToPtr(@TypeOf(reg), @ptrToInt(reg) + 0x1000).* = val;
}

/// Atomically OR a peripheral register by a given value (set bits).
/// reg must point to the register.
pub fn orReg(reg: anytype, val: @TypeOf(reg.*)) callconv(.Inline) void {
    @intToPtr(@TypeOf(reg), @ptrToInt(reg) + 0x2000).* = val;
}

/// Atomically AND a perhipheral register by the NOT of a given value (clear bits).
/// reg must point to the register.
pub fn andNotReg(reg: anytype, val: @TypeOf(reg.*)) callconv(.Inline) void {
    @intToPtr(@TypeOf(reg), @ptrToInt(reg) + 0x3000).* = val;
}

/// Atomically set some bits of a peripheral register to a given value.
/// reg must point to the register. bits is a bitmask of bits of val to set in reg.
/// This operation is not atomic if performed concurrently on the same bits.
pub fn setRegBits(reg: anytype, val: @TypeOf(reg.*), bits: @TypeOf(reg.*)) callconv(.Inline) void {
    // If a given bit in reg is 0 we can xor it by val to set, but if the bit is 1 then we must invert the bit in val
    // so, xor val by whatever is stored in reg to flip the bits of val when a bit in reg is 1.
    xorReg(reg, (val ^ reg.*) & bits);
}