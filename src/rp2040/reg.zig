//! reg.zig: Generic register data structure
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

/// Access constraints for a register. write-to-clear registers contain an integer value to write when clearing.
pub const AccessConstraints = union(enum) {
    read_only,
    write_only,
    read_write,
    write_to_clear: comptime_int,
    read_write_to_clear: comptime_int,
};

/// Define a register whose layout is determined by a bits type with the alignment and size of a container type.
/// The container type must be the same or greater size, in bytes, of the bits type.
/// Unless it is required, do not access fields directly. Instead use helper functions.
/// This is because it will allow you to catch, say, writes to read-only registers, or reads to write-only registers at compile time.
/// It also makes it more clear that the register's value is volatile and won't be cached by the compiler or optimized out for multiple reads/writes.
/// Furthermore it will automatically declare the register contents as volatile. Otherwise you must explicitly annotate a volatile pointer.
/// The bits type determines the layout of the register. It may be a packed struct or an integer.
pub fn Reg(comptime Container: type, comptime Bits: type, comptime constraint: AccessConstraints) type {
    comptime {
        if (@sizeOf(Container) < @sizeOf(Bits)) {
            @compileLog(
                "Register's container is smaller than its bit size. When using a packed struct make sure " ++
                    "non power-of-two ints do not exceed 16 bits.",
                @sizeOf(Container),
                @sizeOf(Bits),
            );
        }

        switch (constraint) {
            .write_to_clear, .read_write_to_clear => |val| {
                if (val > std.math.maxInt(Container)) {
                    @compileLog(
                        "Register is marked as write-to-clear but its clear value is larger than can be " ++
                            "possibly written to its container.",
                        constraint,
                        val,
                        std.math.maxInt(Container),
                    );
                }
            },
            .read_only, .write_only, .read_write => {},
        }
    }

    return extern union {
        /// Actual representation. Do not read directly unless its pointer is volatile.
        bits: Bits,

        /// Full representation of storage space in memory. Do not read directly unless its pointer is volatile.
        full: Container,

        const This = @This();

        const Self = *volatile This;
        const ConstSelf = *const volatile This;

        pub const access_type = constraint;

        /// Initialize register bitfields with a named tuple, where unmentioned bits are initialized to zero.
        /// This does not modify padding bits. So, it is recommended to use bitpacked types for Bits.
        pub inline fn makeBits(bits: anytype) Bits {
            comptime if (@typeInfo(Bits) == .Int) return bits;

            var ret = std.mem.zeroes(Bits);
            const bit_fields = std.meta.fields(@TypeOf(bits));

            inline for (bit_fields) |field| {
                @field(ret, field.name) = @field(bits, field.name);
            }

            return ret;
        }

        /// makeBits; but return the full container value instead. This has the effect of zero-initializing bits that are beyond
        /// the Bits type.
        pub inline fn makeFull(bits: anytype) Container {
            var ret = This{ .full = 0 };
            ret.bits = makeBits(bits);

            return ret.full;
        }

        /// Whether or not this register is readable.
        pub inline fn isReadable() bool {
            return switch (access_type) {
                .read_only, .read_write, .read_write_to_clear => true,

                .write_only, .write_to_clear => false,
            };
        }

        /// Whether or not this register is writable. If the register is write-to-clear, it is not included here.
        pub inline fn isWritable() bool {
            return switch (access_type) {
                .write_only, .read_write => true,

                .read_only,
                .write_to_clear,
                .read_write_to_clear,
                => false,
            };
        }

        /// Get the write-to-clear value, or null if not write-to-clear.
        pub inline fn getClearVal() ?comptime_int {
            return switch (access_type) {
                .write_to_clear, .read_write_to_clear => |val| val,
                .read_only, .read_write, .write_only => null,
            };
        }

        /// Read the entire register memory. Note that because regs are volatile the result of this must be cached if called multiple times,
        /// and if the caller knows that the bit values would not change.
        pub inline fn readFull(this: ConstSelf) Container {
            comptime if (!isReadable()) @compileError("Tried to read write-only register");
            return this.full;
        }

        /// Read bits. Note that because regs are volatile the result of this must be cached if called multiple times,
        /// and if the caller knows that the bit values would not change.
        pub inline fn readBits(this: ConstSelf) Bits {
            comptime if (!isReadable()) @compileError("Tried to read write-only register");
            return this.bits;
        }

        /// Write the entire register memory. Note that this is a volatile write, so multiple writes will always generate multiple writes to memory,
        /// instead of just one.
        pub inline fn writeFull(this: Self, full: Container) void {
            comptime if (!isWritable()) @compileError("Tried to write read-only register or write-to-clear register");
            this.full = full;
        }

        /// Write as bits. Note that this is a volatile write, so multiple writes will always generate multiple writes to memory,
        /// instead of just one. This will zero the entire register.
        pub inline fn writeAllBits(this: Self, bits: Bits) void {
            comptime if (!isWritable()) @compileError("Tried to write read-only register or write-to-clear register");
            this.full = makeFull(bits);
        }

        /// Clear the register. This is only valid for write-to-clear registers.
        /// Multiple writes will generate multiple memory writes, like with other operations.
        pub inline fn writeClearVal(this: Self) void {
            this.full = getClearVal() orelse @compileError("Tried to clear a register which is not write-to-clear.");
        }

        /// Atomically XOR the register's bits by the full value. This is only valid for read-write registers.
        pub inline fn xorFull(this: Self, full: Container) void {
            comptime if (access_type != .read_write) @compileError("Tried to XOR a register which was not read-write.");
            @intToPtr(*Container, @ptrToInt(&this.full) + 0x1000).* = full;
        }

        /// Atomically OR the register's bits by the full value. This is only valid for read-write registers.
        pub inline fn orFull(this: Self, full: Container) void {
            comptime if (access_type != .read_write) @compileError("Tried to OR a register which was not read-write.");
            @intToPtr(*Container, @ptrToInt(&this.full) + 0x2000).* = full;
        }

        /// Atomically AND the register's bits by ~(full); clear the specified bits. This is only valid for read-write registers.
        pub inline fn clearFull(this: Self, full: Container) void {
            comptime if (access_type != .read_write) @compileError("Tried to clear a register's bits which was not read-write.");
            @intToPtr(*Container, @ptrToInt(&this.full) + 0x3000).* = full;
        }

        /// Atomically XOR the register's bits. This is only valid for read-write registers.
        /// Bits not specified in `bits` are assumed to be zero.
        pub inline fn xorBits(this: Self, bits: anytype) void {
            this.xorFull(makeFull(bits));
        }

        /// Atomically OR the register's bits. This is only valid for read-write registers.
        /// Bits not specified in `bits` are assumed to be zero.
        pub inline fn orBits(this: Self, bits: anytype) void {
            this.orFull(makeFull(bits));
        }

        /// Atomically AND the register's bits by ~(bits); clear the specified bits. This is only valid for read-write registers.
        /// Bits not specified in `bits` are assumed to be zero. Note that this only works for values specified in each field;
        /// a field that is specified but has some bits set to 0 in the actual value will not have those bits cleared.
        /// To clear every bit of all specified fields, see clearBits.
        pub inline fn clearBitsMask(this: Self, bits: anytype) void {
            this.clearFull(makeFull(bits));
        }

        /// Atomically AND the register's bits by the NOT of the bits of the specified bitfields in `bits`; clear the bitfields specified in `bits`.
        /// This is only valid for read-write registers.
        /// Note this only works for registers whose bits field is a struct.
        pub inline fn clearBits(this: Self, bits: anytype) void {
            comptime if (This.access_type != .read_write) @compileError("Tried to clear a register's bits which was not read-write.");

            // Here we assume that the user just wants to specify all of the given bits;
            // build a bitmask of the values of each specified bit, where each bit is specified if it is present
            // within the fields in `bits`.
            const bitmask = comptime blk: {
                comptime var temp: Container = 0;
                const bit_fields = std.meta.fields(@TypeOf(bits));

                for (bit_fields) |field| {
                    const first_bit = @bitOffsetOf(Bits, field.name);
                    const bit_size = @bitSizeOf(@TypeOf(@field(this.bits, field.name)));

                    // Set bitmask of all bits covered by this bitfield, shifted by its bit position.
                    temp |= ((@as(Container, 1) << bit_size) - 1) << first_bit;
                }

                break :blk temp;
            };

            this.clearFull(bitmask);
        }

        /// Atomically set specific bits. This is only valid for read-write registers.
        /// Bits not specified in `bits` are assumed to be zero. Note that this is not atomic for
        /// concurrent writes to the specified bits.
        /// Most callers will wish to use setBits which is more convenient. This function exists if they wish instead
        /// to set a part of the bits of a given field.
        /// Note that bits and which_bits must specify the same bits. which_bits may contain bit values of zero;
        /// these bits will not be set.
        pub inline fn setBitsMask(this: Self, bits: anytype, which_bits: anytype) void {
            comptime {
                if (@typeInfo(Bits) != .Int) {
                    const bit_fields = std.meta.fields(@TypeOf(bits));
                    const which_bit_fields = std.meta.fields(@TypeOf(which_bits));

                    if (@TypeOf(bit_fields) != @TypeOf(which_bit_fields) or bit_fields.len != which_bit_fields.len)
                        @compileLog("mismatch between bits and which_bits: must specify the same bitfields, type", bit_fields.len, which_bit_fields.len);

                    comptime var num_eql: usize = 0;

                    for (bit_fields) |field, i| {
                        if (std.mem.eql(u8, field.name, which_bit_fields[i].name)) num_eql += 1;
                    }

                    if (num_eql != bit_fields.len)
                        @compileLog("Some fields are present inside which_bits which aren't present inside bits.", bit_fields.len - num_eql, bit_fields.len);
                }
            }

            // If a given bit in full is 0 we can xor it by bits to set, but if the bit is 1 then we must invert the bit in bits
            // so, xor bits by whatever is stored in full to flip the bits of bits when a bit in full is 1.
            this.xorFull((makeFull(bits) ^ this.readFull()) & makeFull(which_bits));
        }

        /// Atomically set the specified bitfields. This is only valid for read-write registers.
        /// Bits not specified in `bits` are assumed to be zero. Note that this is not atomic for
        /// concurrent writes to the specified bitfields.
        /// Note also that this will overwrite the entire specified bitfields. To avoid this,
        /// use setBitsMask, where each bitfield has its own bitmask to specify how it's set.
        /// This assumes that the `Bits` type is a packed struct. Use setBitsMask if this is not the case.
        pub inline fn setBits(this: Self, bits: anytype) void {
            // Here we assume that the user just wants to specify all of the given bits;
            // build a bitmask of the values of each specified bit, where each bit is specified if it is present
            // within the fields in `bits`.
            const bitmask = comptime blk: {
                comptime var temp: Container = 0;
                const bit_fields = std.meta.fields(@TypeOf(bits));

                for (bit_fields) |field| {
                    const first_bit = @bitOffsetOf(Bits, field.name);
                    const bit_size = @bitSizeOf(@TypeOf(@field(this.bits, field.name)));

                    // Set bitmask of all bits covered by this bitfield, shifted by its bit position.
                    temp |= ((@as(Container, 1) << bit_size) - 1) << first_bit;
                }

                break :blk temp;
            };

            this.xorFull((makeFull(bits) ^ this.readFull()) & bitmask);
        }
    };
}
