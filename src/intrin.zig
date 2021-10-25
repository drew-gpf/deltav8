//! intrin.zig: small collection of ARM instructions
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

/// Disable IRQs.
pub fn cpsidi() callconv(.Inline) void {
    asm volatile("CPSID i" ::: "memory");
}

/// Enable IRQs.
pub fn cpsiei() callconv(.Inline) void {
    asm volatile("CPSIE i" ::: "memory");
}

/// Wait for next IRQ.
pub fn wfi() callconv(.Inline) void {
    asm volatile("WFI" ::: "memory");
}