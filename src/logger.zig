//! logger.zig: std.log.*** implementation
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

const c = @cImport({
    @cInclude("stdio.h");
    @cInclude("pico/stdlib.h");
});

const LogContext = struct {};
const LogError = error{StdoutEof};

const Logger = std.io.Writer(LogContext, LogError, logWrite);
const logger: Logger = .{ .context = undefined };

fn logWriteImpl(bytes: []const u8) LogError!void {
    var was_last_char_cr = false;

    for (bytes) |byte| {
        // Convert bare '\n' into '\r\n' as the SDK would do
        if (byte == '\n' and !was_last_char_cr) {
            if (c.putchar('\r') == c.EOF) return error.StdoutEof;
        } else {
            was_last_char_cr = byte == '\r';
        }

        // We can't guarantee that bytes is NULL-terminated, so print characters using putchar instead of printf.
        if (c.putchar(byte) == c.EOF) return error.StdoutEof;
    }
}

fn logWrite(context: LogContext, bytes: []const u8) LogError!usize {
    _ = context;

    // STOP ZIG FROM UNROLLING THIS INTO A GIANT SERIES OF PUTCHARS
    try @call(.{ .modifier = .never_inline }, logWriteImpl, .{bytes});

    return bytes.len;
}

/// Print to stdout. This does not know if stdio was initialized or if it's available.
pub inline fn log(comptime fmt: []const u8, args: anytype) LogError!void {
    if (stdio_enabled)
        try logger.print(fmt, args);
}

/// Initialize the logger. This must be called before any logging is done.
/// Note that this will init all stdio.
pub fn initLogger() void {
    if (stdio_enabled)
        c.stdio_init_all();
}

/// Whether or not there are potential stdio listeners.
pub const stdio_enabled =
    (@hasDecl(c, "LIB_PICO_STDIO_UART") and c.LIB_PICO_STDIO_UART == 1) or
    (@hasDecl(c, "LIB_PICO_STDIO_USB") and c.LIB_PICO_STDIO_USB == 1) or
    (@hasDecl(c, "LIB_PICO_STDIO_SEMIHOSTING") and c.LIB_PICO_STDIO_SEMIHOSTING == 1);

comptime {
    if (@hasDecl(c, "LIB_PICO_STDIO_UART") and c.LIB_PICO_STDIO_UART == 1)
        @compileError("stdio via UART is not supported; use USB instead.");
}
