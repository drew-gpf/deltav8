//! zig_runner.zig: a small program to interpret the CC command line which we give to the final build script.
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

pub fn main() !u8 {
    // We're given as args a GCC command line and we must interpret them
    // and give them to the main build script.
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();

    const allocator = &arena.allocator;
    const args = try std.process.argsAlloc(allocator);

    // Check the last arg; it should end with cmake_dummy.c (case insensitive).
    // If it does we want to invoke zig build with the given args.
    // In either case we will build the object file verbaitim as we're subverting all C compilation.
    // todo: this is stupid but it works for now
    // todo: we can cache all of this in zig-cache since it needs to be deleted when cmake is ran
    if (std.ascii.endsWithIgnoreCase(args[args.len-1], "cmake_dummy.c")) {
        var build_args = std.ArrayList([]const u8).init(allocator);
        defer build_args.deinit();

        try build_args.append("zig");
        try build_args.append("build");

        // The way we're executed in the build script we know the path of the zig_runner executable;
        // util/zig-out/bin, which is always an absolute path. We want to go back by 3.
        const project_root = try std.fs.path.resolve(allocator, &[_][]const u8{
            std.fs.path.dirname(args[0]) orelse unreachable, // util/zig-out/bin
            "..", // util/zig-out
            "..", // util
            ".." // <root>
        });

        // Because we know the C compiler we need to do another thing: get the sysroot it uses.
        // We then pass this to --sysroot if something was given.
        {
            var child = try std.ChildProcess.init(&[_][]const u8{ args[1], "-print-sysroot" }, allocator);
            defer child.deinit();

            // Set behavior to pipe. In doing so when we spawn the process it will write to the stdout file.
            child.stdout_behavior = .Pipe;
            try child.spawn();

            var should_kill = true;
            errdefer if (should_kill) {
                _ = child.kill() catch {};
            };

            // Read sysroot path from stdout
            const stdout_reader = child.stdout.?.reader();
            const sysroot_path_opt = try stdout_reader.readUntilDelimiterOrEofAlloc(allocator, 0, std.math.maxInt(usize));

            if (sysroot_path_opt) |sysroot_path| {
                // sysroot_path could contain a \n or \r\n sequence which will completely fuck up everything, so try
                // to look for that first.
                const first_bad_char = blk: {
                    for (sysroot_path) |char, i| {
                        if (char == '\n' or char == '\r') break :blk i;
                    }

                    break :blk sysroot_path.len;
                };

                try build_args.append("--search-prefix");
                try build_args.append(sysroot_path[0..first_bad_char]);
            }

            should_kill = false;
            switch (try child.wait()) {
                .Exited => |val| {
                    if (val != 0) return val;
                },
                .Signal => return error.Signal,
                .Stopped => return error.Stopped,
                .Unknown => return error.Unknown,
            }
        }

        var zig_opt: ?[]const u8 = "-Drelease-fast";

        for (args) |arg| {
            // CMake plays nicely here and we can just assume that include directories look like
            // -I<path> and preprocessor defines look like -D<name> or -D<name>=<val>
            // We specify multiple values by just specifying the arg given to zig build for each arg type.
            // We also want to look for the CMake release type so it can be specified.
            if (arg.len > 2) {
                if (std.mem.startsWith(u8, arg, "-I")) {
                    try build_args.append(try std.fmt.allocPrint(allocator, "-Dinclude-dirs={s}", .{ arg[2..] }));
                } else if (std.mem.startsWith(u8, arg, "-D")) {
                    const def = arg[2..];
                    const cmake_build_type_str = "PICO_CMAKE_BUILD_TYPE";

                    // PICO_CMAKE_BUILD_TYPE incidentally contains the CMake build type so we can specify it here.
                    if (std.mem.startsWith(u8, def, cmake_build_type_str)) {
                        // Skip the =" sequence following PICO_CMAKE_BUILD_TYPE
                        const build_type_base = def[cmake_build_type_str.len+2..];
                        const build_type_str = build_type_base[0..build_type_base.len-1];

                        // Debug -> null
                        // Release -> -Drelease-fast
                        // RelWithDebInfo -> -Drelease-safe
                        // MinSizeRel -> -Drelease-small
                        if (std.mem.eql(u8, build_type_str, "Debug")) {
                            zig_opt = null;
                        } else if (std.mem.eql(u8, build_type_str, "Release")) {
                            zig_opt = "-Drelease-fast";
                        } else if (std.mem.eql(u8, build_type_str, "RelWithDebInfo")) {
                            zig_opt = "-Drelease-safe";
                        } else if (std.mem.eql(u8, build_type_str, "MinSizeRel")) {
                            zig_opt = "-Drelease-small";
                        } else unreachable;
                    }

                    try build_args.append(try std.fmt.allocPrint(allocator, "-Ddefs={s}", .{ def }));
                }
            }
        }

        if (zig_opt) |zig_opt_unwr| {
            try build_args.append(zig_opt_unwr);
        }

        // Execute this zig build in the root dir
        var child = try std.ChildProcess.init(build_args.items, allocator);
        defer child.deinit();

        // Note: child.cwd will be deprecated once they fix cwd_dir on windows
        child.cwd = project_root;

        switch (try child.spawnAndWait()) {
            .Exited => |val| {
                if (val != 0) return val;
            },
            .Signal => return error.Signal,
            .Stopped => return error.Stopped,
            .Unknown => return error.Unknown,
        }
    }

    // Compile object files unconditionally. This is made extremely easy by the fact that
    // the compiler is given in the command line.
    var child = try std.ChildProcess.init(args[1..], allocator);
    defer child.deinit();

    return switch (try child.spawnAndWait()) {
        .Exited => |val| val,
        .Signal => error.Signal,
        .Stopped => error.Stopped,
        .Unknown => error.Unknown,
    };
}