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

        // We know the C compiler but we need to grep for its search prefix. The standard way to do this
        // is with -v while using other commands to allow it to function without specifying a file and by
        // stopping after preprocessing.
        // gcc -E -Wp,-v -
        // Of course, if we weren't using UNIX in the year 2021 this would all be sane and there would be some
        // API to query for this instead of relying on parsing text output, but oh well.
        {
            var child = try std.ChildProcess.init(&[_][]const u8{ args[1], "-E", "-Wp,-v", "-" }, allocator);
            defer child.deinit();

            // For some reason the command will output to stderr so we need to pipe stderr
            // and ignore stdout.
            child.stderr_behavior = .Pipe;
            child.stdout_behavior = .Ignore;

            // We also need stdin to have an end-of-file so it doesn't wait on input forever.
            // The zig stdlib lets us do this by setting stdin to .Ignore which pipes it to /dev/null or similar.
            child.stdin_behavior = .Ignore;

            try child.spawn();
            var should_kill = true;

            errdefer if (should_kill) {
                _ = child.kill() catch {};
            };

            // Get the output. In here, somewhere, is the correct prefix.
            // Here Zig will include other files for us so we only need a prefix containing stdio.h.
            const stderr_reader = child.stderr.?.reader();
            const stderr = (try stderr_reader.readUntilDelimiterOrEofAlloc(allocator, 0, std.math.maxInt(usize))).?;

            const search_prefix = blk: {
                // UNIX paths are all kinds of insane because they accept every character except \0 and /.
                // This means they can have \r or \n or \" inside of them which can break our parsing.
                // I don't care. If you want to install your GCC in an invalid path that can't even be created
                // using bash, then go ahead, but it's theoretically impossible to parse anything without knowing
                // when the paths begin and end (without trying to access them manually), in a cross-platform way.
                // So assume everything is nice: the paths we want come just after the line
                // "#include <...> search starts here:"
                // and are prefixed with a space char and are each on their own lines.
                // We also assume that the first invalid path marks the end of the given list.
                var lines = std.mem.tokenize(u8, stderr, "\n\r");
                var find_search_prefix: bool = false;

                while (lines.next()) |line| {
                    if (find_search_prefix) {
                        // Ignore space at the start and go back one directory, while also parsing the
                        // weird ../../ sequences already in the path.
                        const full_path = try std.fs.path.resolve(allocator, &[_][]const u8{ line[1..], ".." });

                        // Open this dir to see if it's valid
                        var path_dir = std.fs.openDirAbsolute(full_path, .{}) catch break;
                        defer path_dir.close();

                        // If we can access include/stdio.h, this is probably the prefix dir.
                        path_dir.access(try std.fs.path.join(allocator, &[_][]const u8{ "include", "stdio.h" }), .{}) catch continue;

                        break :blk full_path;
                    } else if (std.mem.eql(u8, line, "#include <...> search starts here:")) {
                        find_search_prefix = true;
                    }
                }

                std.log.err("Failed to find search prefix. Full GCC output:\n{s}", .{ stderr });
                return error.FailedToFindSearchPrefix;
            };

            try build_args.append("--search-prefix");
            try build_args.append(search_prefix);

            should_kill = false;
            _ = try child.wait();
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