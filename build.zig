//! build.zig: main build script for Zig code. Do not execute this directly; it relies on args given by zig_runner,
//! which relies on compiler flags given by CMake to determine the environment.
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
const CrossTarget = std.zig.CrossTarget;

pub fn build(b: *std.build.Builder) !void {
    const mode = b.standardReleaseOptions();
    const obj = b.addObject("deltav8_zig", "src/main.zig");
    b.getInstallStep().dependOn(&obj.step);

    // Set the target to exactly match the RP2040 IC; a 32-bit ARM Cortex M0+.
    obj.setTarget(.{
        .cpu_arch = .thumb,
        .os_tag = .freestanding,
        .cpu_model = CrossTarget.CpuModel{ .explicit = &std.Target.arm.cpu.cortex_m0plus },
        .abi = .eabi,
    });

    obj.setBuildMode(mode);

    // -ffunction-sections (-fdata-sections not supported)
    obj.link_function_sections = true;

    // Make the build_root/out/ folder
    {
        var build_root_dir = try std.fs.openDirAbsolute(b.build_root, .{});
        defer build_root_dir.close();

        build_root_dir.makeDir("out") catch |e| switch (e) {
            error.PathAlreadyExists => {},
            else => return e,
        };
    }

    // Make our own output dir instead of using zig-cache as we can't install this object file
    // but need to know where it is for any build.
    obj.setOutputDir(b.pathJoin(&.{ b.build_root, "out" }));

    const include_dirs_opt = b.option([]const []const u8, "include-dirs", "List of CC flags for including directories") orelse null;
    const defs_opt = b.option([]const []const u8, "defs", "List of CC flags for preprocessor definitions") orelse null;

    if (include_dirs_opt) |include_dirs| {
        for (include_dirs) |dir| {
            // Assume they just gave us the path
            obj.addIncludeDir(dir);
        }
    }

    if (defs_opt) |defs| {
        for (defs) |def| {
            // Assume general format <name> or <name>=<val>
            // <val> can be an arbitrary string so we don't split; it might contain a tokenizer.
            const name_str = blk: {
                for (def) |char, i| {
                    if (char == '=') {
                        break :blk def[0..i];
                    }
                }

                break :blk def;
            };

            const val_str = if (def.len != name_str.len) blk: {
                break :blk def[name_str.len + 1 ..];
            } else null;

            obj.defineCMacro(name_str, val_str);
        }
    }

    obj.red_zone = false;
    obj.disable_sanitize_c = true;
    obj.strip = true;
}
