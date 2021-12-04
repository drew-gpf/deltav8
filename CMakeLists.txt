# CMakeLists.txt: CMake makefile. Note that gitignore assumes you build in the build/ folder. On windows NMake must be used
# i.e. cmake .. -G "NMake Makefiles"
# Copyright (C) 2021 Drew P.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

cmake_minimum_required(VERSION 3.20)

include(pico_sdk_import.cmake)
project(deltav8)

set(ZIG_OBJ_PATH "${CMAKE_CURRENT_LIST_DIR}/out/deltav8_zig.o")
set(DUMMY_SRC_PATH "${CMAKE_CURRENT_LIST_DIR}/src/cmake_dummy.c")

# Supress warnings about deltav8_zig.o not existing.
# This is kind of a hack but it shouldn't(???) matter as the actual object file is generated
# along with the cmake dummy object file, so it will be present when linking.
# We also want to touch the dummy file so that zig build is ran on every make.
add_custom_target(fix_zig_obj_file
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_LIST_DIR}/out"
    COMMAND ${CMAKE_COMMAND} -E touch "${ZIG_OBJ_PATH}"
    COMMAND ${CMAKE_COMMAND} -E touch_nocreate "${DUMMY_SRC_PATH}"
    BYPRODUCTS "${ZIG_OBJ_PATH}"
)

# Create zig_runner, if needed. This is executed on every make.
add_custom_target(zig_runner
    COMMAND zig build
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/util"
)

add_executable(deltav8 ${DUMMY_SRC_PATH} ${ZIG_OBJ_PATH})
add_dependencies(deltav8 fix_zig_obj_file zig_runner)

# Reduce code size as we always use the V2 bootrom
add_compile_definitions(PICO_FLOAT_SUPPORT_ROM_V1=0)
add_compile_definitions(PICO_DOUBLE_SUPPORT_ROM_V1=0)

pico_sdk_init()

# Link with pico libs
target_link_libraries(deltav8 pico_stdlib hardware_adc hardware_pwm)

# Debug feature; enable stdio over usb. Note that UART0 and UART1 are reserved for hardware devices
# and thus enabling stdio via UART will trigger a compile error.
pico_enable_stdio_usb(deltav8 0)
pico_enable_stdio_uart(deltav8 0)

# Produce uf2 file
pico_add_extra_outputs(deltav8)

# Here we're faced with a problem: we need to give our Zig code the full command line it would normally receive
# such that it knows about pre-included files, directories, definitions, etc.
# To fix this, we execute a dummy C file but subvert the compiler launcher
# to a script which forwards the args to our Zig build script.
# If the file is not recognized it just runs the compiler executable in-place.
# Note that it will still compile the dummy file as CMake will try to link with its object file.
# This works seamlessly as it is the command given to compile our cmake_dummy and thus
# CMake will wait for us to finish before linking; meaning we can generate the Zig object file and link with it.
set_target_properties(deltav8
    PROPERTIES C_COMPILER_LAUNCHER "${CMAKE_CURRENT_LIST_DIR}/util/zig-out/bin/zig_runner"
)