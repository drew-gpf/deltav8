//! adc.zig: ADC interface
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
const uart = @import("uart.zig");

const c = @cImport({
    @cInclude("pico/stdlib.h");
    @cInclude("hardware/adc.h");
    @cInclude("hardware/irq.h");
});

/// The ADC input for GPIO26. This increases linearly until GPIO30, which selects the temperature sensor and is the final pin.
const adc_pin_base = 26;

/// GPIO pin to sample throttle output from.
const throttle_pin = 26;

/// The ADC input for the throttle pin
const throttle_adc_input = throttle_pin - adc_pin_base;

comptime {
    if (throttle_adc_input < 0 or throttle_adc_input > 3) {
        @compileLog(
            "Invalid throttle ADC input selected from throttle pin (must be GPIO26..29)",
            throttle_adc_input,
            throttle_pin,
        );
    }
}

/// Approximate min voltage the throttle reports when not cranked.
const throttle_vmin = 1050;

/// Approximate max voltage the throttle reports when fully cranked.
const throttle_vmax = 3050;

/// No. of seconds per ADC cycle (48MHz)
const adc_period = 1.0 / 48e+6;

/// No. of seconds per ADC conversion (5ms)
const throttle_period = 5.0 / 1.0e+3;

/// No. of ADC cycles within a throttle period, subtracted with the n + 1 offset of the ADC hardware.
const throttle_cycles = (throttle_period / adc_period) - 1.0;

/// The current (relative) voltage reading of the throttle, from throttle_vmin -> throttle_vmax.
var current_throttle_voltage: ?u12 = null;

/// Not included in the SDK headers for some reason
const throttle_fifo_irq = 22;

/// (Relative) exclusive upper-bound of reported throttle voltage
pub const max_throttle_voltage = throttle_vmax - throttle_vmin;

/// Initialize the ADC hardware and designate GPIO 26 as the pin to receive throttle output.
/// It will furthermore configure the ADC such that it fires an IRQ approx. every 5ms with new data from the ADC,
/// which will be converted and sent to getThrottleSpeed.
/// The throttle should be powered by the 3v3 out pin and any ground pin. todo: figure out better voltage ranges.
pub fn init() void {
    current_throttle_voltage = null;

    // Init the ADC hardware and GPIO pins.
    c.adc_init();
    c.adc_gpio_init(throttle_pin);
    c.adc_select_input(throttle_adc_input);

    // Enable the ADC FIFO for 1 sample, and enable the ADC IRQ.
    fifoSetup(true, false, 1, true, false);
    c.irq_set_exclusive_handler(throttle_fifo_irq, throttleIrq);
    c.irq_set_enabled(throttle_fifo_irq, true);
    adcIrqSetEnabled(true);

    // Configure the clock divisor such that a conversion is only performed after ~5ms.
    // Here we can tell the ADC to run every n + 1 ADC cycles, based on a 48MHz clock;
    // because a conversion takes 96 cycles, a value less than or equal to 96 will be equivalent to 0
    // which represents no delay.
    c.adc_set_clkdiv(throttle_cycles);
}

/// Enable the ADC's free running mode, causing it to continuously collect ADC samples.
pub fn enableIrqs() void {
    c.adc_run(true);
}

/// Get the raw (relative) voltage reading of the throttle, from [0, max_throttle_voltage), or null if none available.
/// IRQs must be disabled when entering this function.
pub fn getThrottleVoltage() ?u12 {
    if (current_throttle_voltage) |voltage| {
        current_throttle_voltage = null;
        return voltage - throttle_vmin;
    }

    return null;
}

/// Called when throttle ADC FIFO has been populated.
fn throttleIrq() callconv(.C) void {
    const voltage = c.adc_fifo_get();

    // If the error bit (bit 15) was set, we should just ignore this measurement; see rp2040 datasheet:
    // "Conversion errors produce undefined results, and the corresponding sample should be discarded. They indicate that
    // the comparison of one or more bits failed to complete in the time allowed. Normally this is caused by comparator
    // metastability, i.e. the closer to the comparator threshold the input signal is, the longer it will take to make a decision.
    // The high gain of the comparator reduces the probability that no decision is made"
    if ((voltage & (@as(u16, 1) << 15)) == 0) {
        // Account for slight inaccuracies in measurement by clamping voltage mins and maxs, from [vmin, vmax)
        current_throttle_voltage = switch (@truncate(u12, voltage)) {
            0...throttle_vmin => throttle_vmin,
            throttle_vmax...std.math.maxInt(u12) => throttle_vmax - 1,
            else => |val| val,
        };
    }
}

// The Pico SDK abuses UB in some of its macros so I have to manually tweak them here to avoid errors;
// note that this is autogenerated by the Zig compiler and was copied directly from the cImport cache.
inline fn fifoSetup(arg_en: bool, arg_dreq_en: bool, arg_dreq_thresh: u16, arg_err_in_fifo: bool, arg_byte_shift: bool) void {
    var en = arg_en;
    var dreq_en = arg_dreq_en;
    var dreq_thresh = arg_dreq_thresh;
    var err_in_fifo = arg_err_in_fifo;
    var byte_shift = arg_byte_shift;
    c.hw_write_masked(&@intToPtr([*c]volatile c.adc_hw_t, @as(c_uint, 1074053120)).*.fcs, ((((@as(c_uint, @boolToInt(!!en)) << @intCast(@import("std").math.Log2Int(c_uint), 0)) | (@as(c_uint, @boolToInt(!!dreq_en)) << @intCast(@import("std").math.Log2Int(c_uint), 3))) | (@as(c_uint, @as(c_uint, dreq_thresh)) << @intCast(@import("std").math.Log2Int(c_uint), 24))) | (@as(c_uint, @boolToInt(!!err_in_fifo)) << @intCast(@import("std").math.Log2Int(c_uint), 2))) | (@as(c_uint, @boolToInt(!!byte_shift)) << @intCast(@import("std").math.Log2Int(c_uint), 1)), (((@as(c_uint, 1) | @as(c_uint, 8)) | @as(c_uint, 251658240)) | @as(c_uint, 4)) | @as(c_uint, 2));
}

inline fn adcIrqSetEnabled(arg_enabled: bool) void {
    var enabled = arg_enabled;
    @intToPtr([*c]volatile c.adc_hw_t, @as(c_uint, 1074053120)).*.inte = @as(u32, @boolToInt(!!enabled));
}
