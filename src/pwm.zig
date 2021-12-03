//! pwm.zig: PWM interface
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

const c = @cImport({
    @cInclude("pico/stdlib.h");
    @cInclude("hardware/pwm.h");
    @cInclude("hardware/clocks.h");
});

const servo_pwm_gpio = 22;
const servo_pwm_slice = c.pwm_gpio_to_slice_num(servo_pwm_gpio);
const servo_pwm_chan = c.pwm_gpio_to_channel(servo_pwm_gpio);

const servo_period = 20.0 / 1.0e+3;
const servo_pwm_wrap = 65535;
const servo_wrap = @intToFloat(comptime_float, servo_pwm_wrap + 1);

/// The exact number of PWM cycles per half-millisecond.
/// This is because the PWM runs pwm_clock/divider = pwm_clock/((servo_period * pwm_clock)/wrap)
/// = (pwm_clock * wrap)/(servo_period * pwm_clock) = wrap/servo_period times per second.
const servo_half_ms_level = (servo_wrap / servo_period) * (0.5 / 1.0e+3);

/// The rounded number of cycles to rotate the servo clockwise at full speed.
const servo_clockwise_level = @floatToInt(u16, @round(servo_half_ms_level));

/// The rounded number of cycles to stop rotating the servo (0.5ms * 3 = 1.5ms).
const servo_stop_level = @floatToInt(u16, @round(servo_half_ms_level * 3.0));

/// The rounded number of cycles to rotate the servo counterclockwise at full speed (0.5ms * 5 = 2.5ms).
const servo_counterclockwise_level = @floatToInt(u16, @round(servo_half_ms_level * 5.0));

/// Initialize the PWM hardware for the continuous servomotor on GPIO22.
pub fn init() !void {
    c.gpio_set_function(servo_pwm_gpio, c.GPIO_FUNC_PWM);

    // We need to configure the PWM freerunning counter such that it has
    // a 20ms period and can be high for 0.5ms, 1.5ms, or 2.5ms;
    // .5ms -> full speed clockwise
    // 1.5ms -> stop
    // 2.5ms -> full speed counter-clockwise
    // By default the PWM counter wraps at 0xFFFF + 1 and runs at the sys clock frequency.
    // Here we can set a PWM frequency divider and change the wrap value.
    // Note that {1 <= divider < 256, divider is an element of reals}.
    // wrap/(pwm_clock/divider) = servo_period
    // (wrap * divider)/pwm_clock = servo_period
    // wrap * divider = servo_period * pwm_clock
    // Here our wrap should be 0xFFFF + 1 (65536) to provide the best granularity;
    // 65536divider = servo_period * pwm_clock
    // divider = (servo_period * pwm_clock) / 65536.
    // This results in a divider ~38.147 for a 125Mhz PWM clock (default).
    const pwm_clock = c.clock_get_hz(c.clk_sys);
    const divider = (servo_period * @intToFloat(f32, pwm_clock)) / servo_wrap;

    if (divider < 1.0 or divider >= 256.0)
        return error.InvalidPwmDivider;

    // Init the servo PWM and output a 1.5ms pulse every 20ms; telling the servo to stop rotating.
    c.pwm_set_clkdiv(servo_pwm_slice, divider);
    c.pwm_set_wrap(servo_pwm_slice, servo_pwm_wrap);
    c.pwm_set_chan_level(servo_pwm_slice, servo_pwm_chan, servo_stop_level);
    pwmSetEnabled(servo_pwm_slice, true);
}

// Toggle the brake. Note that it may take some time for the brake to fully actuate.
pub fn setBrake(brake: bool) void {
    _ = brake;
}

// More Pico SDK UB breaking translate-c
fn pwmSetEnabled(arg_slice_num: c_uint, arg_enabled: bool) void {
    var slice_num = arg_slice_num;
    var enabled = arg_enabled;
    c.check_slice_num_param(slice_num);
    c.hw_write_masked(&@intToPtr([*c]volatile c.pwm_hw_t, @as(c_uint, 1074069504)).*.slice[slice_num].csr, @as(c_uint, @boolToInt(!!enabled)) << @intCast(@import("std").math.Log2Int(c_uint), 0), @as(c_uint, 1));
}
