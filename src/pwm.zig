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
    @cInclude("hardware/irq.h");
});

const pwm_irq_wrap = 4;

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

/// The level to select to brake/rotate the servo.
const servo_rotate_level = servo_counterclockwise_level;

/// Amount of time, in seconds, the servo takes to fully rotate.
const servo_rotate_time = 5.5;

/// Number of 20ms (servo period/PWM wrap) cycles for the servo to fully rotate.
const servo_rotate_cycles = @floatToInt(comptime_int, @ceil(servo_rotate_time / servo_period));

/// The current number of times the servo IRQ has wrapped while braking. Reset when no longer braking.
var servo_wrap_cycles: usize = 0;

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

    // Init the servo PWM but do not output a pulse.
    // Because the handbrake will resist the servo, we can make this easy for ourselves:
    // when not braking, no PWM signal is sent and the servo will naturally reset to a neutral position.
    // Otherwise, we set a rotation pulse for the amount of time needed to brake;
    // to do this efficiently we can just enable IRQs for when the PWM wraps.
    // Then, we disable the IRQ and set the level to the stop position.
    c.pwm_set_clkdiv(servo_pwm_slice, divider);
    c.pwm_set_wrap(servo_pwm_slice, servo_pwm_wrap);
    c.pwm_set_chan_level(servo_pwm_slice, servo_pwm_chan, servo_rotate_level);
    pwmSetEnabled(servo_pwm_slice, true);
}

/// Toggle the brake. Note that the brake takes 5.5 seconds to fully actuate and in this time
/// a wrap IRQ will be fired every 20ms.
pub fn setBrake(brake: bool) void {
    if (brake) {
        // We're braking, so we should set the servo to rotate and enable the wrap IRQ.
        c.pwm_set_counter(servo_pwm_slice, 0);
        c.pwm_set_irq_enabled(servo_pwm_slice, true);
        pwmSetEnabled(servo_pwm_slice, true);
    } else {
        // We don't need to brake, so tell the servo to release.
        // Note that if the PWM output is currently high it will remain high, so we need
        // to manually override the GPIO pin (as explained in the pico SDK).
        // We also need to set the level back to rotate as it may be set to the stop level.
        pwmSetEnabled(servo_pwm_slice, false);
        c.gpio_put(servo_pwm_gpio, false);
        c.pwm_set_irq_enabled(servo_pwm_slice, false);
        c.pwm_set_chan_level(servo_pwm_slice, servo_pwm_chan, servo_rotate_level);
        servo_wrap_cycles = 0;
    }
}

/// Called when the servo PWM block wraps, approx. every 20ms. Only used when braking.
fn servoWrapIrq() callconv(.C) void {
    // Always increment the cycle counter before doing the comparison;
    // this IRQ will be fired 20ms after initially telling the servo to rotate.
    servo_wrap_cycles += 1;

    if (servo_wrap_cycles >= servo_rotate_cycles) {
        // The servo's fully rotated, so we need to tell it to stop.
        // We can also disable this IRQ.
        c.pwm_set_chan_level(servo_pwm_slice, servo_pwm_chan, servo_stop_level);
        c.pwm_set_irq_enabled(servo_pwm_slice, false);
    }
}

// More Pico SDK UB breaking translate-c
fn pwmSetEnabled(arg_slice_num: c_uint, arg_enabled: bool) void {
    var slice_num = arg_slice_num;
    var enabled = arg_enabled;
    c.check_slice_num_param(slice_num);
    c.hw_write_masked(&@intToPtr([*c]volatile c.pwm_hw_t, @as(c_uint, 1074069504)).*.slice[slice_num].csr, @as(c_uint, @boolToInt(!!enabled)) << @intCast(@import("std").math.Log2Int(c_uint), 0), @as(c_uint, 1));
}
