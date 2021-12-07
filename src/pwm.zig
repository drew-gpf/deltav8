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

const servo_freq = 50.0;
const servo_period = 1.0 / servo_freq;
const servo_pwm_wrap = 65535;
const servo_wrap = @intToFloat(comptime_float, servo_pwm_wrap + 1);

/// The exact number of PWM cycles per half-millisecond.
/// This is because the PWM runs wrap*freq times per second.
const servo_half_ms_level = (servo_wrap * servo_freq) * (0.5 / 1.0e+3);

/// The rounded number of cycles to rotate the servo clockwise at full speed (0.5ms).
const servo_clockwise_level = @floatToInt(u16, @round(servo_half_ms_level));

/// The rounded number of cycles to stop rotating the servo (0.5ms * 3 = 1.5ms).
const servo_stop_level = @floatToInt(u16, @round(servo_half_ms_level * 3.0));

/// The rounded number of cycles to rotate the servo counterclockwise at full speed (0.5ms * 4 = 2.5ms).
const servo_counterclockwise_level = @floatToInt(u16, @round(servo_half_ms_level * 5.0));

/// The level to select to brake/rotate the servo.
const servo_rotate_level = servo_clockwise_level;

/// The level to select to unbrake the servo.
const servo_unrotate_level = servo_counterclockwise_level;

/// Amount of time, in seconds, the servo takes to fully rotate.
const servo_rotate_time = 6.5;

/// Number of servo periods for the servo to fully rotate in one direction.
const servo_rotate_cycles = @floatToInt(comptime_int, @ceil(servo_rotate_time * servo_freq));

/// The current number of times the servo IRQ has wrapped while braking. Reset when no longer braking.
var servo_wrap_cycles: usize = 0;
var servo_braking: bool = false;
var servo_actuating: bool = false;

/// Initialize the PWM hardware for the continuous servomotor on GPIO22.
pub fn init() !void {
    c.gpio_set_function(servo_pwm_gpio, c.GPIO_FUNC_PWM);

    // We need to configure the PWM freerunning counter such that it has
    // a 50hz frequency and can be high for 0.5ms, 1.5ms, or 2.5ms;
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
    // This results in a divider ~38.147 for a 125Mhz PWM clock and 50hz period (default).
    const pwm_clock = c.clock_get_hz(c.clk_sys);
    const divider = (servo_period * @intToFloat(f32, pwm_clock)) / servo_wrap;

    if (divider < 1.0 or divider >= 256.0)
        return error.InvalidPwmDivider;

    // Init the servo PWM and continually output a stop pulse (1.5ms).
    // When we need to brake, we rotate the servo and enable the wrap IRQ and count each until about
    // servo_rotate_time seconds had passed; we then set it back to the stop "position".
    // When we need to unbrake we rotate in the opposite direction, instead counting down the counter
    // which should have been incremented earlier.
    c.pwm_set_clkdiv(servo_pwm_slice, divider);
    c.pwm_set_wrap(servo_pwm_slice, servo_pwm_wrap);
    c.pwm_set_chan_level(servo_pwm_slice, servo_pwm_chan, servo_stop_level);
    c.irq_set_exclusive_handler(pwm_irq_wrap, servoWrapIrq);
    c.irq_set_enabled(pwm_irq_wrap, true);
    pwmSetEnabled(servo_pwm_slice, true);
}

/// Toggle the brake. Note that the brake takes 5.5 seconds to fully actuate and in this time
/// a wrap IRQ will be fired every ~20ms.
/// This function must be called with IRQs disabled. If braking, the motor must be off;
/// if unbraking, the caller needs to wait the full period (determined by isBrakeActuating) before
/// turning back on the motor.
pub inline fn setBrake(brake: bool) void {
    if (servo_braking == brake) return;

    // If we're braking, we should set the servo to rotate and enable the wrap IRQ.
    // Otherwise we should rotate in the opposite direction until we've reached the amount of rotation
    // we'd done previously.
    // To simplify logic we can invert the number of wrap cycles and only reset it to 0 once unbraking is complete.
    servo_braking = brake;
    if (!brake) servo_wrap_cycles = servo_rotate_cycles - servo_wrap_cycles;

    c.pwm_set_chan_level(servo_pwm_slice, servo_pwm_chan, if (brake) servo_rotate_level else servo_unrotate_level);
    c.pwm_set_irq_enabled(servo_pwm_slice, true);
    servo_actuating = true;
}

/// Returns whether or not the brake mechanism is still actuating;
/// In other words, this function indicates whether or not any torque is applied onto the brake.
/// So, if this function returns true, the motor speed must be 0.
pub inline fn isBrakeActuating() bool {
    return servo_actuating or servo_braking;
}

/// Called when the servo PWM block wraps every servo period. Used when actuating the brake.
fn servoWrapIrq() callconv(.C) void {
    if (servo_wrap_cycles >= servo_rotate_cycles) {
        // The servo's fully rotated, so we need to tell it to stop;
        // we can also disable this IRQ.
        c.pwm_set_chan_level(servo_pwm_slice, servo_pwm_chan, servo_stop_level);
        c.pwm_set_irq_enabled(servo_pwm_slice, false);
        servo_wrap_cycles = if (servo_braking) servo_rotate_cycles else 0;
        servo_actuating = false;
    } else {
        // Always increment after the check as the change in level should only take effect after this IRQ.
        servo_wrap_cycles += 1;
    }
}

// More Pico SDK UB breaking translate-c
fn pwmSetEnabled(arg_slice_num: c_uint, arg_enabled: bool) void {
    var slice_num = arg_slice_num;
    var enabled = arg_enabled;
    c.check_slice_num_param(slice_num);
    c.hw_write_masked(&@intToPtr([*c]volatile c.pwm_hw_t, @as(c_uint, 1074069504)).*.slice[slice_num].csr, @as(c_uint, @boolToInt(!!enabled)) << @intCast(@import("std").math.Log2Int(c_uint), 0), @as(c_uint, 1));
}
