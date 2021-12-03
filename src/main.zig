//! main.zig: deltav8 entry point
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
const logger = @import("logger.zig");
const uart = @import("uart.zig");
const intrin = @import("intrin.zig");
const clock = @import("clock.zig");
const adc = @import("adc.zig");
const pwm = @import("pwm.zig");

const c = @cImport({
    @cInclude("pico/stdlib.h");
    @cInclude("hardware/watchdog.h");
});

// Uncomment or change to enable logs for any build mode
//pub const log_level: std.log.Level = .debug;

/// stdlib log handler; no logging is done if stdio is disabled.
pub fn log(comptime level: std.log.Level, comptime scope: @TypeOf(.EnumLiteral), comptime format: []const u8, args: anytype) void {
    _ = scope;

    const prefix = "[" ++ @tagName(level) ++ "]: ";
    logger.log(prefix ++ format ++ "\n", args) catch {};
}

const decel = 1.108219628;
const max_vel = 2.882992752;
const stopping_dist = (max_vel * max_vel) / (2.0 * decel) + 1.0;
const stopping_dist_cm = @floatToInt(comptime_int, @round(stopping_dist * 100.0));

comptime {
    if (stopping_dist_cm > 1200) {
        @compileLog("Stopping dist too large", stopping_dist_cm);
    }

    if (stopping_dist_cm <= 0) {
        @compileLog("Stopping dist too small", stopping_dist_cm);
    }
}

fn mainWrap() !void {
    const watchdog_caused_reboot = c.watchdog_caused_reboot();

    if (watchdog_caused_reboot) {
        // Flash the LED on if the last reboot was caused by the watchdog. Note that
        // this will also flash the LED after the flash memory has been written,
        // because the bootrom will use the watchdog to reboot.
        c.gpio_init(c.PICO_DEFAULT_LED_PIN);
        c.gpio_set_dir(c.PICO_DEFAULT_LED_PIN, true);
        c.gpio_put(c.PICO_DEFAULT_LED_PIN, true);
    }

    // Configure a generous watchdog timer of 1 second in case something goes wrong.
    // This should be more than enough time for the sensor to init everything;
    // during testing it seems the sensor needs ~450ms to initialize (minus the 50ms delay for a valid data packet).
    // If stdio is enabled we use a 10 second timeout instead as UART failures will wait five seconds for someone
    // to connect the USB cable.
    c.watchdog_enable(if (logger.stdio_enabled) 10000 else 1000, true);

    // Disable some clocks to save power
    clock.configureClocks();
    logger.initLogger();

    // Init the ADC peripherals first. The UART is initialized last as it could take up to 500ms.
    adc.init();
    try pwm.init();
    try uart.init();

    // Magic sleep value of 50ms because the sensor's data is unreliable before this period,
    // where dist seems to be some value smaller than it actually should be.
    // This seems to only be needed after the sensor is initialized, so its value can remain fixed.
    c.watchdog_update();
    c.sleep_ms(50);
    c.watchdog_update();

    // Tell the ADC that we're ready to receive samples
    adc.enableIrqs();

    // Turn the watchdog LED back off. If the pico gets caught in an infinite reset loop,
    // the LED will flash every 50ms or so.
    if (watchdog_caused_reboot)
        c.gpio_put(c.PICO_DEFAULT_LED_PIN, false);

    // Set a new, lower watchdog of 50ms which is enough time to transmit 5 data packets.
    // Because we reset independent of the sensor there won't be much time between resets if something randomly goes wrong.
    c.watchdog_enable(if (logger.stdio_enabled) 500 else 50, true);

    // Whether or not each component had responded.
    var adc_response = false;
    var luna_response = false;

    // Whether or not we should be braking, and whether or not we were braking.
    var should_brake = false;
    var was_braking = false;

    // The next throttle voltage, or 0 if we are braking; and the last-known throttle voltage.
    var current_voltage: u12 = 0;
    var last_voltage = current_voltage;

    // Main event loop. We have two peripherals, the sensor and the throttle, continuously trying to give us some data;
    // the sensor through the UART RX IRQ and the throttle through the ADC's FIFO IRQ.
    // The sensor runs every ~10ms and the throttle (ADC) runs every ~5ms. To optimize power
    // we tell the M0+ to sleep until either one of these events occur; we then do something if either
    // are reporting new information. Following we then just go back to sleep.
    while (true) {
        // Prevent race conditions by masking IRQs; assumes that IRQs are unmasked at this point
        intrin.cpsidi();

        if (uart.getNextLuna()) |luna| {
            // If we already had an ADC response we can update the watchdog here.
            if (adc_response) {
                c.watchdog_update();

                luna_response = false;
                adc_response = false;
            } else {
                luna_response = true;
            }

            if (luna.getValidDist()) |dist| {
                should_brake = dist <= stopping_dist_cm;
            } else {
                // Assume that invalid distances represent far-away values or are too close for braking to matter
                should_brake = false;
            }
        }

        if (adc.getThrottleVoltage()) |voltage| {
            // If we already had a Luna response we can update the watchdog here.
            if (luna_response) {
                c.watchdog_update();

                luna_response = false;
                adc_response = false;
            } else {
                adc_response = true;
            }

            current_voltage = voltage;
        }

        // Always reset the current voltage to 0 if we should be braking. This covers the case where
        // voltage is nonzero but we need to brake and don't have a new throttle voltage,
        // because we will then set the speed to 0.
        if (should_brake)
            current_voltage = 0;

        const should_set_brake = should_brake != was_braking;
        was_braking = should_brake;

        const should_change_voltage = current_voltage != last_voltage;
        last_voltage = current_voltage;

        if (should_change_voltage) {
            // If we're releasing the brakes, we should do so before setting speed.
            if (should_set_brake and !should_brake) {
                std.debug.assert(current_voltage > 0);
                pwm.setBrake(false);
            }

            // controlSpeed should be called with IRQs unmasked as it performs integer division and may wait for
            // the UART hardware. If new throttle voltage is available we need to enable IRQs and set it here,
            // and then see what arrived in the meantime without executing WFI.
            intrin.cpsiei();
            uart.controlSpeed(current_voltage, .clockwise, .left);

            // If we're engaging the brakes, we should do so after setting speed.
            if (should_set_brake and should_brake) {
                std.debug.assert(current_voltage == 0);
                pwm.setBrake(true);
            }
        } else {
            // If not changing the motor speed, we can unconditionally set the brakes however.
            // In this case the motor speed will always be 0.
            if (should_set_brake) {
                std.debug.assert(current_voltage == 0);
                pwm.setBrake(should_brake);
            }

            // Wait for IRQs with IRQs masked to prevent any from getting lost.
            intrin.wfi();
            intrin.cpsiei();
        }
    }
}

export fn main() void {
    mainWrap() catch |e| {
        // Flash the LED
        c.gpio_init(c.PICO_DEFAULT_LED_PIN);
        c.gpio_set_dir(c.PICO_DEFAULT_LED_PIN, true);
        c.gpio_put(c.PICO_DEFAULT_LED_PIN, true);

        // Send message infinitely and wait to get killed by the watchdog
        while (true) {
            std.log.warn("Error: {}", .{e});
            c.sleep_ms(50);

            intrin.loopHint();
        }
    };
}
