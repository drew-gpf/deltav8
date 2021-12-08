# deltav8
A motor control/break actuation system for the Raspberry Pi Pico using the RP2040 SOC.

# Requirements
Generally requires the latest [Zig compiler](https://ziglang.org/download/), [CMake](https://cmake.org/download/) 3.20 or later (will need to use custom triggers for apt), a GCC embedded ARM toolchain compiler (arm-none-eabi), and the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk).

If using a Debian-based distro (ex. Ubuntu) or WSL with such a distro you can just follow the [official instructions](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) to acquire the dependencies (except of course for the Zig compiler). As mentioned previously you will need [custom triggers to install the latest CMake](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line).

If you aren't using a Debian-based distro you probably know what you're doing. If you can't find the arm-none-eabi cross compiler in your package manager, you can try to [build your own](https://wiki.osdev.org/GCC_Cross-Compiler). Unfortunately `zig cc` can't be used to cross-compile the SDK components as the SDK doesn't yet properly support Clang.

For any platform the latest Zig compiler can be installed trivially by decompressing the provided archive on the [download page](https://ziglang.org/download/) for your platform and adding the zig executable to your PATH. It has no dependencies otherwise.

On Windows you can get all the required dependencies including the Pico SDK from [this unofficially supported Windows intaller](https://github.com/ndabas/pico-setup-windows) (see the Releases tag). The included CMake will be the correct version to use with this project.

# Build instructions
Doing all the above you can just run this from the project root (assuming a UNIX system):

```
mkdir build
cd build
cmake ..
make
```

Or running the equivalent for your preferred build system.
Note that on Windows you need to generate NMake Makefiles due to eccentricities with MSBuild:

```
cmake .. -G "NMake Makefiles"
nmake
```

Although you will need NMake in your path.

On VSCode, it will help to install the default CMake extensions provided by the text editor. This will perform the above steps for you and streamline the build process. To use NMake on Windows, go to File -> Preferences -> Settings, search "cmake" and under "Workspace" look for an option called "Generator"; set it to "NMake Makefiles".

To flash the output file (build/deltav8.uf2) see the [official instructions](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html).

# Electrical
First, configure the SmartDriveDuo-30 MDDS30 such that it is in Serial Simplified input mode with 115200 bps (DIP SW1->SW6; 110111). Connect the IN1 pin to GPIO4, and on the same header block the GND pin to any of the Pico's grounds. Connect your motor to the MLA/MLB ports; corresponding to the "left" motor, although only the "left" motor is ever controlled. A 50 watt motor powered at ~18 volts was used.

The TF Luna's RX pin must be connected to GPIO0 and the TX pin must be connected to GPIO1. The remaining pins can be ignored; make sure they're never connected to anything. The Pico and the sensor should share the same ground reference, which can come from the Pico if they aren't already powered from the same source. All peripherals ideally receive 5 volts with about 1 Amp able to power all of them.

To change the set stopping distance, see stopping_dist_cm in src/main.zig. It is based on a fixed worst-case deceleration and maxmimum velocity.

The LS-955CR continuous servomotor's PWM input must be connected to GPIO22. In the scooter it pulls on the manual brake via a wire and a lever extension. Like with the throttle and sensor, the servo and Pico must have the same ground. See servo_rotate_time in src/pwm.zig: this is the amount of time, in seconds, that the servo takes to rotate in either direction.

Finally, connect a throttle (or any potentiometer) such that it's powered by the Pico's 3V3 output through the Pico's ground. The output voltage pin must be connected to GPIO26. Note that depending on throttle make there may be variance with output voltages such that the proper range of speeds aren't read from the ADC hardware. See the throttle_vmin and throttle_vmax constants in src/adc.zig; these constants were only found to work for the throttle used with the final design. The adc_test branch can be pulled (which also controls the motor), which will output the relative and scaled values. The potentiometer output must never exceed 3.3 Volts as to avoid damaging the Pico's hardware (this is avoided by powering it with a known 3.3 Volt power source via the 3V3 pin).

Alternatively, the motor controller could be removed and the throttle could be connected directly to the motor through an appropriate relay. By configuring the relay via a transistor connected to the Pico, the same (or greater) functionality can be achieved for much lower cost; the motor controller design was chosen because it was provided to us. If this method is chosen, make sure that the relay is closed while not braking and open while braking. This may also provide additional safety if the Pico ever resets or loses power but the motor does not.

Limitations with the sensor unfortunately dictate that the assembly will wait ~500ms when initially plugged in to respond to input of any kind. Any signal loss due to cabling may cause the Pico to reset, shown via the on-board LED.

# Troubleshooting
If nothing happens, something isn't wired correctly or something isn't powered. Alternatively, there is a strange issue with how the motor controller is used; see [postmortem](#Postmortem); although this will cause the LED to start blinking. If the LED is not blinking, everything is wired correctly, and nothing happens, it may be an issue with the throttle or servo. Pull the adc_test branch and look at the output produced by the Pico with the throttle connected. It will also drive the motor controller. At rest the motor controller value should be 0 (do nothing) and when cranked it should be 63 (max speed).

If the LED lights up and mostly stays on, most likely the TF Luna is failing to be initialized, because it is not powered or the RX and TX pins aren't properly connected. Make sure at least that the TF Luna is on: looking directly into it you should see a red glow.

If the LED starts blinking, either the Pico's ADC or the UART are not firing IRQs. Here, something is either horribly broken or caught in an infinite loop somewhere. Rest assured that all the components are initializing correctly, especially the TF Luna. Most likely it may be an issue with the motor controller; see [postmortem](#Postmortem).

If the servo is not pulling the brake, it might not have enough power, might not be wired correctly, might not be a continuous servo, or might expect a slightly different pulse width or frequency. Our servo expects a 500us pulse to mean full speed clockwise, and a 2500us pulse to mean full speed anticlockwise with a 50hz signal.

# Postmortem
Despite working perfectly in the adc_test branch, the motor controller's UART; UART1; wouldn't respond to any input. Furthering this, when waiting for the UART's TX holding register to be cleared the Pico will enter an infinite loop. This causes the blinking LED. It's unclear if this is a fluke with our Pico, the rewritten UART interface, the wiring, or the motor controller, although it only seems to manifest if UART0 is also used for the sensor. Commenting out the line to wait for the TX FIFO to not be full will solve the problem but the motor controller never registers any input from the throttle; the braking system will work but the scooter won't run.

We also had to use a power supply to run any tests as our batteries for the motor were accidentally discharged and the batteries for the peripherals could not power the servo, and had slightly too high of a voltage. The final demo only demonstrates the braking system being able to stop the scooter's back wheel turned from the (manually controlled) motor.
