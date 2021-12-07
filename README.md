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
First, configure the SmartDriveDuo-30 MDDS30 such that it is in Serial Simplified input mode with 115200 bps (DIP SW1->SW6; 110111). Connect the IN1 pin to GPIO4, and on the same header block the GND pin to any of the Pico's grounds. Connect your motor to the MLA/MLB ports; corresponding to the "left" motor, although only the "left" motor is ever controlled. A 50 watt motor powered at ~22 volts was used.

The TF Luna's RX pin must be connected to GPIO0 and the TX pin must be connected to GPIO1. The remaining pins can be ignored; make sure they're never connected to anything. The Pico and the sensor should share the same ground reference. This can be done by powering them from the same source, which is ideal anyways as they have similar input voltages and don't consume much power.

Finally, connect a throttle (or any potentiometer) such that it's powered by the Pico's 3V3 output and drains to the Pico's ground. The output voltage pin must be connected to GPIO26. Note that depending on throttle make there may be variance with output voltages such that the proper range of speeds aren't read from the ADC hardware. See the throttle_vmin and throttle_vmax constants in arc/adc.zig; these constants were only found to work for the throttle used with the final design. The potentiometer output must never exceed 3.3 Volts as to avoid damaging the Pico's hardware (this is avoided by powering it with a known 3.3 Volt power source via the 3V3 pin).

Alternatively, the motor controller could be removed and the throttle could be connected directly to the motor through an appropriate relay. By configuring the relay via a transistor connected to the Pico, the same (or greater) functionality can be achieved for much lower cost; the motor controller design was chosen because it was provided to us. If this method is chosen, make sure that the relay is closed while not braking and open while braking. This may also provide additional safety if the Pico ever resets or loses power but the motor does not.

Also of note; the stopping distance chosen was simply a conservative value for the scooter's max speed of ~10.4 km/h and weak braking power. It could also be changed as a function of time if velocity were known.

The LS-955CR continuous servomotor's PWM input must be connected to GPIO22. In the scooter it pulls on the manual brake via a wire and a lever extension. Like with the throttle and sensor, the servo and Pico must have the same ground.

Limitations with the sensor unfortunately dictate that the assembly will wait ~500ms when initially plugged in to respond to input of any kind.

# Troubleshooting
Usually something just isn't wired correctly. Verify that each component works in some form; the sensor should emit a red light from its lens if powered.

Make sure also you have the correct pinout and are connecting each pin correctly, and that each connection is correct. For example, the sensor's RX must connect to the Pico's UART0 TX, and the sensor's TX to the Pico's UART0 RX.
