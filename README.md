# deltav8
A motor control/break actuation system for the Raspberry Pi Pico using the RP2040 SOC with a TF-Luna LiDAR sensor (GPIO 0 and 1 allocated to UART0).

# Requirements
The following instructions are intended to be understandable by a student with experience in CMPT 128 and so they may seem simplistic.

Generally requires the latest [Zig compiler](https://ziglang.org/download/), [CMake](https://cmake.org/download/) 3.20 or later (will need to use custom triggers for apt), a GCC embedded ARM toolchain compiler (arm-none-eabi), and the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk).

If using a Debian-based distro (ex. Ubuntu) or WSL with such a distro you can just follow the [official instructions](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) to acquire the dependencies (except of course for the Zig compiler), except that as mentioned previously you will need [custom triggers to install the latest CMake](https://askubuntu.com/questions/355565/how-do-i-install-the-latest-version-of-cmake-from-the-command-line).

If you aren't using a Debian-based distro you probably know what you're doing. If you can't find the arm-none-eabi cross compiler in your package manager, you can try to [build your own](https://wiki.osdev.org/GCC_Cross-Compiler).

For any platform the latest Zig compiler can be installed trivially by decompressing the provided archive on the [download page](https://ziglang.org/download/) for your platform and adding the zig executable to your PATH. It has no dependencies otherwise.

On Windows you can get all the required dependencies, including the Pico SDK from [this unofficially supported Windows intaller](https://github.com/ndabas/pico-setup-windows) (see the Releases tag). The included CMake will be the correct version to use with this project.

# Build instructions
With zig, arm-none-eabi-gcc (and binutils), cmake in the path and PICO_SDK_PATH set to the path of the SDK you downloaded (may be done automatically by scripts mentioned above), one can simply:

```
mkdir build
cd build
cmake ..
make -j<JOBS>
```

Where JOBS is of course (NUM_CORES + 1) * HYPER_THREADS; a 10-core CPU with hyperthreading (20 threads) can use 22 jobs, for example. The -j parameter is optional and is only used to decrease compile times.
Note that on Windows you need to generate NMake Makefiles due to eccentricities with MSBuild. Put simply,

```
cmake .. -G "NMake Makefiles"
nmake
```

Although you will need NMake in your path.

On VSCode, it will help to install the default CMake extension (CTRL+SHIFT+X, search for "CMake" by twxs, "CMake Tools" by Microsoft). This will perform the above steps for you and streamline the build process. To use NMake on Windows, go to File -> Preferences -> Settings, search "cmake" and under "Workspace" look for an option called "Generator"; set it to "NMake Makefiles".

Building should then just work. Note that building does not work if the GCC cross compiler is installed in a path containing newline or carriage return characters (only applies to UNIX systems).

# Flashing
The build process should generate a file called deltav8.uf2 in the build folder. Take your RPI Pico and, holding down the BOOTSEL button, plug it into your PC. Drag the deltav8.uf2 file into the new storage device that shows up. This will flash the Pico.

# Electrical
For now, connect the TF Luna's RX pin to GPIO0 and the TX pin to GPIO1. Both of these pins must be connected when the microcontroller is turned on. The remaining pins not used for power can be ignored. Do not power them or short them to anything.

It is normal for a half-second delay to occur if the sensor and the microcontroller are powered on at the same time; this is because of limitations with the sensor.

Currently a demo is in place where the on-board LED will light up if the sensor detects an object within 35cm of it, like your hand. So, to figure out if the sensor and board work you can just wave your hand over the sensor.

# Troubleshooting
Usually the sensor just isn't wired correctly. Make sure it's powered by looking into the lenses and looking for the red light emitted by the IR sensor.

Make sure also you have the correct pinout and are connecting each pin correctly, and that each connection is correct. The sensor's RX must connect to the Pico's TX, and the sensor's TX to the Pico's RX.