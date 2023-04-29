# Control for Motor2040 Board over Serial Comms

This project builds a script for driving the [Pimoroni Motor2040 Motor Controller](https://shop.pimoroni.com/products/motor-2040) board via USB, offering velocity control for each of the four motors.

Target velocities are sent via an 16 byte string, which is parsed as four 32 bit floats which control the speed of each motor in rotations per second. Remember to take care with the order!

Please get in contact with any suggestions for improvements.

## TODO
- [x] Check that a fresh clone of the repo works properly
- [x] Stream the current velocity back to the master PC
- [ ] Stream the current position back to the master PC
- [x] Expose relevant information to allow for actual velocity in task space to be set
- [ ] Add error checking (xor on the incoming bytes?)

## Build

The CMakeLists.txt assumes this project is in the same directory as the [pimoroni-pico](https://github.com/pimoroni/pimoroni-pico) library.

You should should ensure your `PICO_SDK_PATH` is set to the correct path, either by editing `~/.profile` or specifying it in [settings.json](https://github.com/adamwhats/xavbot_motor2040/blob/main/.vscode/settings.json).

Building will generate a `.uf2` file in the build colder, copy this over to the motor2040 to flash it.

## Test

To test, run `test_comms.py`.