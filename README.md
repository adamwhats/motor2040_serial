# Control for Motor2040 Board over Serial Comms

This project builds a script for driving the [Pimoroni Motor2040 Motor Controller](https://shop.pimoroni.com/products/motor-2040) board via USB, offering velocity control for each of the four motors.

Target velocities are sent via an 8 byte string, which is parsed as four 16 bit signed integers which control the speed of each motor - take care with the order!

## TODO
- [ ] Check that a fresh clone of the repo works properly
- [ ] Stream the current velocity back to the master PC
- [ ] Stream the current position back to the master PC
- [ ] Expose relevant information to allow for actual velocity in task space to be set. Maybe do this through an abstract C++ class on the master PC side which ROS can interact with

## Build

The CMakeLists.txt assumes this project is in the same directory as the [pimoroni-pico](https://github.com/pimoroni/pimoroni-pico) library.

You should should ensure your `PICO_SDK_PATH` is set to the correct path, either by editing `~/.profile` or specifying it in [settings.json](https://github.com/adamwhats/xavbot_motor2040/blob/main/.vscode/settings.json).

Building will generate a `.uf2` file in the build colder, copy this over to the motor2040 to flash it.

## Test

To test, run `test_comms.py`.