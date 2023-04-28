import math
import time

import serial


def encode_vels(a: float, b: float, c: float, d: float) -> bytes:
    """ Encode the four motor velocities (between 1 and -1) to an 8 byte string """
    b_arr = [int(v * (2 ** 15 - 1)).to_bytes(2, byteorder='little', signed=True) for v in [a, b, c, d]]
    return b''.join(b_arr)


if __name__ == '__main__':

    try:

        # Connect
        ser = serial.Serial('/dev/ttyACM0')
        print(f"Connected to {ser.name}, {ser.is_open}")

        # Move forwards
        ser.write(encode_vels(1, 1, 1, 1))
        time.sleep(4)

        # Move forwards
        ser.write(encode_vels(-1, -1, -1, -1))
        time.sleep(4)

        # Individual control
        step = 0
        while step < (4 * math.pi):
            vels = [math.sin(step + (n * math.pi / 2)) for n in range(4)]
            ser.write(encode_vels(*vels))
            step += 0.002
            time.sleep(0.001)

    except Exception as ex:
        print(ex)

    ser.write(encode_vels(0, 0, 0, 0))
    ser.close()
