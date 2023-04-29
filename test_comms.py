import math
import struct
import time

import serial


def encode_vels(a: float, b: float, c: float, d: float) -> bytes:
    """ Encode the four motor velocities (in rotation per second) to a 16 byte string (4 x float32)"""
    byte_string = b''.join(struct.pack('f', v) for v in [a, b, c, d])
    return byte_string


if __name__ == '__main__':

    try:

        # Connect
        ser = serial.Serial('/dev/ttyACM0')
        print(f"Connected to {ser.name}, {ser.is_open}")

        # Move forwards
        ser.write(encode_vels(1, 1, 1, 1))
        time.sleep(3)

        # Move forwards
        ser.write(encode_vels(-2, -2, -2, -2))
        time.sleep(3)

        # Move forwards
        ser.write(encode_vels(3, 3, 3, 3))
        time.sleep(3)

        # Move forwards
        ser.write(encode_vels(-4, -4, -4, -4))
        time.sleep(3)

        # Individual control
        step = 0
        while step < (8 * math.pi):
            # Send vels
            vels = [3 * math.sin(step + (n * math.pi / 2)) for n in range(4)]
            ser.reset_input_buffer()

            # Wait for response
            ser.write(encode_vels(*vels))
            response_bytes = ser.read(16)
            response_floats = struct.unpack('4f', response_bytes)

            # Print response
            if round(step, 2) % 1 == 0:
                print(f"------{round(step, 2)}------")
                for n in range(4):
                    print(f"{n}: {vels[n]: .2f}, {response_floats[n]: .2f}")
            step += 0.02
            time.sleep(0.01)

        ser.write(encode_vels(0, 0, 0, 0))

    except Exception as ex:
        print(ex)

    except KeyboardInterrupt:
        pass

    ser.write(encode_vels(0, 0, 0, 0))
    ser.close()
