import Pyro4
import serial
import io
from collections import namedtuple
from itertools import takewhile, count
from argparse import ArgumentParser
from math import pi

Status = namedtuple("Status", "speed_left speed_right distance_left distance_right lidar")


@Pyro4.behavior(instance_mode="single")
class HAL(object):

    # all distances are in mm
    # all angles are in radians
    WHEEL_BASE = 222.0
    WHEEL_CIRCUMFERENCE = 38.0
    TICKS_PER_ROTATION = 320.0
    TICKS_TO_DISTANCE = pi * WHEEL_CIRCUMFERENCE / TICKS_PER_ROTATION
    BEAM_FOV = 25.0 * pi / 180.0
    BEAM_ANGLES = [pi * i - pi / 2 for i in range(5)]
    SERVOS_MIN = [0, 0, 0, 0]
    SERVOS_MAX = [pi, pi, pi, pi]
    SERVOS_DEFAULT = [pi/2, pi/2, pi/2, pi/2]
    SPEED_MAX = 190.0

    def __init__(self, device):
        self._motors_enabled = False
        self._servos_enabled = False
        self._servos_last = list(self.SERVOS_DEFAULT)
        self._serial = serial.Serial(device, 115200)
        self._sio = io.TextIOWrapper(io.BufferedRWPair(self._serial, self._serial))
        self._hello()

    @Pyro4.expose
    def enable_motors(self, value):
        self._motors_enabled = value

    @Pyro4.expose
    def enable_servos(self, value):
        self._servos_enabled = value

    @Pyro4.expose
    def get_max_speed(self):
        return self.SPEED_MAX

    @Pyro4.expose
    def get_wheelbase(self):
        return self.WHEEL_BASE

    @Pyro4.expose
    def get_lidar_fov(self):
        return self.BEAM_FOV

    @Pyro4.expose
    def get_lidar_angles(self):
        return self.BEAM_ANGLES

    @Pyro4.expose
    def get_arm_default(self):
        return self.SERVOS_DEFAULT

    @Pyro4.expose
    def get_arm_last(self):
        return self._servos_last

    @Pyro4.expose
    def get_arm_range(self):
        return self.SERVOS_MIN, self.SERVOS_MAX

    @Pyro4.expose
    def set_speed(self, speed, theta, max_dst=0):
        left = (2.0 * speed + theta) / 2.0
        right = (2.0 * speed - theta) / 2.0
        self._motors(self._motors_enabled, left, right)

    @Pyro4.expose
    def set_arm(self, joints):
        assert(len(joints) == 4)
        self._servos_last = list(joints)
        self._servos(self._arm_enabled, joints)

    @Pyro4.expose
    def status(self):
        return self._status()

    @Pyro4.expose
    def debug(self):
        return self._debug()

    # communication with driver board
    def _motors(self, enable, speed_left, speed_right, max_dst_left=0, max_dst_right=0):
        max_ticks_left, max_ticks_right = self.TICKS_TO_DISTANCE * max_dst_left, self.TICKS_TO_DISTANCE * max_dst_right
        self._sio.write("MOTOR {} {} {} {} {}".format([0, 1][enable], speed_left, speed_right, max_ticks_left, max_ticks_right))
        self._sio.flush()
        result = self._sio.readline()
        if result != "OK":
            raise SystemError(result)

    def _differential(self, enable, speed, theta, max_dst):
        self._sio.write("DIFF {} {} {} {}".format([0, 1][enable], speed, theta, max_dst * self.TICKS_TO_DISTANCE))
        self._sio.flush()
        result = self._sio.readline()
        if result != "OK":
            raise SystemError(result)

    def _servos(self, enable, joints):
        self._sio.write("SERVO {} {}".format([0, 1][enable], " ".join(str(int(i * 180 / pi)) for i in joints)))
        self._sio.flush()
        result = self._sio.readline()
        if result != "OK":
            raise SystemError(result)

    def _status(self):
        self._sio.write("STATUS")
        self._sio.flush()
        result = self._sio.readline().split()
        result = [float(i) for i in result]
        return Status(result[0], result[1], result[2] / self.TICKS_TO_DISTANCE, result[3] / self.TICKS_TO_DISTANCE, result[4:])

    def _debug(self):
        self._sio.write("DEBUG")
        self._sio.flush()
        return self._sio.readline().split()

    def _hello(self):
        self._sio.write("HELLO")
        self._sio.flush()
        lines = list(takewhile(lambda x: x, (self._sio.readline() for _ in count())))
        if lines[0] != "Autonomous Robot Controller":
            raise SystemError("Could not find the driver board")


def main():
    parser = ArgumentParser(description="Remote Hardware Abstraction Layer for robot driver board")
    parser.add_argument("name", help="Remote name of HAL")
    parser.add_argument("-d", "--device", default="/dev/ttySAC3", help="serial device of the driver board")
    parser.add_argument("-n", "--nameserver", default="", help="hostname of the name server (default: empty)")
    parser.add_argument("-H", "--host", default="localhost", help="Hostname of the server (default: localhost)")
    parser.add_argument("-p", "--port", default=0, help="Port of the server (default: 0)")

    args = parser.parse_args()

    hal = HAL(args.device)

    with Pyro4.Daemon(host=args.host, port=args.port) as daemon:
        if args.nameserver:
            ns = Pyro4.locateNS(host=args.nameserver)
        else:
            ns = Pyro4.locateNS()
        uri = daemon.register(hal)
        ns.register(args.name, uri)

        daemon.requestLoop()


if __name__ == "__main__":
    main()
