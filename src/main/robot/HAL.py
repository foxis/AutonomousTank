import Pyro4
import serial
import io
from collections import namedtuple
from itertools import takewhile, count
from argparse import ArgumentParser

Status = namedtuple("Status", "speed_left speed_right lidar")


@Pyro4.behavior(instance_mode="single")
class HAL(object):

    def __init__(self, device):
        self._motors_enabled = False
        self._servos_enabled = False
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
        return 190.0

    @Pyro4.expose
    def get_default_arm(self):
        return [90, 90, 90, 90]

    @Pyro4.expose
    def set_speed(self, speed, theta):
        left = (2.0 * speed + theta) / 2.0
        right = (2.0 * speed - theta) / 2.0
        self._motors(self._motors_enabled, left, right)

    @Pyro4.expose
    def set_arm(self, joints):
        assert(len(joints) == 4)
        self._servos(self._arm_enabled, joints)

    @Pyro4.expose
    def status(self):
        return self._status()

    @Pyro4.expose
    def debug(self):
        return self._debug()

    # communication with driver board
    def _motors(self, enable, speed_left, speed_right):
        self._sio.write("MOTOR {} {} {}".format([0, 1][enable], speed_left, speed_right))
        self._sio.flush()
        result = self._sio.readline()
        if result != "OK":
            raise SystemError(result)

    def _servos(self, enable, joints):
        self._sio.write("SERVO {} {}".format([0, 1][enable], " ".join(str(i) for i in joints)))
        self._sio.flush()
        result = self._sio.readline()
        if result != "OK":
            raise SystemError(result)

    def _status(self):
        self._sio.write("STATUS")
        self._sio.flush()
        result = self._sio.readline().split()
        result = [float(i) for i in result]
        return Status(result[0], result[1], result[2:])

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
