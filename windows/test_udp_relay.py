"""Exercise command/response forwarding and sensor source filtering on loopback."""
from pathlib import Path
import socket
import subprocess
import sys
import time
import unittest


def available_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class RelayTest(unittest.TestCase):
    def test_roundtrip_and_source_filter(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sensor, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stranger:
            sensor.bind(('127.0.0.1', 0))
            client.bind(('127.0.0.1', 0))
            pc_port, relay_port = available_port(), available_port()
            while relay_port == pc_port:
                relay_port = available_port()
            process = subprocess.Popen([
                sys.executable, str(Path(__file__).with_name('lidar_udp_relay.py')),
                '--sensor-ip', '127.0.0.1', '--sensor-port', str(sensor.getsockname()[1]),
                '--pc-ip', '127.0.0.1', '--pc-port', str(pc_port),
                '--relay-port', str(relay_port),
            ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            try:
                sensor.settimeout(0.2)
                deadline = time.monotonic() + 5
                while True:
                    self.assertIsNone(process.poll(), 'relay exited before startup')
                    client.sendto(b'start', ('127.0.0.1', relay_port))
                    try:
                        command, sender = sensor.recvfrom(1024)
                        break
                    except (TimeoutError, ConnectionResetError):
                        if time.monotonic() > deadline:
                            self.fail('relay startup timed out')
                self.assertEqual(command, b'start')
                self.assertEqual(sender, ('127.0.0.1', pc_port))
                sensor.sendto(b'frame', sender)
                client.settimeout(2)
                while True:
                    try:
                        response, origin = client.recvfrom(1024)
                        break
                    except ConnectionResetError:
                        continue
                self.assertEqual(response, b'frame')
                self.assertEqual(origin, ('127.0.0.1', relay_port))
                stranger.sendto(b'not-the-sensor', sender)
                client.settimeout(0.2)
                with self.assertRaises(TimeoutError):
                    client.recvfrom(1024)
            finally:
                process.terminate()
                process.communicate(timeout=5)


if __name__ == '__main__':
    unittest.main()
