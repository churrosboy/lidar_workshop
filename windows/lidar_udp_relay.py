"""Relay fixed-destination GL5 UDP through a Windows loopback endpoint."""
import argparse
import ipaddress
import selectors
import socket


def port(value):
    value = int(value)
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError('port must be between 1 and 65535')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor-ip', type=ipaddress.IPv4Address, required=True)
    parser.add_argument('--sensor-port', type=port, default=2000)
    parser.add_argument('--pc-ip', type=ipaddress.IPv4Address, required=True)
    parser.add_argument('--pc-port', type=port, default=3000)
    parser.add_argument('--relay-port', type=port, default=13000)
    args = parser.parse_args()
    target = (str(args.sensor_ip), args.sensor_port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sensor, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as control, \
            selectors.DefaultSelector() as selector:
        sensor.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sensor.bind((str(args.pc_ip), args.pc_port))
        control.bind(('127.0.0.1', args.relay_port))
        selector.register(sensor, selectors.EVENT_READ)
        selector.register(control, selectors.EVENT_READ)
        peer = None
        print(f'Ready: sensor={target}, PC={sensor.getsockname()}, relay={control.getsockname()}', flush=True)
        while True:
            for key, _ in selector.select():
                payload, sender = key.fileobj.recvfrom(65535)
                if key.fileobj is control:
                    peer = sender
                    sensor.sendto(payload, target)
                elif sender == target and peer:
                    control.sendto(payload, peer)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
