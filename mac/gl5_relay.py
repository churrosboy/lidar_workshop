#!/usr/bin/env python3
"""macOS에서 GL5 스트림을 받을 수 있게 해 주는 중계기.

GL5는 포인트 스트림을 **이더넷 브로드캐스트**로 보냅니다. IP 목적지는 유니캐스트
`10.110.1.3:3000`인데 링크 계층 목적지는 `ff:ff:ff:ff:ff:ff`입니다. BSD 계열인
macOS의 IP 스택은 이 조합을 소켓에 넘기지 않고 버립니다. 리눅스는 통과시킵니다.
그래서 같은 센서가 Ubuntu에서는 잘 동작하고 Mac에서는 아무것도 오지 않습니다.

이 중계기가 그 틈을 메웁니다. tcpdump로 프레임을 직접 떠서 페이로드만 꺼내고,
평범한 유니캐스트 UDP 데이터그램으로 다시 쏩니다. 그러면 SDK도 ROS 드라이버도
컨테이너도 고칠 것 없이 그대로 동작합니다.

명령 방향도 함께 중계합니다. SDK가 로컬 주소로 보낸 명령을 실제 센서로 넘깁니다.
덕분에 SDK 설정을 전부 루프백으로 둘 수 있습니다.

  # 실습: 컨테이너로 넘기기
  sudo python3 mac/gl5_relay.py --iface en6 --listen 0.0.0.0:2000
  bash mac/start-lidar.sh            # 다른 터미널

  # 호스트에서 바로 확인
  sudo python3 mac/gl5_relay.py --iface en6
  python3 mac/gl5_probe.py --sensor-ip 127.0.0.1 --pc-ip 127.0.0.1

tcpdump 를 쓰므로 root 권한이 필요합니다.
"""

import argparse
import errno
import os
import pwd
import signal
import socket
import struct
import subprocess
import sys
import threading

PCAP_GLOBAL_HEADER = 24
PCAP_RECORD_HEADER = 16
ETHERNET_HEADER = 14


class PcapStream:
    """tcpdump 의 -w - 출력을 읽으며 UDP 페이로드를 하나씩 내놓습니다."""

    def __init__(self, stream):
        header = self._read_exactly(stream, PCAP_GLOBAL_HEADER)
        if header is None:
            raise RuntimeError("tcpdump 가 pcap 헤더를 내놓지 않았습니다")
        magic, = struct.unpack("<I", header[:4])
        if magic in (0xA1B2C3D4, 0xA1B23C4D):
            self.endian = "<"
        elif magic in (0xD4C3B2A1, 0x4D3CB2A1):
            self.endian = ">"
        else:
            raise RuntimeError(f"알 수 없는 pcap 매직 0x{magic:08X}")
        self.stream = stream

    @staticmethod
    def _read_exactly(stream, count):
        chunks = []
        remaining = count
        while remaining > 0:
            chunk = stream.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def __iter__(self):
        while True:
            record = self._read_exactly(self.stream, PCAP_RECORD_HEADER)
            if record is None:
                return
            _, _, caplen, _ = struct.unpack(self.endian + "IIII", record)
            frame = self._read_exactly(self.stream, caplen)
            if frame is None:
                return
            payload = self._udp_payload(frame)
            if payload:
                yield payload

    @staticmethod
    def _udp_payload(frame):
        # 이더넷 + IPv4 + UDP 만 다룹니다. 그 외는 무시합니다.
        if len(frame) < ETHERNET_HEADER + 20 + 8:
            return None
        if frame[12:14] != b"\x08\x00":
            return None
        ip_start = ETHERNET_HEADER
        version_ihl = frame[ip_start]
        if version_ihl >> 4 != 4:
            return None
        ihl = (version_ihl & 0x0F) * 4
        if frame[ip_start + 9] != 17:  # UDP
            return None
        udp_start = ip_start + ihl
        if len(frame) < udp_start + 8:
            return None
        udp_length, = struct.unpack_from("!H", frame, udp_start + 4)
        payload = frame[udp_start + 8:udp_start + udp_length]
        return payload or None


def tcpdump_drop_user():
    """tcpdump 자신이 권한을 내려놓게 할 사용자 이름.

    우리 프로세스는 root 로 남아야 합니다. 권한을 내려놓으면 root 로 띄운 tcpdump
    자식을 종료할 수 없어 Ctrl+C 때 EPERM 이 납니다.
    """
    name = os.environ.get("SUDO_USER")
    if not name:
        return None
    try:
        pwd.getpwnam(name)
    except KeyError:
        return None
    return name


def stream_worker(process, forward_socket, destination, state, quiet):
    try:
        for payload in PcapStream(process.stdout):
            forward_socket.sendto(payload, destination)
            state["packets"] += 1
            if not quiet and state["packets"] % 240 == 0:
                print(f"  중계 {state['packets']}패킷 "
                      f"(프레임 약 {state['packets'] // 6}개)", flush=True)
    except Exception as err:  # tcpdump 종료 등
        if not state["stopping"]:
            print(f"캡처 중단 :: {err}", file=sys.stderr, flush=True)
    finally:
        state["stopping"] = True


def command_worker(listen_socket, uplink_socket, sensor, state, quiet):
    """SDK 가 로컬로 보낸 명령을 실제 센서로 넘깁니다."""
    while not state["stopping"]:
        try:
            data, sender = listen_socket.recvfrom(65535)
        except OSError:
            return
        try:
            uplink_socket.sendto(data, sensor)
        except OSError as err:
            print(f"명령 전달 실패 :: {err}", file=sys.stderr, flush=True)
            continue
        state["commands"] += 1
        if not quiet:
            print(f"  명령 중계 {len(data)}바이트 :: {sender[0]}:{sender[1]} "
                  f"-> {sensor[0]}:{sensor[1]}", flush=True)


def split_hostport(text, option):
    host, _, port = text.rpartition(":")
    if not host or not port.isdigit():
        raise SystemExit(f"{option} 는 IP:PORT 형식이어야 합니다: {text}")
    return host, int(port)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="GL5 브로드캐스트 스트림을 일반 UDP 로 중계")
    parser.add_argument("--iface", default="en6", help="센서가 붙은 인터페이스")
    parser.add_argument("--sensor", default="10.110.1.2:2000", metavar="IP:PORT")
    parser.add_argument("--pc-ip", default="10.110.1.3",
                        help="센서로 명령을 보낼 때 쓸 출발지 주소")
    parser.add_argument("--pc-port", type=int, default=3000,
                        help="센서가 스트림을 보내는 목적지 포트")
    parser.add_argument("--forward", default="127.0.0.1:3000", metavar="IP:PORT",
                        help="스트림을 다시 쏠 곳 (기본 127.0.0.1:3000)")
    parser.add_argument("--listen", default="127.0.0.1:2000", metavar="IP:PORT",
                        help="SDK 의 명령을 받을 곳 (기본 127.0.0.1:2000)")
    parser.add_argument("--no-command-relay", action="store_true",
                        help="명령 중계를 끄고 스트림만 중계")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    sensor = split_hostport(args.sensor, "--sensor")
    forward = split_hostport(args.forward, "--forward")
    listen = split_hostport(args.listen, "--listen")

    # --iface "$IFACE" 를 IFACE 를 담지 않은 창에서 실행하면 빈 문자열이 들어옵니다.
    # 그대로 두면 tcpdump 가 조용히 실패해 "pcap 헤더" 오류로만 보입니다.
    if not args.iface.strip():
        print("--iface 가 비어 있습니다. 이 창에서 먼저 실행하세요:", file=sys.stderr)
        print('  IFACE=$(bash mac/find_iface.sh) && echo "라이다 어댑터: $IFACE"', file=sys.stderr)
        return 2

    if os.geteuid() != 0:
        print("tcpdump 를 쓰므로 sudo 로 실행해야 합니다.", file=sys.stderr)
        return 2

    # 센서가 보내는 스트림만 뜹니다. 중계한 패킷을 다시 뜨는 되먹임을 피하려고
    # 출발지를 센서로 한정합니다.
    bpf = f"udp and src host {sensor[0]} and src port {sensor[1]} and dst port {args.pc_port}"
    # --immediate-mode 가 없으면 macOS 의 BPF 는 버퍼가 차거나 tcpdump 의 1초 읽기
    # 제한이 지나야 패킷을 넘깁니다. 스트림은 초당 약 240 KB 라 버퍼보다 제한 시간이
    # 먼저 와서, 40 Hz 가 1초에 한 번씩 몰려 들어오고 RViz 가 뚝뚝 끊깁니다.
    command = ["tcpdump", "-i", args.iface, "-n", "-s", "0", "-U",
               "--immediate-mode", "-w", "-"]
    drop_user = tcpdump_drop_user()
    if drop_user:
        command += ["-Z", drop_user]
    command.append(bpf)
    process = subprocess.Popen(command, stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL, bufsize=0)

    uplink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        uplink.bind((args.pc_ip, args.pc_port))
    except OSError as err:
        print(f"{args.pc_ip}:{args.pc_port} bind 실패 :: {err}", file=sys.stderr)
        if err.errno == errno.EADDRINUSE:
            # 다른 창에서 이미 중계기를 띄운 경우가 대부분입니다. 주소 문제로 안내하면
            # 멀쩡한 설정을 다시 건드리게 됩니다.
            print("이미 다른 터미널에서 중계기가 실행 중인 것 같습니다. "
                  "'pgrep -fl gl5_relay' 로 확인하고, 하나만 켜 두세요.", file=sys.stderr)
        else:
            print("인터페이스에 그 주소가 있는지 확인하세요.", file=sys.stderr)
        process.terminate()
        return 2

    # 중계한 패킷의 출발지가 센서 주소로 보이도록, 명령을 받는 소켓으로 스트림도
    # 내보냅니다. SDK 는 정상 경로에서 출발지를 엄격히 검사하므로, 임의의 포트에서
    # 쏘면 전부 버려집니다.
    forwarder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    forwarder.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        forwarder.bind(listen)
    except OSError as err:
        print(f"{listen[0]}:{listen[1]} bind 실패 :: {err}", file=sys.stderr)
        process.terminate()
        return 2
    listener = None if args.no_command_relay else forwarder

    print(f"인터페이스 {args.iface} 에서 {sensor[0]}:{sensor[1]} 의 스트림을 뜹니다.")
    print(f"스트림 중계 :: -> {forward[0]}:{forward[1]}")
    if listener is not None:
        print(f"명령 중계   :: {listen[0]}:{listen[1]} -> {sensor[0]}:{sensor[1]} "
              f"(출발지 {args.pc_ip}:{args.pc_port})")
    print("Ctrl+C 로 종료합니다.\n")

    state = {"packets": 0, "commands": 0, "stopping": False}
    threads = [threading.Thread(target=stream_worker, daemon=True,
                                args=(process, forwarder, forward, state, args.quiet))]
    if listener is not None:
        threads.append(threading.Thread(target=command_worker, daemon=True,
                                        args=(listener, uplink, sensor, state, args.quiet)))
    for thread in threads:
        thread.start()

    def stop(*_):
        state["stopping"] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while not state["stopping"]:
            threads[0].join(0.5)
            if not threads[0].is_alive():
                break
    finally:
        state["stopping"] = True
        process.terminate()
        forwarder.close()
        uplink.close()
        print(f"\n종료. 스트림 {state['packets']}패킷, 명령 {state['commands']}건 중계함.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
