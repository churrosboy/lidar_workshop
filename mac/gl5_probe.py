#!/usr/bin/env python3
"""실물 GL5에 직접 말을 거는 진단 도구.

SDK도 Docker도 ROS도 없이 순수 Python으로 동작합니다. SDK는 리눅스용 공유
라이브러리를 만들기 때문에 macOS와 Windows에서는 빌드되지 않습니다. 그런 PC에서
센서가 살아 있는지, 어디로 무엇을 보내는지 확인할 유일한 방법입니다.

답하려는 질문은 하나입니다. **GL5는 스트림을 어디로 보내는가.** 명령을 보낸
출발지로 보내는지, 펌웨어에 저장된 PC 주소로 보내는지에 따라 컨테이너로 데이터를
넣는 방법이 완전히 달라집니다.

  python3 mac/gl5_probe.py --sensor-ip 10.110.1.2 --pc-ip 10.110.1.3

pc_ip 로 bind 되지 않는 환경(컨테이너 등)에서는 --any-source 를 씁니다.
"""

import argparse
import json
import socket
import struct
import sys
import time

from gl5_protocol import (
    BI_DEV2PC, BI_PC2DEV, CAT_ETHERNET_INFO, CAT_FW_VERSION, CAT_SERIAL_NUM,
    CAT_STREAM_DATA, CAT_STREAM_ENABLE, META_BYTES, SM_GET,
    SM_SET, build_packet, parse_ethernet_info, parse_packet,
)


def send(sock, target, cat, sm, payload):
    sock.sendto(build_packet(cat, sm, BI_PC2DEV, 0, 1, payload), target)


def await_response(sock, cat, timeout):
    """해당 CAT 의 응답 하나를 기다립니다. 그 사이 온 다른 패킷도 세어 둡니다."""
    deadline = time.monotonic() + timeout
    others = []
    while time.monotonic() < deadline:
        sock.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            data, sender = sock.recvfrom(65535)
        except socket.timeout:
            break
        except OSError as err:
            return None, None, others, str(err)
        parsed = parse_packet(data, expect_bi=BI_DEV2PC)
        if parsed["error"] is None and parsed["cat"] == cat:
            return parsed, sender, others, None
        others.append((sender, parsed))
    return None, None, others, "시간 초과"


def query(sock, target, label, cat, timeout, decoder=None):
    """GET 명령 하나를 보내고 응답을 풀어 출력합니다."""
    send(sock, target, cat, SM_GET, b"\x01")
    parsed, sender, _, err = await_response(sock, cat, timeout)
    if parsed is None:
        print(f"  {label}: 응답 없음 ({err})")
        return None
    print(f"  {label}: {sender[0]}:{sender[1]} 에서 응답")
    if decoder is None:
        payload = parsed["payload"]
        text = payload.split(b"\x00")[0].decode("ascii", "replace")
        # 문자열이 아닌 응답도 있습니다. 펌웨어 버전은 연·월·일 3바이트입니다.
        if text.strip():
            print(f"    {text}")
            return text
        if len(payload) >= 3:
            print(f"    바이트 {payload.hex(' ')}  (연월일로 읽으면 "
                  f"{payload[0]}-{payload[1]:02d}-{payload[2]:02d})")
        else:
            print(f"    바이트 {payload.hex(' ')}")
        return payload.hex(" ")
    decoded = decoder(parsed["payload"])
    if decoded.get("error"):
        print(f"    해석 실패: {decoded['error']}")
        return None
    return decoded


def main(argv=None):
    parser = argparse.ArgumentParser(description="GL5 UDP 진단")
    parser.add_argument("--sensor-ip", default="10.110.1.2")
    parser.add_argument("--sensor-port", type=int, default=2000)
    parser.add_argument("--pc-ip", default="10.110.1.3")
    parser.add_argument("--pc-port", type=int, default=3000)
    parser.add_argument("--seconds", type=float, default=5.0, help="스트림 관찰 시간")
    parser.add_argument("--timeout", type=float, default=3.0, help="명령 응답 대기 시간")
    parser.add_argument("--any-source", action="store_true",
                        help="pc_ip 대신 0.0.0.0 에 bind. 컨테이너 등 그 주소가 없는 환경용")
    parser.add_argument("--output", help="결과를 JSON 으로 저장할 경로")
    parser.add_argument("--stop-only", action="store_true",
                        help="StreamEnable false 만 보내고 바로 종료. 임시 포트를 쓰므로 "
                             "중계기가 pc_port 를 잡고 있어도 됩니다")
    parser.add_argument("--self-test", action="store_true",
                        help="소켓을 열지 않고 코덱만 검사하고 종료")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()

    if args.stop_only:
        # SDK 의 GL5 파서는 header 와 조립 상태를 스트림과 명령이 공유합니다.
        # 스트림이 흐르는 중에 명령을 보내면 parseCommand 가 깨져 streamStart 가
        # 실패합니다. 노드를 띄우기 전에 스트림을 꺼 두는 용도입니다.
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((args.pc_ip, 0))
            send(sock, (args.sensor_ip, args.sensor_port), CAT_STREAM_ENABLE, SM_SET, b"\x00")
            print(f"스트리밍 중지 명령을 {args.sensor_ip}:{args.sensor_port} 로 보냈습니다.")
        except OSError as err:
            print(f"중지 명령 실패 :: {err}", file=sys.stderr)
            return 2
        finally:
            sock.close()
        return 0

    target = (args.sensor_ip, args.sensor_port)
    bind_ip = "0.0.0.0" if args.any_source else args.pc_ip
    report = {
        "sensor": f"{args.sensor_ip}:{args.sensor_port}",
        "requested_pc": f"{args.pc_ip}:{args.pc_port}",
    }

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((bind_ip, args.pc_port))
    except OSError as err:
        print(f"bind {bind_ip}:{args.pc_port} 실패 :: {err}", file=sys.stderr)
        if not args.any_source:
            print("이 PC 에 그 주소가 없습니다. 이더넷 설정을 확인하거나 "
                  "--any-source 로 다시 실행하세요.", file=sys.stderr)
        return 2
    report["bound"] = "%s:%d" % sock.getsockname()
    print(f"bind :: {report['bound']}  ->  센서 {args.sensor_ip}:{args.sensor_port}")

    print("\n[1] 센서 정보 조회")
    serial = query(sock, target, "시리얼 번호", CAT_SERIAL_NUM, args.timeout)
    fw = query(sock, target, "펌웨어", CAT_FW_VERSION, args.timeout)
    eth = query(sock, target, "EthernetInfo", CAT_ETHERNET_INFO, args.timeout,
                decoder=parse_ethernet_info)
    report["serial"] = serial
    report["firmware"] = fw
    report["ethernet_info"] = eth

    if eth:
        print(f"    센서가 저장하고 있는 PC 주소 :: {eth['pc_ip']}:{eth['pc_port']}")
        print(f"    센서 자신의 주소          :: {eth['sensor_ip']}:{eth['sensor_port']}")
        print(f"    서브넷 {eth['subnet_mask']}  게이트웨이 {eth['gateway']}  MAC {eth['mac']}")
        if (eth["pc_ip"], eth["pc_port"]) != (args.pc_ip, args.pc_port):
            print(f"    [경고] 요청한 {args.pc_ip}:{args.pc_port} 와 다릅니다. "
                  "센서는 저장된 주소로 스트림을 보낼 수 있습니다.")

    print(f"\n[2] 스트리밍 {args.seconds:.0f}초 관찰")
    send(sock, target, CAT_STREAM_ENABLE, SM_SET, b"\x01")
    ack, ack_sender, _, err = await_response(sock, CAT_STREAM_ENABLE, args.timeout)
    report["start_ack"] = ack is not None
    if ack is None:
        print(f"  시작 명령에 응답이 없습니다 ({err})")
    else:
        print(f"  시작 응답 :: {ack_sender[0]}:{ack_sender[1]}")

    sources = {}
    pages = {}
    # 프레임당 페이지 수는 기종마다 다릅니다(GL5 6, GL3 4). 패킷 헤더의 값을 씁니다.
    page_length = None
    packets = 0
    frames = 0
    first_frame = None
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        sock.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            data, sender = sock.recvfrom(65535)
        except socket.timeout:
            break
        except OSError:
            break
        packets += 1
        parsed = parse_packet(data)
        key = f"{sender[0]}:{sender[1]}"
        sources[key] = sources.get(key, 0) + 1
        if parsed["error"] is not None:
            continue
        if parsed["cat"] == CAT_STREAM_DATA:
            pages[parsed["page_idx"]] = pages.get(parsed["page_idx"], 0) + 1
            page_length = parsed["page_length"]
            if parsed["page_idx"] == page_length - 1:
                frames += 1
            if first_frame is None and parsed["page_idx"] == 0:
                first_frame = parsed

    send(sock, target, CAT_STREAM_ENABLE, SM_SET, b"\x00")
    stop, _, _, _ = await_response(sock, CAT_STREAM_ENABLE, args.timeout)
    report["stop_ack"] = stop is not None
    sock.close()

    report["packets"] = packets
    report["frames"] = frames
    report["sources"] = sources
    report["pages_seen"] = {str(k): v for k, v in sorted(pages.items())}

    print(f"  패킷 {packets}개, 완성 프레임 {frames}개")
    print(f"  출발지별 패킷 수: {sources if sources else '없음'}")
    if pages:
        print(f"  페이지 번호 분포: {report['pages_seen']}")
        print(f"  프레임당 페이지 수(헤더): {page_length}  (GL5 6, GL3 4)")
        report["page_length"] = page_length
        if sorted(pages) != list(range(page_length)):
            print(f"    [경고] 0~{page_length - 1} 이 모두 보이지 않습니다.")
    if first_frame is not None:
        n = struct.unpack_from("<H", first_frame["payload"], 0)[0] \
            if len(first_frame["payload"]) >= 2 else 0
        print(f"  첫 페이지가 보고한 포인트 수: {n}")
        report["points_reported"] = n

    print("\n[3] 판정")
    if frames > 0:
        print("  스트림이 이 소켓까지 도달했습니다.")
        if eth and len(sources) == 1:
            src_ip = next(iter(sources)).split(":")[0]
            print(f"  스트림 출발지는 {src_ip} 하나입니다.")
    elif packets > 0:
        print("  패킷은 왔지만 프레임이 완성되지 않았습니다. 페이지 분포를 보세요.")
    elif report["start_ack"]:
        print("  명령 응답은 받았지만 스트림이 오지 않습니다.")
        if eth:
            print(f"  센서는 {eth['pc_ip']}:{eth['pc_port']} 로 쏘고 있을 가능성이 큽니다. "
                  "이 소켓의 주소와 비교하세요.")
    else:
        print("  센서가 전혀 응답하지 않습니다. 배선, IP, 포트를 확인하세요.")

    if args.output:
        with open(args.output, "w") as handle:
            json.dump(report, handle, indent=2, ensure_ascii=False, sort_keys=True)
        print(f"\n결과를 {args.output} 에 저장했습니다.")

    return 0 if frames > 0 else 4


def self_test():
    info = {"pc_ip": "10.110.1.3", "sensor_ip": "10.110.1.2",
            "subnet_mask": "255.255.255.0", "gateway": "0.0.0.0",
            "mac": "00-11-22-33-44-55", "pc_port": 3000, "sensor_port": 2000}
    from gl5_protocol import build_ethernet_info
    packed = build_ethernet_info(**info)
    assert len(packed) == 26, len(packed)
    decoded = parse_ethernet_info(packed)
    for key, value in info.items():
        assert decoded[key] == value, (key, decoded[key], value)

    request = build_packet(CAT_ETHERNET_INFO, SM_GET, BI_PC2DEV, 0, 1, b"\x01")
    parsed = parse_packet(request, expect_bi=BI_PC2DEV)
    assert parsed["error"] is None and parsed["cat"] == CAT_ETHERNET_INFO

    # 깨진 패킷을 이유와 함께 돌려주는지.
    broken = bytearray(build_packet(CAT_STREAM_ENABLE, SM_SET, BI_DEV2PC, 0, 1, b"\x01"))
    broken[-1] ^= 0xFF
    assert parse_packet(bytes(broken))["error"] == "체크섬 불일치"

    print("자체 검사 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
