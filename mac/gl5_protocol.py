"""GL5 UDP 규약 인코더와 디코더.

SDK 소스를 링크하지 않고 규약만 보고 구현했습니다. mac/gl5_probe.py 가 씁니다.
표준 라이브러리 외에는 의존성이 없어 macOS, Windows, Linux 어디서든 돕니다.
"""

import struct

SYNC = b"\xc3\x51\xa1\xf8"
PACKET_END = 0xC2
HEADER_BYTES = 12
# 헤더 12 + PE 1 + CS 1. total_length 는 페이로드에 이 값을 더한 것입니다.
FRAME_OVERHEAD = HEADER_BYTES + 2

SM_SET, SM_GET, SM_STREAM, SM_ERROR = 0, 1, 2, 255
BI_PC2DEV, BI_DEV2PC = 0x21, 0x12

CAT_CONSOLE = 0x0001
CAT_OPERATION_MODE = 0x0101
CAT_STREAM_DATA = 0x0102
CAT_STREAM_ENABLE = 0x0103
CAT_AREA_LEVEL_DATA = 0x0203
CAT_AREA_DATA_FINISH = 0x0204
CAT_SERIAL_NUM = 0x020A
CAT_ETHERNET_INFO = 0x020B
CAT_FW_VERSION = 0x020C

CAT_NAMES = {
    CAT_CONSOLE: "Console",
    CAT_OPERATION_MODE: "OperationMode",
    CAT_STREAM_DATA: "StreamData",
    CAT_STREAM_ENABLE: "StreamEnable",
    CAT_AREA_LEVEL_DATA: "AreaLevelData",
    CAT_AREA_DATA_FINISH: "AreaDataFinish",
    CAT_SERIAL_NUM: "SerialNum",
    CAT_ETHERNET_INFO: "EthernetInfo",
    CAT_FW_VERSION: "FWVersion",
}

SM_NAMES = {SM_SET: "SET", SM_GET: "GET", SM_STREAM: "STREAM", SM_ERROR: "ERROR"}

# GL5.h 의 고정값. 프레임 하나는 반드시 6페이지이고 화각은 270도입니다.
# GL3.h 는 4페이지, 1000점, 180도입니다. 진단 도구는 패킷 헤더의 page_length 를 씁니다.
PAGES_PER_FRAME = 6
NUM_POINTS = 1500
H_FOV_DEG = 270.0
# parseStreamData 가 누적 페이로드에서 빼는 메타데이터 크기. 앞 2바이트가 포인트
# 개수이고 나머지 20바이트는 뒤쪽 여백입니다.
META_BYTES = 22
TRAILER_BYTES = META_BYTES - 2


def checksum(data):
    """CS 를 뺀 앞부분 전체의 XOR. 초기값은 0x00 입니다."""
    value = 0
    for byte in data:
        value ^= byte
    return value


def build_packet(cat, sm, bi, page_idx, page_length, payload):
    """12바이트 헤더 + 페이로드 + PE + CS."""
    total_length = len(payload) + FRAME_OVERHEAD
    packet = bytearray(SYNC)
    packet += struct.pack("<H", total_length)  # TL 은 리틀엔디언
    packet.append(page_idx)
    packet.append(page_length)
    packet.append(sm)
    packet.append(bi)
    packet += struct.pack(">H", cat)  # CAT 은 빅엔디언
    packet += payload
    packet.append(PACKET_END)
    packet.append(checksum(packet))
    return bytes(packet)


def parse_packet(data, expect_bi=None):
    """패킷을 dict 로 풉니다. 규약에 어긋나면 이유를 담은 dict 를 돌려줍니다.

    깨진 패킷도 진단에 쓸모가 있으므로 None 대신 error 를 채워 돌려줍니다.
    """
    if len(data) < FRAME_OVERHEAD:
        return {"error": f"너무 짧음 ({len(data)}바이트)"}
    if data[:4] != SYNC:
        return {"error": f"동기 바이트 불일치 {data[:4].hex()}"}

    total_length = struct.unpack_from("<H", data, 4)[0]
    result = {
        "error": None,
        "total_length": total_length,
        "page_idx": data[6],
        "page_length": data[7],
        "sm": data[8],
        "bi": data[9],
        "cat": struct.unpack_from(">H", data, 10)[0],
        "length_matches": total_length == len(data),
    }
    result["cat_name"] = CAT_NAMES.get(result["cat"], f"0x{result['cat']:04X}")
    result["sm_name"] = SM_NAMES.get(result["sm"], str(result["sm"]))

    if data[-2] != PACKET_END:
        result["error"] = f"PE 가 0xC2 가 아님 (0x{data[-2]:02X})"
        return result
    if checksum(data[:-1]) != data[-1]:
        result["error"] = "체크섬 불일치"
        return result
    if expect_bi is not None and result["bi"] != expect_bi:
        result["error"] = f"방향 바이트 0x{result['bi']:02X}, 기대 0x{expect_bi:02X}"
        return result

    payload_len = total_length - FRAME_OVERHEAD if total_length >= FRAME_OVERHEAD + 1 else 0
    result["payload"] = data[HEADER_BYTES:HEADER_BYTES + payload_len]
    return result


def parse_ethernet_info(payload):
    """EthernetInfo GET 응답 26바이트.

    GL5.cpp 의 parseEthernetInfoAck 와 같은 순서입니다. 센서가 플래시에 저장하고
    있는 목적지 PC 주소를 읽는 것이 목적입니다.
    """
    if len(payload) < 26:
        return {"error": f"페이로드가 26바이트보다 짧음 ({len(payload)})"}
    dotted = lambda off: ".".join(str(b) for b in payload[off:off + 4])
    return {
        "error": None,
        "pc_ip": dotted(0),
        "sensor_ip": dotted(4),
        "subnet_mask": dotted(8),
        "gateway": dotted(12),
        "mac": "-".join(f"{b:02X}" for b in payload[16:22]),
        "pc_port": struct.unpack_from("<H", payload, 22)[0],
        "sensor_port": struct.unpack_from("<H", payload, 24)[0],
    }


def build_ethernet_info(pc_ip, sensor_ip, subnet_mask, gateway, mac, pc_port, sensor_port):
    """parse_ethernet_info 의 역방향. 가짜 센서가 응답을 만들 때 씁니다."""
    packed = bytearray()
    for dotted in (pc_ip, sensor_ip, subnet_mask, gateway):
        packed += bytes(int(part) for part in dotted.split("."))
    packed += bytes(int(part, 16) for part in mac.split("-"))
    packed += struct.pack("<HH", pc_port, sensor_port)
    return bytes(packed)


def frame_payload(distances_mm, pulsewidths):
    """6페이지에 걸쳐 누적될 스트림 페이로드 전체.

    구조는 [포인트 개수 2바이트][포인트 4바이트 * N][여백 20바이트] 입니다.
    parseStreamData 는 (전체 - 22) / 4 로 포인트 수를 다시 계산하므로 이 크기가
    맞아야 합니다.
    """
    payload = bytearray(struct.pack("<H", len(distances_mm)))
    for distance, pulsewidth in zip(distances_mm, pulsewidths):
        payload += struct.pack("<HH", distance, pulsewidth)
    payload += bytes(TRAILER_BYTES)
    return bytes(payload)


def split_pages(payload, pages=PAGES_PER_FRAME):
    """페이로드를 정확히 6조각으로 나눕니다. 마지막 조각만 길이가 다릅니다."""
    chunk = -(-len(payload) // pages)  # 올림
    parts = [payload[i * chunk:(i + 1) * chunk] for i in range(pages)]
    if any(len(part) == 0 for part in parts):
        raise ValueError("페이로드가 6페이지로 나누기에 너무 짧습니다")
    return parts
