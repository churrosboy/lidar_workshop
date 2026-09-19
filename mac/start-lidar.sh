#!/usr/bin/env bash
# macOS Docker Desktop 에서 드라이버 + 장애물 감지 + RViz 를 한 번에 띄웁니다.
#
#   bash mac/start-lidar.sh           # 실습 실행 (RViz 는 VNC 로 봅니다)
#   bash mac/start-lidar.sh shell     # 같은 네트워크 설정으로 컨테이너 셸
#
# 다른 터미널에서 mac/gl5_relay.py 를 먼저 띄워 두어야 합니다. GL5 는 스트림과 명령
# 응답을 이더넷 브로드캐스트로 보내는데 macOS 커널이 그런 프레임을 소켓에 넘기지 않고
# 버립니다. 중계기가 그 프레임을 떠서 유니캐스트로 다시 보내고, 컨테이너는 센서 대신
# 호스트(host.docker.internal)에 접속합니다.
set -euo pipefail

MAC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAB_ROOT="$(cd -- "$MAC_DIR/.." && pwd)"
IMAGE="${GL5_IMAGE:-lidar-workshop:humble-arm64}"
CONTAINER="${GL5_CONTAINER:-lidar-workshop}"
PARAMS="$LAB_ROOT/src/gl5_driver/config/gl5.yaml"
RUNTIME="$MAC_DIR/.local"
VNC_PORT="${GL5_VNC_PORT:-5901}"
VNC_PASSWORD="${GL5_VNC_PASSWORD:-gl5lab}"
ACTION="${1:-workshop}"
[ $# -gt 0 ] && shift

# PyYAML 없이 읽습니다. macOS 기본 python3 에는 PyYAML 이 없습니다.
param() {
  sed -nE "s/^[[:space:]]*$1:[[:space:]]*\"?([^\"#[:space:]]+)\"?.*/\1/p" "$PARAMS" | head -1
}
SENSOR_IP="$(param sensor_ip)"
PC_IP="$(param pc_ip)"
PC_PORT="$(param pc_port)"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "이미지 $IMAGE 가 없습니다. 먼저 빌드하세요:" >&2
  echo "  docker build --platform linux/arm64 -t $IMAGE ." >&2
  exit 1
fi
if docker container inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "컨테이너 $CONTAINER 가 이미 있습니다. 실행 중인 실습을 Ctrl+C 로 끄거나" >&2
  echo "  docker stop $CONTAINER" >&2
  exit 1
fi

HOST_IP="$(docker run --rm "$IMAGE" getent ahostsv4 host.docker.internal 2>/dev/null | awk '{print $1; exit}')"
[ -z "$HOST_IP" ] && { echo "host.docker.internal 을 찾을 수 없습니다" >&2; exit 2; }

mkdir -p "$RUNTIME"
# sudo 로 실행되면 root 소유가 되어 다음 실행에서 영역을 저장하지 못합니다.
[ -n "${SUDO_USER:-}" ] && chown "$SUDO_USER" "$RUNTIME" 2>/dev/null || true
sed -E "s/^([[:space:]]*sensor_ip:).*/\1 \"$HOST_IP\"/" "$PARAMS" > "$RUNTIME/gl5.yaml"

# 펌웨어에 저장된 PC 포트로 데이터가 오므로 그 포트를 컨테이너로 넘깁니다.
# 루프백에만 공개합니다. 0.0.0.0 으로 열면 중계기가 잡고 있는 포트와 부딪힙니다.
# src 는 읽기·쓰기로 물립니다. 이미지가 --symlink-install 로 빌드돼 있어 여기의
# 파이썬 파일을 Mac 에서 고치면 다시 빌드하지 않아도 다음 실행에 반영됩니다.
RUN_FLAGS=(--rm --name "$CONTAINER" -p "127.0.0.1:$PC_PORT:$PC_PORT/udp"
  -v "$RUNTIME:/mac-runtime"
  -v "$LAB_ROOT/scripts:/opt/lidar_workshop/scripts:ro"
  -v "$LAB_ROOT/src:/opt/lidar_workshop/src"
  -e GL5_PARAMS_FILE=/mac-runtime/gl5.yaml
  -e "GL5_OBSTACLE_PARAMS_FILE=${GL5_OBSTACLE_PARAMS_FILE:-}"
  -e GL5_REGION_FILE=/mac-runtime/gl5_region.json)

echo "중계기 경유 :: 컨테이너 -> 호스트 $HOST_IP (실제 센서 $SENSOR_IP), 수신 포트 $PC_PORT"
echo

case "$ACTION" in
  workshop)
    # 센서가 이미 스트리밍 중이면 SDK 가 스트림 페이지를 명령 응답으로 잘못 읽어
    # streamStart 가 실패합니다. 먼저 스트림을 끕니다.
    python3 "$MAC_DIR/gl5_probe.py" --stop-only \
      --sensor-ip "$SENSOR_IP" --pc-ip "$PC_IP" >/dev/null 2>&1 || true
    sleep 1

    # macOS 의 XQuartz 는 OpenGL 2.1 만 제공해 RViz 가 뜨지 않습니다. 컨테이너 안의
    # 가상 화면에 그리고 VNC 로 내보냅니다.
    TTY_FLAGS=(-i); [ -t 0 ] && TTY_FLAGS=(-it)
    echo "RViz 가 뜨면 화면 공유로 접속하세요."
    echo
    echo "    open vnc://localhost:$VNC_PORT"
    echo "  암호: $VNC_PASSWORD"
    echo
    docker run "${TTY_FLAGS[@]}" "${RUN_FLAGS[@]}" \
      -p "127.0.0.1:$VNC_PORT:$VNC_PORT" \
      -e "GL5_VNC_PORT=$VNC_PORT" -e "GL5_VNC_PASSWORD=$VNC_PASSWORD" \
      "$IMAGE" /opt/lidar_workshop/scripts/workshop_vnc.sh \
      bash /opt/lidar_workshop/scripts/run.sh "$@"
    ;;
  shell)
    docker run -it "${RUN_FLAGS[@]}" "$IMAGE" bash
    ;;
  *)
    echo "사용법: bash mac/start-lidar.sh [workshop|shell]" >&2
    exit 2
    ;;
esac
