#!/usr/bin/env bash
# macOS Docker Desktop 에서 녹화(rosbag2)를 재생하며 장애물 감지 + RViz 를 띄웁니다.
# 라이다 실물, USB 이더넷 어댑터, 관리자 비밀번호가 모두 필요 없습니다.
#
#   bash mac/start-bag.sh                                           # 들어 있는 녹화 목록
#   bash mac/start-bag.sh gl5_sopcom_stationary                     # 실습 실행
#   bash mac/start-bag.sh gl5_sopcom_stationary scan_matcher:=false # launch 인자 통과
#   bash mac/start-bag.sh shell                                     # 컨테이너 셸
#
# RViz 는 컨테이너 안에서 그려 VNC 로 내보냅니다. 화면은 `open vnc://localhost:5901` 로 봅니다.
set -euo pipefail

MAC_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LAB_ROOT="$(cd -- "$MAC_DIR/.." && pwd)"
IMAGE="${GL5_IMAGE:-lidar-workshop:humble-arm64}"
CONTAINER="${GL5_CONTAINER:-lidar-workshop}"
BAGS_DIR="$LAB_ROOT/bags"
RUNTIME="$MAC_DIR/.local"
VNC_PORT="${GL5_VNC_PORT:-5901}"
VNC_PASSWORD="${GL5_VNC_PASSWORD:-gl5lab}"

list_bags() {
  local meta found=1
  for meta in "$BAGS_DIR"/*/metadata.yaml; do
    [ -f "$meta" ] || continue
    [ $found -eq 1 ] && echo "쓸 수 있는 녹화:"
    found=0
    echo "  $(basename "$(dirname "$meta")")"
  done
  [ $found -eq 0 ] || echo "$BAGS_DIR 에 녹화가 없습니다." >&2
}

usage() {
  echo "사용법: bash mac/start-bag.sh <녹화 이름> [launch 인자...]" >&2
  echo "        bash mac/start-bag.sh shell" >&2
  echo >&2
  list_bags >&2
}

# 첫 인자가 launch 인자(name:=value)면 녹화 이름이 빠진 것입니다.
BAG=""
ACTION="workshop"
if [ $# -gt 0 ] && [[ "$1" != *":="* ]]; then
  if [ "$1" = "shell" ]; then ACTION="shell"; else BAG="$1"; fi
  shift
fi

if [ "$ACTION" = "workshop" ] && [ -z "$BAG" ]; then
  usage
  exit 2
fi
if [ -n "$BAG" ] && [ ! -f "$BAGS_DIR/$BAG/metadata.yaml" ]; then
  echo "녹화를 찾지 못했습니다: $BAG" >&2
  echo "($BAGS_DIR/$BAG 에 metadata.yaml 이 있어야 합니다.)" >&2
  echo >&2
  list_bags >&2
  exit 2
fi

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

mkdir -p "$RUNTIME"
# sudo 로 실행되면 root 소유가 되어 다음 실행에서 영역을 저장하지 못합니다.
[ -n "${SUDO_USER:-}" ] && chown "$SUDO_USER" "$RUNTIME" 2>/dev/null || true

# 녹화는 이미지에 넣지 않고 호스트에서 읽기 전용으로 물립니다. bags/ 에 새 녹화를 넣어도
# 이미지를 다시 빌드할 필요가 없습니다.
# src 는 읽기·쓰기로 물립니다. 이미지가 --symlink-install 로 빌드돼 있어 여기의
# 파이썬 파일을 Mac 에서 고치면 다시 빌드하지 않아도 다음 실행에 반영됩니다.
RUN_FLAGS=(--rm --name "$CONTAINER"
  -v "$RUNTIME:/mac-runtime"
  -v "$BAGS_DIR:/opt/lidar_workshop/bags:ro"
  -v "$LAB_ROOT/scripts:/opt/lidar_workshop/scripts:ro"
  -v "$LAB_ROOT/src:/opt/lidar_workshop/src"
  -e "GL5_OBSTACLE_PARAMS_FILE=${GL5_OBSTACLE_PARAMS_FILE:-}"
  -e GL5_REGION_FILE=/mac-runtime/gl5_region.json)

case "$ACTION" in
  workshop)
    # macOS 의 XQuartz 는 OpenGL 2.1 만 제공해 RViz 가 뜨지 않습니다. 컨테이너 안의
    # 가상 화면에 그리고 VNC 로 내보냅니다.
    TTY_FLAGS=(-i); [ -t 0 ] && TTY_FLAGS=(-it)
    echo "녹화 재생 :: $BAG (끝까지 가면 처음부터 다시 재생됩니다)"
    echo
    echo "RViz 가 뜨면 화면 공유로 접속하세요."
    echo
    echo "    open vnc://localhost:$VNC_PORT"
    echo "  암호: $VNC_PASSWORD"
    echo
    docker run "${TTY_FLAGS[@]}" "${RUN_FLAGS[@]}" \
      -p "127.0.0.1:$VNC_PORT:$VNC_PORT" \
      -e "GL5_VNC_PORT=$VNC_PORT" -e "GL5_VNC_PASSWORD=$VNC_PASSWORD" \
      -e "GL5_VNC_GEOMETRY=${GL5_VNC_GEOMETRY:-1400x900x24}" \
      "$IMAGE" /opt/lidar_workshop/scripts/workshop_vnc.sh \
      bash /opt/lidar_workshop/scripts/run.sh "bag:=$BAG" "$@"
    ;;
  shell)
    docker run -it "${RUN_FLAGS[@]}" "$IMAGE" bash
    ;;
esac
