#!/usr/bin/env bash
# 컨테이너 안에서 가상 화면을 띄우고 그 위에 실습을 실행합니다.
#
# macOS 의 XQuartz 는 OpenGL 2.1 만 제공하고 Ogre 가 요구하는 fbConfig 를 내주지
# 않습니다. RViz 는 "No matching fbConfigs or visuals found" 로 항상 실패합니다.
# 그래서 호스트의 X 서버를 쓰지 않고, 컨테이너 안에 Xvfb 로 화면을 만들어 소프트웨어
# 렌더링하고 VNC 로 내보냅니다. 클릭도 그대로 전달되므로 영역 지정 실습이 됩니다.
#
# 이 스크립트는 컨테이너 안에서 실행됩니다. 호스트에서는 mac/start-lidar.sh 를
# 쓰세요.
set -u

DISPLAY_NUM="${GL5_VNC_DISPLAY:-99}"
VNC_PORT="${GL5_VNC_PORT:-5901}"
SCREEN="${GL5_VNC_GEOMETRY:-1400x900x24}"
export DISPLAY=":$DISPLAY_NUM"

# Ogre 는 하드웨어 가속을 못 찾으면 멈춥니다. llvmpipe 로 명시해 둡니다.
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_X11_NO_MITSHM=1
# numpy 의 OpenBLAS 는 코어 수만큼 스레드를 만들고, 일이 끝나도 잠깐 바쁘게 돌며 기다립니다.
# 40 Hz 스캔은 그 대기가 끝나기 전에 다시 들어와서 노드마다 코어 두세 개를 헛돌게 합니다.
# 여기서 다루는 행렬은 작아 1 스레드여도 느려지지 않습니다.
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p "$XDG_RUNTIME_DIR"

cleanup() {
  [ -n "${VNC_PID:-}" ] && kill "$VNC_PID" 2>/dev/null
  [ -n "${XVFB_PID:-}" ] && kill "$XVFB_PID" 2>/dev/null
}
trap cleanup EXIT

Xvfb "$DISPLAY" -screen 0 "$SCREEN" -nolisten tcp &
XVFB_PID=$!

# X 서버가 준비될 때까지 기다립니다. 바로 붙으면 x11vnc 가 실패합니다.
# xdpyinfo 는 이미지에 없으므로 유닉스 소켓이 생겼는지로 판단합니다.
SOCKET="/tmp/.X11-unix/X$DISPLAY_NUM"
for _ in $(seq 1 40); do
  [ -S "$SOCKET" ] && break
  sleep 0.25
done
if [ ! -S "$SOCKET" ]; then
  echo "가상 화면 $DISPLAY 을 띄우지 못했습니다." >&2
  kill "$XVFB_PID" 2>/dev/null
  exit 1
fi

# macOS 화면 공유는 인증 없는 VNC 를 거부하고 자격 증명을 계속 물어봅니다.
# 암호를 걸어야 붙습니다. VNC 암호는 8자까지만 유효합니다.
VNC_PASSWORD="${GL5_VNC_PASSWORD:-gl5lab}"
# -forever 로 뷰어가 끊겨도 살아 있게 하고, -shared 로 여러 명이 볼 수 있게 합니다.
# -threads 가 없으면 x11vnc 는 한 루프에서 화면 폴링과 클라이언트 입출력을 번갈아 합니다.
# 이 화면은 라이다 40 Hz 로 쉬지 않고 변해서 폴링이 루프를 계속 잡고, 마우스 클릭이 그 뒤에
# 줄을 섭니다. 버튼이 한참 뒤에 눌리는 원인이라 입출력을 별도 스레드로 뺍니다.
x11vnc -display "$DISPLAY" -rfbport "$VNC_PORT" -forever -shared -threads \
  -passwd "$VNC_PASSWORD" -quiet -noxdamage >/tmp/x11vnc.log 2>&1 &
VNC_PID=$!
sleep 1

echo "가상 화면 $DISPLAY ($SCREEN), VNC $VNC_PORT 번 대기 중"
echo

exec "$@"
