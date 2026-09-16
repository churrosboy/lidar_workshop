#!/usr/bin/env bash
# macOS 에서 라이다가 꽂힌 유선 이더넷 어댑터 이름(en6, en8 ...)을 찾습니다.
#
#   IFACE=$(bash mac/find_iface.sh) && echo "라이다 어댑터: $IFACE"
#
# 제품 이름으로 찾지 않습니다. "USB 10/100 LAN", "USB 10/100/1000 LAN" 처럼
# 어댑터마다 이름이 달라 한 가지 패턴으로는 다른 참여자의 Mac 에서 빈 값이 나옵니다.
# 대신 유선 LAN/Ethernet 포트 가운데 링크가 올라온(status: active) 것을 고릅니다.
# 그래서 라이다 전원을 켜고 케이블을 꽂은 뒤에 실행해야 합니다.
#
# 이름만 표준 출력으로 내보내고 안내는 표준 에러로 보냅니다. 정확히 하나가 아니면
# 빈 값으로 이어지지 않도록 종료 코드 1 로 끝냅니다.
set -euo pipefail

if [ "$(uname)" != Darwin ]; then
  echo "macOS 전용입니다. 리눅스는 'ip -br link', Windows 는 'Get-NetAdapter' 로 확인하세요." >&2
  exit 2
fi

wired=()
while IFS='|' read -r device port; do
  wired+=("$device|$port")
done < <(networksetup -listallhardwareports | awk '
  /^Hardware Port/ { sub(/^Hardware Port: /, ""); port = $0 }
  /^Device/ && port ~ /LAN|Ethernet/ && port !~ /Thunderbolt|Bridge/ { print $2 "|" port }')

active=()
for entry in ${wired[@]+"${wired[@]}"}; do
  device="${entry%%|*}"
  ifconfig "$device" 2>/dev/null | grep -q "status: active" && active+=("$entry")
done

if [ "${#active[@]}" -eq 1 ]; then
  echo "${active[0]%%|*}"
  exit 0
fi

if [ "${#active[@]}" -eq 0 ]; then
  echo "링크가 올라온 유선 어댑터가 없습니다. 라이다 전원과 케이블, USB 어댑터를 확인하세요." >&2
else
  echo "링크가 올라온 유선 어댑터가 여러 개입니다. 라이다만 남기고 뽑거나, 이름을 직접 쓰세요." >&2
fi
echo "유선 어댑터 목록:" >&2
for entry in ${wired[@]+"${wired[@]}"}; do
  device="${entry%%|*}"
  status="$(ifconfig "$device" 2>/dev/null | awk '/status:/ {print $2}')"
  echo "  $device  ${entry#*|}  (${status:-unknown})" >&2
done
exit 1
