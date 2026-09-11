#!/usr/bin/env bash
# Only a temporary Ethernet profile; no default gateway and no saved sensor changes.
set -euo pipefail
ACTION="${1:-show}"
PROFILE=gl5-lab-temp
IFACE="${GL5_INTERFACE:-enp0s31f6}"
ADDRESS="${GL5_PC_CIDR:-10.110.1.3/24}"
case "$ACTION" in
  show)
    ip -br addr show "$IFACE"
    ip route
    ip neigh show dev "$IFACE"
    ;;
  up)
    if nmcli -g connection.id connection show "$PROFILE" >/dev/null 2>&1; then
      echo "$PROFILE already exists. Inspect with: nmcli connection show $PROFILE"
      echo "To change addresses, run network.sh down first."
      exit 1
    fi
    nmcli connection add save no type ethernet con-name "$PROFILE" ifname "$IFACE" \
      ipv4.method manual ipv4.addresses "$ADDRESS" ipv4.never-default yes \
      ipv6.method disabled connection.autoconnect no
    nmcli connection up "$PROFILE"
    ;;
  down)
    nmcli connection delete "$PROFILE"
    ;;
  *) echo "Usage: bash scripts/network.sh {show|up|down}" >&2; exit 2 ;;
esac
