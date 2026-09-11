#!/usr/bin/env bash
# Real hardware only. Does not modify the network or publish simulated data.
set -eo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS="${1:-$LAB_ROOT/src/gl5_driver/config/gl5.yaml}"
PARAMS="$(realpath "$PARAMS")"
RUN_DIR="$LAB_ROOT/artifacts/hardware_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
export ROS_LOG_DIR="$RUN_DIR/ros_log"
source /opt/ros/humble/setup.bash
source "$LAB_ROOT/install/setup.bash"
mapfile -t SETTINGS < <(python3 - "$PARAMS" <<'PY'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))['gl5_node']['ros__parameters']
for k in ('sensor_ip', 'sensor_port', 'pc_ip', 'pc_port'):
    print(p[k])
PY
)
if [[ ${#SETTINGS[@]} != 4 ]]; then echo "Invalid GL5 parameter file" >&2; exit 2; fi
cp "$PARAMS" "$RUN_DIR/gl5.yaml"
"$LAB_ROOT/build/standalone/gl5_receive" "${SETTINGS[@]}" 30 "$RUN_DIR/sdk.json" 2>&1 | tee "$RUN_DIR/sdk.log"
NODE_PID=""
BAG_PID=""
cleanup() {
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID"; wait "$BAG_PID" || true
  fi
  if [[ -n "$NODE_PID" ]] && kill -0 "$NODE_PID" 2>/dev/null; then
    kill -INT "$NODE_PID"; wait "$NODE_PID" || true
  fi
}
trap cleanup EXIT
"$LAB_ROOT/install/gl5_driver/lib/gl5_driver/gl5_node" \
  --ros-args --params-file "$PARAMS" > "$RUN_DIR/node.log" 2>&1 &
NODE_PID=$!
timeout --signal=INT --kill-after=10 12 ros2 bag record -o "$RUN_DIR/bag" /scan /points > "$RUN_DIR/bag.log" 2>&1 &
BAG_PID=$!
python3 "$LAB_ROOT/tools/verify_topics.py" --seconds 60 --output "$RUN_DIR/ros.json" | tee "$RUN_DIR/verify.log"
kill -0 "$NODE_PID"
python3 "$LAB_ROOT/tools/plot_scan.py" "$RUN_DIR/ros_scan.json" --output "$RUN_DIR/scan.png"
ros2 bag info "$RUN_DIR/bag" | tee "$RUN_DIR/bag_info.txt"
python3 - "$RUN_DIR/bag/metadata.yaml" <<'PY'
import sys, yaml
info = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
counts = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
if any(counts.get(t, 0) == 0 for t in ('/scan', '/points')):
    raise SystemExit('Bag recording is missing scan or points messages')
PY
echo "Real hardware verification passed. Evidence: $RUN_DIR"
echo "Run bash scripts/run.sh params_file:=$PARAMS to inspect the live scan in RViz."
