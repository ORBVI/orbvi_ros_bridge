#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/orbvi_ros_bridge_rate_check.sh [options]

Run a ROS1 topic frequency check from the ORBVI SDK ROS bridge side.
The primary measurement is rostopic hz over /orbvi/* topics, not board-side
/livox/* or SDK HTTP counters.
All rates are measured on the ROS master/bridge host where this script runs.
When --ssh is used, the script is copied over stdin and executed on that remote
bridge host; the device host is only used to start the bridge process.

Common examples:
  # Run directly on the ROS bridge host, using an already running bridge.
  scripts/orbvi_ros_bridge_rate_check.sh \
    --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash

  # From a dev machine, run on a remote bridge host, start a temporary bridge,
  # and check all /orbvi/* rates on that remote ROS master.
  SSH_PASSWORD=<password> scripts/orbvi_ros_bridge_rate_check.sh \
    --ssh <user>@<bridge-host> \
    --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
    --device-host <device-ip> \
    --start-bridge

  # Check only MID360 ROS bridge topics.
  SSH_PASSWORD=<password> scripts/orbvi_ros_bridge_rate_check.sh \
    --ssh <user>@<bridge-host> \
    --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
    --topics /orbvi/lidar/custom,/orbvi/lidar/imu

Options:
  --ssh USER@HOST              Run the check on a remote ROS bridge host via ssh.
                               If SSH_PASSWORD is set, sshpass is used.
  --ros-setup PATH             ROS setup.bash path (default: /opt/ros/noetic/setup.bash).
  --bridge-setup PATH          Bridge workspace setup.bash path. Strongly recommended.
  --topic-prefix PREFIX        Topic prefix to discover (default: /orbvi).
  --topics CSV                 Explicit topic list. Skips discovery.
  --sample-sec SECONDS         rostopic hz duration per topic (default: 20).
  --window COUNT               rostopic hz window size (default: 100).
  --output-dir DIR             Output directory for logs and CSV (default: /tmp/...).
  --start-bridge               Start a temporary orbvi_ros_bridge before checking.
  --keep-bridge                Do not stop the temporary bridge on exit.
  --device-host HOST           Device host used only to launch the bridge.
  --control-port PORT          Device control port for bridge launch (default: 18088).
  --streams CSV                Bridge streams when launched (default: raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio).
  --image-mode MODE            Bridge image mode when launched (default: raw-only).
  --bridge-wait-sec SECONDS    Wait after starting bridge (default: 10).
  -h, --help                   Show this help.
USAGE
}

ssh_target=""
remote_run=0
ros_setup="/opt/ros/noetic/setup.bash"
bridge_setup=""
topic_prefix="/orbvi"
topics_csv=""
sample_sec=20
window_size=100
output_dir=""
start_bridge=0
keep_bridge=0
device_host=""
control_port=18088
streams="raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio"
image_mode="raw-only"
bridge_wait_sec=10

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ssh) ssh_target="${2:-}"; shift 2 ;;
    --remote-run) remote_run=1; shift ;;
    --ros-setup) ros_setup="${2:-}"; shift 2 ;;
    --bridge-setup) bridge_setup="${2:-}"; shift 2 ;;
    --topic-prefix) topic_prefix="${2:-}"; shift 2 ;;
    --topics) topics_csv="${2:-}"; shift 2 ;;
    --sample-sec) sample_sec="${2:-}"; shift 2 ;;
    --window) window_size="${2:-}"; shift 2 ;;
    --output-dir) output_dir="${2:-}"; shift 2 ;;
    --start-bridge) start_bridge=1; shift ;;
    --keep-bridge) keep_bridge=1; shift ;;
    --device-host) device_host="${2:-}"; shift 2 ;;
    --control-port) control_port="${2:-}"; shift 2 ;;
    --streams) streams="${2:-}"; shift 2 ;;
    --image-mode) image_mode="${2:-}"; shift 2 ;;
    --bridge-wait-sec) bridge_wait_sec="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

run_remote() {
  local -a args=(
    --remote-run
    --ros-setup "$ros_setup"
    --topic-prefix "$topic_prefix"
    --sample-sec "$sample_sec"
    --window "$window_size"
    --control-port "$control_port"
    --streams "$streams"
    --image-mode "$image_mode"
    --bridge-wait-sec "$bridge_wait_sec"
  )
  [[ -n "$bridge_setup" ]] && args+=(--bridge-setup "$bridge_setup")
  [[ -n "$topics_csv" ]] && args+=(--topics "$topics_csv")
  [[ -n "$output_dir" ]] && args+=(--output-dir "$output_dir")
  [[ "$start_bridge" -eq 1 ]] && args+=(--start-bridge)
  [[ "$keep_bridge" -eq 1 ]] && args+=(--keep-bridge)
  [[ -n "$device_host" ]] && args+=(--device-host "$device_host")

  local remote_cmd
  printf -v remote_cmd '%q ' bash -s -- "${args[@]}"

  local -a ssh_cmd=(ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$ssh_target")
  if [[ -n "${SSH_PASSWORD:-}" ]]; then
    if ! command -v sshpass >/dev/null 2>&1; then
      echo "SSH_PASSWORD is set but sshpass is not available" >&2
      exit 2
    fi
    ssh_cmd=(sshpass -p "$SSH_PASSWORD" "${ssh_cmd[@]}")
  fi

  "${ssh_cmd[@]}" "$remote_cmd" < "$0"
}

if [[ -n "$ssh_target" && "$remote_run" -eq 0 ]]; then
  run_remote
  exit $?
fi

if [[ -z "$output_dir" ]]; then
  output_dir="${TMPDIR:-/tmp}/orbvi_ros_bridge_rate_check-$(date +%Y%m%d-%H%M%S)"
fi
mkdir -p "$output_dir/topics"

summary_log="${output_dir}/summary.log"
result_csv="${output_dir}/rates.csv"
bridge_log="${output_dir}/orbvi_ros_bridge.log"
bridge_pid=""

log() {
  printf '%s %s\n' "$(date '+%F %T')" "$*" | tee -a "$summary_log"
}

source_ros_env() {
  if [[ -f "$ros_setup" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$ros_setup"
    set -u
  else
    echo "ROS setup not found: $ros_setup" >&2
    exit 2
  fi

  if [[ -n "$bridge_setup" ]]; then
    if [[ -f "$bridge_setup" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "$bridge_setup"
      set -u
    else
      echo "Bridge setup not found: $bridge_setup" >&2
      exit 2
    fi
  fi
}

stop_bridge() {
  if [[ "$start_bridge" -eq 1 && "$keep_bridge" -ne 1 ]]; then
    if [[ -n "$bridge_pid" ]]; then
      kill "$bridge_pid" >/dev/null 2>&1 || true
      wait "$bridge_pid" >/dev/null 2>&1 || true
    fi
    pkill -x orbvi_ros_bridge_node >/dev/null 2>&1 || true
  fi
}

cleanup() {
  local rc=$?
  stop_bridge
  exit "$rc"
}
trap cleanup EXIT INT TERM

start_temp_bridge() {
  if [[ -z "$device_host" ]]; then
    echo "--device-host is required with --start-bridge" >&2
    exit 2
  fi

  log "starting temporary orbvi_ros_bridge host=${device_host} port=${control_port} streams=${streams}"
  pkill -x orbvi_ros_bridge_node >/dev/null 2>&1 || true
  roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
    host:="$device_host" \
    control_port:="$control_port" \
    streams:="$streams" \
    image_mode:="$image_mode" \
    queue_size:=2 \
    max_receive_queue_depth:=4 \
    max_decode_queue_depth:=4 \
    >"$bridge_log" 2>&1 &
  bridge_pid=$!
  sleep "$bridge_wait_sec"

  if ! rosnode list 2>/dev/null | grep -qx '/orbvi_ros_bridge'; then
    echo "orbvi_ros_bridge did not appear in rosnode list; see $bridge_log" >&2
    tail -80 "$bridge_log" >&2 || true
    exit 1
  fi
}

discover_topics() {
  if [[ -n "$topics_csv" ]]; then
    printf '%s\n' "$topics_csv" | tr ',' '\n' | awk 'NF {print}'
    return
  fi

  rostopic list 2>/dev/null \
    | awk -v prefix="$topic_prefix" 'index($0, prefix "/") == 1 {print}' \
    | sort -u
}

safe_name() {
  printf '%s' "$1" | sed -E 's#^/##; s#[^A-Za-z0-9_.-]+#_#g'
}

extract_rate() {
  local file="$1"
  grep 'average rate:' "$file" | tail -1 | sed -E 's/.*average rate:[[:space:]]*([0-9.]+).*/\1/' || true
}

extract_field() {
  local file="$1"
  local key="$2"
  grep 'average rate:' "$file" | tail -1 | sed -E "s/.*${key}:[[:space:]]*([0-9.]+)s.*/\\1/" || true
}

measure_topic() {
  local topic="$1"
  local name type log_file rate min_v max_v std_v status
  name="$(safe_name "$topic")"
  log_file="${output_dir}/topics/${name}.hz.log"
  type="$(rostopic type "$topic" 2>&1 || true)"

  if [[ -z "$type" || "$type" == *"Cannot load message class"* || "$type" == *"ERROR:"* ]]; then
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
      "$topic" "${type:-UNKNOWN}" "TYPE_ERROR" "" "" "" "" "$log_file" "" \
      >> "$result_csv"
    return
  fi

  timeout "$sample_sec" rostopic hz "$topic" --window "$window_size" >"$log_file" 2>&1 || true
  rate="$(extract_rate "$log_file")"
  min_v="$(extract_field "$log_file" min)"
  max_v="$(extract_field "$log_file" max)"
  std_v="$(grep 'average rate:' "$log_file" | tail -1 | sed -E 's/.*std dev:[[:space:]]*([0-9.]+)s.*/\1/' || true)"

  if [[ -n "$rate" ]]; then
    status="OK"
  else
    status="NO_RATE"
  fi

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$topic" "$type" "$status" "$rate" "$min_v" "$max_v" "$std_v" "$log_file" "" \
    >> "$result_csv"
}

print_table() {
  python3 - "$result_csv" <<'PY'
import csv
import sys

path = sys.argv[1]
rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
rows.sort(key=lambda r: r["topic"])
print("")
print("topic,type,status,rate_hz")
for row in rows:
    print("{topic},{type},{status},{rate}".format(**row))

bad = [r for r in rows if r["status"] != "OK"]
print("")
print(f"summary total={len(rows)} ok={len(rows)-len(bad)} bad={len(bad)}")
if bad:
    for row in bad:
        print(f"bad topic={row['topic']} status={row['status']} type={row['type']} log={row['log']}")
PY
}

main() {
  : > "$summary_log"
  source_ros_env
  log "rate_check_start output_dir=${output_dir}"
  log "ros_master=${ROS_MASTER_URI:-unset}"
  log "measurement_host=local_ros_master topic_prefix=${topic_prefix}"

  if [[ "$start_bridge" -eq 1 ]]; then
    start_temp_bridge
  fi

  mapfile -t topics < <(discover_topics)
  if [[ "${#topics[@]}" -eq 0 ]]; then
    echo "No topics found. Check ROS_MASTER_URI, bridge setup, or --topics." >&2
    exit 1
  fi

  printf 'topic,type,status,rate,min_period,max_period,stddev,log,note\n' > "$result_csv"
  log "measuring ${#topics[@]} topic(s), sample_sec=${sample_sec}, window=${window_size}"

  local topic
  for topic in "${topics[@]}"; do
    log "measuring topic=${topic}"
    measure_topic "$topic"
  done

  print_table | tee -a "$summary_log"
  log "rate_check_done result_csv=${result_csv}"
}

main "$@"
