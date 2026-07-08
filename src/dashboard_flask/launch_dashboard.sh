#!/usr/bin/env bash
#
# launch_dashboard.sh — source the ROS2 environment and start the
# dashboard_flask node with sensible defaults, overridable via env vars
# or CLI flags.
#
# Usage:
#   ./launch_dashboard.sh
#   ./launch_dashboard.sh --image-topic /rdk_camera/image_raw --port 8080
#
# Env var overrides (used if the matching flag isn't passed):
#   ROS_SETUP        path to the base ROS2 setup.bash (default: /opt/ros/humble/setup.bash)
#   TROS_SETUP       path to TogetherROS setup.bash, sourced if it exists
#                     (default: /opt/tros/humble/setup.bash — RDK X5 layout)
#   WS_SETUP         path to this workspace's install/setup.bash
#                     (default: auto-detected relative to this script)
#   IMAGE_TOPIC, MAP_TOPIC, POSE_TOPIC, CMD_VEL_TOPIC, HOLONOMIC_TOPIC
#   JPEG_QUALITY, PORT
#
set -euo pipefail

# ---------- defaults ----------
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# assumes this script lives at the package root: <ws>/src/dashboard_flask/launch_dashboard.sh
WS_SETUP="${WS_SETUP:-$SCRIPT_DIR/../../install/setup.bash}"

IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image_raw}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
POSE_TOPIC="${POSE_TOPIC:-/amcl_pose}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
HOLONOMIC_TOPIC="${HOLONOMIC_TOPIC:-/holonomic_mode}"
JPEG_QUALITY="${JPEG_QUALITY:-70}"
PORT="${PORT:-5000}"

# ---------- CLI flags (override env/defaults above) ----------
usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

  --image-topic TOPIC       default: $IMAGE_TOPIC
  --map-topic TOPIC         default: $MAP_TOPIC
  --pose-topic TOPIC        default: $POSE_TOPIC
  --cmd-vel-topic TOPIC     default: $CMD_VEL_TOPIC
  --holonomic-topic TOPIC   default: $HOLONOMIC_TOPIC
  --jpeg-quality N          default: $JPEG_QUALITY
  --port N                  default: $PORT
  --ros-setup PATH          default: $ROS_SETUP
  --tros-setup PATH         default: $TROS_SETUP (skipped if file doesn't exist)
  --ws-setup PATH           default: $WS_SETUP
  -h, --help                show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image-topic) IMAGE_TOPIC="$2"; shift 2 ;;
    --map-topic) MAP_TOPIC="$2"; shift 2 ;;
    --pose-topic) POSE_TOPIC="$2"; shift 2 ;;
    --cmd-vel-topic) CMD_VEL_TOPIC="$2"; shift 2 ;;
    --holonomic-topic) HOLONOMIC_TOPIC="$2"; shift 2 ;;
    --jpeg-quality) JPEG_QUALITY="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --ros-setup) ROS_SETUP="$2"; shift 2 ;;
    --tros-setup) TROS_SETUP="$2"; shift 2 ;;
    --ws-setup) WS_SETUP="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

# ---------- source environments ----------
if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
else
  echo "WARNING: ROS2 setup not found at $ROS_SETUP — assuming ROS2 is already sourced in this shell." >&2
fi

if [[ -f "$TROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$TROS_SETUP"
  echo "Sourced TogetherROS: $TROS_SETUP"
fi

if [[ -f "$WS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$WS_SETUP"
else
  echo "ERROR: workspace setup not found at $WS_SETUP" >&2
  echo "Build it first, e.g.: colcon build --packages-select dashboard_flask --merge-install" >&2
  exit 1
fi

echo "-------------------------------------------------------------"
echo " dashboard_flask launch"
echo "   image_topic       = $IMAGE_TOPIC"
echo "   map_topic         = $MAP_TOPIC"
echo "   pose_topic        = $POSE_TOPIC"
echo "   cmd_vel_topic     = $CMD_VEL_TOPIC"
echo "   holonomic_topic   = $HOLONOMIC_TOPIC"
echo "   jpeg_quality      = $JPEG_QUALITY"
echo "   web port          = $PORT"
echo "-------------------------------------------------------------"
echo " Open: http://$(hostname -I 2>/dev/null | awk '{print $1}'):$PORT/"
echo "-------------------------------------------------------------"

exec ros2 run dashboard_flask flask_node --ros-args \
  -p image_topic:="$IMAGE_TOPIC" \
  -p map_topic:="$MAP_TOPIC" \
  -p pose_topic:="$POSE_TOPIC" \
  -p cmd_vel_topic:="$CMD_VEL_TOPIC" \
  -p holonomic_mode_topic:="$HOLONOMIC_TOPIC" \
  -p jpeg_quality:="$JPEG_QUALITY" \
  -p port:="$PORT"
