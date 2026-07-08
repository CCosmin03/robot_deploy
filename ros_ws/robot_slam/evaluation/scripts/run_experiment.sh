#!/bin/bash
# Ruleaza UN singur experiment si colecteaza toate metricile.
# Apelat de run_scenario.sh — nu rula direct.
#
# Utilizare:
#   run_experiment.sh <config> <scenario> <run_num> <bag_file> [gt_file] [gt_map_pgm] [gt_map_yaml]

set -euo pipefail

CONFIG="$1"
SCENARIO="$2"
RUN_NUM="$3"
BAG_FILE="$4"
GT_FILE="${5:-}"
GT_MAP_PGM="${6:-}"
GT_MAP_YAML="${7:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVAL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RESULT_DIR="$EVAL_DIR/results/${CONFIG}_${SCENARIO}_r${RUN_NUM}"
mkdir -p "$RESULT_DIR"

LOG="$RESULT_DIR/run.log"

declare -A LAUNCH
LAUNCH["c1"]="robot_gmapping.launch open_rviz:=false"
LAUNCH["c2"]="robot_gmapping_imu.launch open_rviz:=false load_robot_description:=false"
LAUNCH["c3"]="robot_hector.launch open_rviz:=false"
LAUNCH["c4"]="robot_cartographer_no_imu.launch open_rviz:=false"
LAUNCH["c5"]="robot_cartographer_ekf.launch open_rviz:=false"
LAUNCH["c6"]="robot_cartographer.launch open_rviz:=false"

declare -A SLAM_PROC
SLAM_PROC["c1"]="slam_gmapping"
SLAM_PROC["c2"]="slam_gmapping"
SLAM_PROC["c3"]="hector_mapping"
SLAM_PROC["c4"]="cartographer_nod"
SLAM_PROC["c5"]="cartographer_nod"
SLAM_PROC["c6"]="cartographer_nod"

# c2/c5 ruleaza EKF, care publica odom_ekf -> base_footprint. /tf din bag
# contine odom -> base_footprint (inregistrat cu vechiul odometry_node, care
# il publica necondiționat) — daca il redam, intra in conflict cu EKF pe
# acelasi child frame. Excludem /tf la replay pentru aceste doua configuratii
# si pastram doar /tf_static (transformarile statice ale robotului).
declare -A BAG_TOPICS
BAG_TOPICS["c1"]="/scan /odom /imu /tf /tf_static"
BAG_TOPICS["c2"]="/scan /odom /imu /tf_static"
BAG_TOPICS["c3"]="/scan /odom /imu /tf /tf_static"
BAG_TOPICS["c4"]="/scan /odom /imu /tf /tf_static"
BAG_TOPICS["c5"]="/scan /odom /imu /tf_static"
BAG_TOPICS["c6"]="/scan /odom /imu /tf /tf_static"

TUM_FILE="$RESULT_DIR/trajectory.tum"
STARTUP_FILE="$RESULT_DIR/startup.txt"
RESOURCE_FILE="$RESULT_DIR/resources.yaml"
MAP_BASE="$RESULT_DIR/map"
METRICS_FILE="$RESULT_DIR/metrics.yaml"

echo "[$CONFIG $SCENARIO r$RUN_NUM] START" | tee "$LOG"

# ── Cleanup ────────────────────────────────────────────────────────────────────
ROSCORE_PID="" SLAM_PID="" TUM_PID="" MON_PID="" HZ_PID=""

cleanup() {
    [ -n "$MON_PID" ] && kill "$MON_PID" 2>/dev/null || true
    [ -n "$TUM_PID" ] && kill "$TUM_PID" 2>/dev/null || true
    [ -n "$HZ_PID"  ] && kill "$HZ_PID"  2>/dev/null || true
    [ -n "$SLAM_PID" ] && kill "$SLAM_PID" 2>/dev/null || true
    sleep 3
    [ -n "$ROSCORE_PID" ] && kill "$ROSCORE_PID" 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT

# ── 1. roscore ─────────────────────────────────────────────────────────────────
roscore &> "$RESULT_DIR/roscore.log" &
ROSCORE_PID=$!
sleep 3
rosparam set /use_sim_time true

# ── 2. SLAM ────────────────────────────────────────────────────────────────────
# shellcheck disable=SC2086
roslaunch robot_slam ${LAUNCH[$CONFIG]} use_sim_time:=true \
    &> "$RESULT_DIR/slam.log" &
SLAM_PID=$!
echo "[$CONFIG $SCENARIO r$RUN_NUM] SLAM pid=$SLAM_PID" | tee -a "$LOG"
sleep 10

# ── 3. tf_to_tum_node ──────────────────────────────────────────────────────────
rosrun robot_slam tf_to_tum_node \
    _output_file:="$TUM_FILE" \
    _startup_file:="$STARTUP_FILE" \
    _rate_hz:=10.0 \
    &> "$RESULT_DIR/tum.log" &
TUM_PID=$!

# ── 4. Monitor CPU/RAM ─────────────────────────────────────────────────────────
python3 "$SCRIPT_DIR/monitor_resources.py" \
    --config "$CONFIG" \
    --output "$RESOURCE_FILE" \
    &> "$RESULT_DIR/monitor.log" &
MON_PID=$!

# ── 5. Frecventa /map ──────────────────────────────────────────────────────────
MAP_HZ_LOG="$RESULT_DIR/map_hz_raw.txt"
rostopic hz /map &> "$MAP_HZ_LOG" &
HZ_PID=$!

# ── 6. Play bag ────────────────────────────────────────────────────────────────
echo "[$CONFIG $SCENARIO r$RUN_NUM] play bag" | tee -a "$LOG"
# shellcheck disable=SC2086
rosbag play "$BAG_FILE" --clock --rate 1.0 --delay 3 --topics ${BAG_TOPICS[$CONFIG]} \
    &>> "$LOG"
sleep 5

# ── 7. Salveaza harta ──────────────────────────────────────────────────────────
rosrun map_server map_saver -f "$MAP_BASE" &>> "$LOG" || true
sleep 2

# ── 8. Opreste procese auxiliare ───────────────────────────────────────────────
kill "$MON_PID" "$TUM_PID" "$HZ_PID" 2>/dev/null || true
sleep 1

# Strip trailing whitespace din TUM — evo e strict la format
[ -f "$TUM_FILE" ] && sed -i 's/[[:space:]]*$//' "$TUM_FILE"

# ── 9. Extrage frecventa medie /map ────────────────────────────────────────────
MAP_HZ=$(grep -oP 'average rate: \K[0-9.]+' "$MAP_HZ_LOG" 2>/dev/null \
         | tail -5 \
         | awk '{ s+=$1; n++ } END { if(n) printf "%.3f", s/n; else print "-1" }')
MAP_HZ="${MAP_HZ:--1}"

# ── 10. Calculeaza metrici ─────────────────────────────────────────────────────
SCENARIO_CFG="$EVAL_DIR/scenario_config.yaml"
MAP_YAML="${MAP_BASE}.yaml"

python3 "$SCRIPT_DIR/compute_metrics.py" \
    --config   "$CONFIG" \
    --scenario "$SCENARIO" \
    --run      "$RUN_NUM" \
    --traj     "$TUM_FILE" \
    ${GT_FILE:+--gt "$GT_FILE"} \
    --map      "$MAP_YAML" \
    ${GT_MAP_PGM:+--gt-map "$GT_MAP_PGM"} \
    ${GT_MAP_YAML:+--gt-map-yaml "$GT_MAP_YAML"} \
    --scenario-cfg "$SCENARIO_CFG" \
    --startup-file "$STARTUP_FILE" \
    --map-hz   "$MAP_HZ" \
    --output   "$METRICS_FILE" \
    &>> "$LOG"

echo "[$CONFIG $SCENARIO r$RUN_NUM] DONE — $METRICS_FILE" | tee -a "$LOG"
