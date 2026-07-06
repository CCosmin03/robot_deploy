#!/usr/bin/env bash
# Recalculeaza metricile pentru toate run-urile S2 existente,
# FARA a re-rula SLAM. Citeste map_hz din map_hz_raw.txt existent.
# CPU/RAM raman neatinse (sunt in resources.yaml separat).
#
# Rulare:
#   chmod +x scripts/recalc_metrics_s2.sh
#   ./scripts/recalc_metrics_s2.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVAL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RESULTS_DIR="$EVAL_DIR/results"
GT_DIR="$EVAL_DIR/ground_truth"
SCENARIO_CFG="$EVAL_DIR/scenario_config.yaml"

GT_MAP_PGM="$GT_DIR/real_map_s2_sofa.pgm"
GT_MAP_YAML="$GT_DIR/real_map_s2_sofa.yaml"

# Regenereaza GT cu noile modificari (dulapuri ingrosat etc.)
echo "[recalc] Regenerez harta GT S2..."
python3 "$SCRIPT_DIR/generate_real_map_s2_sofa.py" \
    --config "$SCENARIO_CFG" \
    --output "$GT_DIR/real_map_s2_sofa"

DONE=0
SKIP=0

for RESULT_DIR in "$RESULTS_DIR"/c*_s2_r*; do
    [ -d "$RESULT_DIR" ] || continue

    DIRNAME="$(basename "$RESULT_DIR")"
    # Extrage config, run din directorul c1_s2_r1
    CONFIG="${DIRNAME%%_s2_*}"
    RUN_NUM="${DIRNAME##*_r}"

    MAP_YAML="$RESULT_DIR/map.yaml"
    MAP_PGM="$RESULT_DIR/map.pgm"
    TUM_FILE="$RESULT_DIR/trajectory.tum"
    STARTUP_FILE="$RESULT_DIR/startup.txt"
    METRICS_FILE="$RESULT_DIR/metrics.yaml"
    MAP_HZ_LOG="$RESULT_DIR/map_hz_raw.txt"
    GT_FILE="$GT_DIR/s2_gt_r${RUN_NUM}.tum"

    # Verifica ca harta exista
    if [ ! -f "$MAP_YAML" ] || [ ! -f "$MAP_PGM" ]; then
        echo "[SKIP] $DIRNAME — map.yaml / map.pgm lipsa"
        ((SKIP++)) || true
        continue
    fi

    # Extrage map_hz din log-ul original
    MAP_HZ="-1"
    if [ -f "$MAP_HZ_LOG" ]; then
        MAP_HZ=$(grep -oP 'average rate: \K[0-9.]+' "$MAP_HZ_LOG" 2>/dev/null \
                 | tail -5 \
                 | awk '{ s+=$1; n++ } END { if(n) printf "%.3f", s/n; else print "-1" }')
        MAP_HZ="${MAP_HZ:--1}"
    fi

    echo "[recalc] $DIRNAME  map_hz=$MAP_HZ"

    python3 "$SCRIPT_DIR/compute_metrics.py" \
        --config        "$CONFIG" \
        --scenario      "s2" \
        --run           "$RUN_NUM" \
        --traj          "$TUM_FILE" \
        ${GT_FILE:+--gt "$GT_FILE"} \
        --map           "$MAP_YAML" \
        --gt-map        "$GT_MAP_PGM" \
        --gt-map-yaml   "$GT_MAP_YAML" \
        --scenario-cfg  "$SCENARIO_CFG" \
        ${STARTUP_FILE:+--startup-file "$STARTUP_FILE"} \
        --map-hz        "$MAP_HZ" \
        --output        "$METRICS_FILE"

    ((DONE++)) || true
done

echo ""
echo "[recalc] Gata: $DONE recalculate, $SKIP sarite."
