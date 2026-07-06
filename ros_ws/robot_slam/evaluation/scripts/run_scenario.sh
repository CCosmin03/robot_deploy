#!/bin/bash
# Ruleaza toate cele 6 configuratii x pana la 5 bag-uri pentru UN scenariu.
# Bag-urile sunt descoperite automat din directorul dat (sortate dupa nume = data).
# GT-ul din /odom e generat automat daca nu exista.
#
# Utilizare:
#   chmod +x run_scenario.sh run_experiment.sh
#   ./run_scenario.sh s2 /path/la/bags/scenariul2/
#
# Exemplu cu bag-urile cu nume generice de data:
#   ./run_scenario.sh s2 ~/recordings/loop/
#
# Structura rezultate generata:
#   evaluation/results/c1_s2_r1/  c2_s2_r1/ ... c6_s2_r5/

set -euo pipefail

SCENARIO="${1:?Lipsa scenariu: s1 s2 s3a s3b}"
BAGS_DIR="${2:?Lipsa director cu bag-uri}"
BAGS_DIR="$(realpath "$BAGS_DIR")"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVAL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
GT_DIR="$EVAL_DIR/ground_truth"
RESULTS_DIR="$EVAL_DIR/results"
mkdir -p "$GT_DIR" "$RESULTS_DIR"

# ── Harta GT geometrica pentru scenariile care au dimensiuni fizice ────────────
# Citeste dimensiunile din scenario_config.yaml si genereaza PGM (o singura data)
GT_MAP_PGM=""
GT_MAP_YAML=""

SCN_CFG="$EVAL_DIR/scenario_config.yaml"

if [ "$SCENARIO" = "s1" ] && [ -f "$SCN_CFG" ]; then
    GT_MAP_BASE="$GT_DIR/real_map_s1"
    GT_MAP_PGM="${GT_MAP_BASE}.pgm"
    GT_MAP_YAML="${GT_MAP_BASE}.yaml"
    if [ ! -f "$GT_MAP_PGM" ]; then
        echo "[GT] Generez harta geometrica S1..."
        REAL_L=$(python3 -c "import yaml; d=yaml.safe_load(open('$SCN_CFG')); print(d['s1']['reference_length_m'])")
        REAL_W=$(python3 -c "import yaml; d=yaml.safe_load(open('$SCN_CFG')); print(d['s1']['reference_width_m'])")
        python3 "$SCRIPT_DIR/generate_corridor_map.py" \
            --length "$REAL_L" --width "$REAL_W" \
            --output "$GT_MAP_BASE"
        echo "[GT] Harta GT salvata: $GT_MAP_PGM"
    else
        echo "[GT] Harta geometrica S1 exista deja: $GT_MAP_PGM"
    fi
fi

if [ "$SCENARIO" = "s2" ] && [ -f "$SCN_CFG" ]; then
    GT_MAP_BASE="$GT_DIR/real_map_s2_sofa"
    GT_MAP_PGM="${GT_MAP_BASE}.pgm"
    GT_MAP_YAML="${GT_MAP_BASE}.yaml"
    if [ ! -f "$GT_MAP_PGM" ]; then
        echo "[GT] Generez harta geometrica S2 (camera + canapea)..."
        python3 "$SCRIPT_DIR/generate_real_map_s2_sofa.py" \
            --config "$SCN_CFG" \
            --output "$GT_MAP_BASE"
        echo "[GT] Harta GT S2 salvata: $GT_MAP_PGM"
    else
        echo "[GT] Harta geometrica S2 exista deja: $GT_MAP_PGM"
    fi
fi

if { [ "$SCENARIO" = "s3a" ] || [ "$SCENARIO" = "s3b" ]; } && [ -f "$SCN_CFG" ]; then
    GT_MAP_BASE="$GT_DIR/real_map_s3_lab"
    GT_MAP_PGM="${GT_MAP_BASE}.pgm"
    GT_MAP_YAML="${GT_MAP_BASE}.yaml"
    if [ ! -f "$GT_MAP_PGM" ]; then
        echo "[GT] Generez harta geometrica S3 (lab)..."
        python3 "$SCRIPT_DIR/generate_real_map_s3_lab.py" \
            --config "$SCN_CFG" \
            --output "$GT_MAP_BASE"
        echo "[GT] Harta GT S3 salvata: $GT_MAP_PGM"
    else
        echo "[GT] Harta geometrica S3 exista deja: $GT_MAP_PGM"
    fi
fi

CONFIGS=(c1 c2 c3 c4 c5 c6)

# ── Descopera bag-urile ────────────────────────────────────────────────────────
mapfile -t BAG_FILES < <(find "$BAGS_DIR" -maxdepth 1 -name "*.bag" | sort)

if [ "${#BAG_FILES[@]}" -eq 0 ]; then
    echo "EROARE: niciun .bag gasit in $BAGS_DIR"
    exit 1
fi

N_RUNS="${#BAG_FILES[@]}"
[ "$N_RUNS" -gt 5 ] && N_RUNS=5   # maxim 5 rulari

echo "============================================================"
echo " Scenariu : $SCENARIO"
echo " Bags dir : $BAGS_DIR"
echo " Bag-uri  : $N_RUNS (din ${#BAG_FILES[@]} gasite)"
for i in $(seq 0 $((N_RUNS-1))); do
    echo "   run$((i+1)): $(basename "${BAG_FILES[$i]}")"
done
echo " Configs  : ${CONFIGS[*]}"
echo " Total    : $((N_RUNS * ${#CONFIGS[@]})) experimente"
echo "============================================================"
echo ""

# ── Genereaza GT per run din /odom ────────────────────────────────────────────
# Fiecare bag isi are propriul GT — permite ATE cu std non-zero intre rulari
echo "[GT] Generez ground truth per run..."
for run_idx in $(seq 0 $((N_RUNS-1))); do
    RUN_NUM=$((run_idx + 1))
    GT_FILE="$GT_DIR/${SCENARIO}_gt_r${RUN_NUM}.tum"
    if [ -f "$GT_FILE" ]; then
        echo "[GT] r$RUN_NUM exista deja: $GT_FILE"
    else
        python3 "$SCRIPT_DIR/gen_gt_odom.py" \
            --bag "${BAG_FILES[$run_idx]}" \
            --output "$GT_FILE"
        # Strip trailing whitespace — evo e strict la format
        sed -i 's/[[:space:]]*$//' "$GT_FILE"
        echo "[GT] r$RUN_NUM salvat: $GT_FILE"
    fi
done
echo ""

# ── Ruleaza experimentele ──────────────────────────────────────────────────────
TOTAL=$((N_RUNS * ${#CONFIGS[@]}))
DONE=0
FAIL=0
START_ALL=$(date +%s)

for run_idx in $(seq 0 $((N_RUNS-1))); do
    RUN_NUM=$((run_idx + 1))
    BAG="${BAG_FILES[$run_idx]}"
    GT_FILE="$GT_DIR/${SCENARIO}_gt_r${RUN_NUM}.tum"

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo " Rulare $RUN_NUM/$N_RUNS — $(basename "$BAG")"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    for CONFIG in "${CONFIGS[@]}"; do
        DONE_MARKER="$RESULTS_DIR/${CONFIG}_${SCENARIO}_r${RUN_NUM}/.done"

        if [ -f "$DONE_MARKER" ]; then
            echo "  [SKIP] $CONFIG r$RUN_NUM — deja facut"
            ((DONE++)) || true
            continue
        fi

        EXP_START=$(date +%s)
        echo ""
        echo "  ── $CONFIG | $SCENARIO | run=$RUN_NUM ──"

        if bash "$SCRIPT_DIR/run_experiment.sh" \
                "$CONFIG" "$SCENARIO" "$RUN_NUM" "$BAG" "$GT_FILE" \
                "$GT_MAP_PGM" "$GT_MAP_YAML"; then
            touch "$DONE_MARKER"
            ((DONE++)) || true
            EXP_S=$(( $(date +%s) - EXP_START ))
            echo "  [OK] ${CONFIG} r${RUN_NUM} (${EXP_S}s)"
        else
            ((FAIL++)) || true
            echo "  [FAIL] ${CONFIG} r${RUN_NUM}"
        fi

        # Estimare timp ramas
        ELAPSED=$(( $(date +%s) - START_ALL ))
        REMAINING=$(( TOTAL - DONE - FAIL ))
        if [ "$DONE" -gt 0 ] && [ "$REMAINING" -gt 0 ]; then
            AVG=$(( ELAPSED / DONE ))
            ETA=$(( REMAINING * AVG / 60 ))
            echo "  ETA: ~${ETA}min ramas ($REMAINING experimente)"
        fi

        # Pauza intre experimente pentru a lasa ROS sa se curete
        sleep 5
    done
done

# ── Sumar final ────────────────────────────────────────────────────────────────
TOTAL_S=$(( $(date +%s) - START_ALL ))
TOTAL_MIN=$(( TOTAL_S / 60 ))

echo ""
echo "============================================================"
echo " SCENARIU $SCENARIO FINALIZAT"
echo " OK: $DONE  FAIL: $FAIL  Timp: ${TOTAL_MIN}min"
echo "============================================================"
echo ""
echo "Genereaza raportul partial:"
echo "  python3 $SCRIPT_DIR/generate_report.py --results-dir $RESULTS_DIR"
