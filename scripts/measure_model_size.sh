#!/usr/bin/env bash
# scripts/measure_model_size.sh
#
# Measure the initial ILP model size for each (benchmark, condition):
#   4 conditions x 9 benchmarks = 36 builds.
# Each build calls `sprl.py --measure-only`, which constructs the MIP
# model up to `model.update()` and reports `(NumVars, NumConstrs,
# NumNZs)` without invoking the solver. Even the largest baseline
# configuration (S3 with neither pre-optimization) finishes in under
# one second.
#
# Output: results/model_size.csv (Table III in the APCCAS 2026 paper).
#
# Defaults match the paper. Override via environment variables:
#
#   PYTHON='uv run python' bash scripts/measure_model_size.sh
#   BENCHES='B3 S3' bash scripts/measure_model_size.sh
#   OUT=results/model_size_test.csv bash scripts/measure_model_size.sh
#
#   # Cap Gurobi parallelism if the host is memory-constrained.
#   # 0 = Gurobi default (= all cores); a positive integer caps it.
#   THREADS=4 bash scripts/measure_model_size.sh

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

PYTHON="${PYTHON:-python}"
BENCH_DIR="${BENCH_DIR:-benchmarks}"
OUT="${OUT:-results/model_size.csv}"
THREADS="${THREADS:-0}"

if [[ -n "${BENCHES:-}" ]]; then
    read -ra BENCH_LIST <<< "${BENCHES}"
else
    BENCH_LIST=(E1 E2 B1 B2 B3 F1 S1 S2 S3)
fi

# (label | flags) pairs.  --method is irrelevant here (no solving),
# so we leave it at sprl.py's default.
CONDITIONS=(
    "baseline|--no-reach-prune --no-ml-num"
    "+ml-num|--no-reach-prune  --ml-num"
    "+reach|--reach-prune     --no-ml-num"
    "full|--reach-prune    --ml-num"
)

mkdir -p "$(dirname "${OUT}")"
rm -f "${OUT}"

cat <<EOF
===================================================================
Model-size probe (build only, no solve)
  Benchmarks (${#BENCH_LIST[@]}) : ${BENCH_LIST[*]}
  Conditions (${#CONDITIONS[@]}) : $(printf '%s ' "${CONDITIONS[@]%%|*}")
  Output         : ${OUT}
  Python         : ${PYTHON}
  Threads        : ${THREADS}
  Started        : $(date)
===================================================================
EOF

for bench in "${BENCH_LIST[@]}"; do
    bench_path="${BENCH_DIR}/${bench}.map"
    if [[ ! -f "${bench_path}" ]]; then
        echo "[SKIP] ${bench_path} not found"
        continue
    fi

    for entry in "${CONDITIONS[@]}"; do
        label="${entry%%|*}"
        flags="${entry#*|}"

        echo
        echo "----- ${bench} / ${label} -----"
        ${PYTHON} sprl.py "${bench_path}" \
            ${flags} \
            --measure-only \
            --gurobi-threads "${THREADS}" \
            --condition "${label}" \
            --results "${OUT}"
    done
done

cat <<EOF

===================================================================
Finished : $(date)
Results  : ${OUT}
===================================================================
EOF
