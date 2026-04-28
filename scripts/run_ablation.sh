#!/usr/bin/env bash
# scripts/run_ablation.sh
#
# Run the APCCAS 2026 ablation matrix:
#   5 conditions x 9 benchmarks = 45 runs.
# Each run appends one CSV row to results/ablation.csv (header is
# written automatically by sprl.py when the file is new).
#
# Defaults match the paper. Override via environment variables, e.g.:
#
#   # Use a different Python launcher
#   PYTHON='uv run python' bash scripts/run_ablation.sh
#
#   # Run only a subset of benchmarks
#   BENCHES='S1 S2 S3' bash scripts/run_ablation.sh
#
#   # Tighter per-call time limit
#   TIME_LIMIT=120 bash scripts/run_ablation.sh
#
#   # Custom output path
#   OUT=results/ablation_2026-04-30.csv bash scripts/run_ablation.sh
#
# For long runs (S3 baseline can take 20+ minutes), prefer:
#   nohup bash scripts/run_ablation.sh > ablation.log 2>&1 &
#
# Note: do NOT use `set -e`; one failing run must not stop the whole
# matrix. sprl.py records its own status to the CSV (status='ok' or
# 'error: ...') so the resulting file always has 45 rows.

set -uo pipefail

# Move to repo root regardless of where this script is invoked from.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

PYTHON="${PYTHON:-python}"
BENCH_DIR="${BENCH_DIR:-benchmarks}"
OUT="${OUT:-results/ablation.csv}"
TIME_LIMIT="${TIME_LIMIT:-600}"

# Benchmark list: env override, or the full APCCAS 2026 set.
if [[ -n "${BENCHES:-}" ]]; then
    read -ra BENCH_LIST <<< "${BENCHES}"
else
    BENCH_LIST=(E1 E2 B1 B2 B3 F1 S1 S2 S3)
fi

# (label | flags) pairs corresponding to Table II columns of the paper.
# `|` separates the label from the CLI flag string.
CONDITIONS=(
    "baseline|--no-reach-prune --no-ml-num --method INC"
    "+ml-num|--no-reach-prune  --ml-num    --method INC"
    "+reach|--reach-prune     --no-ml-num --method INC"
    "full-INC|--reach-prune    --ml-num    --method INC"
    "full-BS|--reach-prune    --ml-num    --method BS"
)

# Ensure output directory exists; remove any stale CSV so the header
# is freshly written by sprl.py on the first run.
mkdir -p "$(dirname "${OUT}")"
rm -f "${OUT}"

cat <<EOF
===================================================================
APCCAS 2026 ablation matrix
  Benchmarks (${#BENCH_LIST[@]}) : ${BENCH_LIST[*]}
  Conditions (${#CONDITIONS[@]}) : $(printf '%s ' "${CONDITIONS[@]%%|*}")
  Output          : ${OUT}
  Time limit/call : ${TIME_LIMIT}s
  Python          : ${PYTHON}
  Started         : $(date)
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
        # ${flags} must be unquoted so word-splitting separates
        # individual CLI tokens.
        ${PYTHON} sprl.py "${bench_path}" \
            ${flags} \
            --time-limit "${TIME_LIMIT}" \
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
