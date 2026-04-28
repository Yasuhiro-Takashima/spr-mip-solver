# SPRL Solver — Set-Pair Routing Problem with a Layer-Indexed ILP Formulation

A Mixed Integer Programming (MIP) solver for the Set-Pair Routing
Problem (SPR) using a layer-indexed 0–1 ILP formulation that
structurally avoids disconnected sub-loops without auxiliary
constraints.

## Overview

This solver implements an exact algorithm for SPR using Gurobi. SPR
asks for `m` vertex-disjoint paths connecting a source-pin set and a
sink-pin set on a (possibly irregular) grid graph, minimizing the
maximum route length and, subject to that, maximizing the minimum
route length.

Two pre-optimization techniques can be toggled independently for
ablation studies:

- **`reach`** — breadth-first search from sinks gives a routing lower
  bound `Rl(v)` for every vertex; variables that cannot reach any
  sink within the current `max-length` are removed at model creation.
- **`ml-num`** — a polynomial-time min-cost max-flow pre-solve provides
  a feasible (though not length-optimal) routing whose layer count is
  used as the initial `max-length`. Without this technique the solver
  falls back to the trivial bound `|V| − |S| − |T|`.

The outer search alternates a min–max stage (minimize the maximum
length) and a max–min stage (maximize the minimum length under the
previously found maximum). Each stage can use either an **INC**
(incremental) or **BS** (binary-search) probe schedule.

## Features

- Layer-indexed ILP formulation that prevents sub-loops by construction
- Two toggleable pre-optimization techniques (`reach`, `ml-num`) for
  ablation experiments
- Two outer-search strategies: incremental (`INC`) and binary search
  (`BS`)
- Configurable Gurobi parameters (`Method`, `Threads`)
- Built-in CSV output mode for batch experiments and model-size probes
- Reproducible benchmark suite

## Requirements

- Python 3.10 or later (uses `dataclass(slots=True)` and
  `argparse.BooleanOptionalAction`)
- gurobipy (Gurobi 11+ recommended; results in the APCCAS 2026 paper
  used Gurobi 13.0.1)
- numpy
- networkx
- matplotlib

## Installation

```bash
git clone https://github.com/Yasuhiro-Takashima/spr-mip-solver.git
cd spr-mip-solver
pip install -r requirements.txt
```

A valid Gurobi license is required.

## Usage

### Basic

```bash
python sprl.py benchmarks/S1.map
```

### With options

```bash
# Binary-search outer loop, 5-minute per-call cap
python sprl.py benchmarks/B1.map --method BS --time-limit 300

# Disable reach pruning only (i.e., the "+ml-num" condition)
python sprl.py benchmarks/E1.map --no-reach-prune

# Disable both pre-optimizations (i.e., the "baseline" condition)
python sprl.py benchmarks/E1.map --no-reach-prune --no-ml-num
```

### Command-line options

| Option | Default | Description |
|---|---|---|
| `benchmark` | (positional) | Path to a `.map` benchmark file. |
| `-t`, `--time-limit SECS` | 600 | Per-ILP-call wall-clock limit. |
| `-m`, `--method {INC,BS}` | INC | Outer-search strategy. |
| `--reach-prune` / `--no-reach-prune` | enabled | Toggle creation-time reachability pruning. |
| `--ml-num` / `--no-ml-num` | enabled | Toggle min-cost max-flow layer-count pre-optimization. |
| `--gurobi-method N` | 3 | Root-LP algorithm. `-1` keeps Gurobi's default; `3` is non-deterministic concurrent. |
| `--gurobi-threads N` | 0 | Gurobi `Threads` parameter. `0` lets Gurobi pick (= all cores). |
| `--results FILE` | — | Append a CSV summary row to `FILE`. Header is written automatically when the file is new. |
| `--condition LABEL` | `default` | Label written to the `condition` column of the results CSV (e.g., `baseline`, `full-INC`). |
| `--measure-only` | off | Build the ILP model and report `(vars, cons, nonzeros)` without solving. Used by the model-size ablation. |
| `-d`, `--debug` | off | Verbose logging. |
| `-o`, `--output` | off | Pop up matplotlib visualisations. |

## Ablation study (APCCAS 2026)

The five conditions reported in the APCCAS 2026 paper (Table II) are
all single invocations of `sprl.py` with different flag combinations:

```bash
# baseline (no pre-optimization)
python sprl.py benchmarks/E1.map --no-reach-prune --no-ml-num \
    --method INC --condition baseline --results results/ablation.csv

# +ml-num only
python sprl.py benchmarks/E1.map --no-reach-prune \
    --method INC --condition +ml-num --results results/ablation.csv

# +reach only
python sprl.py benchmarks/E1.map --no-ml-num \
    --method INC --condition +reach  --results results/ablation.csv

# full-INC (default)
python sprl.py benchmarks/E1.map \
    --method INC --condition full-INC --results results/ablation.csv

# full-BS
python sprl.py benchmarks/E1.map --method BS \
    --condition full-BS --results results/ablation.csv
```

To run the entire ablation matrix (5 conditions × 9 benchmarks),
use the provided driver script:

```bash
bash scripts/run_ablation.sh
```

The script writes one CSV row per `(benchmark, condition)` pair to
`results/ablation.csv`, including timing breakdown and final lengths.

### Model-size probe (Table III)

The `--measure-only` mode builds the ILP model and reports its size
without calling the solver. This is used for the model-size table in
the APCCAS 2026 paper:

```bash
python sprl.py benchmarks/S3.map --measure-only \
    --no-reach-prune --no-ml-num \
    --condition base --results results/model_size.csv

python sprl.py benchmarks/S3.map --measure-only \
    --condition full --results results/model_size.csv
```

Or run all `(benchmark, condition)` combinations:

```bash
bash scripts/measure_model_size.sh
```

## Versions and tags

| Tag | Paper | Code state |
|---|---|---|
| `v1.0-cad2025` | IEICE Trans. Fundamentals, vol. E109-A, no. 3, pp. 590–595, Mar. 2026 (DOI: 10.1587/transfun.2025VLP0001) | Original release. Pre-optimizations always enabled; no CLI ablation flags. |
| `v1.1-apccas2026` | APCCAS 2026 | Consolidated CLI with ablation flags (`--reach-prune`, `--ml-num`), Gurobi parameter exposure, CSV-friendly `--results` output, and `--measure-only` mode for the model-size probe. |

To reproduce a specific paper's experiments, check out the
corresponding tag:

```bash
git checkout v1.0-cad2025      # journal version
git checkout v1.1-apccas2026   # APCCAS version (with ablation)
```

## Benchmarks

The benchmark files in the `benchmarks/` directory were created by:

- **Yuta NAKATANI** (Tokyo Institute of Technology, now Institute of Science Tokyo)
- **Atsushi TAKAHASHI** (Tokyo Institute of Technology, now Institute of Science Tokyo)

These benchmarks were published in:

> Yuta NAKATANI and Atsushi TAKAHASHI, "A Length Matching Routing Algorithm for Set-Pair Routing Problem,"
> IEICE Transactions on Fundamentals of Electronics, Communications and Computer Sciences,
> vol. E98-A, no. 12, pp. 2565–2574, December 2015.
> DOI: [10.1587/transfun.E98.A.2565](https://doi.org/10.1587/transfun.E98.A.2565)

The benchmark files are included in this repository with permission
from the original authors. Please cite the original paper if you use
these benchmarks in your research.

## Input format

Map files use the following format:
- First line: `width height`
- Following lines: a `height × width` grid where each cell is one of:
  - `*` — obstacle
  - `.` — empty cell
  - `s` — source pin
  - `t` — sink pin

Example:
```
12 12
* * * * * * * * * * * *
* . s . . . . . . . . *
* . . * * . . . . . . *
...
```

## Citation

If you use this solver in your research, please cite the relevant
paper(s):

```bibtex
@inproceedings{takashima2024sprl,
  author    = {Takashima, Yasuhiro},
  title     = {Set-Pair Routing Solver with Layer-by-Layer Formulation on ILP},
  booktitle = {Proc.\ Int.\ VLSI Symp.\ Technology, Systems and Applications (VLSI-TSA)},
  address   = {HsinChu, Taiwan},
  year      = {2024},
  doi       = {10.1109/VLSITSA60681.2024.10546370}
}

@article{takashima2026sprl,
  author  = {Takashima, Yasuhiro},
  title   = {Complexity and Exact Solution of Set-Pair Routing Problem},
  journal = {IEICE Transactions on Fundamentals of Electronics, Communications and Computer Sciences},
  volume  = {E109-A},
  number  = {3},
  pages   = {590--595},
  month   = {March},
  year    = {2026},
  doi     = {10.1587/transfun.2025VLP0001}
}

@inproceedings{takashima2026apccas,
  author    = {Takashima, Yasuhiro},
  title     = {Layer-Indexed {ILP} Formulation for the Set-Pair Routing Problem},
  booktitle = {Proc.\ IEEE Asia Pacific Conf.\ Circuits and Systems (APCCAS)},
  year      = {2026}
}
```

For the benchmarks, please cite:

```bibtex
@article{nakatani2015length,
  author  = {Nakatani, Yuta and Takahashi, Atsushi},
  title   = {A Length Matching Routing Algorithm for Set-Pair Routing Problem},
  journal = {IEICE Transactions on Fundamentals of Electronics, Communications and Computer Sciences},
  volume  = {E98-A},
  number  = {12},
  pages   = {2565--2574},
  year    = {2015},
  doi     = {10.1587/transfun.E98.A.2565}
}
```

## License

This solver is released under the MIT License. See `LICENSE` for
details.

The benchmark files in the `benchmarks/` directory are provided for
research purposes and are subject to the terms specified in the
original publication.

## Author

Yasuhiro TAKASHIMA
University of Kitakyushu
Email: takasima@kitakyu-u.ac.jp

## Acknowledgments

We gratefully acknowledge the original creators and contributors of
the benchmark instances used in this solver: Yuta NAKATANI and
Atsushi TAKAHASHI. We also thank Prof. Sato of Shinshu University and
Prof. Shimoda of the Institute of Science Tokyo for sharing benchmark
data and for fruitful discussions.

## Publications

This work has been presented and published in the following venues:

> Yasuhiro TAKASHIMA, "Set-Pair Routing Solver with Layer-by-Layer Formulation on ILP,"
> 2024 International VLSI Symposium on Technology, Systems and Applications (VLSI-TSA),
> HsinChu, Taiwan, April 2024.
> DOI: [10.1109/VLSITSA60681.2024.10546370](https://doi.org/10.1109/VLSITSA60681.2024.10546370)

> Yasuhiro TAKASHIMA, "Complexity and Exact Solution of Set-Pair Routing Problem,"
> IEICE Transactions on Fundamentals of Electronics, Communications and Computer Sciences,
> vol. E109-A, no. 3, pp. 590–595, March 2026.
> DOI: [10.1587/transfun.2025VLP0001](https://doi.org/10.1587/transfun.2025VLP0001)

> Yasuhiro TAKASHIMA, "Layer-Indexed ILP Formulation for the Set-Pair Routing Problem,"
> 2026 IEEE Asia Pacific Conference on Circuits and Systems (APCCAS), 2026.
