# SPRL Solver - Set-Pair Routing Problem with Layer

A Mixed Integer Programming (MIP) solver for the Set-Pair Routing Problem with Layer (SPRL).

## Overview

This solver implements optimization algorithms for the Set-Pair Routing Problem with Layer using Gurobi optimizer. The SPRL problem involves finding optimal routing paths in a grid-based environment with obstacles, where multiple source-sink pairs need to be connected using different layers.

## Features

- Mixed Integer Programming formulation for SPRL
- Support for multiple optimization methods (INC, BS)
- Visualization of solutions
- Benchmark problem support

## Requirements

- Python 3.7+
- gurobipy (Gurobi Optimizer)
- numpy
- networkx
- matplotlib

## Installation

1. Clone this repository:
```bash
git clone https://github.com/Yasuhiro-Takashima/spr-mip-solver.git
cd spr-mip-solver
```

2. Install required packages:
```bash
pip install -r requirements.txt
```

3. Ensure you have a valid Gurobi license installed.

## Usage

Basic usage:
```bash
python sprl.py benchmarks/S1.map
```

With options:
```bash
python sprl.py benchmarks/B1.map --method BS --time-limit 300
```

### Command-line Options

- `benchmark`: Path to benchmark file (required, positional argument)
- `--method {INC,BS}`: Optimization method (default: INC)
- `--time-limit SECONDS`: Time limit for optimization (default: 600)
- `--debug`: Enable debug mode with detailed logging
- `--output`: Enable graph visualization

## Benchmarks

The benchmark files in the `benchmarks/` directory were created by:

- **Yuta NAKATANI** (Tokyo Institute of Technology, now Institute of Science Tokyo)
- **Atsushi TAKAHASHI** (Tokyo Institute of Technology, now Institute of Science Tokyo)

These benchmarks were published in:

> Yuta NAKATANI and Atsushi TAKAHASHI, "A Length Matching Routing Algorithm for Set-Pair Routing Problem," 
> IEICE Transactions on Fundamentals of Electronics, Communications and Computer Sciences, 
> vol.E98-A, no.12, pp.2565-2574, December 2015.
> DOI: [10.1587/transfun.E98.A.2565](https://doi.org/10.1587/transfun.E98.A.2565)

The benchmark files are included in this repository with permission from the original authors. Please cite the original paper if you use these benchmarks in your research.

## Input Format

Map files use the following format:
- First line: `rows columns`
- Following lines: Grid representation where:
  - `*` = obstacle
  - `.` = empty cell
  - `s` = source
  - `t` = sink

Example:
```
12 12
* * * * * * * * * * * * 
* . s . . . . . . . . * 
* . . * * . . . . . . * 
...
```

## Citation

If you use this solver in your research, please cite:

```bibtex
@inproceedings{takashima2024sprl,
  author = {Takashima, Yasuhiro},
  title = {Set-Pair Routing Solver with Layer-by-Layer Formulation on ILP},
  booktitle = {2024 International VLSI Symposium on Technology, Systems and Applications (VLSI TSA)},
  year = {2024},
  doi = {10.1109/VLSITSA60681.2024.10546370}
}
```

For the benchmarks, please cite:

```bibtex
@article{nakatani2015length,
  author = {Nakatani, Yuta and Takahashi, Atsushi},
  title = {A Length Matching Routing Algorithm for Set-Pair Routing Problem},
  journal = {IEICE Transactions on Fundamentals of Electronics, Communications and Computer Sciences},
  volume = {E98-A},
  number = {12},
  pages = {2565--2574},
  year = {2015},
  doi = {10.1587/transfun.E98.A.2565}
}
```

## License

This solver is released under the MIT License. See LICENSE file for details.

Note: The benchmark files in the `benchmarks/` directory are provided for research purposes and are subject to the terms specified in the original publication.

## Author

Yasuhiro TAKASHIMA  
University of Kitakyushu  
Email: takasima@kitakyu-u.ac.jp

## Acknowledgments

We gratefully acknowledge Yuta NAKATANI and Atsushi TAKAHASHI for providing the benchmark instances used in this solver.

## Publication

This work was presented at:

> Yasuhiro TAKASHIMA, "Set-Pair Routing Solver with Layer-by-Layer Formulation on ILP,"
> 2024 International VLSI Symposium on Technology, Systems and Applications (VLSI TSA),
> 2024.
> DOI: [10.1109/VLSITSA60681.2024.10546370](https://doi.org/10.1109/VLSITSA60681.2024.10546370)
