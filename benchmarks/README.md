# Benchmark Instances for Set-Pair Routing Problem

## Credits

These benchmark instances were created by:

- **Yuta NAKATANI**  
  Tokyo Institute of Technology (now Institute of Science Tokyo)

- **Atsushi TAKAHASHI**  
  Tokyo Institute of Technology (now Institute of Science Tokyo)

## Publication

These benchmarks were originally published in:

> Yuta NAKATANI and Atsushi TAKAHASHI,  
> "A Length Matching Routing Algorithm for Set-Pair Routing Problem,"  
> *IEICE Transactions on Fundamentals of Electronics, Communications and Computer Sciences*,  
> vol.E98-A, no.12, pp.2565-2574, December 2015.  
> DOI: [10.1587/transfun.E98.A.2565](https://doi.org/10.1587/transfun.E98.A.2565)

## Usage Terms

These benchmark files are included with permission from the original authors and are provided for research and evaluation purposes. If you use these benchmarks in your research, please cite the original paper.

## BibTeX Citation

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

## Benchmark Files

This directory contains the following benchmark instances:
- Small Instances: **B1.map, E1.map, E2.map, R1.map, S1.map**
- Middle Instaces: **B2.map, F1.map, S2.map**
- Large Instaces: **B3.map, S3.map**

## File Format

Each `.map` file contains:
- First line: Grid dimensions (rows columns)
- Following lines: Grid representation
  - `*` = obstacle
  - `.` = empty cell
  - `s` = source node
  - `t` = sink node

## Contact

For questions about these benchmark instances, please refer to the original publication or contact the authors.
