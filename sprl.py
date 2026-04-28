"""! @file sprl.py
@brief SPR Solver -- Set-Pair Routing Problem with a Layer-Indexed ILP
       Formulation.

This module implements an exact solver for the Set-Pair Routing Problem
(SPR) using a Mixed Integer Programming (MIP) formulation in which 0-1
variables are indexed jointly by edges and layers, so that disconnected
sub-loops are structurally impossible.

The solver supports two pre-optimization techniques (each individually
toggleable for the ablation study):

  * \b reach pruning -- breadth-first search from sinks gives a routing
    lower bound R\\ell(v) for every vertex; variables that cannot reach
    any sink within the current maximum length are removed at model
    creation.
  * \b ml-num -- a polynomial-time min-cost max-flow pre-solve provides a
    feasible (though not length-optimal) routing whose layer count is
    used as the initial \\textit{max-length}. Without this technique the
    solver falls back to the trivial bound \\textit{|V|-|S|-|T|}.

The outer search alternates two stages: a min-max stage (minimize the
maximum path length) and a max-min stage (maximize the minimum path
length under the previously found max). Each stage can use either an
\b INC (incremental) or \b BS (binary-search) probe schedule.

CLI usage examples:

  python sprl.py benchmarks/S1.map
  python sprl.py benchmarks/B1.map --time-limit 300 --method BS
  python sprl.py benchmarks/E1.map --no-reach-prune --no-ml-num
  python sprl.py benchmarks/S3.map --condition full-INC \\
      --results results/ablation.csv
  python sprl.py benchmarks/S3.map --measure-only \\
      --condition baseline --no-reach-prune --no-ml-num \\
      --results results/model_size.csv

@author Yasuhiro Takashima
@date   2026-04-29
"""

import argparse
import csv
import math
import os
import sys
import time
import traceback
from collections import defaultdict
from dataclasses import dataclass, field
from itertools import product
from typing import Optional

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
DEFAULT_TIME_LIMIT = 600       ##< Default time limit per ILP call [s]
DEFAULT_GUROBI_METHOD = 3      ##< Non-deterministic concurrent root LP
DEFAULT_GUROBI_THREADS = 0     ##< 0 = leave Gurobi default (= all cores)
METHOD_NAMES = ['INC', 'BS']   ##< Outer search strategies
NODE_OBSTACLE = '*'            ##< Obstacle marker in .map files
NODE_SOURCE = 's'              ##< Source-pin marker in .map files
NODE_SINK = 't'                ##< Sink-pin marker in .map files

# CSV columns produced by --results in the two operating modes.
ABLATION_HEADER = [
    'benchmark', 'condition',
    'reach_prune', 'ml_num', 'method',
    'n_nodes', 'n_edges', 'n_sources',
    'init_time', 'max_time', 'min_time', 'total_time',
    'max_length', 'min_length', 'status',
]
MEASURE_HEADER = [
    'benchmark', 'condition',
    'reach_prune', 'ml_num',
    'init_max_length', 'n_vars', 'n_cons', 'n_nzs',
]


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class CacheStats:
    """! @brief Cache statistics container.

    Tracks hit/miss statistics for caching operations.
    """
    max_hits: int = 0
    max_misses: int = 0
    min_hits: int = 0
    min_misses: int = 0
    total_hits: int = 0
    total_misses: int = 0


@dataclass
class ImprovedRoute:
    """! @brief Enhanced route management class with caching capabilities.

    Manages routing paths and provides cached length calculations for
    repeated queries.

    @param debug_mode Enable debug output (default: False).
    """
    debug_mode: bool = False
    path: dict[tuple[int, int], tuple[int, int]] = field(default_factory=dict)
    _max_length: Optional[int] = field(default=None, repr=False)
    _min_length: Optional[int] = field(default=None, repr=False)
    _total_length: Optional[int] = field(default=None, repr=False)
    cache_stats: CacheStats = field(default_factory=CacheStats)

    def bulk_add_edges(self, edges):
        """! Add multiple edges at once and invalidate the length cache."""
        self.path.update(edges)
        self._max_length = None
        self._min_length = None
        self._total_length = None
        if self.debug_mode:
            print(f"[DEBUG] Bulk added {len(edges)} edges, cache invalidated")

    def compute_all_lengths(self, source_set, sink_set):
        """! Recompute max, min, and total lengths in a single pass."""
        if self.debug_mode:
            print("[DEBUG] Computing all lengths at once...")

        max_length = 0
        min_length = float('inf')
        total_length = 0

        for source in source_set:
            length = 0
            current = source
            visited = set()

            while current not in sink_set:
                if current in visited:
                    if self.debug_mode:
                        print(f"[WARNING] Cycle detected from {source}")
                    break
                visited.add(current)
                current = self.path.get(current)
                if current is None:
                    if self.debug_mode:
                        print(f"[WARNING] Path broken from {source}")
                    break
                length += 1

            if length > max_length:
                max_length = length
            if length < min_length:
                min_length = length
            total_length += length

        self._max_length = max_length
        self._min_length = min_length if min_length != float('inf') else 0
        self._total_length = total_length

        if self.debug_mode:
            print(f"[DEBUG] Computed: max={max_length}, "
                  f"min={self._min_length}, total={total_length}")

    def get_max_length(self):
        if self._max_length is not None:
            self.cache_stats.max_hits += 1
            self.cache_stats.total_hits += 1
            return self._max_length
        self.cache_stats.max_misses += 1
        self.cache_stats.total_misses += 1
        raise RuntimeError("Length not computed! Call compute_all_lengths first.")

    def get_min_length(self):
        if self._min_length is not None:
            self.cache_stats.min_hits += 1
            self.cache_stats.total_hits += 1
            return self._min_length
        self.cache_stats.min_misses += 1
        self.cache_stats.total_misses += 1
        raise RuntimeError("Length not computed! Call compute_all_lengths first.")

    def get_total_length(self):
        if self._total_length is not None:
            return self._total_length
        raise RuntimeError("Length not computed! Call compute_all_lengths first.")

    def get_next(self, node):
        return self.path.get(node)

    def report_cache_stats(self):
        if not self.cache_stats.total_hits and not self.cache_stats.total_misses:
            return
        print("\n" + "=" * 80)
        print("Cache Statistics")
        print("=" * 80)
        print(f"Max Length - Hits: {self.cache_stats.max_hits}, "
              f"Misses: {self.cache_stats.max_misses}")
        print(f"Min Length - Hits: {self.cache_stats.min_hits}, "
              f"Misses: {self.cache_stats.min_misses}")
        print(f"Total      - Hits: {self.cache_stats.total_hits}, "
              f"Misses: {self.cache_stats.total_misses}")
        total_max = self.cache_stats.max_hits + self.cache_stats.max_misses
        total_min = self.cache_stats.min_hits + self.cache_stats.min_misses
        total_all = self.cache_stats.total_hits + self.cache_stats.total_misses
        if total_max > 0:
            print(f"Max Length Cache Hit Rate: "
                  f"{self.cache_stats.max_hits / total_max * 100:.1f}%")
        if total_min > 0:
            print(f"Min Length Cache Hit Rate: "
                  f"{self.cache_stats.min_hits / total_min * 100:.1f}%")
        if total_all > 0:
            print(f"Overall Cache Hit Rate: "
                  f"{self.cache_stats.total_hits / total_all * 100:.1f}%")
        print("=" * 80)


@dataclass(slots=True)
class GraphConfig:
    """! @brief Configuration container for the SPR solver.

    @param bench               Benchmark .map path (positional CLI argument).
    @param time_limit          Per-ILP-call wall-clock limit [s].
    @param method              Outer search method (0 = INC, 1 = BS).
    @param debug_mode          Verbose logging.
    @param output_graph        Pop up a matplotlib visualisation of the
                               input/output graphs (interactive use only).
    @param enable_reach_prune  Toggle creation-time reachability pruning.
    @param enable_ml_num       Toggle min-cost max-flow layer-count
                               pre-optimization.
    @param gurobi_method       Gurobi root-LP algorithm (-1 = solver
                               default, 3 = non-deterministic concurrent).
    @param gurobi_threads      Gurobi Threads parameter (0 = solver
                               default = all cores).
    @param results_file        If set, append a CSV row with the run's
                               summary to this file (header auto-written).
    @param condition_label     Label written to the CSV's condition
                               column (e.g., "baseline", "full-INC").
    @param measure_only        Build the ILP model, read its size, then
                               exit without solving. Used by the model-
                               size measurement script.
    """
    bench: str
    time_limit: int = DEFAULT_TIME_LIMIT
    method: int = 0
    debug_mode: bool = False
    output_graph: bool = False
    enable_reach_prune: bool = True
    enable_ml_num: bool = True
    gurobi_method: int = DEFAULT_GUROBI_METHOD
    gurobi_threads: int = DEFAULT_GUROBI_THREADS
    results_file: Optional[str] = None
    condition_label: str = 'default'
    measure_only: bool = False


# ---------------------------------------------------------------------------
# Graph construction
# ---------------------------------------------------------------------------
class GridGraphBuilder:
    """! @brief Constructs grid graphs from .map files.

    Handles loading the map, building a NetworkX directed graph, and
    cleaning up obstacles, isolated vertices, dead-ends, and edges that
    point into sources or out of sinks.
    """

    @staticmethod
    def load_map_file(filename):
        with open(filename) as f:
            first_line = f.readline()
            w_str, h_str = first_line.split()
            width = int(w_str)
            height = int(h_str)
        data = np.genfromtxt(filename, skip_header=1, encoding=None, dtype=None)
        return data, width, height

    def build_graph(self, data, width, height):
        graph = nx.grid_2d_graph(width, height, create_using=nx.DiGraph)
        obstacles = self._find_nodes_by_type(data, width, height, NODE_OBSTACLE)
        graph.remove_nodes_from(obstacles)
        isolated = [v for v in graph.nodes() if nx.degree(graph)[v] == 0]
        graph.remove_nodes_from(isolated)
        source_set = set(self._find_nodes_by_type(data, width, height, NODE_SOURCE))
        sink_set = set(self._find_nodes_by_type(data, width, height, NODE_SINK))
        self._remove_dead_ends(graph, source_set, sink_set)
        self._remove_invalid_edges(graph, source_set, sink_set)
        return graph, source_set, sink_set

    @staticmethod
    def _find_nodes_by_type(data, width, height, node_type):
        return [(x, y) for (y, x) in product(range(height), range(width))
                if data[y][x] == node_type]

    @staticmethod
    def _remove_dead_ends(graph, source_set, sink_set):
        special_nodes = source_set | sink_set
        dead_ends = [v for v in graph.nodes()
                     if nx.degree(graph)[v] == 1 and v not in special_nodes]
        while dead_ends:
            graph.remove_nodes_from(dead_ends)
            dead_ends = [v for v in graph.nodes()
                         if nx.degree(graph)[v] == 1 and v not in special_nodes]

    @staticmethod
    def _remove_invalid_edges(graph, source_set, sink_set):
        invalid_edges = []
        for source in source_set:
            for predecessor in graph.predecessors(source):
                invalid_edges.append((predecessor, source))
        for sink in sink_set:
            for successor in graph.successors(sink):
                invalid_edges.append((sink, successor))
        graph.remove_edges_from(invalid_edges)


# ---------------------------------------------------------------------------
# Optimizer
# ---------------------------------------------------------------------------
class ImprovedRouteOptimizer:
    """! @brief MIP-based optimizer for the Set-Pair Routing Problem.

    Implements the layer-indexed ILP formulation, the two pre-optimization
    techniques (reach pruning and ml-num layer estimation), and the
    iterative outer search (INC/BS).
    """

    def __init__(self, graph, source_set, sink_set, config):
        self.graph = graph
        self.source_set = source_set
        self.sink_set = sink_set
        self.config = config
        self.max_length = 0
        self.reachability_dist = {}
        self.current_route = ImprovedRoute(config.debug_mode)

    # ---- Initial bounds ---------------------------------------------------
    def estimate_initial_bounds(self):
        """! Estimate (max_length, total_length) from the underlying graph.

        When ``config.enable_ml_num`` is True, uses min-cost max-flow on
        the graph to obtain a feasible (though not length-optimal)
        routing whose statistics are returned. When False, returns the
        trivial upper bound ``|V|-|S|-|T|``.
        """
        if not self.config.enable_ml_num:
            return self._trivial_initial_bounds()

        if self.config.debug_mode:
            print("[DEBUG] Building flow graph...")

        flow_graph = self._build_flow_graph()

        if self.config.debug_mode:
            print(f"[DEBUG] Flow graph: {flow_graph.number_of_nodes()} nodes, "
                  f"{flow_graph.number_of_edges()} edges")
            print("[DEBUG] Computing max flow...")

        flow_dict = nx.max_flow_min_cost(flow_graph, 'S', 'T')

        # Extract route from the flow solution
        edges = []
        for v1_name, v2_name, data in flow_graph.edges(data=True):
            if flow_dict[v1_name][v2_name] == 1 and data['weight'] == 1:
                if v1_name.endswith('_o') and v2_name.endswith('_i'):
                    v1_str = v1_name[:-2]
                    v2_str = v2_name[:-2]
                    v1_parts = v1_str.split('_')
                    v2_parts = v2_str.split('_')
                    if len(v1_parts) == 2 and len(v2_parts) == 2:
                        v1 = (int(v1_parts[0]), int(v1_parts[1]))
                        v2 = (int(v2_parts[0]), int(v2_parts[1]))
                        edges.append((v1, v2))

        if self.config.debug_mode:
            print(f"[DEBUG] Extracted {len(edges)} edges from flow solution")

        self.current_route = ImprovedRoute(self.config.debug_mode)
        self.current_route.bulk_add_edges(edges)
        self.current_route.compute_all_lengths(self.source_set, self.sink_set)

        max_length = self.current_route.get_max_length()
        total_length = self.current_route.get_total_length()

        if self.config.debug_mode:
            print(f"[DEBUG] Initial bounds: max={max_length}, "
                  f"total={total_length}")

        return max_length, total_length

    def _trivial_initial_bounds(self):
        """! Trivial fallback when ml-num is disabled.

        Sets max_length to ``|V|-|S|-|T|`` (a loose but always-valid
        upper bound on a simple path's length in a graph where source
        and sink pins are excluded from the interior). Also installs a
        placeholder ``current_route`` so downstream code that calls
        ``get_min_length`` etc. does not fail before the first ILP solve.
        """
        n_nodes = self.graph.number_of_nodes()
        max_length = max(1, n_nodes - len(self.source_set) - len(self.sink_set))
        total_length = max_length * len(self.source_set)

        self.current_route = ImprovedRoute(self.config.debug_mode)
        self.current_route._max_length = max_length
        self.current_route._min_length = 0
        self.current_route._total_length = total_length

        if self.config.debug_mode:
            print(f"[DEBUG] ml_num disabled: trivial max_length={max_length}")

        return max_length, total_length

    def _build_flow_graph(self):
        """! Build a flow network with node-splitting for unit capacity."""
        flow_graph = nx.DiGraph()
        for vertex in self.graph:
            x, y = vertex
            v_name = f'{x}_{y}'
            if vertex in self.source_set:
                flow_graph.add_edge('S', f'{v_name}_o', capacity=1, weight=0)
            elif vertex in self.sink_set:
                flow_graph.add_edge(f'{v_name}_i', 'T', capacity=1, weight=0)
            else:
                flow_graph.add_edge(f'{v_name}_i', f'{v_name}_o',
                                    capacity=1, weight=0)
        for v1, v2 in self.graph.edges():
            x1, y1 = v1
            x2, y2 = v2
            flow_graph.add_edge(f'{x1}_{y1}_o', f'{x2}_{y2}_i', weight=1)
        return flow_graph

    # ---- Reachability ------------------------------------------------------
    def calculate_reachability(self, max_bound):
        """! BFS from every sink-pin gives R\\ell(v) for each vertex v.

        Returns ``(updated_max_bound, reachability_dict)``. The bound is
        widened to fit the maximum source-to-sink BFS distance, since no
        feasible routing can be shorter than that.
        """
        if self.config.debug_mode:
            print("[DEBUG] Calculating reachability from sinks...")

        reachability = {}
        for distance, node_layer in enumerate(
                nx.bfs_layers(self.graph.reverse(), self.sink_set)):
            for node in node_layer:
                reachability[node] = distance

        for source in self.source_set:
            if source not in reachability:
                print('No feasible solution')
                sys.exit()
            max_bound = max(max_bound, reachability[source])

        if self.config.debug_mode:
            print(f"[DEBUG] Reachability calculated: max_bound={max_bound}")
        return max_bound, reachability

    # ---- MIP construction --------------------------------------------------
    def build_mip_model(self, model_name):
        """! Build the layer-indexed ILP model.

        The Gurobi parameters most relevant to the published experiments
        (``Threads``, ``Method``) are configured here, taking the values
        from ``self.config``. Set ``gurobi_method=-1`` to leave Gurobi's
        default (which differs across releases).
        """
        if self.config.debug_mode:
            print(f"[DEBUG] Building MIP model: {model_name}")

        model = gp.Model(model_name)
        model.setParam('LogToConsole', 0)
        model.setParam('DisplayInterval', 30)
        model.setParam('NodefileStart', 1)
        model.setParam('LogFile', f'{model_name}.log')
        model.setParam('TimeLimit', self.config.time_limit)
        model.setParam('SolutionLimit', 1)
        if self.config.gurobi_method != -1:
            model.setParam('Method', self.config.gurobi_method)
        if self.config.gurobi_threads > 0:
            model.setParam('Threads', self.config.gurobi_threads)

        variables = self._create_variables(model)
        model.update()

        if self.config.debug_mode:
            var_count = sum(len(layer_vars) for layer_vars in variables)
            print(f"[DEBUG] Created {var_count} variable groups")
            print("[DEBUG] Adding constraints...")

        self._add_constraints(model, variables)
        model.update()

        if self.config.debug_mode:
            print(f"[DEBUG] Model built: {model.NumVars} variables, "
                  f"{model.NumConstrs} constraints")
        return model, variables

    def _create_variables(self, model):
        """! Create the layer-indexed binary variables.

        When ``config.enable_reach_prune`` is True, variables that
        cannot reach any sink within the current ``max_length`` are
        omitted at creation. When False, all edge-layer combinations
        reachable from sources by BFS expansion are created.
        """
        variables = [{} for _ in range(self.max_length + 1)]
        current_vertices = self.source_set

        for layer in range(self.max_length + 1):
            next_vertices = set()
            for v1 in current_vertices:
                if v1 in self.sink_set:
                    variables[layer][v1] = {
                        't': model.addVar(
                            vtype=GRB.BINARY,
                            name=f'x_{layer}_{v1[0]}_{v1[1]}_t',
                        )
                    }
                    continue

                variables[layer][v1] = {}
                for v2 in self.graph[v1]:
                    if (self.config.enable_reach_prune and
                            self.reachability_dist[v2] + layer
                            >= self.max_length):
                        continue
                    variables[layer][v1][v2] = model.addVar(
                        vtype=GRB.BINARY, obj=1,
                        name=f'x_{layer}_{v1[0]}_{v1[1]}_{v2[0]}_{v2[1]}',
                    )
                    next_vertices.add(v2)
            current_vertices = next_vertices
        return variables

    def _add_constraints(self, model, variables):
        """! Source, sink, ordinary-pin, and flow-conservation constraints."""
        for source in self.source_set:
            model.addConstr(
                gp.quicksum(variables[0][source][v2]
                            for v2 in variables[0][source].keys()) == 1,
                f'source_{source[0]}_{source[1]}',
            )

        for v1 in self.graph:
            if v1 in self.source_set:
                continue
            if v1 in self.sink_set:
                model.addConstr(
                    gp.quicksum(variables[layer][v1]['t']
                                for layer in range(1, self.max_length + 1)
                                if v1 in variables[layer].keys()) == 1,
                    f'sink_{v1[0]}_{v1[1]}',
                )
            else:
                sos_set = []
                for layer in range(1, self.max_length):
                    if v1 in variables[layer].keys():
                        sos_set.extend(variables[layer][v1][v2]
                                       for v2 in variables[layer][v1].keys())
                if sos_set:
                    model.addConstr(
                        gp.quicksum(sos_set) <= 1,
                        f'out_degree_{v1[0]}_{v1[1]}',
                    )

            for layer in range(1, self.max_length + 1):
                if v1 in variables[layer].keys():
                    outflow = gp.quicksum(
                        variables[layer][v1][v2]
                        for v2 in variables[layer][v1].keys()
                    )
                    inflow = gp.quicksum(
                        variables[layer - 1][v2][v1]
                        for v2 in variables[layer - 1].keys()
                        if v1 in variables[layer - 1][v2].keys()
                    )
                    model.addConstr(
                        outflow - inflow == 0,
                        f'flow_{layer}_{v1[0]}_{v1[1]}',
                    )

    # ---- Solution extraction ----------------------------------------------
    def extract_route_from_solution(self, variables):
        edges = []
        for layer in range(self.max_length):
            for v1, v2_dict in variables[layer].items():
                for v2, var in v2_dict.items():
                    if var.X > 0.8:
                        edges.append((v1, v2))
        if self.config.debug_mode:
            print(f"[DEBUG] Extracted {len(edges)} edges from solution")
        route = ImprovedRoute(self.config.debug_mode)
        route.bulk_add_edges(edges)
        route.compute_all_lengths(self.source_set, self.sink_set)
        return route

    # ---- Outer search: max stage ------------------------------------------
    def optimize_max_path(self, initial_bound):
        if self.config.debug_mode:
            print("\n[DEBUG] === Starting Maximum Path Optimization ===")

        max_bound, self.reachability_dist = self.calculate_reachability(
            initial_bound)
        model, variables = self.build_mip_model('sprl_max')
        model.ModelSense = GRB.MINIMIZE

        if self.config.method == 0:
            result = self._optimize_max_incremental(model, variables)
        else:
            result = self._optimize_max_binary_search(
                model, variables, max_bound)

        if self.config.debug_mode:
            print("[DEBUG] === Maximum Path Optimization Complete ===\n")
        return result

    def _optimize_max_incremental(self, model, variables):
        current_length = self.max_length
        print(f'Current Length = {current_length}', flush=True)

        iteration = 0
        while True:
            iteration += 1
            if self.config.debug_mode:
                print(f"[DEBUG] Max optimization iteration {iteration}")

            self._set_upper_bounds(variables, current_length - 1)
            model.optimize()

            if model.Status == GRB.TIME_LIMIT:
                print('Time out', flush=True)

            if model.SolCount > 0:
                self.current_route = self.extract_route_from_solution(variables)
                current_length = self.current_route.get_max_length()
                print(f'Current Length = {current_length}', flush=True)
            else:
                break
        return current_length

    def _optimize_max_binary_search(self, model, variables, max_bound):
        left = max_bound
        self._set_upper_bounds(variables, max_bound)
        model.optimize()

        if model.Status == GRB.TIME_LIMIT:
            print('Time out', flush=True)

        if model.SolCount > 0:
            self.current_route = self.extract_route_from_solution(variables)
            right = max_bound
            print(f'{max_bound} OK', flush=True)
        else:
            right = self.max_length
            print(f'{max_bound} NG', flush=True)

        iteration = 0
        while right - left > 1:
            iteration += 1
            if self.config.debug_mode:
                print(f"[DEBUG] Binary search iteration {iteration}: "
                      f"[{left}, {right}]")
            mid = (left + right) // 2
            self._set_upper_bounds(variables, mid)
            model.optimize()
            if model.Status == GRB.TIME_LIMIT:
                print('Time out', flush=True)
            if model.SolCount > 0:
                self.current_route = self.extract_route_from_solution(variables)
                right = self.current_route.get_max_length()
                print(f'{right} OK', flush=True)
            else:
                left = mid
                print(f'{mid} NG', flush=True)
        return right

    def _set_upper_bounds(self, variables, threshold):
        """! Force routes to fit within ``threshold`` by zeroing UBs.

        This uses the BFS reachability bound and applies regardless of
        whether ``enable_reach_prune`` was set: it is the mechanism by
        which the outer search progressively tightens the maximum length,
        not the creation-time pruning.
        """
        for layer in range(self.max_length + 1):
            for v1, v2_dict in variables[layer].items():
                if self.reachability_dist[v1] + layer > threshold:
                    for var in v2_dict.values():
                        var.UB = 0
                else:
                    for var in v2_dict.values():
                        var.UB = 1

    # ---- Outer search: min stage ------------------------------------------
    def optimize_min_path(self):
        if self.config.debug_mode:
            print("\n[DEBUG] === Starting Minimum Path Optimization ===")

        model, variables = self.build_mip_model('sprl_min')
        model.ModelSense = GRB.MAXIMIZE

        min_length = self.current_route.get_min_length()
        print(f'Initial Min Length = {min_length}', flush=True)

        if self.config.method == 0:
            result = self._optimize_min_incremental(
                model, variables, min_length)
        else:
            result = self._optimize_min_binary_search(
                model, variables, min_length)

        if self.config.debug_mode:
            print("[DEBUG] === Minimum Path Optimization Complete ===\n")
        return result

    def _optimize_min_incremental(self, model, variables, min_length):
        iteration = 0
        while True:
            iteration += 1
            if self.config.debug_mode:
                print(f"[DEBUG] Min optimization iteration {iteration}")
            self._set_lower_bounds(variables, min_length + 1)
            model.optimize()
            if model.Status == GRB.TIME_LIMIT:
                print('Time out', flush=True)
            if model.SolCount > 0:
                self.current_route = self.extract_route_from_solution(variables)
                min_length = self.current_route.get_min_length()
                print(f'Current Length = {min_length}', flush=True)
            else:
                break
        return min_length

    def _optimize_min_binary_search(self, model, variables, min_length):
        right = self.max_length
        self._set_lower_bounds(variables, self.max_length)
        model.optimize()
        if model.Status == GRB.TIME_LIMIT:
            print('Time out', flush=True)
        if model.SolCount > 0:
            self.current_route = self.extract_route_from_solution(variables)
            min_length = self.current_route.get_min_length()
            print(f'Current Length = {min_length}', flush=True)
        left = min_length
        iteration = 0
        while right - left > 1:
            iteration += 1
            if self.config.debug_mode:
                print(f"[DEBUG] Binary search iteration {iteration}: "
                      f"[{left}, {right}]")
            mid = (left + right) // 2
            self._set_lower_bounds(variables, mid)
            model.optimize()
            if model.Status == GRB.TIME_LIMIT:
                print('Time out', flush=True)
            if model.SolCount > 0:
                self.current_route = self.extract_route_from_solution(variables)
                left = self.current_route.get_min_length()
                print(f'{left} OK', flush=True)
            else:
                right = mid
                print(f'{mid} NG', flush=True)
        return left

    def _set_lower_bounds(self, variables, threshold):
        """! Forbid early sink arrival by zeroing the relevant terminal vars."""
        for sink in self.sink_set:
            for layer in range(threshold):
                if sink in variables[layer] and 't' in variables[layer][sink]:
                    var = variables[layer][sink]['t']
                    var.LB = 0
                    var.UB = 0
            for layer in range(threshold, self.max_length + 1):
                if sink in variables[layer] and 't' in variables[layer][sink]:
                    var = variables[layer][sink]['t']
                    var.LB = 0
                    var.UB = 1


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------
def _benchmark_name(bench_path):
    """! Strip directory and extension: 'benchmarks/S3.map' -> 'S3'."""
    return os.path.splitext(os.path.basename(bench_path))[0]


def _append_csv_row(path, header, row):
    """! Append a row to ``path``; write ``header`` first if file is new."""
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    file_exists = os.path.exists(path) and os.path.getsize(path) > 0
    with open(path, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(header)
        writer.writerow(row)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Set-Pair Routing solver with a layer-indexed ILP '
                    'formulation.',
        add_help=True,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default full solver
  python sprl.py benchmarks/S1.map

  # With time limit and binary-search outer loop
  python sprl.py benchmarks/B1.map --time-limit 300 --method BS

  # Ablation: disable reach pruning
  python sprl.py benchmarks/E1.map --no-reach-prune

  # Ablation: disable both pre-optimizations (= "baseline")
  python sprl.py benchmarks/E1.map --no-reach-prune --no-ml-num

  # Append a CSV row labelled "full-INC" to results/ablation.csv
  python sprl.py benchmarks/S3.map --condition full-INC \\
      --results results/ablation.csv

  # Build the model only and report (vars, cons, nzs)
  python sprl.py benchmarks/S3.map --measure-only \\
      --condition baseline --no-reach-prune --no-ml-num \\
      --results results/model_size.csv
        """
    )
    parser.add_argument('benchmark',
                        help='Path to a .map benchmark file '
                             '(e.g., benchmarks/S1.map).')
    parser.add_argument('-t', '--time-limit', type=int,
                        default=DEFAULT_TIME_LIMIT, dest='time_limit',
                        help='Per-ILP-call wall-clock limit in seconds '
                             f'(default: {DEFAULT_TIME_LIMIT}).')
    parser.add_argument('-m', '--method', type=str, default='INC',
                        choices=['INC', 'BS'],
                        help='Outer search method (default: INC).')
    parser.add_argument('-d', '--debug', action='store_true', dest='debug',
                        help='Verbose logging.')
    parser.add_argument('-o', '--output', action='store_true',
                        help='Show input/output graphs in matplotlib.')

    parser.add_argument('--reach-prune',
                        action=argparse.BooleanOptionalAction, default=True,
                        dest='enable_reach_prune',
                        help='Toggle creation-time reachability-based '
                             'variable pruning (default: enabled).')
    parser.add_argument('--ml-num',
                        action=argparse.BooleanOptionalAction, default=True,
                        dest='enable_ml_num',
                        help='Toggle min-cost max-flow-based layer-count '
                             'pre-optimization (default: enabled).')
    parser.add_argument('--gurobi-method', type=int,
                        default=DEFAULT_GUROBI_METHOD,
                        help='Gurobi root-LP algorithm '
                             '(-1=Gurobi default, 0=primal simplex, '
                             '1=dual simplex, 2=barrier, '
                             '3=non-deterministic concurrent, '
                             '4=deterministic concurrent, '
                             '5=concurrent barrier; '
                             f'default: {DEFAULT_GUROBI_METHOD}).')
    parser.add_argument('--gurobi-threads', type=int,
                        default=DEFAULT_GUROBI_THREADS,
                        help='Gurobi Threads parameter '
                             '(0 = solver default = all cores; '
                             f'default: {DEFAULT_GUROBI_THREADS}).')

    parser.add_argument('--results', dest='results_file', default=None,
                        help='If set, append a CSV summary row to this '
                             'file. Header is written automatically when '
                             'the file is new.')
    parser.add_argument('--condition', dest='condition_label',
                        default='default',
                        help='Label for the condition column of the '
                             'results CSV (e.g., "baseline", "full-INC").')
    parser.add_argument('--measure-only', action='store_true',
                        help='Build the ILP model and report '
                             '(vars, cons, nonzeros) without solving. '
                             'Used for the model-size ablation.')

    args = parser.parse_args()

    method_index = METHOD_NAMES.index(args.method) \
        if args.method in METHOD_NAMES else 0

    return GraphConfig(
        bench=args.benchmark,
        time_limit=args.time_limit,
        method=method_index,
        debug_mode=args.debug,
        output_graph=args.output,
        enable_reach_prune=args.enable_reach_prune,
        enable_ml_num=args.enable_ml_num,
        gurobi_method=args.gurobi_method,
        gurobi_threads=args.gurobi_threads,
        results_file=args.results_file,
        condition_label=args.condition_label,
        measure_only=args.measure_only,
    )


# ---------------------------------------------------------------------------
# Top-level execution modes
# ---------------------------------------------------------------------------
def _print_banner(config):
    print("=" * 80)
    print("SPRL - Set-Pair Routing Solver (Layer-Indexed ILP)")
    print("=" * 80)
    print(f"Benchmark    : {config.bench}")
    print(f"Method       : {METHOD_NAMES[config.method]}")
    print(f"Time limit   : {config.time_limit}s")
    print(f"reach_prune  : {config.enable_reach_prune}")
    print(f"ml_num       : {config.enable_ml_num}")
    print(f"Gurobi Method: {config.gurobi_method}")
    print(f"Gurobi Threads: {config.gurobi_threads}")
    if config.condition_label != 'default':
        print(f"Condition    : {config.condition_label}")
    if config.results_file:
        print(f"Results CSV  : {config.results_file}")
    if config.measure_only:
        print("Mode         : MEASURE-ONLY (no optimize)")
    if config.debug_mode:
        print("Debug Mode   : ENABLED")
    print("=" * 80)
    print()


def run_measure_mode(config):
    """! Build the ILP model, log its size, optionally write a CSV row.

    Used for the model-size ablation. ``optimize()`` is never called, so
    even the largest baseline configurations finish in well under a
    second.
    """
    builder = GridGraphBuilder()
    data, width, height = builder.load_map_file(config.bench)
    graph, source_set, sink_set = builder.build_graph(data, width, height)

    print(f"Graph loaded: {len(graph.nodes())} nodes, "
          f"{len(graph.edges())} edges")
    print(f"Sources: {len(source_set)}, Sinks: {len(sink_set)}")
    print()

    optimizer = ImprovedRouteOptimizer(graph, source_set, sink_set, config)

    # Mirror the head of optimize_max_path so build_mip_model sees a
    # fully initialised state.
    initial_max_length, _total = optimizer.estimate_initial_bounds()
    optimizer.max_length = initial_max_length

    max_length, optimizer.reachability_dist = optimizer.calculate_reachability(
        initial_max_length)
    optimizer.max_length = max_length

    model, _vars = optimizer.build_mip_model('measure')
    n_vars = model.NumVars
    n_cons = model.NumConstrs
    n_nzs = model.NumNZs

    print(f"  init_max_length={max_length}  vars={n_vars}  "
          f"cons={n_cons}  nzs={n_nzs}")

    if config.results_file:
        _append_csv_row(
            config.results_file, MEASURE_HEADER,
            [_benchmark_name(config.bench),
             config.condition_label,
             config.enable_reach_prune,
             config.enable_ml_num,
             max_length, n_vars, n_cons, n_nzs],
        )
        print(f"  -> appended row to {config.results_file}")


def run_optimize_mode(config):
    """! Full optimization: estimate bounds, run min-max and max-min stages."""
    builder = GridGraphBuilder()
    data, width, height = builder.load_map_file(config.bench)
    graph, source_set, sink_set = builder.build_graph(data, width, height)

    if config.output_graph:
        pos = {v: (v[0], height - v[1]) for v in graph.nodes()}
        node_colors = ['red' if v in source_set
                       else 'blue' if v in sink_set
                       else 'green' for v in graph.nodes()]
        nx.draw(graph, pos, node_color=node_colors)
        plt.show()

    n_nodes = len(graph.nodes())
    n_edges = len(graph.edges())
    n_sources = len(source_set)

    print(f"Graph loaded: {n_nodes} nodes, {n_edges} edges")
    print(f"Sources: {n_sources}, Sinks: {len(sink_set)}")
    print()

    optimizer = ImprovedRouteOptimizer(graph, source_set, sink_set, config)

    init_time = max_time = min_time = total_time = 0.0
    max_length = -1
    min_length = -1
    status = 'ok'
    t0 = time.time()

    try:
        max_length_init, total_length = optimizer.estimate_initial_bounds()
        t1 = time.time()
        init_time = t1 - t0
        print(f'Initialization time: {init_time:.6f}s\n')

        optimizer.max_length = max_length_init
        initial_bound = math.ceil(total_length / max(1, len(sink_set)))
        max_length = optimizer.optimize_max_path(initial_bound)
        t2 = time.time()
        max_time = t2 - t1
        print(f'Maximum Length = {max_length}')
        print(f'Search Max time: {max_time:.6f}s\n')

        optimizer.max_length = max_length
        min_length = optimizer.optimize_min_path()
        t3 = time.time()
        min_time = t3 - t2
        print(f'Minimum Length = {min_length}')
        print(f'Search Min time: {min_time:.6f}s')

        total_time = t3 - t0
        print(f'Total runtime  : {total_time:.6f}s\n')
    except SystemExit as e:
        # sprl raises SystemExit('No feasible solution') on bad inputs.
        status = f'sysexit: {e}'
        total_time = time.time() - t0
        print(f'[ERROR] SystemExit: {e}', file=sys.stderr)
    except Exception as e:
        status = f'error: {type(e).__name__}: {e}'
        total_time = time.time() - t0
        traceback.print_exc()

    if config.output_graph and status == 'ok':
        path_dict = optimizer.current_route.path
        edge_colors = ['red' if path_dict.get(e[0]) == e[1] else 'black'
                       for e in graph.edges()]
        edge_width = [3 if path_dict.get(e[0]) == e[1] else 1
                      for e in graph.edges()]
        nx.draw(graph, pos, node_color=node_colors,
                edge_color=edge_colors, width=edge_width)
        plt.show()

    if config.debug_mode and status == 'ok':
        optimizer.current_route.report_cache_stats()

    if config.results_file:
        _append_csv_row(
            config.results_file, ABLATION_HEADER,
            [_benchmark_name(config.bench),
             config.condition_label,
             config.enable_reach_prune,
             config.enable_ml_num,
             METHOD_NAMES[config.method],
             n_nodes, n_edges, n_sources,
             f'{init_time:.3f}',
             f'{max_time:.3f}',
             f'{min_time:.3f}',
             f'{total_time:.3f}',
             max_length, min_length, status],
        )
        print(f'-> appended row to {config.results_file}')


def main():
    config = parse_arguments()
    _print_banner(config)
    if config.measure_only:
        run_measure_mode(config)
    else:
        run_optimize_mode(config)


if __name__ == '__main__':
    main()
