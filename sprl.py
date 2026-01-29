"""! @file sprl.py
@brief SPRL - Set-Pair Routing Problem with Layer

This module implements a solver for the Set-Pair Routing Problem with Layer (SPRL)
using Mixed Integer Programming (MIP) optimization.

@author Yasuhiro Takashima
@date 2026-01-29
"""

import sys
import time
import math
import argparse
from itertools import product
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB


# Constants
DEFAULT_TIME_LIMIT = 600  ##< Default time limit for optimization in seconds
METHOD_NAMES = ['INC', 'BS']  ##< Available optimization method names
NODE_OBSTACLE = '*'  ##< Character representing obstacle nodes in map files
NODE_SOURCE = 's'  ##< Character representing source nodes in map files
NODE_SINK = 't'  ##< Character representing sink nodes in map files


@dataclass
class CacheStats:
    """
    @brief Cache statistics container

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
    """
    @brief Enhanced route management class with caching capabilities

    This class manages routing paths and provides optimized length calculations
    through caching mechanisms to improve performance.

    @param debug_mode Enable debug output (default: False)
    """
    debug_mode: bool = False
    path: dict[tuple[int, int], tuple[int, int]] = field(default_factory=dict)
    _max_length: Optional[int] = field(default=None, repr=False)
    _min_length: Optional[int] = field(default=None, repr=False)
    _total_length: Optional[int] = field(default=None, repr=False)
    cache_stats: CacheStats = field(default_factory=CacheStats)
    
    def bulk_add_edges(self, edges):
        """
        @brief Add multiple edges at once and invalidate cache
        
        @param edges List of edge tuples (source_node, destination_node)
        """
        self.path.update(edges)
        # Invalidate cache
        self._max_length = None
        self._min_length = None
        self._total_length = None
        if self.debug_mode:
            print(f"[DEBUG] Bulk added {len(edges)} edges, cache invalidated")
    
    def compute_all_lengths(self, source_set, sink_set):
        """
        @brief Compute all path lengths at once and cache results for efficiency
        
        This method calculates maximum, minimum, and total path lengths in a single
        pass for improved performance.
        
        @param source_set Set of source nodes
        @param sink_set Set of sink nodes
        """
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
                        print(f"[WARNING] Cycle detected in path from {source}")
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
            print(f"[DEBUG] Computed: max={max_length}, min={self._min_length}, total={total_length}")
        
        
    def get_max_length(self):
        """
        @brief Get the cached maximum path length
        
        @return Maximum path length
        @throws RuntimeError If lengths have not been computed
        """
        if self._max_length is not None:
            self.cache_stats.max_hits += 1
            self.cache_stats.total_hits += 1
            if self.debug_mode:
                print(f"[DEBUG] Cache HIT for max_length: {self._max_length}")
            return self._max_length

        self.cache_stats.max_misses += 1
        self.cache_stats.total_misses += 1
        if self.debug_mode:
            print("[DEBUG] Cache MISS for max_length")
        raise RuntimeError("Length not computed! Call compute_all_lengths first.")
    
    def get_min_length(self):
        """
        @brief Get the cached minimum path length
        
        @return Minimum path length
        @throws RuntimeError If lengths have not been computed
        """
        if self._min_length is not None:
            self.cache_stats.min_hits += 1
            self.cache_stats.total_hits += 1
            if self.debug_mode:
                print(f"[DEBUG] Cache HIT for min_length: {self._min_length}")
            return self._min_length

        self.cache_stats.min_misses += 1
        self.cache_stats.total_misses += 1
        if self.debug_mode:
            print("[DEBUG] Cache MISS for min_length")
        raise RuntimeError("Length not computed! Call compute_all_lengths first.")
    
    def get_total_length(self):
        """
        @brief Get the cached total path length
        
        @return Total path length across all routes
        @throws RuntimeError If lengths have not been computed
        """
        if self._total_length is not None:
            if self.debug_mode:
                print(f"[DEBUG] Cache HIT for total_length: {self._total_length}")
            return self._total_length
        
        if self.debug_mode:
            print("[DEBUG] Cache MISS for total_length")
        raise RuntimeError("Length not computed! Call compute_all_lengths first.")
    
    def get_next(self, node):
        """
        @brief Get the next node in the route
        
        @param node Current node coordinates
        @return Next node coordinates, or None if not found
        """
        return self.path.get(node)
        
    def report_cache_stats(self):
        """
        @brief Report cache statistics

        Outputs hit/miss statistics for maximum, minimum, and total length caches,
        including cache hit rates.
        """
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
            hit_rate_max = self.cache_stats.max_hits / total_max * 100
            print(f"Max Length Cache Hit Rate: {hit_rate_max:.1f}%")

        if total_min > 0:
            hit_rate_min = self.cache_stats.min_hits / total_min * 100
            print(f"Min Length Cache Hit Rate: {hit_rate_min:.1f}%")

        if total_all > 0:
            hit_rate_all = self.cache_stats.total_hits / total_all * 100
            print(f"Overall Cache Hit Rate: {hit_rate_all:.1f}%")

        print("=" * 80)


@dataclass(slots=True)
class GraphConfig:
    """
    @brief Configuration container for graph optimization parameters

    Stores all configuration parameters including benchmark settings,
    time limits, optimization methods, and debug options.

    @param bench Benchmark name
    @param time_limit Time limit in seconds (default: DEFAULT_TIME_LIMIT)
    @param method Optimization method: 0=INC, 1=BS (default: 0)
    @param debug_mode Enable debug mode (default: False)
    @param output_graph Display graph (default: False)
    """
    bench: str
    time_limit: int = DEFAULT_TIME_LIMIT
    method: int = 0
    debug_mode: bool = False
    output_graph: bool = False


class GridGraphBuilder:
    """
    @brief Builder class for constructing grid graphs from map files
    
    This class handles loading map files and constructing NetworkX graph
    representations with sources, sinks, and obstacles.
    """
    
    @staticmethod
    def load_map_file(filename):
        """
        @brief Load a map file and parse grid data
        
        @param filename Path to the map file
        @return Tuple of (grid_data, width, height)
        @throws FileNotFoundError If the file does not exist
        @throws ValueError If the file format is invalid
        """
        with open(filename) as f:
            first_line = f.readline()
            w_str, h_str = first_line.split()
            width = int(w_str)
            height = int(h_str)
        data = np.genfromtxt(filename, skip_header=1, encoding=None, dtype=None)
        return data, width, height
    
    def build_graph(self, data, width, height):
        """
        @brief Build a directed graph from grid data
        
        Creates a NetworkX directed graph where nodes represent grid cells
        and edges represent valid movements between adjacent cells.
        
        @param data 2D grid of cell characters
        @param width Grid width
        @param height Grid height
        @return Tuple of (graph, source_set, sink_set)
        """
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
        """
        @brief Find all nodes of a specific type in the grid
        
        @param data 2D grid data
        @param width Grid width
        @param height Grid height
        @param node_type Type of node to find (NODE_OBSTACLE, NODE_SOURCE, NODE_SINK)
        @return List of node coordinates
        """
        return [(x, y) for (y, x) in product(range(height), range(width)) 
                if data[y][x] == node_type]
    
    @staticmethod
    def _remove_dead_ends(graph, source_set, sink_set):
        """
        @brief Remove dead-end nodes that are not sources or sinks
        
        @param graph Graph to modify
        @param source_set Set of source nodes
        @param sink_set Set of sink nodes
        """
        special_nodes = source_set | sink_set
        dead_ends = [v for v in graph.nodes() 
                     if nx.degree(graph)[v] == 1 and v not in special_nodes]
        while dead_ends:
            graph.remove_nodes_from(dead_ends)
            dead_ends = [v for v in graph.nodes() 
                        if nx.degree(graph)[v] == 1 and v not in special_nodes]
    
    @staticmethod
    def _remove_invalid_edges(graph, source_set, sink_set):
        """
        @brief Remove edges coming into sources and going out of sinks
        
        @param graph Graph to modify
        @param source_set Set of source nodes
        @param sink_set Set of sink nodes
        """
        invalid_edges = []
        for source in source_set:
            for predecessor in graph.predecessors(source):
                invalid_edges.append((predecessor, source))
        for sink in sink_set:
            for successor in graph.successors(sink):
                invalid_edges.append((sink, successor))
        graph.remove_edges_from(invalid_edges)


class ImprovedRouteOptimizer:
    """
    @brief Main optimizer class for set-pair routing problems
    
    This class implements MIP-based optimization algorithms for finding
    optimal routes that minimize maximum and minimum path lengths.
    """
    
    def __init__(self, graph, source_set, sink_set, config):
        """
        @brief Initialize the route optimizer
        
        @param graph NetworkX directed graph representing the grid
        @param source_set Set of source node coordinates
        @param sink_set Set of sink node coordinates
        @param config Configuration parameters
        """
        self.graph = graph
        self.source_set = source_set
        self.sink_set = sink_set
        self.config = config
        self.max_length = 0
        self.reachability_dist = {}
        self.current_route = ImprovedRoute(config.debug_mode)
        
    def estimate_initial_bounds(self):
        """
        @brief Estimate initial bounds for maximum and total path lengths
        
        Uses max-flow min-cost algorithm to compute conservative bounds.
        
        @return Tuple of (max_length, total_length)
        """
        if self.config.debug_mode:
            print("[DEBUG] Building flow graph...")
        
        flow_graph = self._build_flow_graph()
        
        if self.config.debug_mode:
            print(f"[DEBUG] Flow graph: {flow_graph.number_of_nodes()} nodes, "
                  f"{flow_graph.number_of_edges()} edges")
            print("[DEBUG] Computing max flow...")
        
        flow_dict = nx.max_flow_min_cost(flow_graph, 'S', 'T')
        
        # Extract route from flow solution
        edges = []
        for v1_name, v2_name, data in flow_graph.edges(data=True):
            if flow_dict[v1_name][v2_name] == 1 and data['weight'] == 1:
                # Parse node names back to coordinates
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
            print(f"[DEBUG] Initial bounds: max={max_length}, total={total_length}")
        
        return max_length, total_length
    
    def _build_flow_graph(self):
        """
        @brief Build flow network for initial solution estimation
        
        Creates a flow network with node splitting (in/out nodes) to enforce
        unit capacity constraints on nodes.
        
        @return Flow network as directed graph
        """
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
    
    def calculate_reachability(self, max_bound):
        """
        @brief Calculate reachability distances from sinks
        
        Uses BFS to compute the shortest distance from each node to any sink.
        
        @param max_bound Current maximum bound estimate
        @return Tuple of (updated_max_bound, reachability_dict)
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
    
    def build_mip_model(self, model_name):
        """
        @brief Build MIP model for routing optimization
        
        Creates a Gurobi MIP model with layered variables representing
        possible node positions at each time step.
        
        @param model_name Name for the Gurobi model
        @return Tuple of (Gurobi model, variable dictionary)
        """
        if self.config.debug_mode:
            print(f"[DEBUG] Building MIP model: {model_name}")
        
        model = gp.Model(model_name)
        
        # Configure Gurobi parameters
        model.setParam('LogToConsole', 0)
        model.setParam('DisplayInterval', 30)
        model.setParam('NodefileStart', 1)
        model.setParam('LogFile', f'{model_name}.log')
        model.setParam('TimeLimit', self.config.time_limit)
        model.setParam('SolutionLimit', 1)
        # model.setParam('Heuristics', 0.05)
        # model.setParam('Presolve', 2)
        # model.setParam('Cuts', 1)
        # model.setParam('PDHGGPU', 1)
        # model.setParam('Method', 6)                
        
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
        """
        @brief Create decision variables for the MIP model
        
        Creates binary variables for each possible edge at each layer,
        using reachability information to prune infeasible variables.
        
        @param model Gurobi model
        @return List of variable dictionaries, one per layer
        """
        variables = [{} for _ in range(self.max_length + 1)]
        current_vertices = self.source_set
        
        for layer in range(self.max_length + 1):
            next_vertices = set()
            
            for v1 in current_vertices:
                if v1 in self.sink_set:
                    variables[layer][v1] = {
                        't': model.addVar(vtype=GRB.BINARY, 
                                        name=f'x_{layer}_{v1[0]}_{v1[1]}_t')
                    }
                    continue
                
                variables[layer][v1] = {}
                for v2 in self.graph[v1]:
                    # Prune variables that cannot reach any sink in time
                    if self.reachability_dist[v2] + layer >= self.max_length:
                        continue
                    
                    variables[layer][v1][v2] = model.addVar(
                        vtype=GRB.BINARY,
                        obj=1,
                        name=f'x_{layer}_{v1[0]}_{v1[1]}_{v2[0]}_{v2[1]}'
                    )
                    next_vertices.add(v2)
            
            current_vertices = next_vertices
        
        return variables
    
    def _add_constraints(self, model, variables):
        """
        @brief Add constraints to the MIP model
        
        Adds source constraints (one outgoing edge per source),
        sink constraints (one incoming edge per sink),
        and flow conservation constraints (in = out at intermediate nodes).
        
        @param model Gurobi model
        @param variables Variable dictionary
        """
        # Source constraints: each source must have exactly one outgoing edge
        for source in self.source_set:
            model.addConstr(
                gp.quicksum(variables[0][source][v2] 
                          for v2 in variables[0][source].keys()) == 1,
                f'source_{source[0]}_{source[1]}'
            )
        
        # Node constraints
        for v1 in self.graph:
            if v1 in self.source_set:
                continue
            
            if v1 in self.sink_set:
                # Sink constraint: exactly one incoming edge over all layers
                model.addConstr(
                    gp.quicksum(variables[layer][v1]['t'] 
                              for layer in range(1, self.max_length + 1) 
                              if v1 in variables[layer].keys()) == 1,
                    f'sink_{v1[0]}_{v1[1]}'
                )
            else:
                # At most one outgoing edge per node across all layers
                sos_set = []
                for layer in range(1, self.max_length):
                    if v1 in variables[layer].keys():
                        sos_set.extend(variables[layer][v1][v2] 
                                     for v2 in variables[layer][v1].keys())
                
                if sos_set:
                    model.addConstr(
                        gp.quicksum(sos_set) <= 1,
                        f'out_degree_{v1[0]}_{v1[1]}'
                    )
            
            # Flow conservation: inflow = outflow at each layer
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
                        f'flow_{layer}_{v1[0]}_{v1[1]}'
                    )
        
    
    def extract_route_from_solution(self, variables):
        """
        @brief Extract routing solution from solved model variables
        
        @param variables Variable dictionary from the solved model
        @return ImprovedRoute object containing the solution paths
        """
        # Collect all edges where variable = 1
        edges = []
        for layer in range(self.max_length):
            for v1, v2_dict in variables[layer].items():
                for v2, var in v2_dict.items():
                    if var.X > 0.8:  # Tolerance for binary variable
                        edges.append((v1, v2))
        
        if self.config.debug_mode:
            print(f"[DEBUG] Extracted {len(edges)} edges from solution")
        
        # Create route and compute all lengths immediately
        route = ImprovedRoute(self.config.debug_mode)
        route.bulk_add_edges(edges)
        route.compute_all_lengths(self.source_set, self.sink_set)
        
        return route
    
    def optimize_max_path(self, initial_bound):
        """
        @brief Optimize to minimize the maximum path length
        
        @param initial_bound Initial upper bound for the maximum length
        @return Optimized maximum path length
        """
        if self.config.debug_mode:
            print("\n[DEBUG] === Starting Maximum Path Optimization ===")
        
        max_bound, self.reachability_dist = self.calculate_reachability(initial_bound)
        model, variables = self.build_mip_model('sprl_max')
        model.ModelSense = GRB.MINIMIZE
        
        if self.config.method == 0:
            result = self._optimize_max_incremental(model, variables)
        else:
            result = self._optimize_max_binary_search(model, variables, max_bound)
        
        if self.config.debug_mode:
            print("[DEBUG] === Maximum Path Optimization Complete ===\n")
        
        return result
    
    def _optimize_max_incremental(self, model, variables):
        """
        @brief Optimize maximum path using incremental method
        
        Iteratively decreases the upper bound until no feasible solution exists.
        
        @param model Gurobi model
        @param variables Variable dictionary
        @return Optimized maximum path length
        """
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
        """
        @brief Optimize maximum path using binary search method
        
        Uses binary search to find the minimum feasible maximum path length.
        
        @param model Gurobi model
        @param variables Variable dictionary
        @param max_bound Upper bound for binary search
        @return Optimized maximum path length
        """
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
                print(f"[DEBUG] Binary search iteration {iteration}: [{left}, {right}]")
            
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
        """
        @brief Set upper bounds on variables based on threshold
        
        Forces all paths to complete within the given threshold by setting
        upper bounds to 0 for variables that would exceed the threshold.
        
        @param variables Variable dictionary
        @param threshold Maximum allowed path length
        """
        for layer in range(self.max_length + 1):
            for v1, v2_dict in variables[layer].items():
                if self.reachability_dist[v1] + layer > threshold:
                    for var in v2_dict.values():
                        var.UB = 0
                else:
                    for var in v2_dict.values():
                        var.UB = 1
        

    def optimize_min_path(self):
        """
        @brief Optimize to maximize the minimum path length
        
        After finding the maximum path length, this method optimizes
        to increase the minimum path length as much as possible.
        
        @return Optimized minimum path length
        """
        if self.config.debug_mode:
            print("\n[DEBUG] === Starting Minimum Path Optimization ===")
        
        model, variables = self.build_mip_model('sprl_min')
        model.ModelSense = GRB.MAXIMIZE
        
        min_length = self.current_route.get_min_length()
        print(f'Initial Min Length = {min_length}', flush=True)
        
        if self.config.method == 0:
            result = self._optimize_min_incremental(model, variables, min_length)
        else:
            result = self._optimize_min_binary_search(model, variables, min_length)
        
        if self.config.debug_mode:
            print("[DEBUG] === Minimum Path Optimization Complete ===\n")
        
        return result
    
    def _optimize_min_incremental(self, model, variables, min_length):
        """
        @brief Optimize minimum path using incremental method
        
        Iteratively increases the lower bound until no feasible solution exists.
        
        @param model Gurobi model
        @param variables Variable dictionary
        @param min_length Current minimum path length
        @return Optimized minimum path length
        """
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
        """
        @brief Optimize minimum path using binary search method
        
        Uses binary search to find the maximum feasible minimum path length.
        
        @param model Gurobi model
        @param variables Variable dictionary
        @param min_length Current minimum path length
        @return Optimized minimum path length
        """
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
                print(f"[DEBUG] Binary search iteration {iteration}: [{left}, {right}]")
            
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
        """
        @brief Set lower bounds on sink variables based on threshold
        
        Prevents paths from reaching sinks before the given threshold.
        
        @param variables Variable dictionary
        @param threshold Minimum required path length
        """
        for sink in self.sink_set:
            # Forbid arrival before threshold
            for layer in range(threshold):
                if sink in variables[layer] and 't' in variables[layer][sink]:
                    var = variables[layer][sink]['t']
                    var.LB = 0
                    var.UB = 0
            
            # Allow arrival at or after threshold
            for layer in range(threshold, self.max_length + 1):
                if sink in variables[layer] and 't' in variables[layer][sink]:
                    var = variables[layer][sink]['t']
                    var.LB = 0
                    var.UB = 1
        


def parse_arguments():
    """
    @brief Parse command line arguments
    
    @return GraphConfig object with parsed parameters
    """
    parser = argparse.ArgumentParser(
        description='Set-Pair Routing Optimization with Layer (SPRL)',
        add_help=True,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage
  python sprl.py benchmarks/S1.map

  # With time limit and method
  python sprl.py benchmarks/B1.map --time-limit 300 --method BS
        
  # Output graph
  python sprl.py benchmarks/S1.map --output
  
  # Debug mode
  python sprl.py benchmarks/S1.map --debug
        """
    )
    parser.add_argument('benchmark',
                       help='Path to benchmark file (e.g., benchmarks/S1.map)')
    parser.add_argument('-t', '--time-limit', type=int, default=DEFAULT_TIME_LIMIT,
                       dest='time',
                       help=f'Time limit in seconds (default: {DEFAULT_TIME_LIMIT})')
    parser.add_argument('-m', '--method', type=str, default='INC',
                       choices=['INC', 'BS'],
                       help='Optimization method: INC or BS (default: INC)')
    parser.add_argument('-d', '--debug', action='store_true',
                       dest='debug',
                       help='Enable debug mode (detailed logging and reports)')
    parser.add_argument('-o', '--output', action='store_true',
                       help='Output graph visualization')
    
    args = parser.parse_args()
    
    # Convert method name to index
    method_index = METHOD_NAMES.index(args.method) if args.method in METHOD_NAMES else 0
    
    return GraphConfig(
        bench=args.benchmark,
        time_limit=args.time,
        method=method_index,
        debug_mode=args.debug,
        output_graph=args.output
    )


def main():
    """
    @brief Main execution function
    
    Orchestrates the entire optimization process including:
    - Loading and parsing command line arguments
    - Building the graph from input files
    - Estimating initial bounds
    - Optimizing maximum path length
    - Optimizing minimum path length
    - Reporting results
    """
    config = parse_arguments()
    
    print("=" * 80)
    print("SPRL - Set-Pair Routing Problem with Layer")
    print("=" * 80)
    print(f"Method: {METHOD_NAMES[config.method]}")
    print(f"Benchmark: {config.bench}")
    print(f"Time Limit: {config.time_limit}s")
    if config.debug_mode:
        print("Debug Mode: ENABLED")
    if config.output_graph:
        print("Output Graph: ENABLED")
    print("=" * 80)
    print()
    
    # Load graph
    builder = GridGraphBuilder()
    filename = f"{config.bench}"
    
    if config.debug_mode:
        print(f"[DEBUG] Loading map file: {filename}")
    
    data, width, height = builder.load_map_file(filename)
    graph, source_set, sink_set = builder.build_graph(data, width, height)
    
    if config.output_graph:
        pos = {v: (v[0], height - v[1]) for v in graph.nodes()}
        node_colors = ['red' if v in source_set else 'blue' if v in sink_set else 'green' for v in graph.nodes()]
        nx.draw(graph, pos, node_color = node_colors)
        plt.show()
    
    print(f"Graph loaded: {len(graph.nodes())} nodes, {len(graph.edges())} edges")
    print(f"Sources: {len(source_set)}, Sinks: {len(sink_set)}")
    print()
    
    # Create optimizer
    optimizer = ImprovedRouteOptimizer(graph, source_set, sink_set, config)
    
    # Estimate initial bounds
    start_time = time.time()
    max_length, total_length = optimizer.estimate_initial_bounds()
    init_time = time.time()
    print(f'Initialization time: {init_time - start_time:.6f}s')
    print()
    
    # Optimize maximum path
    optimizer.max_length = max_length
    initial_bound = math.ceil(total_length / len(sink_set))
    max_length = optimizer.optimize_max_path(initial_bound)
    print(f'Maximum Length = {max_length}')
    
    search_max_time = time.time()
    print(f'Search Max time: {search_max_time - init_time:.6f}s')
    print()
    
    # Optimize minimum path
    optimizer.max_length = max_length
    min_length = optimizer.optimize_min_path()
    print(f'Minimum Length = {min_length}')
    
    search_min_time = time.time()
    print(f'Search Min time: {search_min_time - search_max_time:.6f}s')
    print(f'Total runtime: {search_min_time - start_time:.6f}s')
    print()
    
    if config.output_graph:
        path_dict = optimizer.current_route.path
        edge_colors = ['red' if path_dict.get(e[0]) == e[1] else 'black' for e in graph.edges()]
        edge_width = [3 if path_dict.get(e[0]) == e[1] else 1 for e in graph.edges()]
        nx.draw(graph, pos, node_color = node_colors, edge_color = edge_colors, width = edge_width)
        plt.show()
    
    
    # Debug reports
    if config.debug_mode:
        optimizer.current_route.report_cache_stats()
        
        print("\n" + "=" * 80)
        print("Debug Analysis Complete")
        print("=" * 80)


if __name__ == '__main__':
    main()
