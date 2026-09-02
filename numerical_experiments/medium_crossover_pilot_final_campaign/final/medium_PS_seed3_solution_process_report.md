# medium_PS_seed3 Solution Process Report

- Snapshot SHA-256: `b0c64a76399a18ddee847b7988d4cc486726e225864ddc3886aca44733552ea8`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.18719924383748385` / `0.18719924383748385` / `0.0`
- Compact incumbent/bound/gap: `0.1871992438374813` / `0.18719924383748135` / `0.0`
- BPC runtime/nodes: `58.85157871246338` / `1`
- Compact runtime/nodes: `27.515923738479614` / `39124.0`
- Compact first incumbent: `4.855999946594238` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 58.68103075027466,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": 0.18719924383749786,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 2458740.0,
  "root_compact_node_count": 25130.0,
  "root_compact_objective_full": 0.18719924383749784,
  "root_compact_route_paths": [
    [
      "Source",
      "C1",
      "C12",
      "H2",
      "DUP:C5",
      "DUP:C6",
      "DUP:C10",
      "Sink"
    ],
    [
      "Source",
      "H3",
      "DUP:C7",
      "DUP:C14",
      "DUP:C15",
      "DUP:C19",
      "C9",
      "C3",
      "C18",
      "Sink"
    ],
    [
      "Source",
      "C11",
      "H1",
      "C13",
      "C8",
      "Sink"
    ],
    [
      "Source",
      "C4",
      "C16",
      "C20",
      "C17",
      "C2",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.18719924383748385,
  "root_incumbent_at_fathom_full": 0.18719924383748385,
  "root_lower_bound_full": 0.18719924383748385,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.2575342916454303,
  "columns_added_farkas": 0,
  "columns_added_standard": 346,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 350,
  "pricing_complete_routes_generated": 3515,
  "pricing_cpu_core_equivalent_max": 3.714322418783203,
  "pricing_extensions_attempted": 19781,
  "pricing_extensions_rejected_by_deadline": 5517,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 3815,
  "pricing_labels_generated": 14384,
  "pricing_labels_pruned": 2047,
  "pricing_labels_purged": 60,
  "pricing_negative_routes_inserted": 346,
  "pricing_negative_routes_verified": 405,
  "pricing_process_cpu_time": 8.84375,
  "pricing_standard_bound_pruned": 2047,
  "standard_pricing_calls": 10,
  "total_routes": 350
}
```

## Cuts and Branching

```json
{
  "branching_nodes": 0,
  "child_nodes_created": 0,
  "conditioned_arc_branches": 0,
  "customer_pair_branches": 0,
  "launch_pad_branches": 0,
  "open_nodes_at_termination": 0,
  "postroot_nodes_processed": 0,
  "postroot_open_nodes": 0,
  "queue_bound_fathoms": 0,
  "sr_cuts_added": 7,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 7
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 10,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 2,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 105,
    "pricing_leaf_tasks_created": 120
  },
  "workers": {
    "pricing_idle_work_requests": 22,
    "pricing_parallel_calls": 10,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 10,
    "pricing_pool_shutdown_time": 0.14754557609558105,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.965796947479248,
    "pricing_process_cpu_time": 8.84375,
    "pricing_task_submission_time": 0.003000974655151367,
    "pricing_worker_busy_seconds": 9.07113003730774,
    "pricing_worker_idle_seconds": 39.47298240661621
  }
}
```

## Route Pool and Incumbent

```json
{
  "heuristic_calls": 0,
  "heuristic_full_pool_calls": 0,
  "heuristic_full_pool_feasible": 0,
  "heuristic_full_pool_incumbent_updates": 0,
  "heuristic_full_pool_time": 0.0,
  "heuristic_hard_pool_feasible_solves": 0,
  "heuristic_hard_pool_solves": 0,
  "heuristic_hard_pool_time": 0.0,
  "heuristic_incumbent_updates": 0,
  "heuristic_time": 0.0,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 41.74527597427368
}
```

## BPC Routes

- Route 0: truck `['Source', 'C1', 'C12', 'H2', 'Sink']`; drones `{'H2': ['C5', 'C6', 'C10']}`; return `80.90352768889028`; services `{'C1': 12.092237765239652, 'C12': 22.964520939420467, 'C5': 43.11731776653356, 'C6': 42.87955906148582, 'C10': 40.75377567122988}`
- Route 2: truck `['Source', 'C11', 'H1', 'C13', 'C8', 'Sink']`; drones `{}`; return `167.72067451175704`; services `{'C11': 36.560638435321835, 'C13': 84.50507550326529, 'C8': 104.19173543194732}`
- Route 3: truck `['Source', 'C4', 'C16', 'C20', 'C17', 'C2', 'Sink']`; drones `{}`; return `147.60725448428687`; services `{'C4': 23.954632979171723, 'C16': 30.46814544649112, 'C20': 48.31003366129838, 'C17': 77.35052756955236, 'C2': 95.72838568663319}`
- Route 1: truck `['Source', 'H3', 'C9', 'C3', 'C18', 'Sink']`; drones `{'H3': ['C7', 'C14', 'C15', 'C19']}`; return `120.96730009112834`; services `{'C7': 42.667668991180825, 'C14': 43.92214042724759, 'C15': 36.192480397786795, 'C19': 37.365819994145696, 'C9': 62.73730228084488, 'C3': 77.94328184403525, 'C18': 89.56924879614043}`

## Compact Routes

- Route 0: truck `['Source', 'C4', 'C16', 'C20', 'C17', 'C2', 'Sink']`; drones `{}`; return `147.60725448428687`; services `{'C4': 23.954632979171723, 'C16': 30.46814544649112, 'C20': 48.31003366129838, 'C17': 77.35052756955236, 'C2': 95.72838568663319}`
- Route 1: truck `['Source', 'C11', 'H1', 'C13', 'C8', 'Sink']`; drones `{}`; return `167.72067451175704`; services `{'C11': 36.560638435321835, 'C13': 84.50507550326529, 'C8': 104.19173543194732}`
- Route 2: truck `['Source', 'H3', 'C9', 'C3', 'C18', 'Sink']`; drones `{'H3': ['C7', 'C14', 'C15', 'C19']}`; return `120.96730009112834`; services `{'C7': 42.667668991180825, 'C14': 43.92214042724759, 'C15': 36.192480397786795, 'C19': 37.365819994145696, 'C9': 62.73730228084488, 'C3': 77.94328184403525, 'C18': 89.56924879614043}`
- Route 3: truck `['Source', 'C1', 'C12', 'H2', 'Sink']`; drones `{'H2': ['C5', 'C6', 'C10']}`; return `80.90352768889028`; services `{'C1': 12.092237765239652, 'C12': 22.964520939420467, 'C5': 43.11731776653356, 'C6': 42.87955906148582, 'C10': 40.75377567122988}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 436.3125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 7.2595945279340475,
    "mean_cpu_percent_of_logical_machine": 60.49662106611706,
    "peak_interval_core_equivalent": 11.985330430155251,
    "peak_process_count": 13,
    "peak_rss_bytes": 792940544,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PS_seed3\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 60.101497173309326
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 266.03125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 9.16690515505601,
    "mean_cpu_percent_of_logical_machine": 76.39087629213343,
    "peak_interval_core_equivalent": 11.417916922809951,
    "peak_process_count": 1,
    "peak_rss_bytes": 241410048,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PS_seed3\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 29.020835876464844
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
