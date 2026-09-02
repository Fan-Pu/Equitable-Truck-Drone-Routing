# pilot15_PS_seed101 Solution Process Report

- Snapshot SHA-256: `7f77a47649dd6e6159ae7c6420568394f828ef5560b5f38e7db8b4a08cdb4447`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.20157723131523725` / `0.20157723131523725` / `0.0`
- Compact incumbent/bound/gap: `0.20157723131523678` / `0.20157723131523675` / `0.0`
- BPC runtime/nodes: `17.184343576431274` / `1`
- Compact runtime/nodes: `3.373040199279785` / `9380.0`
- Compact first incumbent: `0.02700018882751465` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 16.967262506484985,
  "root_compact_accepted_columns": 3,
  "root_compact_bound_full": 0.2015772313152378,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 645307.0,
  "root_compact_node_count": 9780.0,
  "root_compact_objective_full": 0.20157723131523778,
  "root_compact_route_paths": [
    [
      "Source",
      "H1",
      "DUP:C4",
      "DUP:C8",
      "DUP:C14",
      "C1",
      "C5",
      "Sink"
    ],
    [
      "Source",
      "C7",
      "C12",
      "C9",
      "C11",
      "H2",
      "DUP:C13",
      "C6",
      "C15",
      "Sink"
    ],
    [
      "Source",
      "H1",
      "C10",
      "C2",
      "C3",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.20157723131523725,
  "root_incumbent_at_fathom_full": 0.20157723131523725,
  "root_lower_bound_full": 0.20157723131523725,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 3,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.22524867475407412,
  "columns_added_farkas": 0,
  "columns_added_standard": 286,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 289,
  "pricing_complete_routes_generated": 1966,
  "pricing_cpu_core_equivalent_max": 2.902566786785578,
  "pricing_extensions_attempted": 10778,
  "pricing_extensions_rejected_by_deadline": 2541,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 1955,
  "pricing_labels_generated": 8345,
  "pricing_labels_pruned": 1141,
  "pricing_labels_purged": 45,
  "pricing_negative_routes_inserted": 286,
  "pricing_negative_routes_verified": 323,
  "pricing_process_cpu_time": 4.21875,
  "pricing_standard_bound_pruned": 1141,
  "standard_pricing_calls": 9,
  "total_routes": 289
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
  "sr_cuts_added": 12,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 12
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 9,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 0,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 99,
    "pricing_leaf_tasks_created": 108
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 9,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 9,
    "pricing_pool_shutdown_time": 0.19208264350891113,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.533461570739746,
    "pricing_process_cpu_time": 4.21875,
    "pricing_task_submission_time": 0.005999088287353516,
    "pricing_worker_busy_seconds": 4.106459856033325,
    "pricing_worker_idle_seconds": 22.229885578155518
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
  "time_to_first_incumbent": 3.899381160736084
}
```

## BPC Routes

- Route 1: truck `['Source', 'C7', 'C12', 'C9', 'C11', 'H2', 'C6', 'C15', 'Sink']`; drones `{'H2': ['C13']}`; return `179.50673866193017`; services `{'C7': 18.819974691626616, 'C12': 42.649777081270884, 'C9': 59.12634860343975, 'C11': 64.13366045311727, 'C13': 91.69166612025167, 'C6': 99.488548880123, 'C15': 111.62779191085848}`
- Route 2: truck `['Source', 'H1', 'C10', 'C2', 'C3', 'Sink']`; drones `{}`; return `128.06483237946685`; services `{'C10': 37.462554152740545, 'C2': 51.60313489497709, 'C3': 82.42529809372277}`
- Route 0: truck `['Source', 'H1', 'C1', 'C5', 'Sink']`; drones `{'H1': ['C4', 'C8', 'C14']}`; return `128.91070426404636`; services `{'C4': 44.38070134943315, 'C8': 49.074683118303426, 'C14': 41.00172697441125, 'C1': 77.8266233766661, 'C5': 83.99691131274542}`

## Compact Routes

- Route 0: truck `['Source', 'H1', 'C1', 'C5', 'Sink']`; drones `{'H1': ['C4', 'C8', 'C14']}`; return `128.91070426404636`; services `{'C4': 44.38070134943315, 'C8': 49.074683118303426, 'C14': 41.00172697441125, 'C1': 77.8266233766661, 'C5': 83.99691131274542}`
- Route 1: truck `['Source', 'H1', 'C10', 'C2', 'C3', 'Sink']`; drones `{}`; return `128.06483237946685`; services `{'C10': 37.462554152740545, 'C2': 51.60313489497709, 'C3': 82.42529809372277}`
- Route 2: truck `['Source', 'C7', 'C12', 'C9', 'C11', 'H2', 'C6', 'C15', 'Sink']`; drones `{'H2': ['C13']}`; return `179.50673866193017`; services `{'C7': 18.819974691626616, 'C12': 42.649777081270884, 'C9': 59.12634860343975, 'C11': 64.13366045311727, 'C13': 91.69166612025167, 'C6': 99.488548880123, 'C15': 111.62779191085848}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 38.8125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.102156581493289,
    "mean_cpu_percent_of_logical_machine": 17.517971512444074,
    "peak_interval_core_equivalent": 5.911413790784656,
    "peak_process_count": 13,
    "peak_rss_bytes": 752254976,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n15\\cases\\pilot15_PS_seed101\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 18.46318221092224
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 18.34375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 3.558466242902656,
    "mean_cpu_percent_of_logical_machine": 29.65388535752213,
    "peak_interval_core_equivalent": 5.592400769771321,
    "peak_process_count": 1,
    "peak_rss_bytes": 107864064,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n15\\cases\\pilot15_PS_seed101\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 5.154959678649902
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
