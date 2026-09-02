# pilot20_PC_seed105 Solution Process Report

- Snapshot SHA-256: `6dd138d9c5476d3639a061490324702a5315c5d0f3ca9d00357adefadcf69a51`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.13552436738280074` / `0.13552436738280074` / `0.0`
- Compact incumbent/bound/gap: `0.1355243673828006` / `0.13552436738280071` / `0.0`
- BPC runtime/nodes: `34.26029062271118` / `1`
- Compact runtime/nodes: `34.38516283035278` / `29844.0`
- Compact first incumbent: `4.707000017166138` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 34.08058142662048,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": 0.13552436738280005,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 1664318.0,
  "root_compact_node_count": 8764.0,
  "root_compact_objective_full": 0.13552436738280008,
  "root_compact_route_paths": [
    [
      "Source",
      "C6",
      "C12",
      "H3",
      "C13",
      "H2",
      "DUP:C1",
      "DUP:C9",
      "C17",
      "H1",
      "Sink"
    ],
    [
      "Source",
      "C11",
      "C15",
      "C3",
      "C19",
      "H3",
      "DUP:C4",
      "DUP:C8",
      "C5",
      "C20",
      "H1",
      "Sink"
    ],
    [
      "Source",
      "H1",
      "DUP:C18",
      "C10",
      "C16",
      "Sink"
    ],
    [
      "Source",
      "C7",
      "C14",
      "C2",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.13552436738280074,
  "root_incumbent_at_fathom_full": 0.13552436738280074,
  "root_lower_bound_full": 0.13552436738280074,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.22636122694336414,
  "columns_added_farkas": 0,
  "columns_added_standard": 292,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 296,
  "pricing_complete_routes_generated": 3675,
  "pricing_cpu_core_equivalent_max": 5.656579606069499,
  "pricing_extensions_attempted": 19423,
  "pricing_extensions_rejected_by_deadline": 6745,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 2273,
  "pricing_labels_generated": 12762,
  "pricing_labels_pruned": 1584,
  "pricing_labels_purged": 214,
  "pricing_negative_routes_inserted": 292,
  "pricing_negative_routes_verified": 406,
  "pricing_process_cpu_time": 7.078125,
  "pricing_standard_bound_pruned": 1584,
  "standard_pricing_calls": 7,
  "total_routes": 296
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
  "sr_cuts_added": 0,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 0
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 7,
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
    "pricing_leaf_tasks_closed": 61,
    "pricing_leaf_tasks_created": 84
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 7,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 7,
    "pricing_pool_shutdown_time": 0.16158151626586914,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.567190885543823,
    "pricing_process_cpu_time": 7.078125,
    "pricing_task_submission_time": 0.005510091781616211,
    "pricing_worker_busy_seconds": 7.434024810791016,
    "pricing_worker_idle_seconds": 10.942589044570923
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
  "time_to_first_incumbent": 21.379260301589966
}
```

## BPC Routes

- Route 1: truck `['Source', 'C11', 'C15', 'C3', 'C19', 'H3', 'C5', 'C20', 'H1', 'Sink']`; drones `{'H3': ['C4', 'C8']}`; return `166.5606489268176`; services `{'C11': 22.611763577237987, 'C15': 24.344427846215748, 'C3': 25.45570639401615, 'C19': 26.60268805150737, 'C4': 58.37807886893515, 'C8': 58.13778959587399, 'C5': 87.08743932209308, 'C20': 114.43442876280119}`
- Route 0: truck `['Source', 'C6', 'C12', 'H3', 'C13', 'H2', 'C17', 'H1', 'Sink']`; drones `{'H2': ['C1', 'C9']}`; return `145.08353175023373`; services `{'C6': 20.356863479475123, 'C12': 53.76883962504965, 'C13': 80.42371645515472, 'C1': 83.17123421248525, 'C9': 82.80946702287878, 'C17': 86.17461958787452}`
- Route 3: truck `['Source', 'C7', 'C14', 'C2', 'Sink']`; drones `{}`; return `69.1029642559199`; services `{'C7': 23.503442091975106, 'C14': 44.92254187370682, 'C2': 45.53809562173997}`
- Route 2: truck `['Source', 'H1', 'C10', 'C16', 'Sink']`; drones `{'H1': ['C18']}`; return `133.0184404827891`; services `{'C18': 28.371230685800406, 'C10': 45.586931860073875, 'C16': 78.04158540569404}`

## Compact Routes

- Route 0: truck `['Source', 'C6', 'C12', 'H3', 'C13', 'H2', 'C17', 'H1', 'Sink']`; drones `{'H2': ['C1', 'C9']}`; return `145.08353175023373`; services `{'C6': 20.356863479475123, 'C12': 53.76883962504965, 'C13': 80.42371645515472, 'C1': 83.17123421248525, 'C9': 82.80946702287878, 'C17': 86.17461958787452}`
- Route 1: truck `['Source', 'H1', 'C10', 'C16', 'Sink']`; drones `{'H1': ['C18']}`; return `133.0184404827891`; services `{'C18': 28.371230685800406, 'C10': 45.586931860073875, 'C16': 78.04158540569404}`
- Route 2: truck `['Source', 'C7', 'C14', 'C2', 'Sink']`; drones `{}`; return `69.1029642559199`; services `{'C7': 23.503442091975106, 'C14': 44.92254187370682, 'C2': 45.53809562173997}`
- Route 3: truck `['Source', 'C11', 'C15', 'C3', 'C19', 'H3', 'C5', 'C20', 'H1', 'Sink']`; drones `{'H3': ['C4', 'C8']}`; return `166.5606489268176`; services `{'C11': 22.611763577237987, 'C15': 24.344427846215748, 'C3': 25.45570639401615, 'C19': 26.60268805150737, 'C4': 58.37807886893515, 'C8': 58.13778959587399, 'C5': 87.08743932209308, 'C20': 114.43442876280119}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 216.703125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 6.008000714784617,
    "mean_cpu_percent_of_logical_machine": 50.06667262320514,
    "peak_interval_core_equivalent": 11.642651020951059,
    "peak_process_count": 13,
    "peak_rss_bytes": 784785408,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n20\\cases\\pilot20_PC_seed105\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 36.06909108161926
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 342.46875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 9.441319303023802,
    "mean_cpu_percent_of_logical_machine": 78.67766085853168,
    "peak_interval_core_equivalent": 11.645389816261076,
    "peak_process_count": 1,
    "peak_rss_bytes": 220082176,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n20\\cases\\pilot20_PC_seed105\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 36.2733998298645
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
