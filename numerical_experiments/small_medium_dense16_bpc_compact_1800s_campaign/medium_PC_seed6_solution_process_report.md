# medium_PC_seed6 Solution Process Report

- Snapshot SHA-256: `0d0de0795356a92917fc39835a33e7dea287c8d00a19210f84214176bdf93d34`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.058787830517553485` / `0.058787830517553485` / `0.0`
- Compact incumbent/bound/gap: `0.058787830094376645` / `-0.0338733175721814` / `1.5761960854449288`
- BPC runtime/nodes: `303.4762849807739` / `1`
- Compact runtime/nodes: `1800.2841901779175` / `572882.0`
- Compact first incumbent: `5.75600004196167` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 303.2079908847809,
  "root_compact_accepted_columns": 3,
  "root_compact_bound_full": -0.09487895976506133,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 3160750.0,
  "root_compact_node_count": 16818.0,
  "root_compact_objective_full": 0.062288140166199005,
  "root_compact_route_paths": [
    [
      "Source",
      "C10",
      "H1",
      "DUP:C1",
      "DUP:C17",
      "C18",
      "H2",
      "C12",
      "C5",
      "C2",
      "Sink"
    ],
    [
      "Source",
      "C6",
      "H2",
      "C9",
      "C14",
      "C3",
      "C7",
      "Sink"
    ],
    [
      "Source",
      "C13",
      "C4",
      "C8",
      "H2",
      "DUP:C16",
      "C20",
      "H3",
      "DUP:C15",
      "DUP:C19",
      "C11",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.06228814016620121,
  "root_incumbent_at_fathom_full": 0.058787830517553485,
  "root_lower_bound_full": 0.058787830517553485,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 3,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.11519348445958338,
  "columns_added_farkas": 0,
  "columns_added_standard": 952,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 955,
  "pricing_complete_routes_generated": 197215,
  "pricing_cpu_core_equivalent_max": 9.088106951766257,
  "pricing_extensions_attempted": 829501,
  "pricing_extensions_rejected_by_deadline": 76710,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 234849,
  "pricing_labels_generated": 753043,
  "pricing_labels_pruned": 93078,
  "pricing_labels_purged": 2415,
  "pricing_negative_routes_inserted": 952,
  "pricing_negative_routes_verified": 1442,
  "pricing_process_cpu_time": 1343.890625,
  "pricing_standard_bound_pruned": 93078,
  "standard_pricing_calls": 21,
  "total_routes": 955
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
  "queue_bound_fathoms": 1,
  "sr_cuts_added": 32,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 32
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 21,
    "pricing_dynamic_bytes_transferred": 7190251,
    "pricing_dynamic_child_tasks_created": 196,
    "pricing_dynamic_labels_transferred": 17199,
    "pricing_dynamic_split_candidates": 87,
    "pricing_dynamic_split_control_time": 0.15263628959655762,
    "pricing_dynamic_split_rejected_elapsed": 31,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 1894,
    "pricing_dynamic_splits_performed": 39,
    "pricing_leaf_tasks_closed": 327,
    "pricing_leaf_tasks_created": 448
  },
  "workers": {
    "pricing_idle_work_requests": 1265,
    "pricing_parallel_calls": 21,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 21,
    "pricing_pool_shutdown_time": 0.15461349487304688,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.438366413116455,
    "pricing_process_cpu_time": 1343.890625,
    "pricing_task_submission_time": 0.02851390838623047,
    "pricing_worker_busy_seconds": 1379.4889960289001,
    "pricing_worker_idle_seconds": 1290.7888276576996
  }
}
```

## Route Pool and Incumbent

```json
{
  "heuristic_calls": 1,
  "heuristic_full_pool_calls": 1,
  "heuristic_full_pool_feasible": 1,
  "heuristic_full_pool_incumbent_updates": 1,
  "heuristic_full_pool_time": 0.05516862869262695,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.05516862869262695,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.057169198989868164,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.16415357589722
}
```

## BPC Routes

- Route 939: truck `['Source', 'C10', 'H1', 'C18', 'H2', 'C20', 'C15', 'Sink']`; drones `{'H1': ['C1', 'C2', 'C17']}`; return `137.79983615963658`; services `{'C10': 24.38745206065201, 'C1': 34.935358509383626, 'C2': 34.00499720061504, 'C17': 33.50641485663066, 'C18': 42.4910813093766, 'C20': 56.429614131711034, 'C15': 77.9930668631011}`
- Route 754: truck `['Source', 'C13', 'C4', 'C8', 'H2', 'C12', 'C5', 'H3', 'C11', 'Sink']`; drones `{'H2': ['C16'], 'H3': ['C19']}`; return `144.5286762884749`; services `{'C13': 33.59522939753896, 'C4': 37.80903869930433, 'C8': 39.12690750710124, 'C16': 41.71811949283156, 'C12': 43.931254639641985, 'C5': 52.335102201917714, 'C19': 81.31133548573118, 'C11': 84.30359408686684}`
- Route 1: truck `['Source', 'C6', 'H2', 'C9', 'C14', 'C3', 'C7', 'Sink']`; drones `{}`; return `152.5949156229508`; services `{'C6': 31.129381780360646, 'C9': 50.26085056670825, 'C14': 56.46188187732048, 'C3': 87.61635157143706, 'C7': 90.41185055348458}`

## Compact Routes

- Route 0: truck `['Source', 'C13', 'C4', 'C8', 'H2', 'C12', 'C5', 'H3', 'C11', 'Sink']`; drones `{'H2': ['C16'], 'H3': ['C19']}`; return `144.5286762884749`; services `{'C13': 33.59522939753896, 'C4': 37.80903869930433, 'C8': 39.12690750710124, 'C16': 41.71811949283156, 'C12': 43.931254639641985, 'C5': 52.335102201917714, 'C19': 81.31133548573118, 'C11': 84.30359408686684}`
- Route 1: truck `['Source', 'C6', 'H2', 'C9', 'C14', 'C3', 'C7', 'Sink']`; drones `{}`; return `152.5949156229508`; services `{'C6': 31.129381780360646, 'C9': 50.26085056670825, 'C14': 56.46188187732048, 'C3': 87.61635157143706, 'C7': 90.41185055348458}`
- Route 2: truck `['Source', 'C10', 'H1', 'C18', 'H2', 'C20', 'C15', 'Sink']`; drones `{'H1': ['C1', 'C2', 'C17']}`; return `137.79983615963658`; services `{'C10': 24.38745206065201, 'C1': 34.935358509383626, 'C2': 34.00499720061504, 'C17': 33.50641485663066, 'C18': 42.4910813093766, 'C20': 56.429614131711034, 'C15': 77.9930668631011}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 1726.34375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.664486073207459,
    "mean_cpu_percent_of_logical_machine": 47.204050610062154,
    "peak_interval_core_equivalent": 11.267111980059724,
    "peak_process_count": 13,
    "peak_rss_bytes": 1119543296,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\medium_PC_seed6\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 304.76617431640625
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 10343.03125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.741216846555789,
    "mean_cpu_percent_of_logical_machine": 47.84347372129824,
    "peak_interval_core_equivalent": 6.145973993818424,
    "peak_process_count": 1,
    "peak_rss_bytes": 363380736,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\medium_PC_seed6\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.5399045944214
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 21 artifacts are stored in the JSON report.
