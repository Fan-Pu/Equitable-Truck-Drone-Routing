# small_PC_seed6 Solution Process Report

- Snapshot SHA-256: `bdbe21489473ed4ad5769ca1f23b32333487fd11ecceb9ae1e718464e3704e03`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.18714183322315073` / `0.18714183322315073` / `0.0`
- Compact incumbent/bound/gap: `0.1871418332231509` / `0.1871412651425545` / `3.0355617801372363e-06`
- BPC runtime/nodes: `4.339584827423096` / `1`
- Compact runtime/nodes: `0.08910703659057617` / `1.0`
- Compact first incumbent: `0.015999794006347656` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 4.056128740310669,
  "root_compact_accepted_columns": 2,
  "root_compact_bound_full": 0.18714126514255436,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 557.0,
  "root_compact_node_count": 1.0,
  "root_compact_objective_full": 0.18714183322315325,
  "root_compact_route_paths": [
    [
      "Source",
      "C2",
      "H1",
      "DUP:C1",
      "C5",
      "Sink"
    ],
    [
      "Source",
      "C4",
      "H1",
      "C3",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.18714183322315073,
  "root_incumbent_at_fathom_full": 0.18714183322315073,
  "root_lower_bound_full": 0.18714183322315073,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 2,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.18247288986354976,
  "columns_added_farkas": 0,
  "columns_added_standard": 36,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 38,
  "pricing_complete_routes_generated": 112,
  "pricing_cpu_core_equivalent_max": 5.842734026745914,
  "pricing_extensions_attempted": 402,
  "pricing_extensions_rejected_by_deadline": 0,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 37,
  "pricing_labels_generated": 438,
  "pricing_labels_pruned": 75,
  "pricing_labels_purged": 0,
  "pricing_negative_routes_inserted": 36,
  "pricing_negative_routes_verified": 36,
  "pricing_process_cpu_time": 0.203125,
  "pricing_standard_bound_pruned": 75,
  "standard_pricing_calls": 6,
  "total_routes": 38
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
  "sr_cuts_added": 3,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 3
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 6,
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
    "pricing_leaf_tasks_closed": 72,
    "pricing_leaf_tasks_created": 72
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 6,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 6,
    "pricing_pool_shutdown_time": 0.26439619064331055,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 3.740859031677246,
    "pricing_process_cpu_time": 0.203125,
    "pricing_task_submission_time": 0.0022101402282714844,
    "pricing_worker_busy_seconds": 0.13017892837524414,
    "pricing_worker_idle_seconds": 0.546159029006958
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
  "time_to_first_incumbent": 0.08946943283081055
}
```

## BPC Routes

- Route 0: truck `['Source', 'C2', 'H1', 'C5', 'Sink']`; drones `{'H1': ['C1']}`; return `89.5090369212175`; services `{'C2': 39.01419244733794, 'C1': 42.62528289468887, 'C5': 48.732164795707696}`
- Route 1: truck `['Source', 'C4', 'H1', 'C3', 'Sink']`; drones `{}`; return `82.52313099541742`; services `{'C4': 41.2572566466271, 'C3': 44.6362512587301}`

## Compact Routes

- Route 0: truck `['Source', 'C4', 'H1', 'C3', 'Sink']`; drones `{}`; return `82.52313099541742`; services `{'C4': 41.2572566466271, 'C3': 44.6362512587301}`
- Route 1: truck `['Source', 'C2', 'H1', 'C5', 'Sink']`; drones `{'H1': ['C1']}`; return `89.5090369212175`; services `{'C2': 39.01419244733794, 'C1': 42.62528289468887, 'C5': 48.732164795707696}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 18.296875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.928329647227858,
    "mean_cpu_percent_of_logical_machine": 24.402747060232148,
    "peak_interval_core_equivalent": 6.10536984738608,
    "peak_process_count": 13,
    "peak_rss_bytes": 705974272,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PC_seed6\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 6.248229265213013
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 1.03125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.4982401269486119,
    "mean_cpu_percent_of_logical_machine": 4.152001057905099,
    "peak_interval_core_equivalent": 2.482894487592347,
    "peak_process_count": 1,
    "peak_rss_bytes": 47853568,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PC_seed6\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 2.0697851181030273
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
