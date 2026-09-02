# small_PC_seed8 Solution Process Report

- Snapshot SHA-256: `f924d268739eb9ce0628bd4f32103674eddf33bece8f5954204b62d95e6f6bcc`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.22246049811954266` / `0.22246049811954266` / `0.0`
- Compact incumbent/bound/gap: `0.22246049811954136` / `0.2224604981195413` / `0.0`
- BPC runtime/nodes: `5.965398073196411` / `1`
- Compact runtime/nodes: `0.24527311325073242` / `50.0`
- Compact first incumbent: `0.04799985885620117` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 5.613359451293945,
  "root_compact_accepted_columns": 2,
  "root_compact_bound_full": 0.22246049811953983,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 710.0,
  "root_compact_node_count": 1.0,
  "root_compact_objective_full": 0.22246049811954016,
  "root_compact_route_paths": [
    [
      "Source",
      "C3",
      "H2",
      "DUP:C5",
      "C1",
      "H1",
      "DUP:C4",
      "Sink"
    ],
    [
      "Source",
      "C2",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.22246049811954266,
  "root_incumbent_at_fathom_full": 0.22246049811954266,
  "root_lower_bound_full": 0.22246049811954266,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 2,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.0864973732835162,
  "columns_added_farkas": 0,
  "columns_added_standard": 19,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 21,
  "pricing_complete_routes_generated": 45,
  "pricing_cpu_core_equivalent_max": 3.212391549433851,
  "pricing_extensions_attempted": 245,
  "pricing_extensions_rejected_by_deadline": 0,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 42,
  "pricing_labels_generated": 269,
  "pricing_labels_pruned": 43,
  "pricing_labels_purged": 0,
  "pricing_negative_routes_inserted": 19,
  "pricing_negative_routes_verified": 19,
  "pricing_process_cpu_time": 0.078125,
  "pricing_standard_bound_pruned": 43,
  "standard_pricing_calls": 4,
  "total_routes": 21
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
    "pricing_balanced_process_dynamic_calls": 4,
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
    "pricing_leaf_tasks_closed": 48,
    "pricing_leaf_tasks_created": 48
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 4,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 4,
    "pricing_pool_shutdown_time": 0.33202695846557617,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 5.317000150680542,
    "pricing_process_cpu_time": 0.078125,
    "pricing_task_submission_time": 0.0050356388092041016,
    "pricing_worker_busy_seconds": 0.0943002700805664,
    "pricing_worker_idle_seconds": 0.46286439895629883
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
  "time_to_first_incumbent": 0.0785074234008789
}
```

## BPC Routes

- Route 1: truck `['Source', 'C2', 'Sink']`; drones `{}`; return `93.31557856662072`; services `{'C2': 46.65778928331036}`
- Route 0: truck `['Source', 'C3', 'H2', 'C1', 'H1', 'Sink']`; drones `{'H2': ['C5'], 'H1': ['C4']}`; return `102.48163670360931`; services `{'C3': 50.034504348935045, 'C5': 50.678728759132305, 'C1': 53.44608745629792, 'C4': 54.37041321086328}`

## Compact Routes

- Route 0: truck `['Source', 'C2', 'Sink']`; drones `{}`; return `93.31557856662072`; services `{'C2': 46.65778928331036}`
- Route 1: truck `['Source', 'C3', 'H2', 'C1', 'H1', 'Sink']`; drones `{'H2': ['C5'], 'H1': ['C4']}`; return `102.48163670360931`; services `{'C3': 50.034504348935045, 'C5': 50.678728759132305, 'C1': 53.44608745629792, 'C4': 54.37041321086328}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 21.09375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.423056900821883,
    "mean_cpu_percent_of_logical_machine": 20.19214084018236,
    "peak_interval_core_equivalent": 4.900425823632553,
    "peak_process_count": 13,
    "peak_rss_bytes": 682504192,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PC_seed8\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 8.705429077148438
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 1.5625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.48723737833934844,
    "mean_cpu_percent_of_logical_machine": 4.060311486161237,
    "peak_interval_core_equivalent": 2.9152160374840603,
    "peak_process_count": 1,
    "peak_rss_bytes": 56156160,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PC_seed8\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 3.2068557739257812
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
