# small_PC_seed7 Solution Process Report

- Snapshot SHA-256: `168f0015559891e5119422923b75f2b750018b95910aa7908525d264127fdbe5`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.004960280636406855` / `0.004960280636406855` / `0.0`
- Compact incumbent/bound/gap: `0.004960280636409853` / `0.004960280636409881` / `0.0`
- BPC runtime/nodes: `4.6005332469940186` / `1`
- Compact runtime/nodes: `0.1074516773223877` / `1.0`
- Compact first incumbent: `0.015999794006347656` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 4.26467227935791,
  "root_compact_accepted_columns": 1,
  "root_compact_bound_full": 0.004960280636405245,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 470.0,
  "root_compact_node_count": 1.0,
  "root_compact_objective_full": 0.004960280636405301,
  "root_compact_route_paths": [
    [
      "Source",
      "C2",
      "C1",
      "C5",
      "H1",
      "C4",
      "H2",
      "DUP:C3",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.004960280636406855,
  "root_incumbent_at_fathom_full": 0.004960280636406855,
  "root_lower_bound_full": 0.004960280636406855,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 1,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.04565230486212338,
  "columns_added_farkas": 0,
  "columns_added_standard": 26,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 27,
  "pricing_complete_routes_generated": 125,
  "pricing_cpu_core_equivalent_max": 5.331525497267908,
  "pricing_extensions_attempted": 454,
  "pricing_extensions_rejected_by_deadline": 0,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 81,
  "pricing_labels_generated": 472,
  "pricing_labels_pruned": 21,
  "pricing_labels_purged": 0,
  "pricing_negative_routes_inserted": 26,
  "pricing_negative_routes_verified": 26,
  "pricing_process_cpu_time": 0.21875,
  "pricing_standard_bound_pruned": 21,
  "standard_pricing_calls": 3,
  "total_routes": 27
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
    "pricing_balanced_process_dynamic_calls": 3,
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
    "pricing_leaf_tasks_closed": 36,
    "pricing_leaf_tasks_created": 36
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 3,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 3,
    "pricing_pool_shutdown_time": 0.32082319259643555,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 4.04900050163269,
    "pricing_process_cpu_time": 0.21875,
    "pricing_task_submission_time": 0.0018875598907470703,
    "pricing_worker_busy_seconds": 0.14620733261108398,
    "pricing_worker_idle_seconds": 0.4141044616699219
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
  "time_to_first_incumbent": 0.07374405860900879
}
```

## BPC Routes

- Route 0: truck `['Source', 'C2', 'C1', 'C5', 'H1', 'C4', 'H2', 'Sink']`; drones `{'H2': ['C3']}`; return `122.22307183115078`; services `{'C2': 55.5679643268575, 'C1': 55.863631355208625, 'C5': 57.97566304529661, 'C4': 60.48194606172868, 'C3': 63.3933679240676}`

## Compact Routes

- Route 0: truck `['Source', 'C2', 'C1', 'C5', 'H1', 'C4', 'H2', 'Sink']`; drones `{'H2': ['C3']}`; return `122.22307183115078`; services `{'C2': 55.5679643268575, 'C1': 55.863631355208625, 'C5': 57.97566304529661, 'C4': 60.48194606172868, 'C3': 63.3933679240676}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 18.9375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 3.011803419593861,
    "mean_cpu_percent_of_logical_machine": 25.09836182994884,
    "peak_interval_core_equivalent": 5.987595025373812,
    "peak_process_count": 13,
    "peak_rss_bytes": 698789888,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PC_seed7\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 6.287760972976685
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 0.984375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.4636341187857157,
    "mean_cpu_percent_of_logical_machine": 3.8636176565476306,
    "peak_interval_core_equivalent": 3.130600936275915,
    "peak_process_count": 1,
    "peak_rss_bytes": 42770432,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PC_seed7\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 2.1231720447540283
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
