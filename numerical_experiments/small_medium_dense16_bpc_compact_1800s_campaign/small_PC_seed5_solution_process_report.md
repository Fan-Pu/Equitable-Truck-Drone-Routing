# small_PC_seed5 Solution Process Report

- Snapshot SHA-256: `6a949ccc659e27f2f2fcc3d252d40d3b45219c9c4e14d0b4edcd51e7cd926c04`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.0067675651612496834` / `0.0067675651612496834` / `0.0`
- Compact incumbent/bound/gap: `0.0067675651612484344` / `0.006767565161247269` / `0.0`
- BPC runtime/nodes: `4.691836595535278` / `1`
- Compact runtime/nodes: `0.04652047157287598` / `1.0`
- Compact first incumbent: `0.013000011444091797` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 4.419217586517334,
  "root_compact_accepted_columns": 1,
  "root_compact_bound_full": 0.006767565161250516,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 189.0,
  "root_compact_node_count": 1.0,
  "root_compact_objective_full": 0.006767565161250544,
  "root_compact_route_paths": [
    [
      "Source",
      "C5",
      "C3",
      "H2",
      "DUP:C4",
      "C2",
      "C1",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.0067675651612496834,
  "root_incumbent_at_fathom_full": 0.0067675651612496834,
  "root_lower_bound_full": 0.0067675651612496834,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 1,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.04659291974647839,
  "columns_added_farkas": 0,
  "columns_added_standard": 10,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 11,
  "pricing_complete_routes_generated": 140,
  "pricing_cpu_core_equivalent_max": 5.23095342618829,
  "pricing_extensions_attempted": 372,
  "pricing_extensions_rejected_by_deadline": 0,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 48,
  "pricing_labels_generated": 396,
  "pricing_labels_pruned": 28,
  "pricing_labels_purged": 0,
  "pricing_negative_routes_inserted": 10,
  "pricing_negative_routes_verified": 10,
  "pricing_process_cpu_time": 0.15625,
  "pricing_standard_bound_pruned": 28,
  "standard_pricing_calls": 4,
  "total_routes": 11
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
    "pricing_pool_shutdown_time": 0.2561631202697754,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 4.191854238510132,
    "pricing_process_cpu_time": 0.15625,
    "pricing_task_submission_time": 0.0030062198638916016,
    "pricing_worker_busy_seconds": 0.12471532821655273,
    "pricing_worker_idle_seconds": 0.4174976348876953
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
  "time_to_first_incumbent": 0.07490253448486328
}
```

## BPC Routes

- Route 0: truck `['Source', 'C5', 'C3', 'H2', 'C2', 'C1', 'Sink']`; drones `{'H2': ['C4']}`; return `129.55020809670145`; services `{'C5': 59.476801372974926, 'C3': 60.81446399829574, 'C4': 64.38536798779135, 'C2': 67.10422670136734, 'C1': 68.88413104436238}`

## Compact Routes

- Route 0: truck `['Source', 'C5', 'C3', 'H2', 'C2', 'C1', 'Sink']`; drones `{'H2': ['C4']}`; return `129.55020809670145`; services `{'C5': 59.476801372974926, 'C3': 60.81446399829574, 'C4': 64.38536798779135, 'C2': 67.10422670136734, 'C1': 68.88413104436238}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 18.984375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.992057690107312,
    "mean_cpu_percent_of_logical_machine": 24.933814084227603,
    "peak_interval_core_equivalent": 5.359195870348332,
    "peak_process_count": 13,
    "peak_rss_bytes": 684957696,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PC_seed5\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 6.344922780990601
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 1.03125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.4945133793970137,
    "mean_cpu_percent_of_logical_machine": 4.120944828308447,
    "peak_interval_core_equivalent": 1.0002740396989864,
    "peak_process_count": 1,
    "peak_rss_bytes": 38449152,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PC_seed5\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 2.085383415222168
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
