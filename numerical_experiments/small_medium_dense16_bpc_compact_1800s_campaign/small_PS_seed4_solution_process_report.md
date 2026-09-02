# small_PS_seed4 Solution Process Report

- Snapshot SHA-256: `9e428dcfceeb20131b4088596f84a77ebe9ad74f2d0cf1f1b10ce1ff720fcf45`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.2718881699732445` / `0.2718881699732445` / `0.0`
- Compact incumbent/bound/gap: `0.27188816997324844` / `0.2718881699732485` / `0.0`
- BPC runtime/nodes: `4.424454927444458` / `1`
- Compact runtime/nodes: `0.11576700210571289` / `35.0`
- Compact first incumbent: `0.01100015640258789` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 4.224320650100708,
  "root_compact_accepted_columns": 2,
  "root_compact_bound_full": 0.2718881699732483,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 823.0,
  "root_compact_node_count": 33.0,
  "root_compact_objective_full": 0.2718881699732483,
  "root_compact_route_paths": [
    [
      "Source",
      "C4",
      "H1",
      "DUP:C2",
      "DUP:C5",
      "C3",
      "Sink"
    ],
    [
      "Source",
      "C1",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.2718881699732445,
  "root_incumbent_at_fathom_full": 0.2718881699732445,
  "root_lower_bound_full": 0.2718881699732445,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 2,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.21865204286756124,
  "columns_added_farkas": 0,
  "columns_added_standard": 8,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 10,
  "pricing_complete_routes_generated": 24,
  "pricing_cpu_core_equivalent_max": 2.5944576405384008,
  "pricing_extensions_attempted": 109,
  "pricing_extensions_rejected_by_deadline": 3,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 14,
  "pricing_labels_generated": 124,
  "pricing_labels_pruned": 22,
  "pricing_labels_purged": 0,
  "pricing_negative_routes_inserted": 8,
  "pricing_negative_routes_verified": 8,
  "pricing_process_cpu_time": 0.03125,
  "pricing_standard_bound_pruned": 22,
  "standard_pricing_calls": 3,
  "total_routes": 10
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
    "pricing_pool_shutdown_time": 0.18804311752319336,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 4.033069372177124,
    "pricing_process_cpu_time": 0.03125,
    "pricing_task_submission_time": 0.0029442310333251953,
    "pricing_worker_busy_seconds": 0.0452420711517334,
    "pricing_worker_idle_seconds": 0.16417551040649414
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
  "time_to_first_incumbent": 0.1021413803100586
}
```

## BPC Routes

- Route 1: truck `['Source', 'C1', 'Sink']`; drones `{}`; return `109.0787743790047`; services `{'C1': 54.53938718950235}`
- Route 0: truck `['Source', 'C4', 'H1', 'C3', 'Sink']`; drones `{'H1': ['C2', 'C5']}`; return `114.02784463159628`; services `{'C4': 36.61608836736884, 'C2': 46.576289601361516, 'C5': 51.04634166022272, 'C3': 77.13375401763494}`

## Compact Routes

- Route 0: truck `['Source', 'C4', 'H1', 'C3', 'Sink']`; drones `{'H1': ['C2', 'C5']}`; return `114.02784463159628`; services `{'C4': 36.61608836736884, 'C2': 46.576289601361516, 'C5': 51.04634166022272, 'C3': 77.13375401763494}`
- Route 1: truck `['Source', 'C1', 'H1', 'Sink']`; drones `{}`; return `109.0787743790047`; services `{'C1': 54.53938718950235}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 18.046875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.8705089139116726,
    "mean_cpu_percent_of_logical_machine": 23.920907615930606,
    "peak_interval_core_equivalent": 5.852393352898451,
    "peak_process_count": 13,
    "peak_rss_bytes": 698261504,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PS_seed4\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 6.286994934082031
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 1.03125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.4928415433976767,
    "mean_cpu_percent_of_logical_machine": 4.107012861647306,
    "peak_interval_core_equivalent": 0.9942090346782145,
    "peak_process_count": 1,
    "peak_rss_bytes": 35753984,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PS_seed4\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 2.0924575328826904
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
