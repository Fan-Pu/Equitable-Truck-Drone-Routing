# small_PS_seed3 Solution Process Report

- Snapshot SHA-256: `7a58b67e3e5e8f577787fa92f2103484d192b6dccd4579a88e0cb536ea7bb603`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.25836826845079364` / `0.25836826845079364` / `0.0`
- Compact incumbent/bound/gap: `0.25836826845079885` / `0.2583682684507988` / `0.0`
- BPC runtime/nodes: `4.47203516960144` / `1`
- Compact runtime/nodes: `0.0538334846496582` / `1.0`
- Compact first incumbent: `0.012000083923339844` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 4.2586236000061035,
  "root_compact_accepted_columns": 2,
  "root_compact_bound_full": 0.25836826845079575,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 925.0,
  "root_compact_node_count": 20.0,
  "root_compact_objective_full": 0.25836826845079575,
  "root_compact_route_paths": [
    [
      "Source",
      "C1",
      "H2",
      "C4",
      "C5",
      "C2",
      "Sink"
    ],
    [
      "Source",
      "H1",
      "DUP:C3",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.25836826845079364,
  "root_incumbent_at_fathom_full": 0.25836826845079364,
  "root_lower_bound_full": 0.25836826845079364,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 2,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.10334556961717362,
  "columns_added_farkas": 0,
  "columns_added_standard": 5,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 7,
  "pricing_complete_routes_generated": 18,
  "pricing_cpu_core_equivalent_max": 1.5530593867007916,
  "pricing_extensions_attempted": 63,
  "pricing_extensions_rejected_by_deadline": 0,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 3,
  "pricing_labels_generated": 75,
  "pricing_labels_pruned": 12,
  "pricing_labels_purged": 0,
  "pricing_negative_routes_inserted": 5,
  "pricing_negative_routes_verified": 5,
  "pricing_process_cpu_time": 0.015625,
  "pricing_standard_bound_pruned": 12,
  "standard_pricing_calls": 2,
  "total_routes": 7
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
    "pricing_balanced_process_dynamic_calls": 2,
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
    "pricing_leaf_tasks_closed": 24,
    "pricing_leaf_tasks_created": 24
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 2,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 2,
    "pricing_pool_shutdown_time": 0.200453519821167,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 4.076362133026123,
    "pricing_process_cpu_time": 0.015625,
    "pricing_task_submission_time": 0.0040438175201416016,
    "pricing_worker_busy_seconds": 0.02820110321044922,
    "pricing_worker_idle_seconds": 0.11914515495300293
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
  "time_to_first_incumbent": 0.10349631309509277
}
```

## BPC Routes

- Route 0: truck `['Source', 'C1', 'H2', 'C4', 'C5', 'C2', 'Sink']`; drones `{}`; return `131.06538587512193`; services `{'C1': 12.092237765239652, 'C4': 47.807264155310655, 'C5': 59.116992204712744, 'C2': 79.18651707746825}`
- Route 1: truck `['Source', 'H1', 'Sink']`; drones `{'H1': ['C3']}`; return `39.23305241488923`; services `{'C3': 19.616526207444615}`

## Compact Routes

- Route 0: truck `['Source', 'H1', 'Sink']`; drones `{'H1': ['C3']}`; return `39.23305241488923`; services `{'C3': 19.616526207444615}`
- Route 1: truck `['Source', 'C1', 'H2', 'C4', 'C5', 'C2', 'Sink']`; drones `{}`; return `131.06538587512193`; services `{'C1': 12.092237765239652, 'C4': 47.807264155310655, 'C5': 59.116992204712744, 'C2': 79.18651707746825}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 18.046875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.8670923754339372,
    "mean_cpu_percent_of_logical_machine": 23.892436461949476,
    "peak_interval_core_equivalent": 5.336445879356717,
    "peak_process_count": 13,
    "peak_rss_bytes": 685461504,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PS_seed3\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 6.29448676109314
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 0.984375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.47125819768822824,
    "mean_cpu_percent_of_logical_machine": 3.9271516474019017,
    "peak_interval_core_equivalent": 0.9413391250184107,
    "peak_process_count": 1,
    "peak_rss_bytes": 51699712,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PS_seed3\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 2.088823080062866
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
