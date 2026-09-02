# small_PS_seed1 Solution Process Report

- Snapshot SHA-256: `f33991c5bc8c37554f83c6e9558001acb7c8eda0d04fa47a2fae3f0efcf07488`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.263195971607575` / `0.263195971607575` / `0.0`
- Compact incumbent/bound/gap: `0.2631959716075897` / `0.26319597160758973` / `0.0`
- BPC runtime/nodes: `4.2571022510528564` / `1`
- Compact runtime/nodes: `0.22719693183898926` / `137.0`
- Compact first incumbent: `0.013000011444091797` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 4.0486180782318115,
  "root_compact_accepted_columns": 2,
  "root_compact_bound_full": 0.26319597160759395,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 641.0,
  "root_compact_node_count": 1.0,
  "root_compact_objective_full": 0.263195971607594,
  "root_compact_route_paths": [
    [
      "Source",
      "C5",
      "C3",
      "H1",
      "DUP:C1",
      "C2",
      "Sink"
    ],
    [
      "Source",
      "C4",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.263195971607575,
  "root_incumbent_at_fathom_full": 0.263195971607575,
  "root_lower_bound_full": 0.263195971607575,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 2,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.1580314208535789,
  "columns_added_farkas": 0,
  "columns_added_standard": 9,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 11,
  "pricing_complete_routes_generated": 41,
  "pricing_cpu_core_equivalent_max": 3.895310363956967,
  "pricing_extensions_attempted": 186,
  "pricing_extensions_rejected_by_deadline": 16,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 6,
  "pricing_labels_generated": 188,
  "pricing_labels_pruned": 52,
  "pricing_labels_purged": 0,
  "pricing_negative_routes_inserted": 9,
  "pricing_negative_routes_verified": 9,
  "pricing_process_cpu_time": 0.0625,
  "pricing_standard_bound_pruned": 52,
  "standard_pricing_calls": 3,
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
    "pricing_pool_shutdown_time": 0.1914505958557129,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 3.806577444076538,
    "pricing_process_cpu_time": 0.0625,
    "pricing_task_submission_time": 0.0030007362365722656,
    "pricing_worker_busy_seconds": 0.057900190353393555,
    "pricing_worker_idle_seconds": 0.17740750312805176
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
  "time_to_first_incumbent": 0.14191842079162598
}
```

## BPC Routes

- Route 1: truck `['Source', 'C4', 'Sink']`; drones `{}`; return `92.76762976422023`; services `{'C4': 46.383814882110116}`
- Route 0: truck `['Source', 'C5', 'C3', 'H1', 'C2', 'Sink']`; drones `{'H1': ['C1']}`; return `127.43570026573846`; services `{'C5': 21.643230034354794, 'C3': 45.400588961557844, 'C1': 72.63393397021575, 'C2': 86.45536052110555}`

## Compact Routes

- Route 0: truck `['Source', 'C4', 'Sink']`; drones `{}`; return `92.76762976422023`; services `{'C4': 46.383814882110116}`
- Route 1: truck `['Source', 'C5', 'C3', 'H1', 'C2', 'Sink']`; drones `{'H1': ['C1']}`; return `127.43570026573846`; services `{'C5': 21.643230034354794, 'C3': 45.400588961557844, 'C1': 72.63393397021575, 'C2': 86.45536052110555}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 18.8125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 3.000378881501046,
    "mean_cpu_percent_of_logical_machine": 25.00315734584205,
    "peak_interval_core_equivalent": 6.348866164998675,
    "peak_process_count": 13,
    "peak_rss_bytes": 690065408,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PS_seed1\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 6.270041465759277
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 1.0,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.4801886177901224,
    "mean_cpu_percent_of_logical_machine": 4.001571814917686,
    "peak_interval_core_equivalent": 2.1952902555857032,
    "peak_process_count": 1,
    "peak_rss_bytes": 47308800,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PS_seed1\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 2.082515001296997
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
