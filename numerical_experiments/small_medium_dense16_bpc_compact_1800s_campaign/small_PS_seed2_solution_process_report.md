# small_PS_seed2 Solution Process Report

- Snapshot SHA-256: `4959aa4b98aead252c4f9d5bba2770aa05b733ffe7c80b264c1a1c093e568281`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.2927049793966303` / `0.2927049793966303` / `0.0`
- Compact incumbent/bound/gap: `0.292704979396635` / `0.2927049793966349` / `0.0`
- BPC runtime/nodes: `4.911108016967773` / `1`
- Compact runtime/nodes: `0.1324007511138916` / `33.0`
- Compact first incumbent: `0.009000062942504883` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 4.647837162017822,
  "root_compact_accepted_columns": 2,
  "root_compact_bound_full": 0.2927049793966349,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 736.0,
  "root_compact_node_count": 32.0,
  "root_compact_objective_full": 0.292704979396635,
  "root_compact_route_paths": [
    [
      "Source",
      "C1",
      "C4",
      "C3",
      "Sink"
    ],
    [
      "Source",
      "C2",
      "H2",
      "DUP:C5",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.2927049793966303,
  "root_incumbent_at_fathom_full": 0.2927049793966303,
  "root_lower_bound_full": 0.2927049793966303,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 2,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.10016067157715544,
  "columns_added_farkas": 0,
  "columns_added_standard": 9,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 11,
  "pricing_complete_routes_generated": 15,
  "pricing_cpu_core_equivalent_max": 4.798535603148453,
  "pricing_extensions_attempted": 106,
  "pricing_extensions_rejected_by_deadline": 6,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 10,
  "pricing_labels_generated": 118,
  "pricing_labels_pruned": 14,
  "pricing_labels_purged": 2,
  "pricing_negative_routes_inserted": 9,
  "pricing_negative_routes_verified": 9,
  "pricing_process_cpu_time": 0.03125,
  "pricing_standard_bound_pruned": 14,
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
    "pricing_pool_shutdown_time": 0.24622774124145508,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 4.434393405914307,
    "pricing_process_cpu_time": 0.03125,
    "pricing_task_submission_time": 0.0020046234130859375,
    "pricing_worker_busy_seconds": 0.03557872772216797,
    "pricing_worker_idle_seconds": 0.13843894004821777
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
  "time_to_first_incumbent": 0.1045074462890625
}
```

## BPC Routes

- Route 0: truck `['Source', 'C1', 'C4', 'C3', 'Sink']`; drones `{}`; return `123.42874723059124`; services `{'C1': 21.003872912378988, 'C4': 32.89345704852029, 'C3': 73.60395775143692}`
- Route 1: truck `['Source', 'C2', 'H2', 'Sink']`; drones `{'H2': ['C5']}`; return `110.50520175040617`; services `{'C2': 33.980313102351644, 'C5': 76.73003395346412}`

## Compact Routes

- Route 0: truck `['Source', 'C1', 'C4', 'C3', 'Sink']`; drones `{}`; return `123.42874723059124`; services `{'C1': 21.003872912378988, 'C4': 32.89345704852029, 'C3': 73.60395775143692}`
- Route 1: truck `['Source', 'C2', 'H2', 'Sink']`; drones `{'H2': ['C5']}`; return `110.50520175040617`; services `{'C2': 33.980313102351644, 'C5': 76.73003395346412}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 18.125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.4209646738096318,
    "mean_cpu_percent_of_logical_machine": 20.174705615080263,
    "peak_interval_core_equivalent": 4.951993310295496,
    "peak_process_count": 13,
    "peak_rss_bytes": 633049088,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PS_seed2\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 7.486685037612915
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 0.9375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.43456354829907673,
    "mean_cpu_percent_of_logical_machine": 3.621362902492306,
    "peak_interval_core_equivalent": 0.8798379866657565,
    "peak_process_count": 1,
    "peak_rss_bytes": 37359616,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\small_PS_seed2\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 2.157336950302124
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
