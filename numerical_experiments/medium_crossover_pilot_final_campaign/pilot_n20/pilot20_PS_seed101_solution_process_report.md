# pilot20_PS_seed101 Solution Process Report

- Snapshot SHA-256: `a570e0053baff163c5ffd4726d5489bf1a75d27ca1971b9c477969ef564690d2`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.1424839267205476` / `0.1424839267205476` / `0.0`
- Compact incumbent/bound/gap: `0.14248392672054871` / `-0.050223029444004974` / `1.3524820700829403`
- BPC runtime/nodes: `80.81174683570862` / `1`
- Compact runtime/nodes: `600.4039549827576` / `56371.0`
- Compact first incumbent: `9.056999921798706` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 80.61523985862732,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": -0.08667353145424027,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 3712179.0,
  "root_compact_node_count": 6589.0,
  "root_compact_objective_full": 0.1548181217846003,
  "root_compact_route_paths": [
    [
      "Source",
      "C3",
      "C16",
      "C17",
      "Sink"
    ],
    [
      "Source",
      "C12",
      "H1",
      "DUP:C5",
      "DUP:C11",
      "DUP:C20",
      "C10",
      "C4",
      "H2",
      "C13",
      "Sink"
    ],
    [
      "Source",
      "C18",
      "C14",
      "C2",
      "H2",
      "DUP:C6",
      "DUP:C15",
      "DUP:C19",
      "C8",
      "Sink"
    ],
    [
      "Source",
      "C7",
      "C9",
      "C1",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.15481812178460064,
  "root_incumbent_at_fathom_full": 0.1424839267205476,
  "root_lower_bound_full": 0.14248392672054755,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.22833243663747826,
  "columns_added_farkas": 0,
  "columns_added_standard": 479,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 483,
  "pricing_complete_routes_generated": 10978,
  "pricing_cpu_core_equivalent_max": 8.21391104462169,
  "pricing_extensions_attempted": 69331,
  "pricing_extensions_rejected_by_deadline": 21961,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 16786,
  "pricing_labels_generated": 47490,
  "pricing_labels_pruned": 3455,
  "pricing_labels_purged": 221,
  "pricing_negative_routes_inserted": 479,
  "pricing_negative_routes_verified": 774,
  "pricing_process_cpu_time": 38.203125,
  "pricing_standard_bound_pruned": 3455,
  "standard_pricing_calls": 10,
  "total_routes": 483
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
  "sr_cuts_added": 0,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 0
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 10,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 12,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 58,
    "pricing_leaf_tasks_created": 120
  },
  "workers": {
    "pricing_idle_work_requests": 120,
    "pricing_parallel_calls": 10,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 10,
    "pricing_pool_shutdown_time": 0.13589954376220703,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.489895343780518,
    "pricing_process_cpu_time": 38.203125,
    "pricing_task_submission_time": 0.0050013065338134766,
    "pricing_worker_busy_seconds": 39.97777533531189,
    "pricing_worker_idle_seconds": 58.35222887992859
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
  "heuristic_full_pool_time": 0.026968002319335938,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.026968002319335938,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.027968645095825195,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.18167686462402
}
```

## BPC Routes

- Route 453: truck `['Source', 'C12', 'H1', 'C4', 'C17', 'Sink']`; drones `{'H1': ['C5', 'C11', 'C20']}`; return `152.80804360260072`; services `{'C12': 11.460452035384144, 'C5': 51.81058859482276, 'C11': 54.741895017921834, 'C20': 47.26754505076872, 'C4': 68.44550662438797, 'C17': 87.49921315782386}`
- Route 285: truck `['Source', 'C16', 'H3', 'C9', 'C1', 'Sink']`; drones `{}`; return `113.68661933433205`; services `{'C16': 30.47304564902847, 'C9': 43.90212747047554, 'C1': 64.82586162362729}`
- Route 2: truck `['Source', 'C18', 'C14', 'C2', 'H2', 'C8', 'Sink']`; drones `{'H2': ['C6', 'C15', 'C19']}`; return `148.66856009667424`; services `{'C18': 31.472772962746212, 'C14': 42.67173815002323, 'C2': 51.60313489497708, 'C6': 61.10706246345831, 'C15': 65.31584830529751, 'C19': 71.68837469231951, 'C8': 93.60240783801908}`
- Route 421: truck `['Source', 'C7', 'C10', 'C13', 'H2', 'C3', 'Sink']`; drones `{}`; return `138.0125239240053`; services `{'C7': 18.819974691626616, 'C10': 37.46255415274054, 'C13': 62.216904481244846, 'C3': 92.37298963826122}`

## Compact Routes

- Route 0: truck `['Source', 'C7', 'C10', 'C13', 'H2', 'C3', 'Sink']`; drones `{}`; return `138.0125239240053`; services `{'C7': 18.819974691626616, 'C10': 37.46255415274054, 'C13': 62.216904481244846, 'C3': 92.37298963826122}`
- Route 1: truck `['Source', 'C18', 'C14', 'C2', 'H2', 'C8', 'Sink']`; drones `{'H2': ['C6', 'C15', 'C19']}`; return `148.66856009667424`; services `{'C18': 31.472772962746212, 'C14': 42.67173815002323, 'C2': 51.60313489497708, 'C6': 61.10706246345831, 'C15': 65.31584830529751, 'C19': 71.68837469231951, 'C8': 93.60240783801908}`
- Route 2: truck `['Source', 'C16', 'H3', 'C9', 'C1', 'Sink']`; drones `{}`; return `113.68661933433205`; services `{'C16': 30.47304564902847, 'C9': 43.90212747047554, 'C1': 64.82586162362729}`
- Route 3: truck `['Source', 'C12', 'H1', 'C4', 'C17', 'Sink']`; drones `{'H1': ['C5', 'C11', 'C20']}`; return `152.80804360260072`; services `{'C12': 11.460452035384144, 'C5': 51.81058859482276, 'C11': 54.741895017921834, 'C20': 47.26754505076872, 'C4': 68.44550662438797, 'C17': 87.49921315782386}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 637.515625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 7.758780924332479,
    "mean_cpu_percent_of_logical_machine": 64.65650770277067,
    "peak_interval_core_equivalent": 11.52973750401889,
    "peak_process_count": 13,
    "peak_rss_bytes": 817479680,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n20\\cases\\pilot20_PS_seed101\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 82.16698360443115
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 5991.374999999999,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 9.957285759587045,
    "mean_cpu_percent_of_logical_machine": 82.97738132989203,
    "peak_interval_core_equivalent": 11.810888065822978,
    "peak_process_count": 1,
    "peak_rss_bytes": 705966080,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n20\\cases\\pilot20_PS_seed101\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 601.7076485157013
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
