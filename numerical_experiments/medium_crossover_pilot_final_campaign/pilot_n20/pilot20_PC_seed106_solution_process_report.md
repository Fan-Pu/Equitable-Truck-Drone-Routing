# pilot20_PC_seed106 Solution Process Report

- Snapshot SHA-256: `72b84e3ad522294dccc1899360a8310ddbede9769ca53f4bda5616511aa11baf`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.117327365641789` / `0.117327365641789` / `0.0`
- Compact incumbent/bound/gap: `0.11732731367568153` / `0.06634409734042572` / `0.43453834182366624`
- BPC runtime/nodes: `74.07637977600098` / `1`
- Compact runtime/nodes: `600.1786222457886` / `345774.0`
- Compact first incumbent: `3.439000129699707` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 73.90469884872437,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": -0.038536621481616226,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 7171960.0,
  "root_compact_node_count": 34682.0,
  "root_compact_objective_full": 0.11732736564179458,
  "root_compact_route_paths": [
    [
      "Source",
      "C10",
      "C3",
      "H1",
      "C17",
      "Sink"
    ],
    [
      "Source",
      "C2",
      "H3",
      "DUP:C14",
      "DUP:C16",
      "DUP:C18",
      "C6",
      "Sink"
    ],
    [
      "Source",
      "C20",
      "C12",
      "C4",
      "C15",
      "C5",
      "C11",
      "C1",
      "Sink"
    ],
    [
      "Source",
      "C8",
      "C19",
      "C7",
      "H2",
      "DUP:C9",
      "DUP:C13",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.117327365641789,
  "root_incumbent_at_fathom_full": 0.117327365641789,
  "root_lower_bound_full": 0.117327365641789,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.13877503412824488,
  "columns_added_farkas": 0,
  "columns_added_standard": 334,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 338,
  "pricing_complete_routes_generated": 4513,
  "pricing_cpu_core_equivalent_max": 8.463679803487983,
  "pricing_extensions_attempted": 24657,
  "pricing_extensions_rejected_by_deadline": 4243,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 4770,
  "pricing_labels_generated": 20534,
  "pricing_labels_pruned": 3577,
  "pricing_labels_purged": 87,
  "pricing_negative_routes_inserted": 334,
  "pricing_negative_routes_verified": 478,
  "pricing_process_cpu_time": 12.40625,
  "pricing_standard_bound_pruned": 3577,
  "standard_pricing_calls": 10,
  "total_routes": 338
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
    "pricing_balanced_process_dynamic_calls": 10,
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
    "pricing_leaf_tasks_closed": 91,
    "pricing_leaf_tasks_created": 120
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 10,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 10,
    "pricing_pool_shutdown_time": 0.14817380905151367,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.45013689994812,
    "pricing_process_cpu_time": 12.40625,
    "pricing_task_submission_time": 0.020011186599731445,
    "pricing_worker_busy_seconds": 12.98325490951538,
    "pricing_worker_idle_seconds": 15.310730218887329
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
  "time_to_first_incumbent": 60.154892683029175
}
```

## BPC Routes

- Route 0: truck `['Source', 'C10', 'C3', 'H1', 'C17', 'Sink']`; drones `{}`; return `108.36281213258758`; services `{'C10': 21.72218707267499, 'C3': 65.90809900812391, 'C17': 70.82893614794787}`
- Route 1: truck `['Source', 'C2', 'H3', 'C6', 'Sink']`; drones `{'H3': ['C14', 'C16', 'C18']}`; return `76.6083573177184`; services `{'C2': 19.815551694945835, 'C14': 36.651676211880584, 'C16': 37.92568296195474, 'C18': 35.81434185308495, 'C6': 55.83616833287202}`
- Route 2: truck `['Source', 'C20', 'C12', 'C4', 'C15', 'C5', 'C11', 'C1', 'Sink']`; drones `{}`; return `103.85124730966305`; services `{'C20': 19.967373989697773, 'C12': 22.683828087225308, 'C4': 25.483851000833802, 'C15': 49.766156553808926, 'C5': 55.017177940318206, 'C11': 60.634069511593154, 'C1': 66.80439652022577}`
- Route 3: truck `['Source', 'C8', 'C19', 'C7', 'H2', 'Sink']`; drones `{'H2': ['C9', 'C13']}`; return `93.24721322042996`; services `{'C8': 19.11026293308839, 'C19': 44.606797602690364, 'C7': 47.266388679639284, 'C9': 53.901709341859565, 'C13': 52.95131603572786}`

## Compact Routes

- Route 0: truck `['Source', 'C10', 'C3', 'H1', 'C17', 'Sink']`; drones `{}`; return `108.36281213258758`; services `{'C10': 21.72218707267499, 'C3': 65.90809900812391, 'C17': 70.82893614794787}`
- Route 1: truck `['Source', 'C2', 'H3', 'C6', 'Sink']`; drones `{'H3': ['C14', 'C16', 'C18']}`; return `76.6083573177184`; services `{'C2': 19.815551694945835, 'C14': 36.651676211880584, 'C16': 37.92568296195474, 'C18': 35.81434185308495, 'C6': 55.83616833287202}`
- Route 2: truck `['Source', 'C20', 'C12', 'C4', 'C15', 'C5', 'C11', 'C1', 'Sink']`; drones `{}`; return `103.85124730966305`; services `{'C20': 19.967373989697773, 'C12': 22.683828087225308, 'C4': 25.483851000833802, 'C15': 49.766156553808926, 'C5': 55.017177940318206, 'C11': 60.634069511593154, 'C1': 66.80439652022577}`
- Route 3: truck `['Source', 'C8', 'C19', 'C7', 'H2', 'Sink']`; drones `{'H2': ['C9', 'C13']}`; return `93.24721322042996`; services `{'C8': 19.11026293308839, 'C19': 44.606797602690364, 'C7': 47.266388679639284, 'C9': 53.901709341859565, 'C13': 52.95131603572786}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 638.9375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 8.456079077291085,
    "mean_cpu_percent_of_logical_machine": 70.46732564409237,
    "peak_interval_core_equivalent": 11.548880008006801,
    "peak_process_count": 13,
    "peak_rss_bytes": 792023040,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n20\\cases\\pilot20_PC_seed106\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 75.55954647064209
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 6165.078124999999,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 10.254543251789634,
    "mean_cpu_percent_of_logical_machine": 85.45452709824694,
    "peak_interval_core_equivalent": 12.042007907338164,
    "peak_process_count": 1,
    "peak_rss_bytes": 292835328,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n20\\cases\\pilot20_PC_seed106\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 601.2045562267303
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
