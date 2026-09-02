# medium_PS_seed2 Solution Process Report

- Snapshot SHA-256: `92974c600c7ecb34827bbb2d172d04ab703dc5685d41ec34689e2366748edffc`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.16695675049344685` / `0.16695675049344685` / `0.0`
- Compact incumbent/bound/gap: `0.16695675049344788` / `0.10854232382496952` / `0.34987759701738325`
- BPC runtime/nodes: `85.32576751708984` / `1`
- Compact runtime/nodes: `1800.2789189815521` / `148340.0`
- Compact first incumbent: `9.65499997138977` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 85.16269612312317,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": -0.031229668953083704,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 5124453.0,
  "root_compact_node_count": 13758.0,
  "root_compact_objective_full": 0.16695675049344844,
  "root_compact_route_paths": [
    [
      "Source",
      "C13",
      "C2",
      "Sink"
    ],
    [
      "Source",
      "C6",
      "C10",
      "H1",
      "C15",
      "C16",
      "Sink"
    ],
    [
      "Source",
      "C1",
      "C5",
      "H3",
      "C7",
      "H2",
      "DUP:C12",
      "DUP:C20",
      "C9",
      "H1",
      "Sink"
    ],
    [
      "Source",
      "C4",
      "H1",
      "C18",
      "C19",
      "C14",
      "C8",
      "H3",
      "DUP:C3",
      "DUP:C11",
      "DUP:C17",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.16695675049344685,
  "root_incumbent_at_fathom_full": 0.16695675049344685,
  "root_lower_bound_full": 0.16695675049344685,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.12716795744264897,
  "columns_added_farkas": 0,
  "columns_added_standard": 495,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 499,
  "pricing_complete_routes_generated": 12955,
  "pricing_cpu_core_equivalent_max": 7.534857663669012,
  "pricing_extensions_attempted": 75989,
  "pricing_extensions_rejected_by_deadline": 23472,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 15433,
  "pricing_labels_generated": 52649,
  "pricing_labels_pruned": 6247,
  "pricing_labels_purged": 421,
  "pricing_negative_routes_inserted": 495,
  "pricing_negative_routes_verified": 680,
  "pricing_process_cpu_time": 48.234375,
  "pricing_standard_bound_pruned": 6247,
  "standard_pricing_calls": 11,
  "total_routes": 499
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
    "pricing_balanced_process_dynamic_calls": 11,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 27,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 97,
    "pricing_leaf_tasks_created": 132
  },
  "workers": {
    "pricing_idle_work_requests": 193,
    "pricing_parallel_calls": 11,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 11,
    "pricing_pool_shutdown_time": 0.14006972312927246,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.753051519393921,
    "pricing_process_cpu_time": 48.234375,
    "pricing_task_submission_time": 0.009511470794677734,
    "pricing_worker_busy_seconds": 50.6682231426239,
    "pricing_worker_idle_seconds": 95.84579586982727
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
  "time_to_first_incumbent": 60.159483194351196
}
```

## BPC Routes

- Route 2: truck `['Source', 'C1', 'C5', 'H3', 'C7', 'H2', 'C9', 'H1', 'Sink']`; drones `{'H2': ['C12', 'C20']}`; return `164.0652944229189`; services `{'C1': 21.003872912378988, 'C5': 34.965089354311154, 'C7': 44.70985446018467, 'C12': 78.27698258237308, 'C20': 68.78711481043771, 'C9': 102.1715153677951}`
- Route 0: truck `['Source', 'C13', 'C2', 'Sink']`; drones `{}`; return `130.38319778258284`; services `{'C13': 46.58863114208244, 'C2': 96.40288468023118}`
- Route 3: truck `['Source', 'C4', 'H1', 'C18', 'C19', 'C14', 'C8', 'H3', 'Sink']`; drones `{'H3': ['C3', 'C11', 'C17']}`; return `128.2581637654626`; services `{'C4': 9.114288776237686, 'C18': 39.411700008039524, 'C19': 51.21516982657292, 'C14': 56.68955606990653, 'C8': 60.765793135171904, 'C3': 70.48455712048617, 'C11': 72.15061242098426, 'C17': 74.71255834011266}`
- Route 1: truck `['Source', 'C6', 'C10', 'H1', 'C15', 'C16', 'Sink']`; drones `{}`; return `121.80933651238053`; services `{'C6': 26.712297228216155, 'C10': 34.5059648325348, 'C15': 48.2732099292946, 'C16': 81.07094446671513}`

## Compact Routes

- Route 0: truck `['Source', 'C13', 'C2', 'Sink']`; drones `{}`; return `130.38319778258284`; services `{'C13': 46.58863114208244, 'C2': 96.40288468023118}`
- Route 1: truck `['Source', 'C1', 'C5', 'H3', 'C7', 'H2', 'C9', 'Sink']`; drones `{'H2': ['C12', 'C20']}`; return `164.0652944229189`; services `{'C1': 21.003872912378988, 'C5': 34.965089354311154, 'C7': 44.70985446018467, 'C12': 78.27698258237308, 'C20': 68.78711481043771, 'C9': 102.1715153677951}`
- Route 2: truck `['Source', 'C4', 'H1', 'C18', 'C19', 'C14', 'C8', 'H3', 'Sink']`; drones `{'H3': ['C3', 'C11', 'C17']}`; return `128.2581637654626`; services `{'C4': 9.114288776237686, 'C18': 39.411700008039524, 'C19': 51.21516982657292, 'C14': 56.68955606990653, 'C8': 60.765793135171904, 'C3': 70.48455712048617, 'C11': 72.15061242098426, 'C17': 74.71255834011266}`
- Route 3: truck `['Source', 'C6', 'C10', 'H1', 'C15', 'C16', 'Sink']`; drones `{}`; return `121.80933651238053`; services `{'C6': 26.712297228216155, 'C10': 34.5059648325348, 'C15': 48.2732099292946, 'C16': 81.07094446671513}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 670.203125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 7.673543755322395,
    "mean_cpu_percent_of_logical_machine": 63.94619796101997,
    "peak_interval_core_equivalent": 11.473736864761076,
    "peak_process_count": 13,
    "peak_rss_bytes": 822919168,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PS_seed2\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 87.33945441246033
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 17851.59375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 9.907911770516975,
    "mean_cpu_percent_of_logical_machine": 82.56593142097479,
    "peak_interval_core_equivalent": 11.849535551112547,
    "peak_process_count": 1,
    "peak_rss_bytes": 686460928,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PS_seed2\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.7513844966888
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
