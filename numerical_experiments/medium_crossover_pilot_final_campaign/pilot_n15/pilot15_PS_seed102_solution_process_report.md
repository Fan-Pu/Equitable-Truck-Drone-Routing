# pilot15_PS_seed102 Solution Process Report

- Snapshot SHA-256: `48666b7920a5165efa2e45b0272ee24993587037ba855b51e28c248786aa5f29`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.2578956985125339` / `0.2578956985125339` / `0.0`
- Compact incumbent/bound/gap: `0.2578956985125309` / `0.25789569851253086` / `0.0`
- BPC runtime/nodes: `13.722118616104126` / `1`
- Compact runtime/nodes: `1.6561331748962402` / `2310.0`
- Compact first incumbent: `0.19800019264221191` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 13.540900707244873,
  "root_compact_accepted_columns": 3,
  "root_compact_bound_full": 0.2578956985124594,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 123866.0,
  "root_compact_node_count": 2004.0,
  "root_compact_objective_full": 0.2578956985124595,
  "root_compact_route_paths": [
    [
      "Source",
      "H2",
      "DUP:C1",
      "DUP:C14",
      "C6",
      "H1",
      "DUP:C9",
      "C7",
      "C8",
      "Sink"
    ],
    [
      "Source",
      "C11",
      "H1",
      "DUP:C2",
      "DUP:C3",
      "DUP:C13",
      "C10",
      "C4",
      "Sink"
    ],
    [
      "Source",
      "C5",
      "H1",
      "C15",
      "H2",
      "C12",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.2578956985125339,
  "root_incumbent_at_fathom_full": 0.2578956985125339,
  "root_lower_bound_full": 0.2578956985125339,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 3,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.2700057508673361,
  "columns_added_farkas": 0,
  "columns_added_standard": 332,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 335,
  "pricing_complete_routes_generated": 2482,
  "pricing_cpu_core_equivalent_max": 5.001209776067981,
  "pricing_extensions_attempted": 11781,
  "pricing_extensions_rejected_by_deadline": 2312,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 1476,
  "pricing_labels_generated": 9553,
  "pricing_labels_pruned": 1587,
  "pricing_labels_purged": 23,
  "pricing_negative_routes_inserted": 332,
  "pricing_negative_routes_verified": 490,
  "pricing_process_cpu_time": 5.484375,
  "pricing_standard_bound_pruned": 1587,
  "standard_pricing_calls": 7,
  "total_routes": 335
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
    "pricing_balanced_process_dynamic_calls": 7,
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
    "pricing_leaf_tasks_closed": 53,
    "pricing_leaf_tasks_created": 84
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 7,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 7,
    "pricing_pool_shutdown_time": 0.16693973541259766,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.715631484985352,
    "pricing_process_cpu_time": 5.484375,
    "pricing_task_submission_time": 0.005126953125,
    "pricing_worker_busy_seconds": 5.625859022140503,
    "pricing_worker_idle_seconds": 8.68073320388794
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
  "time_to_first_incumbent": 1.604820728302002
}
```

## BPC Routes

- Route 1: truck `['Source', 'C11', 'H1', 'C10', 'C4', 'Sink']`; drones `{'H1': ['C2', 'C3', 'C13']}`; return `158.97407875249382`; services `{'C11': 39.95162752930735, 'C2': 62.08365940360864, 'C3': 60.90744017459578, 'C13': 60.52204961339903, 'C10': 87.0383229512836, 'C4': 104.36542780616337}`
- Route 2: truck `['Source', 'C5', 'H1', 'C15', 'H2', 'C12', 'Sink']`; drones `{}`; return `134.49149120853994`; services `{'C5': 46.339258187767456, 'C15': 73.34718473877408, 'C12': 109.48947453584606}`
- Route 0: truck `['Source', 'H2', 'C6', 'H1', 'C7', 'C8', 'Sink']`; drones `{'H2': ['C1', 'C14'], 'H1': ['C9']}`; return `209.3760891634533`; services `{'C1': 34.8790749228133, 'C14': 33.676370305559786, 'C6': 54.718354698979034, 'C9': 98.45316016902756, 'C7': 114.93702386574061, 'C8': 137.42780007668216}`

## Compact Routes

- Route 0: truck `['Source', 'H2', 'C6', 'H1', 'C7', 'C8', 'Sink']`; drones `{'H2': ['C1', 'C14'], 'H1': ['C9']}`; return `209.3760891634533`; services `{'C1': 34.8790749228133, 'C14': 33.676370305559786, 'C6': 54.718354698979034, 'C9': 98.45316016902756, 'C7': 114.93702386574061, 'C8': 137.42780007668216}`
- Route 1: truck `['Source', 'C5', 'H1', 'C15', 'H2', 'C12', 'Sink']`; drones `{}`; return `134.49149120853994`; services `{'C5': 46.339258187767456, 'C15': 73.34718473877408, 'C12': 109.48947453584606}`
- Route 2: truck `['Source', 'C11', 'H1', 'C10', 'C4', 'Sink']`; drones `{'H1': ['C2', 'C3', 'C13']}`; return `158.97407875249382`; services `{'C11': 39.95162752930735, 'C2': 62.08365940360864, 'C3': 60.90744017459578, 'C13': 60.52204961339903, 'C10': 87.0383229512836, 'C4': 104.36542780616337}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 25.609375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 1.6653243782807048,
    "mean_cpu_percent_of_logical_machine": 13.877703152339208,
    "peak_interval_core_equivalent": 3.9903035970624483,
    "peak_process_count": 13,
    "peak_rss_bytes": 748916736,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n15\\cases\\pilot15_PS_seed102\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 15.378010034561157
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 6.59375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.1219809877777474,
    "mean_cpu_percent_of_logical_machine": 17.683174898147897,
    "peak_interval_core_equivalent": 5.156100262099007,
    "peak_process_count": 1,
    "peak_rss_bytes": 92344320,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n15\\cases\\pilot15_PS_seed102\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 3.107355833053589
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
