# pilot20_PS_seed102 Solution Process Report

- Snapshot SHA-256: `fc4d4a40e24dc36ebbd16bba0fa25bc66c369a6d874f1d428f54866e1cc1e1ad`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.14099099717812472` / `0.14099099717812472` / `0.0`
- Compact incumbent/bound/gap: `0.14099099717812383` / `0.14099099717812377` / `0.0`
- BPC runtime/nodes: `14.147133588790894` / `1`
- Compact runtime/nodes: `2.1843721866607666` / `1189.0`
- Compact first incumbent: `0.18199992179870605` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 13.99106216430664,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": 0.1409909965974546,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 127776.0,
  "root_compact_node_count": 1581.0,
  "root_compact_objective_full": 0.14099099659745462,
  "root_compact_route_paths": [
    [
      "Source",
      "C14",
      "C18",
      "H3",
      "DUP:C1",
      "Sink"
    ],
    [
      "Source",
      "C6",
      "H3",
      "C3",
      "C13",
      "C4",
      "C10",
      "C16",
      "Sink"
    ],
    [
      "Source",
      "C11",
      "H1",
      "DUP:C7",
      "DUP:C9",
      "DUP:C17",
      "C2",
      "C19",
      "C5",
      "H2",
      "DUP:C8",
      "Sink"
    ],
    [
      "Source",
      "C20",
      "C12",
      "C15",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.14099099717812472,
  "root_incumbent_at_fathom_full": 0.14099099717812472,
  "root_lower_bound_full": 0.14099099717812472,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.20556461133395937,
  "columns_added_farkas": 0,
  "columns_added_standard": 239,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 243,
  "pricing_complete_routes_generated": 1380,
  "pricing_cpu_core_equivalent_max": 4.807115023151332,
  "pricing_extensions_attempted": 7571,
  "pricing_extensions_rejected_by_deadline": 1448,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 1533,
  "pricing_labels_generated": 6207,
  "pricing_labels_pruned": 753,
  "pricing_labels_purged": 21,
  "pricing_negative_routes_inserted": 239,
  "pricing_negative_routes_verified": 339,
  "pricing_process_cpu_time": 2.53125,
  "pricing_standard_bound_pruned": 753,
  "standard_pricing_calls": 7,
  "total_routes": 243
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
    "pricing_leaf_tasks_closed": 63,
    "pricing_leaf_tasks_created": 84
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 7,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 7,
    "pricing_pool_shutdown_time": 0.13804149627685547,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.609459400177002,
    "pricing_process_cpu_time": 2.53125,
    "pricing_task_submission_time": 0.004998922348022461,
    "pricing_worker_busy_seconds": 2.6409049034118652,
    "pricing_worker_idle_seconds": 4.083014965057373
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
  "time_to_first_incumbent": 2.513679265975952
}
```

## BPC Routes

- Route 2: truck `['Source', 'C11', 'H1', 'C2', 'C19', 'C5', 'H2', 'Sink']`; drones `{'H1': ['C7', 'C9', 'C17'], 'H2': ['C8']}`; return `191.982564799371`; services `{'C11': 39.95162752930735, 'C7': 58.17738130045719, 'C9': 57.02042914885087, 'C17': 55.57026833171526, 'C2': 67.85648451159088, 'C19': 69.49713466807822, 'C5': 77.03486213337989, 'C8': 122.9253446396403}`
- Route 0: truck `['Source', 'C14', 'C18', 'H3', 'Sink']`; drones `{'H3': ['C1']}`; return `62.22910998961134`; services `{'C14': 9.01109146904678, 'C18': 17.636367503866815, 'C1': 33.05645262087166}`
- Route 3: truck `['Source', 'C20', 'C12', 'C15', 'Sink']`; drones `{}`; return `124.23184844624654`; services `{'C20': 27.98150523891662, 'C12': 66.23691718442892, 'C15': 86.03882934254054}`
- Route 1: truck `['Source', 'C6', 'H3', 'C3', 'C13', 'C4', 'C10', 'C16', 'Sink']`; drones `{}`; return `140.73961426897395`; services `{'C6': 17.05673180918458, 'C3': 43.079388069460684, 'C13': 49.352708362339534, 'C4': 55.21511726103019, 'C10': 72.54222211590995, 'C16': 83.91087180893184}`

## Compact Routes

- Route 0: truck `['Source', 'C14', 'C18', 'H3', 'Sink']`; drones `{'H3': ['C1']}`; return `62.22910998961134`; services `{'C14': 9.01109146904678, 'C18': 17.636367503866815, 'C1': 33.05645262087166}`
- Route 1: truck `['Source', 'C6', 'H3', 'C3', 'C13', 'C4', 'C10', 'C16', 'Sink']`; drones `{}`; return `140.73961426897395`; services `{'C6': 17.05673180918458, 'C3': 43.079388069460684, 'C13': 49.352708362339534, 'C4': 55.21511726103019, 'C10': 72.54222211590995, 'C16': 83.91087180893184}`
- Route 2: truck `['Source', 'C11', 'H1', 'C2', 'C19', 'C5', 'H2', 'Sink']`; drones `{'H1': ['C7', 'C9', 'C17'], 'H2': ['C8']}`; return `191.982564799371`; services `{'C11': 39.95162752930735, 'C7': 58.17738130045719, 'C9': 57.02042914885087, 'C17': 55.57026833171526, 'C2': 67.85648451159088, 'C19': 69.49713466807822, 'C5': 77.03486213337989, 'C8': 122.9253446396403}`
- Route 3: truck `['Source', 'C20', 'C12', 'C15', 'Sink']`; drones `{}`; return `124.23184844624654`; services `{'C20': 27.98150523891662, 'C12': 66.23691718442892, 'C15': 86.03882934254054}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 27.375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 1.7807610462243366,
    "mean_cpu_percent_of_logical_machine": 14.839675385202804,
    "peak_interval_core_equivalent": 5.6292616224176735,
    "peak_process_count": 13,
    "peak_rss_bytes": 745594880,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n20\\cases\\pilot20_PS_seed102\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 15.37264084815979
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 10.90625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.651995086773113,
    "mean_cpu_percent_of_logical_machine": 22.09995905644261,
    "peak_interval_core_equivalent": 5.625626981865898,
    "peak_process_count": 1,
    "peak_rss_bytes": 99512320,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n20\\cases\\pilot20_PS_seed102\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 4.112469911575317
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
