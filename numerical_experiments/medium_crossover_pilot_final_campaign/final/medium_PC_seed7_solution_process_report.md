# medium_PC_seed7 Solution Process Report

- Snapshot SHA-256: `bbec1f75fd8798798c6dc1a3991f3520c2213a6b8e1496740b58a4713670afec`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.12049387179514157` / `0.12049387179514157` / `0.0`
- Compact incumbent/bound/gap: `0.1204938717951416` / `0.12049387179514162` / `0.0`
- BPC runtime/nodes: `1414.8018572330475` / `91`
- Compact runtime/nodes: `287.4352593421936` / `160916.0`
- Compact first incumbent: `0.2910001277923584` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": true,
  "root_closed": true,
  "root_closure_time": 284.53332328796387,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": 0.013847624084902845,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 5744553.0,
  "root_compact_node_count": 27919.0,
  "root_compact_objective_full": 0.12059477493967907,
  "root_compact_route_paths": [
    [
      "Source",
      "C14",
      "H2",
      "C20",
      "C12",
      "H1",
      "DUP:C1",
      "DUP:C5",
      "DUP:C8",
      "DUP:C17",
      "C4",
      "Sink"
    ],
    [
      "Source",
      "C6",
      "C19",
      "C9",
      "H1",
      "DUP:C13",
      "Sink"
    ],
    [
      "Source",
      "C2",
      "C16",
      "C11",
      "H2",
      "Sink"
    ],
    [
      "Source",
      "C10",
      "C18",
      "C3",
      "C7",
      "H3",
      "DUP:C15",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": null,
  "root_fractional_variable_count": 17,
  "root_incumbent_at_classification_full": 0.12059477493967927,
  "root_incumbent_at_fathom_full": null,
  "root_lower_bound_full": 0.10625964831274712,
  "root_max_integrality_violation": 0.375,
  "root_nonzero_variable_count": 17,
  "root_rmp_is_integer": false
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.12701524375322862,
  "columns_added_farkas": 0,
  "columns_added_standard": 2412,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 2416,
  "pricing_complete_routes_generated": 361973,
  "pricing_cpu_core_equivalent_max": 8.204505253975817,
  "pricing_extensions_attempted": 2204884,
  "pricing_extensions_rejected_by_deadline": 382453,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 417479,
  "pricing_labels_generated": 1826283,
  "pricing_labels_pruned": 232610,
  "pricing_labels_purged": 1279,
  "pricing_negative_routes_inserted": 2412,
  "pricing_negative_routes_verified": 2782,
  "pricing_process_cpu_time": 2776.53125,
  "pricing_standard_bound_pruned": 125558,
  "standard_pricing_calls": 321,
  "total_routes": 2416
}
```

## Cuts and Branching

```json
{
  "branching_nodes": 45,
  "child_nodes_created": 90,
  "conditioned_arc_branches": 0,
  "customer_pair_branches": 45,
  "launch_pad_branches": 0,
  "open_nodes_at_termination": 0,
  "postroot_nodes_processed": 90,
  "postroot_open_nodes": 0,
  "queue_bound_fathoms": 0,
  "sr_cuts_added": 371,
  "sr_cuts_added_postroot": 281,
  "sr_cuts_added_root": 90
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 321,
    "pricing_dynamic_bytes_transferred": 1030545,
    "pricing_dynamic_child_tasks_created": 43,
    "pricing_dynamic_labels_transferred": 1133,
    "pricing_dynamic_split_candidates": 4,
    "pricing_dynamic_split_control_time": 0.014999866485595703,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 1139,
    "pricing_dynamic_splits_performed": 4,
    "pricing_leaf_tasks_closed": 3817,
    "pricing_leaf_tasks_created": 3895
  },
  "workers": {
    "pricing_idle_work_requests": 6736,
    "pricing_parallel_calls": 321,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 321,
    "pricing_pool_shutdown_time": 0.14405536651611328,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.424630641937256,
    "pricing_process_cpu_time": 2776.53125,
    "pricing_task_submission_time": 0.0325322151184082,
    "pricing_worker_busy_seconds": 2818.870394706726,
    "pricing_worker_idle_seconds": 7457.403237819672
  }
}
```

## Route Pool and Incumbent

```json
{
  "heuristic_calls": 45,
  "heuristic_full_pool_calls": 45,
  "heuristic_full_pool_feasible": 45,
  "heuristic_full_pool_incumbent_updates": 2,
  "heuristic_full_pool_time": 1.8443174362182617,
  "heuristic_hard_pool_feasible_solves": 45,
  "heuristic_hard_pool_solves": 45,
  "heuristic_hard_pool_time": 1.8443174362182617,
  "heuristic_incumbent_updates": 2,
  "heuristic_time": 1.9664254188537598,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.13476085662842
}
```

## BPC Routes

- Route 3: truck `['Source', 'C10', 'C18', 'C3', 'C7', 'H3', 'Sink']`; drones `{'H3': ['C15']}`; return `95.66570720661127`; services `{'C10': 17.15915393513894, 'C18': 19.039128965855646, 'C3': 45.83144384040469, 'C7': 48.38932381294432, 'C15': 52.27070860656198}`
- Route 267: truck `['Source', 'C14', 'C2', 'C16', 'C11', 'Sink']`; drones `{}`; return `106.01935841024911`; services `{'C14': 14.059585798152785, 'C2': 15.497652611428592, 'C16': 39.699852615266764, 'C11': 61.90497319898587}`
- Route 1: truck `['Source', 'C6', 'C19', 'C9', 'H1', 'Sink']`; drones `{'H1': ['C13']}`; return `128.82784007993814`; services `{'C6': 15.798707769184881, 'C19': 42.35536171746512, 'C9': 65.43771854846308, 'C13': 77.3649743935416}`
- Route 1533: truck `['Source', 'H2', 'C20', 'C12', 'H1', 'C4', 'Sink']`; drones `{'H1': ['C1', 'C5', 'C8', 'C17']}`; return `108.2748504016682`; services `{'C20': 37.516377955210594, 'C12': 38.8250930396014, 'C1': 55.18135218074575, 'C5': 54.97668193612738, 'C8': 54.10465142581406, 'C17': 54.90508381790464, 'C4': 68.51315639349208}`

## Compact Routes

- Route 0: truck `['Source', 'C14', 'C2', 'C16', 'C11', 'Sink']`; drones `{}`; return `106.01935841024911`; services `{'C14': 14.059585798152785, 'C2': 15.497652611428592, 'C16': 39.699852615266764, 'C11': 61.90497319898587}`
- Route 1: truck `['Source', 'H2', 'C10', 'C18', 'C3', 'C7', 'H3', 'Sink']`; drones `{'H3': ['C15']}`; return `95.66570720661127`; services `{'C10': 17.15915393513894, 'C18': 19.039128965855646, 'C3': 45.83144384040469, 'C7': 48.38932381294432, 'C15': 52.27070860656198}`
- Route 2: truck `['Source', 'C6', 'C19', 'C9', 'H1', 'Sink']`; drones `{'H1': ['C13']}`; return `128.82784007993814`; services `{'C6': 15.798707769184881, 'C19': 42.35536171746512, 'C9': 65.43771854846308, 'C13': 77.3649743935416}`
- Route 3: truck `['Source', 'H2', 'C20', 'C12', 'H1', 'C4', 'Sink']`; drones `{'H1': ['C1', 'C5', 'C8', 'C17']}`; return `108.2748504016682`; services `{'C20': 37.516377955210594, 'C12': 38.8250930396014, 'C1': 55.18135218074575, 'C5': 54.97668193612738, 'C8': 54.10465142581406, 'C17': 54.90508381790464, 'C4': 68.51315639349208}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 3954.375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.790521553965063,
    "mean_cpu_percent_of_logical_machine": 23.254346283042196,
    "peak_interval_core_equivalent": 11.72075221990718,
    "peak_process_count": 13,
    "peak_rss_bytes": 1329397760,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PC_seed7\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1417.0738062858582
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 2961.6875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 10.25780275876251,
    "mean_cpu_percent_of_logical_machine": 85.48168965635426,
    "peak_interval_core_equivalent": 11.66309289128705,
    "peak_process_count": 1,
    "peak_rss_bytes": 344977408,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PC_seed7\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 288.7253313064575
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 209 artifacts are stored in the JSON report.
