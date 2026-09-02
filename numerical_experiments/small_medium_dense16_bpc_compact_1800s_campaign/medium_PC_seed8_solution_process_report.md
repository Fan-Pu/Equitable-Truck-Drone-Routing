# medium_PC_seed8 Solution Process Report

- Snapshot SHA-256: `9e0738e301060e539165ee1e3a6fe14a62496a1e699f4bcc8a5203721c9eb298`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.17401421049025717` / `0.17401421049025717` / `0.0`
- Compact incumbent/bound/gap: `0.17401421049025614` / `0.1740142104902561` / `0.0`
- BPC runtime/nodes: `76.497225522995` / `1`
- Compact runtime/nodes: `117.7529718875885` / `183760.0`
- Compact first incumbent: `4.513000011444092` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 76.34205675125122,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": 0.13414975799316886,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 4042099.0,
  "root_compact_node_count": 42553.0,
  "root_compact_objective_full": 0.17401421049025975,
  "root_compact_route_paths": [
    [
      "Source",
      "C7",
      "C19",
      "Sink"
    ],
    [
      "Source",
      "C20",
      "C15",
      "H1",
      "C17",
      "H3",
      "DUP:C9",
      "Sink"
    ],
    [
      "Source",
      "C12",
      "H1",
      "DUP:C8",
      "C11",
      "C3",
      "Sink"
    ],
    [
      "Source",
      "C16",
      "C4",
      "C13",
      "C5",
      "C14",
      "C10",
      "H2",
      "DUP:C1",
      "DUP:C2",
      "DUP:C6",
      "DUP:C18",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.17401421049025717,
  "root_incumbent_at_fathom_full": 0.17401421049025717,
  "root_lower_bound_full": 0.17401421049025717,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.2578253273221749,
  "columns_added_farkas": 0,
  "columns_added_standard": 331,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 335,
  "pricing_complete_routes_generated": 6424,
  "pricing_cpu_core_equivalent_max": 7.455153067028398,
  "pricing_extensions_attempted": 24499,
  "pricing_extensions_rejected_by_deadline": 4429,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 3547,
  "pricing_labels_generated": 20190,
  "pricing_labels_pruned": 2055,
  "pricing_labels_purged": 12,
  "pricing_negative_routes_inserted": 331,
  "pricing_negative_routes_verified": 465,
  "pricing_process_cpu_time": 17.640625,
  "pricing_standard_bound_pruned": 2055,
  "standard_pricing_calls": 10,
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
  "sr_cuts_added": 24,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 24
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
    "pricing_leaf_tasks_closed": 89,
    "pricing_leaf_tasks_created": 120
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 10,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 10,
    "pricing_pool_shutdown_time": 0.13406610488891602,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.143465280532837,
    "pricing_process_cpu_time": 17.640625,
    "pricing_task_submission_time": 0.004002571105957031,
    "pricing_worker_busy_seconds": 18.117298126220703,
    "pricing_worker_idle_seconds": 17.187832593917847
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
  "time_to_first_incumbent": 60.26321530342102
}
```

## BPC Routes

- Route 2: truck `['Source', 'C12', 'H1', 'C11', 'C3', 'Sink']`; drones `{'H1': ['C8']}`; return `116.14286809494646`; services `{'C12': 31.842935327829743, 'C8': 48.33155086184988, 'C11': 65.68892976700135, 'C3': 68.9014742180848}`
- Route 3: truck `['Source', 'C16', 'C4', 'C13', 'C5', 'C14', 'C10', 'H2', 'Sink']`; drones `{'H2': ['C1', 'C2', 'C6', 'C18']}`; return `130.1425611874621`; services `{'C16': 27.40786333543598, 'C4': 31.8658641505453, 'C13': 54.972289084433854, 'C5': 58.075062529538684, 'C14': 68.52711803050543, 'C10': 72.13531123306423, 'C1': 81.50178381943347, 'C2': 76.22135028579129, 'C6': 75.487768264777, 'C18': 75.99886579202304}`
- Route 1: truck `['Source', 'C20', 'C15', 'H1', 'C17', 'H3', 'Sink']`; drones `{'H3': ['C9']}`; return `144.0327258946116`; services `{'C20': 29.28350793472729, 'C15': 48.166991702403415, 'C17': 92.96066345489244, 'C9': 94.05585561379606}`
- Route 0: truck `['Source', 'C7', 'C19', 'Sink']`; drones `{}`; return `93.3722019081132`; services `{'C7': 45.9435420671071, 'C19': 47.22886058155839}`

## Compact Routes

- Route 0: truck `['Source', 'C12', 'H1', 'C11', 'C3', 'Sink']`; drones `{'H1': ['C8']}`; return `116.14286809494646`; services `{'C12': 31.842935327829743, 'C8': 48.33155086184988, 'C11': 65.68892976700135, 'C3': 68.9014742180848}`
- Route 1: truck `['Source', 'C7', 'C19', 'Sink']`; drones `{}`; return `93.3722019081132`; services `{'C7': 45.9435420671071, 'C19': 47.22886058155839}`
- Route 2: truck `['Source', 'C20', 'C15', 'H1', 'C17', 'H3', 'Sink']`; drones `{'H3': ['C9']}`; return `144.0327258946116`; services `{'C20': 29.28350793472729, 'C15': 48.166991702403415, 'C17': 92.96066345489244, 'C9': 94.05585561379606}`
- Route 3: truck `['Source', 'C16', 'C4', 'C13', 'C5', 'C14', 'C10', 'H2', 'Sink']`; drones `{'H2': ['C1', 'C2', 'C6', 'C18']}`; return `130.1425611874621`; services `{'C16': 27.40786333543598, 'C4': 31.8658641505453, 'C13': 54.972289084433854, 'C5': 58.075062529538684, 'C14': 68.52711803050543, 'C10': 72.13531123306423, 'C1': 81.50178381943347, 'C2': 76.22135028579129, 'C6': 75.487768264777, 'C18': 75.99886579202304}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 611.234375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 7.784453091297034,
    "mean_cpu_percent_of_logical_machine": 64.87044242747528,
    "peak_interval_core_equivalent": 11.542765447096004,
    "peak_process_count": 13,
    "peak_rss_bytes": 812994560,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\medium_PC_seed8\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 78.51988673210144
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 974.4375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 8.186047906767808,
    "mean_cpu_percent_of_logical_machine": 68.21706588973174,
    "peak_interval_core_equivalent": 11.696288653272015,
    "peak_process_count": 1,
    "peak_rss_bytes": 403607552,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\medium_PC_seed8\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 119.03637886047363
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 21 artifacts are stored in the JSON report.
