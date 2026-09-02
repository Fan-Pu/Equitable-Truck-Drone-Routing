# medium_PS_seed1 Solution Process Report

- Snapshot SHA-256: `86b50062b21084b7ae0ce1be71109af8b30d6215bd8063333ce8e1f77c375f7b`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.18622580647010026` / `0.18622580647010026` / `0.0`
- Compact incumbent/bound/gap: `0.18622580647009765` / `0.1862258064700977` / `0.0`
- BPC runtime/nodes: `75.39743137359619` / `1`
- Compact runtime/nodes: `63.30167365074158` / `23495.0`
- Compact first incumbent: `1.3980000019073486` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 75.17133831977844,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": 0.06984844481448592,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 5067106.0,
  "root_compact_node_count": 13771.0,
  "root_compact_objective_full": 0.18622580647010345,
  "root_compact_route_paths": [
    [
      "Source",
      "C5",
      "C14",
      "H3",
      "C17",
      "C18",
      "C15",
      "Sink"
    ],
    [
      "Source",
      "C10",
      "C3",
      "C2",
      "C9",
      "Sink"
    ],
    [
      "Source",
      "H2",
      "DUP:C1",
      "DUP:C12",
      "C7",
      "C19",
      "C13",
      "H3",
      "Sink"
    ],
    [
      "Source",
      "C16",
      "H3",
      "DUP:C6",
      "DUP:C8",
      "DUP:C20",
      "C4",
      "C11",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.18622580647010026,
  "root_incumbent_at_fathom_full": 0.18622580647010026,
  "root_lower_bound_full": 0.18622580647010026,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.2498346452295926,
  "columns_added_farkas": 0,
  "columns_added_standard": 341,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 345,
  "pricing_complete_routes_generated": 3722,
  "pricing_cpu_core_equivalent_max": 5.577294583209225,
  "pricing_extensions_attempted": 16905,
  "pricing_extensions_rejected_by_deadline": 3262,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 2651,
  "pricing_labels_generated": 13739,
  "pricing_labels_pruned": 1741,
  "pricing_labels_purged": 293,
  "pricing_negative_routes_inserted": 341,
  "pricing_negative_routes_verified": 461,
  "pricing_process_cpu_time": 8.671875,
  "pricing_standard_bound_pruned": 1741,
  "standard_pricing_calls": 8,
  "total_routes": 345
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
    "pricing_balanced_process_dynamic_calls": 8,
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
    "pricing_leaf_tasks_closed": 72,
    "pricing_leaf_tasks_created": 96
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 8,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 8,
    "pricing_pool_shutdown_time": 0.2050940990447998,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.922861099243164,
    "pricing_process_cpu_time": 8.671875,
    "pricing_task_submission_time": 0.0030024051666259766,
    "pricing_worker_busy_seconds": 8.895051956176758,
    "pricing_worker_idle_seconds": 29.69857096672058
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
  "time_to_first_incumbent": 60.16147208213806
}
```

## BPC Routes

- Route 1: truck `['Source', 'C10', 'C3', 'C2', 'C9', 'Sink']`; drones `{}`; return `94.53606743607969`; services `{'C10': 17.466321791924965, 'C3': 27.56842128686479, 'C2': 53.55572769144678, 'C9': 74.39276679754367}`
- Route 3: truck `['Source', 'C16', 'H3', 'C4', 'C11', 'Sink']`; drones `{'H3': ['C6', 'C8', 'C20']}`; return `122.45917100337871`; services `{'C16': 23.69753242569934, 'C6': 45.846369603870585, 'C8': 51.38328267129782, 'C20': 48.5691040645627, 'C4': 76.0753561212686, 'C11': 83.80516735527524}`
- Route 0: truck `['Source', 'C5', 'C14', 'H3', 'C17', 'C18', 'C15', 'Sink']`; drones `{}`; return `123.61688009228368`; services `{'C5': 21.643230034354794, 'C14': 31.30693964415208, 'C17': 56.617225056253645, 'C18': 62.283514241243566, 'C15': 81.22022676761145}`
- Route 2: truck `['Source', 'H2', 'C7', 'C19', 'C13', 'H3', 'Sink']`; drones `{'H2': ['C1', 'C12']}`; return `190.83850494690537`; services `{'C1': 53.48431517921158, 'C12': 53.30714902761254, 'C7': 65.03761461114203, 'C19': 85.66180011126474, 'C13': 127.59673740550585}`

## Compact Routes

- Route 0: truck `['Source', 'C10', 'C3', 'C2', 'C9', 'Sink']`; drones `{}`; return `94.53606743607969`; services `{'C10': 17.466321791924965, 'C3': 27.56842128686479, 'C2': 53.55572769144678, 'C9': 74.39276679754367}`
- Route 1: truck `['Source', 'C5', 'C14', 'H3', 'C17', 'C18', 'C15', 'Sink']`; drones `{}`; return `123.61688009228368`; services `{'C5': 21.643230034354794, 'C14': 31.30693964415208, 'C17': 56.617225056253645, 'C18': 62.283514241243566, 'C15': 81.22022676761145}`
- Route 2: truck `['Source', 'C16', 'H3', 'C4', 'C11', 'Sink']`; drones `{'H3': ['C6', 'C8', 'C20']}`; return `122.45917100337871`; services `{'C16': 23.69753242569934, 'C6': 45.846369603870585, 'C8': 51.38328267129782, 'C20': 48.5691040645627, 'C4': 76.0753561212686, 'C11': 83.80516735527524}`
- Route 3: truck `['Source', 'H2', 'C7', 'C19', 'C13', 'H1', 'Sink']`; drones `{'H2': ['C1', 'C12']}`; return `190.83850494690535`; services `{'C1': 53.48431517921158, 'C12': 53.30714902761254, 'C7': 65.03761461114203, 'C19': 85.66180011126474, 'C13': 127.59673740550585}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 589.046875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 7.644602938373556,
    "mean_cpu_percent_of_logical_machine": 63.7050244864463,
    "peak_interval_core_equivalent": 11.342596798467643,
    "peak_process_count": 13,
    "peak_rss_bytes": 799473664,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\medium_PS_seed1\\bpc\\attempt_002\\resource_samples.jsonl",
    "wall_runtime_seconds": 77.05395293235779
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 585.390625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 8.97586076459948,
    "mean_cpu_percent_of_logical_machine": 74.79883970499566,
    "peak_interval_core_equivalent": 11.120772683055396,
    "peak_process_count": 1,
    "peak_rss_bytes": 421466112,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\small_medium_dense16_bpc_compact_1800s_campaign\\cases\\medium_PS_seed1\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 65.21832728385925
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
