# pilot15_PC_seed106 Solution Process Report

- Snapshot SHA-256: `074ef50bff4d7601b81ea05206300740dc286b87526978329ffd5e64c716d38e`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.12354930316176194` / `0.12354930316176194` / `0.0`
- Compact incumbent/bound/gap: `0.12354930316176493` / `0.12354930316176488` / `0.0`
- BPC runtime/nodes: `19.936265468597412` / `1`
- Compact runtime/nodes: `2.199769973754883` / `2682.0`
- Compact first incumbent: `0.29200005531311035` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 19.72028422355652,
  "root_compact_accepted_columns": 3,
  "root_compact_bound_full": 0.12354930316176255,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 130438.0,
  "root_compact_node_count": 1512.0,
  "root_compact_objective_full": 0.12354930316176255,
  "root_compact_route_paths": [
    [
      "Source",
      "C10",
      "C1",
      "C7",
      "H1",
      "DUP:C4",
      "DUP:C9",
      "DUP:C15",
      "C6",
      "Sink"
    ],
    [
      "Source",
      "C5",
      "H2",
      "DUP:C2",
      "DUP:C8",
      "DUP:C11",
      "C12",
      "Sink"
    ],
    [
      "Source",
      "C14",
      "C13",
      "C3",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.12354930316176194,
  "root_incumbent_at_fathom_full": 0.12354930316176194,
  "root_lower_bound_full": 0.12354930316176194,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 3,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.34329408400273864,
  "columns_added_farkas": 0,
  "columns_added_standard": 381,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 384,
  "pricing_complete_routes_generated": 7044,
  "pricing_cpu_core_equivalent_max": 4.574488090692222,
  "pricing_extensions_attempted": 29389,
  "pricing_extensions_rejected_by_deadline": 3910,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 5430,
  "pricing_labels_generated": 25647,
  "pricing_labels_pruned": 4120,
  "pricing_labels_purged": 4,
  "pricing_negative_routes_inserted": 381,
  "pricing_negative_routes_verified": 476,
  "pricing_process_cpu_time": 15.53125,
  "pricing_standard_bound_pruned": 4120,
  "standard_pricing_calls": 14,
  "total_routes": 384
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
  "sr_cuts_added": 12,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 12
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 14,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 4,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 148,
    "pricing_leaf_tasks_created": 168
  },
  "workers": {
    "pricing_idle_work_requests": 43,
    "pricing_parallel_calls": 14,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 14,
    "pricing_pool_shutdown_time": 0.18742680549621582,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.969565868377686,
    "pricing_process_cpu_time": 15.53125,
    "pricing_task_submission_time": 0.013511180877685547,
    "pricing_worker_busy_seconds": 16.59871506690979,
    "pricing_worker_idle_seconds": 56.251171350479126
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
  "time_to_first_incumbent": 1.537079095840454
}
```

## BPC Routes

- Route 0: truck `['Source', 'C10', 'C1', 'C7', 'H1', 'C6', 'Sink']`; drones `{'H1': ['C4', 'C9', 'C15']}`; return `95.0443616825811`; services `{'C10': 39.56239687037591, 'C1': 42.07794295131455, 'C7': 43.176542709426734, 'C4': 47.06222922052748, 'C9': 46.97334928936497, 'C15': 47.4401366890548, 'C6': 53.34350242568839}`
- Route 2: truck `['Source', 'C14', 'C13', 'C3', 'Sink']`; drones `{}`; return `116.01007541547057`; services `{'C14': 20.905396588408024, 'C13': 68.88529577022601, 'C3': 73.59202862225091}`
- Route 1: truck `['Source', 'C5', 'H2', 'C12', 'Sink']`; drones `{'H2': ['C2', 'C8', 'C11']}`; return `110.71635089762285`; services `{'C5': 20.772188984846384, 'C2': 22.224220310365208, 'C8': 23.064656439011202, 'C11': 22.196912735434594, 'C12': 71.42732901659869}`

## Compact Routes

- Route 0: truck `['Source', 'C5', 'H2', 'C12', 'Sink']`; drones `{'H2': ['C2', 'C8', 'C11']}`; return `110.71635089762285`; services `{'C5': 20.772188984846384, 'C2': 22.224220310365208, 'C8': 23.064656439011202, 'C11': 22.196912735434594, 'C12': 71.42732901659869}`
- Route 1: truck `['Source', 'C10', 'C1', 'C7', 'H1', 'C6', 'Sink']`; drones `{'H1': ['C4', 'C9', 'C15']}`; return `95.0443616825811`; services `{'C10': 39.56239687037591, 'C1': 42.07794295131455, 'C7': 43.176542709426734, 'C4': 47.06222922052748, 'C9': 46.97334928936497, 'C15': 47.4401366890548, 'C6': 53.34350242568839}`
- Route 2: truck `['Source', 'C14', 'C13', 'C3', 'Sink']`; drones `{}`; return `116.01007541547057`; services `{'C14': 20.905396588408024, 'C13': 68.88529577022601, 'C3': 73.59202862225091}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 37.65625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 1.7422303118972267,
    "mean_cpu_percent_of_logical_machine": 14.518585932476888,
    "peak_interval_core_equivalent": 4.583089429775838,
    "peak_process_count": 13,
    "peak_rss_bytes": 767864832,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n15\\cases\\pilot15_PC_seed106\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 21.613818645477295
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 11.671875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.8285697645502474,
    "mean_cpu_percent_of_logical_machine": 23.571414704585393,
    "peak_interval_core_equivalent": 5.76014462534177,
    "peak_process_count": 1,
    "peak_rss_bytes": 91885568,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n15\\cases\\pilot15_PC_seed106\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 4.126422882080078
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
