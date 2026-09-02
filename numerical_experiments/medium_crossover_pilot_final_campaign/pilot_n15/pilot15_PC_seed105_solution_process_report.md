# pilot15_PC_seed105 Solution Process Report

- Snapshot SHA-256: `600d8b4b560d4238098a19d0dc952bee0b90401684dfaf5d604c03c67c603b67`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `success`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.10315318253050698` / `0.10315318253050698` / `0.0`
- Compact incumbent/bound/gap: `0.1031531825305187` / `0.10315318253051871` / `0.0`
- BPC runtime/nodes: `10.829227924346924` / `1`
- Compact runtime/nodes: `0.27846527099609375` / `100.0`
- Compact first incumbent: `0.0559999942779541` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 10.649163246154785,
  "root_compact_accepted_columns": 3,
  "root_compact_bound_full": 0.10315318253052873,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 4979.0,
  "root_compact_node_count": 110.0,
  "root_compact_objective_full": 0.10315318253052869,
  "root_compact_route_paths": [
    [
      "Source",
      "C8",
      "C5",
      "C14",
      "C12",
      "C3",
      "H2",
      "DUP:C4",
      "DUP:C7",
      "DUP:C13",
      "C10",
      "Sink"
    ],
    [
      "Source",
      "C15",
      "C6",
      "C1",
      "C9",
      "Sink"
    ],
    [
      "Source",
      "C11",
      "C2",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.10315318253050698,
  "root_incumbent_at_fathom_full": 0.10315318253050698,
  "root_lower_bound_full": 0.10315318253050698,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 3,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.25447863752242267,
  "columns_added_farkas": 0,
  "columns_added_standard": 72,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 75,
  "pricing_complete_routes_generated": 374,
  "pricing_cpu_core_equivalent_max": 6.696305715129085,
  "pricing_extensions_attempted": 1384,
  "pricing_extensions_rejected_by_deadline": 173,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 133,
  "pricing_labels_generated": 1283,
  "pricing_labels_pruned": 220,
  "pricing_labels_purged": 0,
  "pricing_negative_routes_inserted": 72,
  "pricing_negative_routes_verified": 72,
  "pricing_process_cpu_time": 0.375,
  "pricing_standard_bound_pruned": 220,
  "standard_pricing_calls": 6,
  "total_routes": 75
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
    "pricing_balanced_process_dynamic_calls": 6,
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
    "pricing_leaf_tasks_created": 72
  },
  "workers": {
    "pricing_idle_work_requests": 0,
    "pricing_parallel_calls": 6,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 6,
    "pricing_pool_shutdown_time": 0.16173267364501953,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.984846353530884,
    "pricing_process_cpu_time": 0.375,
    "pricing_task_submission_time": 0.008004188537597656,
    "pricing_worker_busy_seconds": 0.605410099029541,
    "pricing_worker_idle_seconds": 0.7530696392059326
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
  "time_to_first_incumbent": 0.28415799140930176
}
```

## BPC Routes

- Route 2: truck `['Source', 'C11', 'C2', 'Sink']`; drones `{}`; return `49.640423398995075`; services `{'C11': 22.64846144049215, 'C2': 26.07555476481515}`
- Route 1: truck `['Source', 'C15', 'C6', 'C1', 'C9', 'Sink']`; drones `{}`; return `121.52213905704357`; services `{'C15': 20.879099308260223, 'C6': 22.226710749599224, 'C1': 61.71434209521091, 'C9': 97.8573060325765}`
- Route 0: truck `['Source', 'C8', 'C5', 'C14', 'C12', 'C3', 'H2', 'C10', 'Sink']`; drones `{'H2': ['C4', 'C7', 'C13']}`; return `154.14377125515728`; services `{'C8': 21.754140368703663, 'C5': 23.690560046047246, 'C14': 27.712193904889787, 'C12': 48.92985740987891, 'C3': 50.438476060232745, 'C4': 90.89970906357387, 'C7': 90.55228213542898, 'C13': 90.08736688764276, 'C10': 93.13827052869146}`

## Compact Routes

- Route 0: truck `['Source', 'C8', 'C5', 'C14', 'C12', 'C3', 'H2', 'C10', 'Sink']`; drones `{'H2': ['C4', 'C7', 'C13']}`; return `154.14377125515728`; services `{'C8': 21.754140368703663, 'C5': 23.690560046047246, 'C14': 27.712193904889787, 'C12': 48.92985740987891, 'C3': 50.438476060232745, 'C4': 90.89970906357387, 'C7': 90.55228213542898, 'C13': 90.08736688764276, 'C10': 93.13827052869146}`
- Route 1: truck `['Source', 'C11', 'C2', 'Sink']`; drones `{}`; return `49.640423398995075`; services `{'C11': 22.64846144049215, 'C2': 26.07555476481515}`
- Route 2: truck `['Source', 'C15', 'C6', 'C1', 'C9', 'Sink']`; drones `{}`; return `121.52213905704357`; services `{'C15': 20.879099308260223, 'C6': 22.226710749599224, 'C1': 61.71434209521091, 'C9': 97.8573060325765}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 12.515625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 1.0155093914285436,
    "mean_cpu_percent_of_logical_machine": 8.462578261904529,
    "peak_interval_core_equivalent": 2.969191736136281,
    "peak_process_count": 13,
    "peak_rss_bytes": 714870784,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n15\\cases\\pilot15_PC_seed105\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 12.324479818344116
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 1.21875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 0.5868221114886797,
    "mean_cpu_percent_of_logical_machine": 4.890184262405664,
    "peak_interval_core_equivalent": 2.839514731369151,
    "peak_process_count": 1,
    "peak_rss_bytes": 75665408,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n15\\cases\\pilot15_PC_seed105\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 2.07686448097229
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
