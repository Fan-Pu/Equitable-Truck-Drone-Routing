# medium_PS_seed4 Solution Process Report

- Snapshot SHA-256: `cbabf6cafe5e414c176c261a5666f5a9c3870b2c10c4b9cd0d20a68582063e81`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.1860002876752752` / `0.1860002876752752` / `0.0`
- Compact incumbent/bound/gap: `0.18673016392812686` / `0.031223207129665725` / `0.8327896978568302`
- BPC runtime/nodes: `72.90974688529968` / `1`
- Compact runtime/nodes: `1800.2846839427948` / `557907.0`
- Compact first incumbent: `15.85700011253357` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 72.73467302322388,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": -0.0799742940689758,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 3386300.0,
  "root_compact_node_count": 6122.0,
  "root_compact_objective_full": 0.18600028767526863,
  "root_compact_route_paths": [
    [
      "Source",
      "C12",
      "C4",
      "C16",
      "H3",
      "DUP:C7",
      "Sink"
    ],
    [
      "Source",
      "C15",
      "H3",
      "C10",
      "C19",
      "H1",
      "DUP:C11",
      "C14",
      "Sink"
    ],
    [
      "Source",
      "C20",
      "H3",
      "C18",
      "C13",
      "C5",
      "C9",
      "Sink"
    ],
    [
      "Source",
      "C3",
      "C6",
      "H2",
      "DUP:C1",
      "DUP:C2",
      "DUP:C8",
      "C17",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.1860002876752752,
  "root_incumbent_at_fathom_full": 0.1860002876752752,
  "root_lower_bound_full": 0.1860002876752752,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.23259877897615303,
  "columns_added_farkas": 0,
  "columns_added_standard": 297,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 301,
  "pricing_complete_routes_generated": 4385,
  "pricing_cpu_core_equivalent_max": 8.01759639916096,
  "pricing_extensions_attempted": 33340,
  "pricing_extensions_rejected_by_deadline": 14057,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 6040,
  "pricing_labels_generated": 19355,
  "pricing_labels_pruned": 3013,
  "pricing_labels_purged": 63,
  "pricing_negative_routes_inserted": 297,
  "pricing_negative_routes_verified": 422,
  "pricing_process_cpu_time": 12.015625,
  "pricing_standard_bound_pruned": 3013,
  "standard_pricing_calls": 6,
  "total_routes": 301
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
    "pricing_dynamic_split_rejected_small_frontier": 3,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 48,
    "pricing_leaf_tasks_created": 72
  },
  "workers": {
    "pricing_idle_work_requests": 27,
    "pricing_parallel_calls": 6,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 6,
    "pricing_pool_shutdown_time": 0.15906953811645508,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.258567810058594,
    "pricing_process_cpu_time": 12.015625,
    "pricing_task_submission_time": 0.004000425338745117,
    "pricing_worker_busy_seconds": 12.182399272918701,
    "pricing_worker_idle_seconds": 9.880281209945679
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
  "time_to_first_incumbent": 60.13153100013733
}
```

## BPC Routes

- Route 0: truck `['Source', 'C12', 'C4', 'C16', 'H3', 'Sink']`; drones `{'H3': ['C7']}`; return `128.76486843563924`; services `{'C12': 29.964493965654245, 'C4': 36.61608836736884, 'C16': 64.60823758969481, 'C7': 83.50345130750011}`
- Route 1: truck `['Source', 'C15', 'H3', 'C10', 'C19', 'H1', 'C14', 'Sink']`; drones `{'H1': ['C11']}`; return `155.04022532697232`; services `{'C15': 21.51407049751765, 'C10': 54.939223131347866, 'C19': 69.22604614157812, 'C11': 85.40056576545881, 'C14': 100.33490749729756}`
- Route 2: truck `['Source', 'C20', 'H3', 'C18', 'C13', 'C5', 'C9', 'Sink']`; drones `{}`; return `190.67397289630648`; services `{'C20': 25.988210133652387, 'C18': 60.525717223773086, 'C13': 85.50780734298826, 'C5': 101.4111539258507, 'C9': 119.50050075574444}`
- Route 3: truck `['Source', 'C3', 'C6', 'H2', 'C17', 'Sink']`; drones `{'H2': ['C1', 'C2', 'C8']}`; return `161.27620309344508`; services `{'C3': 36.89409061396133, 'C6': 51.7263226333198, 'C1': 71.6369114115508, 'C2': 75.3223171523778, 'C8': 70.97495691361611, 'C17': 104.23409606578055}`

## Compact Routes

- Route 0: truck `['Source', 'C12', 'C4', 'C16', 'H3', 'C9', 'H1', 'Sink']`; drones `{}`; return `180.58897846048504`; services `{'C12': 29.964493965654245, 'C4': 36.61608836736884, 'C16': 64.60823758969481, 'C9': 109.41550631992301}`
- Route 1: truck `['Source', 'C13', 'H2', 'C6', 'Sink']`; drones `{'H2': ['C1', 'C2', 'C8']}`; return `140.55828023212752`; services `{'C13': 37.180778727805844, 'C1': 61.55055733892273, 'C2': 65.23596307974974, 'C8': 60.88860284098804, 'C6': 88.83195759880773}`
- Route 2: truck `['Source', 'C15', 'H3', 'C19', 'H1', 'C14', 'Sink']`; drones `{'H3': ['C7', 'C10', 'C18'], 'H1': ['C11']}`; return `156.62414915282616`; services `{'C15': 21.51407049751765, 'C7': 49.60616577023368, 'C10': 51.97905460807022, 'C18': 50.23233316955506, 'C19': 70.80996996743198, 'C11': 86.98448959131267, 'C14': 101.91883132315142}`
- Route 3: truck `['Source', 'C20', 'C17', 'C3', 'C5', 'Sink']`; drones `{}`; return `146.464283448743`; services `{'C20': 25.988210133652387, 'C17': 57.04210702766454, 'C3': 77.19012344136776, 'C5': 93.38015813807472}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 361.609375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.829996574099545,
    "mean_cpu_percent_of_logical_machine": 40.24997145082954,
    "peak_interval_core_equivalent": 6.042034938784025,
    "peak_process_count": 13,
    "peak_rss_bytes": 762580992,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PS_seed4\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 74.86741852760315
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 16993.78125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 9.432520529212994,
    "mean_cpu_percent_of_logical_machine": 78.60433774344162,
    "peak_interval_core_equivalent": 11.872241841689934,
    "peak_process_count": 1,
    "peak_rss_bytes": 392404992,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PS_seed4\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.6161425113678
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
