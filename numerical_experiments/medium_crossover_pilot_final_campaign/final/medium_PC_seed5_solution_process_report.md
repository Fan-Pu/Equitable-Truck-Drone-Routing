# medium_PC_seed5 Solution Process Report

- Snapshot SHA-256: `2669b56ab8619427d14711307aa7669d92a591d5149b1cc0b803ec659a8f7d49`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.12307615214868403` / `0.12307615214868403` / `0.0`
- Compact incumbent/bound/gap: `0.12729781997915976` / `-0.05044231043885464` / `1.3962543148587514`
- BPC runtime/nodes: `92.00721597671509` / `1`
- Compact runtime/nodes: `1800.5957214832306` / `155867.0`
- Compact first incumbent: `5.575000047683716` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 91.77751994132996,
  "root_compact_accepted_columns": 4,
  "root_compact_bound_full": -0.05463576290329203,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 4138988.0,
  "root_compact_node_count": 25538.0,
  "root_compact_objective_full": 0.12730263574006673,
  "root_compact_route_paths": [
    [
      "Source",
      "C8",
      "C12",
      "H2",
      "DUP:C4",
      "DUP:C20",
      "C7",
      "C11",
      "Sink"
    ],
    [
      "Source",
      "C16",
      "C19",
      "H1",
      "DUP:C1",
      "DUP:C15",
      "C17",
      "C13",
      "Sink"
    ],
    [
      "Source",
      "C10",
      "H3",
      "DUP:C14",
      "DUP:C18",
      "C9",
      "C3",
      "Sink"
    ],
    [
      "Source",
      "C6",
      "C5",
      "C2",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.12730263574006442,
  "root_incumbent_at_fathom_full": 0.12307615214868403,
  "root_lower_bound_full": 0.12307615214868403,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 4,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.17839406145847791,
  "columns_added_farkas": 0,
  "columns_added_standard": 505,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 509,
  "pricing_complete_routes_generated": 14528,
  "pricing_cpu_core_equivalent_max": 7.419449790558135,
  "pricing_extensions_attempted": 83590,
  "pricing_extensions_rejected_by_deadline": 21159,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 19562,
  "pricing_labels_generated": 62563,
  "pricing_labels_pruned": 8562,
  "pricing_labels_purged": 130,
  "pricing_negative_routes_inserted": 505,
  "pricing_negative_routes_verified": 741,
  "pricing_process_cpu_time": 58.0,
  "pricing_standard_bound_pruned": 8562,
  "standard_pricing_calls": 11,
  "total_routes": 509
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
  "queue_bound_fathoms": 1,
  "sr_cuts_added": 23,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 23
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
    "pricing_dynamic_split_rejected_small_frontier": 61,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 85,
    "pricing_leaf_tasks_created": 132
  },
  "workers": {
    "pricing_idle_work_requests": 281,
    "pricing_parallel_calls": 11,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 11,
    "pricing_pool_shutdown_time": 0.15766096115112305,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.462805032730103,
    "pricing_process_cpu_time": 58.0,
    "pricing_task_submission_time": 0.00500035285949707,
    "pricing_worker_busy_seconds": 59.1203134059906,
    "pricing_worker_idle_seconds": 152.74457502365112
  }
}
```

## Route Pool and Incumbent

```json
{
  "heuristic_calls": 1,
  "heuristic_full_pool_calls": 1,
  "heuristic_full_pool_feasible": 1,
  "heuristic_full_pool_incumbent_updates": 1,
  "heuristic_full_pool_time": 0.03351092338562012,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.03351092338562012,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.03451061248779297,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.15165424346924
}
```

## BPC Routes

- Route 2: truck `['Source', 'C10', 'H3', 'C9', 'C3', 'Sink']`; drones `{'H3': ['C14', 'C18']}`; return `130.84919073386146`; services `{'C10': 51.22988089357022, 'C14': 53.510262998445754, 'C18': 53.220529609713644, 'C9': 73.64620282900971, 'C3': 83.71612583787606}`
- Route 329: truck `['Source', 'C16', 'C19', 'C15', 'H3', 'C7', 'C11', 'H2', 'Sink']`; drones `{}`; return `130.988157437255`; services `{'C16': 23.45830144428035, 'C19': 47.96025912594017, 'C15': 52.16970498822729, 'C7': 82.71398142336236, 'C11': 84.50444693290027}`
- Route 293: truck `['Source', 'C2', 'H1', 'C17', 'C13', 'Sink']`; drones `{'H1': ['C1']}`; return `137.23915255967188`; services `{'C2': 48.58557478920343, 'C1': 69.0099421462602, 'C17': 79.08571944788034, 'C13': 80.27360002983912}`
- Route 267: truck `['Source', 'C8', 'C12', 'H2', 'C6', 'C5', 'Sink']`; drones `{'H2': ['C4', 'C20']}`; return `134.06576136693`; services `{'C8': 22.814078936819396, 'C12': 22.94445023984583, 'C4': 25.16661753656246, 'C20': 25.522862711475526, 'C6': 54.507110312104786, 'C5': 76.05157741648406}`

## Compact Routes

- Route 0: truck `['Source', 'H2', 'C10', 'C18', 'C11', 'Sink']`; drones `{'H2': ['C4', 'C20']}`; return `118.28344245865958`; services `{'C4': 25.166617536562455, 'C20': 25.522862711475522, 'C10': 55.45723478663231, 'C18': 57.518912082175845, 'C11': 71.79973195430482}`
- Route 1: truck `['Source', 'C16', 'C19', 'H1', 'C17', 'C13', 'Sink']`; drones `{'H1': ['C1', 'C15']}`; return `125.6800244425955`; services `{'C16': 23.45830144428035, 'C19': 47.96025912594017, 'C1': 57.450814029183796, 'C15': 56.625564488156215, 'C17': 67.52659133080394, 'C13': 68.71447191276272}`
- Route 2: truck `['Source', 'H3', 'C9', 'C3', 'Sink']`; drones `{'H3': ['C2', 'C14']}`; return `129.16413215427997`; services `{'C2': 51.60724646639646, 'C14': 51.38928851392866, 'C9': 71.96114424942822, 'C3': 82.03106725829457}`
- Route 3: truck `['Source', 'C8', 'C12', 'H2', 'C7', 'H1', 'C6', 'C5', 'Sink']`; drones `{}`; return `148.25410246679127`; services `{'C8': 22.814078936819396, 'C12': 22.94445023984583, 'C7': 48.27417601389268, 'C6': 68.69545141196606, 'C5': 90.23991851634534}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 689.234375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 7.394799072515325,
    "mean_cpu_percent_of_logical_machine": 61.62332560429437,
    "peak_interval_core_equivalent": 11.69181144643089,
    "peak_process_count": 13,
    "peak_rss_bytes": 863510528,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PC_seed5\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 93.20528769493103
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 18406.09375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 10.211925672871232,
    "mean_cpu_percent_of_logical_machine": 85.09938060726026,
    "peak_interval_core_equivalent": 11.86769778672571,
    "peak_process_count": 1,
    "peak_rss_bytes": 783237120,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\final\\cases\\medium_PC_seed5\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1802.411644935608
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 21 artifacts are stored in the JSON report.
