# pilot25_PC_seed105 Solution Process Report

- Snapshot SHA-256: `69d62224ef0e59baeab01e96633d99edb9d945fc5d564dafd43e823f4f2b33b7`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.11433254414753699` / `0.11433254414753699` / `0.0`
- Compact incumbent/bound/gap: `0.11443803826097432` / `-0.1291111384552748` / `2.128218732313802`
- BPC runtime/nodes: `104.36322259902954` / `1`
- Compact runtime/nodes: `600.1904294490814` / `56352.0`
- Compact first incumbent: `159.97899985313416` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 104.12358117103577,
  "root_compact_accepted_columns": 5,
  "root_compact_bound_full": -0.13057804713451887,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 1945882.0,
  "root_compact_node_count": 5977.0,
  "root_compact_objective_full": 0.16164094651242308,
  "root_compact_route_paths": [
    [
      "Source",
      "C12",
      "H1",
      "DUP:C2",
      "C7",
      "C18",
      "H3",
      "C23",
      "Sink"
    ],
    [
      "Source",
      "C3",
      "C25",
      "C13",
      "C9",
      "C19",
      "H1",
      "Sink"
    ],
    [
      "Source",
      "C5",
      "C14",
      "H3",
      "DUP:C4",
      "DUP:C24",
      "C1",
      "Sink"
    ],
    [
      "Source",
      "C8",
      "H3",
      "C22",
      "H2",
      "DUP:C11",
      "DUP:C21",
      "C6",
      "C16",
      "Sink"
    ],
    [
      "Source",
      "C20",
      "C15",
      "H3",
      "C17",
      "C10",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.1616409465124228,
  "root_incumbent_at_fathom_full": 0.11433254414753699,
  "root_lower_bound_full": 0.11433254414753699,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 5,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.22133769989538743,
  "columns_added_farkas": 0,
  "columns_added_standard": 661,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 666,
  "pricing_complete_routes_generated": 28567,
  "pricing_cpu_core_equivalent_max": 9.119833773363707,
  "pricing_extensions_attempted": 170151,
  "pricing_extensions_rejected_by_deadline": 63131,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 31847,
  "pricing_labels_generated": 107200,
  "pricing_labels_pruned": 8669,
  "pricing_labels_purged": 297,
  "pricing_negative_routes_inserted": 661,
  "pricing_negative_routes_verified": 1046,
  "pricing_process_cpu_time": 106.28125,
  "pricing_standard_bound_pruned": 8669,
  "standard_pricing_calls": 15,
  "total_routes": 666
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
  "sr_cuts_added": 28,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 28
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 15,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 231,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 101,
    "pricing_leaf_tasks_created": 180
  },
  "workers": {
    "pricing_idle_work_requests": 480,
    "pricing_parallel_calls": 15,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 15,
    "pricing_pool_shutdown_time": 0.1440589427947998,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.578303098678589,
    "pricing_process_cpu_time": 106.28125,
    "pricing_task_submission_time": 0.0205228328704834,
    "pricing_worker_busy_seconds": 112.46205615997314,
    "pricing_worker_idle_seconds": 161.27607560157776
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
  "heuristic_full_pool_time": 0.04603075981140137,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.04603075981140137,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.047036170959472656,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.16686391830444
}
```

## BPC Routes

- Route 394: truck `['Source', 'C12', 'H1', 'C7', 'C18', 'H3', 'C17', 'Sink']`; drones `{'H1': ['C2']}`; return `107.84401446830825`; services `{'C12': 22.377107233149143, 'C2': 24.468474641866536, 'C7': 25.877962187683654, 'C18': 47.92202777987121, 'C17': 84.14897387635332}`
- Route 341: truck `['Source', 'C22', 'C20', 'C15', 'H3', 'C1', 'Sink']`; drones `{'H3': ['C4', 'C24']}`; return `159.4948609953357`; services `{'C22': 23.962398035187764, 'C20': 47.75711855416877, 'C15': 51.62065636293121, 'C4': 64.13993557796898, 'C24': 64.5035880893283, 'C1': 99.68706403350305}`
- Route 1: truck `['Source', 'C3', 'C25', 'C13', 'C9', 'C19', 'H1', 'Sink']`; drones `{}`; return `125.83237999411608`; services `{'C3': 21.457851635028092, 'C25': 32.25164431987171, 'C13': 40.70063195279496, 'C9': 71.36517762998129, 'C19': 73.70615983009968}`
- Route 84: truck `['Source', 'C5', 'C14', 'H3', 'C10', 'Sink']`; drones `{}`; return `103.36293729585591`; services `{'C5': 30.847680101386356, 'C14': 51.68146864792796, 'C10': 69.81077635435473}`
- Route 665: truck `['Source', 'C8', 'C23', 'H2', 'C6', 'C16', 'Sink']`; drones `{'H2': ['C11', 'C21']}`; return `125.5118677553497`; services `{'C8': 21.717442505449498, 'C23': 24.666180784669105, 'C11': 62.03767292259877, 'C21': 61.82836082030611, 'C6': 64.69978912206896, 'C16': 66.6029555929905}`

## Compact Routes

- Route 0: truck `['Source', 'C5', 'C14', 'H3', 'C1', 'Sink']`; drones `{}`; return `151.40117955120505`; services `{'C5': 30.847680101386356, 'C14': 51.68146864792796, 'C1': 91.5933825893724}`
- Route 1: truck `['Source', 'C12', 'H1', 'C7', 'C18', 'H3', 'C21', 'Sink']`; drones `{'H1': ['C2', 'C22']}`; return `147.88030246297959`; services `{'C12': 22.377107233149143, 'C2': 24.468474641866536, 'C22': 25.418399893151495, 'C7': 27.777812690253572, 'C18': 49.821878282441126, 'C21': 87.61623048947682}`
- Route 2: truck `['Source', 'C3', 'C17', 'H2', 'C6', 'C16', 'Sink']`; drones `{'H2': ['C11']}`; return `140.71507235580071`; services `{'C3': 21.457851635028092, 'C17': 39.90440617735672, 'C11': 77.2408775230498, 'C6': 79.90299372252, 'C16': 81.80616019344153}`
- Route 3: truck `['Source', 'C8', 'C25', 'C13', 'C9', 'C19', 'H1', 'Sink']`; drones `{}`; return `125.8323799941161`; services `{'C8': 21.717442505449498, 'C25': 32.251644319871716, 'C13': 40.700631952794964, 'C9': 71.3651776299813, 'C19': 73.7061598300997}`
- Route 4: truck `['Source', 'C20', 'C15', 'H3', 'C10', 'Sink']`; drones `{'H3': ['C4', 'C23', 'C24']}`; return `95.07710538776314`; services `{'C20': 31.377605201945332, 'C15': 35.24114301070777, 'C4': 47.76042222574556, 'C23': 45.71796416599075, 'C24': 48.12407473710487, 'C10': 61.52494444626195}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 462.46875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.36596119643637,
    "mean_cpu_percent_of_logical_machine": 36.38300997030308,
    "peak_interval_core_equivalent": 6.880855629713214,
    "peak_process_count": 13,
    "peak_rss_bytes": 976089088,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n25\\cases\\pilot25_PC_seed105\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 105.92598724365234
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 3322.625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.523721759816561,
    "mean_cpu_percent_of_logical_machine": 46.031014665138,
    "peak_interval_core_equivalent": 6.09348264857664,
    "peak_process_count": 1,
    "peak_rss_bytes": 277745664,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n25\\cases\\pilot25_PC_seed105\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 601.519255399704
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 21 artifacts are stored in the JSON report.
