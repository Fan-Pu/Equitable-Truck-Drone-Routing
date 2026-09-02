# pilot25_PC_seed106 Solution Process Report

- Snapshot SHA-256: `143055618a46947242c940cbf81560f2c182a6a21dbb8fa1a0183768957ac696`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.1252904063560579` / `0.1252904063560579` / `0.0`
- Compact incumbent/bound/gap: `0.13118934476690625` / `-0.07781013220574236` / `1.5931132009538833`
- BPC runtime/nodes: `440.6856896877289` / `1`
- Compact runtime/nodes: `600.2208008766174` / `30897.0`
- Compact first incumbent: `42.60000014305115` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 440.4406967163086,
  "root_compact_accepted_columns": 5,
  "root_compact_bound_full": -0.09970849567060958,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 1222760.0,
  "root_compact_node_count": 3057.0,
  "root_compact_objective_full": 0.16275649220859512,
  "root_compact_route_paths": [
    [
      "Source",
      "C15",
      "C20",
      "C13",
      "H3",
      "DUP:C3",
      "DUP:C8",
      "DUP:C18",
      "C6",
      "C24",
      "Sink"
    ],
    [
      "Source",
      "C7",
      "C14",
      "C16",
      "H2",
      "Sink"
    ],
    [
      "Source",
      "C10",
      "H2",
      "DUP:C1",
      "DUP:C11",
      "DUP:C21",
      "C25",
      "C23",
      "Sink"
    ],
    [
      "Source",
      "C22",
      "C9",
      "C5",
      "C2",
      "Sink"
    ],
    [
      "Source",
      "C12",
      "C17",
      "H1",
      "DUP:C4",
      "C19",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.1627564922085947,
  "root_incumbent_at_fathom_full": 0.1252904063560579,
  "root_lower_bound_full": 0.1252904063560579,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 5,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.1892405232604052,
  "columns_added_farkas": 0,
  "columns_added_standard": 840,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 845,
  "pricing_complete_routes_generated": 155062,
  "pricing_cpu_core_equivalent_max": 9.508488031231767,
  "pricing_extensions_attempted": 995619,
  "pricing_extensions_rejected_by_deadline": 302610,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 278913,
  "pricing_labels_generated": 693225,
  "pricing_labels_pruned": 60945,
  "pricing_labels_purged": 6058,
  "pricing_negative_routes_inserted": 840,
  "pricing_negative_routes_verified": 1323,
  "pricing_process_cpu_time": 3004.40625,
  "pricing_standard_bound_pruned": 60945,
  "standard_pricing_calls": 18,
  "total_routes": 845
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
  "sr_cuts_added": 0,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 0
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 18,
    "pricing_dynamic_bytes_transferred": 5637065,
    "pricing_dynamic_child_tasks_created": 85,
    "pricing_dynamic_labels_transferred": 17011,
    "pricing_dynamic_split_candidates": 134,
    "pricing_dynamic_split_control_time": 0.11909008026123047,
    "pricing_dynamic_split_rejected_elapsed": 1,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 1775,
    "pricing_dynamic_splits_performed": 38,
    "pricing_leaf_tasks_closed": 165,
    "pricing_leaf_tasks_created": 301
  },
  "workers": {
    "pricing_idle_work_requests": 847,
    "pricing_parallel_calls": 18,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 18,
    "pricing_pool_shutdown_time": 0.15308403968811035,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.449516296386719,
    "pricing_process_cpu_time": 3004.40625,
    "pricing_task_submission_time": 0.004004955291748047,
    "pricing_worker_busy_seconds": 3201.9261322021484,
    "pricing_worker_idle_seconds": 1125.258303642273
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
  "heuristic_full_pool_time": 0.03837156295776367,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.03837156295776367,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.04036688804626465,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.202067852020264
}
```

## BPC Routes

- Route 289: truck `['Source', 'C10', 'C12', 'C17', 'H1', 'C19', 'Sink']`; drones `{}`; return `73.83350678743274`; services `{'C10': 13.841146519683662, 'C12': 36.47744224532835, 'C17': 38.56111275299211, 'C19': 55.19812151566041}`
- Route 724: truck `['Source', 'C15', 'C20', 'C5', 'C2', 'H3', 'C1', 'Sink']`; drones `{}`; return `128.3401853458131`; services `{'C15': 13.024356035416695, 'C20': 14.245589020963479, 'C5': 16.62832551918048, 'C2': 41.01638499939651, 'C1': 91.29333455637581}`
- Route 645: truck `['Source', 'C22', 'C9', 'C25', 'C23', 'Sink']`; drones `{}`; return `116.24680976756284`; services `{'C22': 23.752158851184227, 'C9': 40.88917479009338, 'C25': 46.93291577697498, 'C23': 74.16390253009068}`
- Route 829: truck `['Source', 'C4', 'C18', 'C13', 'H3', 'C6', 'C24', 'Sink']`; drones `{'H3': ['C3', 'C8', 'C11', 'C21']}`; return `111.15481359461033`; services `{'C4': 20.050942805778732, 'C18': 42.96402688491528, 'C13': 47.12337046017916, 'C3': 51.79081808616984, 'C8': 51.264606958451196, 'C11': 54.063417081069716, 'C21': 54.41944289654838, 'C6': 62.783289239454355, 'C24': 91.22528176096175}`
- Route 1: truck `['Source', 'C7', 'C14', 'C16', 'H2', 'Sink']`; drones `{}`; return `109.8396234982265`; services `{'C7': 22.34375467981016, 'C14': 41.15167377173866, 'C16': 71.25549060545495}`

## Compact Routes

- Route 0: truck `['Source', 'C10', 'C12', 'C17', 'H1', 'C19', 'Sink']`; drones `{}`; return `73.83350678743274`; services `{'C10': 13.841146519683662, 'C12': 36.47744224532835, 'C17': 38.56111275299211, 'C19': 55.19812151566041}`
- Route 1: truck `['Source', 'C22', 'C9', 'C5', 'C2', 'Sink']`; drones `{}`; return `91.8025878884064`; services `{'C22': 23.752158851184227, 'C9': 40.88917479009338, 'C5': 47.59897671324454, 'C2': 71.98703619346057}`
- Route 2: truck `['Source', 'C16', 'H2', 'C25', 'Sink']`; drones `{'H2': ['C1', 'C11', 'C21']}`; return `82.23551631496454`; services `{'C16': 38.450351052920425, 'C1': 40.13241417461846, 'C11': 39.47035896771617, 'C21': 39.01225848667711, 'C25': 67.38359583060807}`
- Route 3: truck `['Source', 'C7', 'C14', 'C4', 'C18', 'H3', 'C23', 'Sink']`; drones `{}`; return `110.3396032812089`; services `{'C7': 22.34375467981016, 'C14': 41.15167377173866, 'C4': 42.60950798832548, 'C18': 65.52259206746203, 'C23': 68.25669604373674}`
- Route 4: truck `['Source', 'C15', 'C20', 'C13', 'H3', 'C6', 'C24', 'Sink']`; drones `{'H3': ['C3', 'C8']}`; return `100.50568136579182`; services `{'C15': 13.024356035416695, 'C20': 14.245589020963479, 'C13': 41.73148785211772, 'C3': 46.398935478108406, 'C8': 45.87272435038976, 'C6': 52.134157010635846, 'C24': 80.57614953214323}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 3294.4375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 7.443638492458866,
    "mean_cpu_percent_of_logical_machine": 62.030320770490555,
    "peak_interval_core_equivalent": 11.445277431276699,
    "peak_process_count": 13,
    "peak_rss_bytes": 1074851840,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n25\\cases\\pilot25_PC_seed106\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 442.5842957496643
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 3207.703125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.329943768592342,
    "mean_cpu_percent_of_logical_machine": 44.41619807160285,
    "peak_interval_core_equivalent": 6.105335886057614,
    "peak_process_count": 1,
    "peak_rss_bytes": 320020480,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n25\\cases\\pilot25_PC_seed106\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 601.8268229961395
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
