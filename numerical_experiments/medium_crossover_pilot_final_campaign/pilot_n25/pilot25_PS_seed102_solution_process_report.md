# pilot25_PS_seed102 Solution Process Report

- Snapshot SHA-256: `d4feed02e32ff1fda182b84f2a92ac758499fae90ac26ec1cf97729791c30561`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `True`
- BPC incumbent/bound/gap: `0.13624702769430905` / `0.13624702769430905` / `0.0`
- Compact incumbent/bound/gap: `0.1362470276943047` / `0.042666611403057476` / `0.6868437269781189`
- BPC runtime/nodes: `79.77784395217896` / `1`
- Compact runtime/nodes: `600.195240020752` / `47772.0`
- Compact first incumbent: `4.828999996185303` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 79.62475895881653,
  "root_compact_accepted_columns": 5,
  "root_compact_bound_full": 0.04179675604216648,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 3096841.0,
  "root_compact_node_count": 6252.0,
  "root_compact_objective_full": 0.13624702769430827,
  "root_compact_route_paths": [
    [
      "Source",
      "C9",
      "C17",
      "C7",
      "C21",
      "C4",
      "Sink"
    ],
    [
      "Source",
      "C22",
      "C18",
      "C11",
      "H2",
      "DUP:C3",
      "DUP:C5",
      "DUP:C19",
      "C13",
      "C24",
      "Sink"
    ],
    [
      "Source",
      "C14",
      "C20",
      "H1",
      "C25",
      "Sink"
    ],
    [
      "Source",
      "C12",
      "H2",
      "C15",
      "C16",
      "C8",
      "Sink"
    ],
    [
      "Source",
      "H1",
      "DUP:C6",
      "DUP:C23",
      "C1",
      "C10",
      "C2",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_existing_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.13624702769430905,
  "root_incumbent_at_fathom_full": 0.13624702769430905,
  "root_lower_bound_full": 0.13624702769430905,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 5,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.16855436357789863,
  "columns_added_farkas": 0,
  "columns_added_standard": 398,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 403,
  "pricing_complete_routes_generated": 5617,
  "pricing_cpu_core_equivalent_max": 5.625572921725734,
  "pricing_extensions_attempted": 32556,
  "pricing_extensions_rejected_by_deadline": 6473,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 8835,
  "pricing_labels_generated": 26203,
  "pricing_labels_pruned": 3531,
  "pricing_labels_purged": 333,
  "pricing_negative_routes_inserted": 398,
  "pricing_negative_routes_verified": 600,
  "pricing_process_cpu_time": 18.828125,
  "pricing_standard_bound_pruned": 3531,
  "standard_pricing_calls": 10,
  "total_routes": 403
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
    "pricing_balanced_process_dynamic_calls": 10,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 2,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 79,
    "pricing_leaf_tasks_created": 120
  },
  "workers": {
    "pricing_idle_work_requests": 22,
    "pricing_parallel_calls": 10,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 10,
    "pricing_pool_shutdown_time": 0.13206195831298828,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.446073770523071,
    "pricing_process_cpu_time": 18.828125,
    "pricing_task_submission_time": 0.017998695373535156,
    "pricing_worker_busy_seconds": 19.157766580581665,
    "pricing_worker_idle_seconds": 51.7398796081543
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
  "time_to_first_incumbent": 60.20242261886597
}
```

## BPC Routes

- Route 3: truck `['Source', 'C12', 'H2', 'C15', 'C16', 'C8', 'Sink']`; drones `{}`; return `167.87833864580324`; services `{'C12': 25.002016672693888, 'C15': 62.174779575967, 'C16': 80.8105029323031, 'C8': 95.93004955903211}`
- Route 2: truck `['Source', 'C14', 'C20', 'H1', 'C25', 'Sink']`; drones `{}`; return `93.57520270463202`; services `{'C14': 9.01109146904678, 'C20': 27.98150523891662, 'C25': 56.69679283295358}`
- Route 1: truck `['Source', 'C22', 'C18', 'C11', 'H2', 'C13', 'C24', 'Sink']`; drones `{'H2': ['C3', 'C5', 'C19']}`; return `171.16063159146336`; services `{'C22': 8.319530973751954, 'C18': 13.752572251734824, 'C11': 50.86343789125181, 'C3': 67.25987509486534, 'C5': 61.6097016267481, 'C19': 58.35033380877382, 'C13': 96.28455040256534, 'C24': 115.82047720302276}`
- Route 0: truck `['Source', 'C9', 'C17', 'C7', 'C21', 'C4', 'Sink']`; drones `{}`; return `158.2381307238331`; services `{'C9': 40.670780622447616, 'C17': 47.7858469328396, 'C7': 64.41425951835657, 'C21': 81.4463006126792, 'C4': 103.62947977750267}`
- Route 4: truck `['Source', 'H1', 'C1', 'C10', 'C2', 'Sink']`; drones `{'H1': ['C6', 'C23']}`; return `124.24294397311496`; services `{'C6': 18.87574054062652, 'C23': 22.08829128558024, 'C1': 36.004504798772295, 'C10': 57.37123712362033, 'C2': 82.20180595024712}`

## Compact Routes

- Route 0: truck `['Source', 'C14', 'C20', 'H1', 'C25', 'Sink']`; drones `{}`; return `93.57520270463202`; services `{'C14': 9.01109146904678, 'C20': 27.98150523891662, 'C25': 56.69679283295358}`
- Route 1: truck `['Source', 'C22', 'C18', 'C11', 'H2', 'C13', 'C24', 'Sink']`; drones `{'H2': ['C3', 'C5', 'C19']}`; return `171.16063159146336`; services `{'C22': 8.319530973751954, 'C18': 13.752572251734824, 'C11': 50.86343789125181, 'C3': 67.25987509486534, 'C5': 61.6097016267481, 'C19': 58.35033380877382, 'C13': 96.28455040256534, 'C24': 115.82047720302276}`
- Route 2: truck `['Source', 'C12', 'H2', 'C15', 'C16', 'C8', 'Sink']`; drones `{}`; return `167.87833864580324`; services `{'C12': 25.002016672693888, 'C15': 62.174779575967, 'C16': 80.8105029323031, 'C8': 95.93004955903211}`
- Route 3: truck `['Source', 'C9', 'C17', 'C7', 'C21', 'C4', 'Sink']`; drones `{}`; return `158.2381307238331`; services `{'C9': 40.670780622447616, 'C17': 47.7858469328396, 'C7': 64.41425951835657, 'C21': 81.4463006126792, 'C4': 103.62947977750267}`
- Route 4: truck `['Source', 'H1', 'C1', 'C10', 'C2', 'Sink']`; drones `{'H1': ['C6', 'C23']}`; return `124.24294397311496`; services `{'C6': 18.87574054062652, 'C23': 22.08829128558024, 'C1': 36.004504798772295, 'C10': 57.37123712362033, 'C2': 82.20180595024712}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 371.15625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.579191392758111,
    "mean_cpu_percent_of_logical_machine": 38.15992827298426,
    "peak_interval_core_equivalent": 6.037068425513099,
    "peak_process_count": 13,
    "peak_rss_bytes": 832557056,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n25\\cases\\pilot25_PS_seed102\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 81.0527925491333
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 3405.875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.658695118566425,
    "mean_cpu_percent_of_logical_machine": 47.15579265472021,
    "peak_interval_core_equivalent": 6.111576433700964,
    "peak_process_count": 1,
    "peak_rss_bytes": 324222976,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n25\\cases\\pilot25_PS_seed102\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 601.883460521698
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 19 artifacts are stored in the JSON report.
