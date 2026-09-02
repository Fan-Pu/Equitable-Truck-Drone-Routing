# pilot25_PS_seed101 Solution Process Report

- Snapshot SHA-256: `25622529e128a3d0eddd607134dd1c1859c618dc628802533c5aa3ea1a2da731`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.15517229135272745` / `0.15517229135272745` / `0.0`
- Compact incumbent/bound/gap: `0.16891499038941069` / `-0.0717278759516405` / `1.4246389014159229`
- BPC runtime/nodes: `160.43650364875793` / `1`
- Compact runtime/nodes: `600.186726808548` / `98859.0`
- Compact first incumbent: `7.691999912261963` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 160.2217779159546,
  "root_compact_accepted_columns": 5,
  "root_compact_bound_full": -0.10983650175957936,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 3659022.0,
  "root_compact_node_count": 9262.0,
  "root_compact_objective_full": 0.1985811810546099,
  "root_compact_route_paths": [
    [
      "Source",
      "C12",
      "C22",
      "C21",
      "C13",
      "C8",
      "H2",
      "DUP:C15",
      "Sink"
    ],
    [
      "Source",
      "C18",
      "C16",
      "H1",
      "C11",
      "H2",
      "C3",
      "Sink"
    ],
    [
      "Source",
      "H3",
      "DUP:C7",
      "DUP:C9",
      "C5",
      "C10",
      "C14",
      "C24",
      "H1",
      "Sink"
    ],
    [
      "Source",
      "C23",
      "H1",
      "DUP:C19",
      "C25",
      "Sink"
    ],
    [
      "Source",
      "C2",
      "C20",
      "C4",
      "C6",
      "H2",
      "DUP:C1",
      "DUP:C17",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.19858118105461023,
  "root_incumbent_at_fathom_full": 0.15517229135272745,
  "root_lower_bound_full": 0.1551722913527275,
  "root_max_integrality_violation": 4.440892098500626e-16,
  "root_nonzero_variable_count": 5,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.21065155685814196,
  "columns_added_farkas": 0,
  "columns_added_standard": 701,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 706,
  "pricing_complete_routes_generated": 36929,
  "pricing_cpu_core_equivalent_max": 8.598233637990841,
  "pricing_extensions_attempted": 211854,
  "pricing_extensions_rejected_by_deadline": 74056,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 36490,
  "pricing_labels_generated": 138002,
  "pricing_labels_pruned": 13046,
  "pricing_labels_purged": 272,
  "pricing_negative_routes_inserted": 701,
  "pricing_negative_routes_verified": 1187,
  "pricing_process_cpu_time": 234.546875,
  "pricing_standard_bound_pruned": 13046,
  "standard_pricing_calls": 17,
  "total_routes": 706
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
  "sr_cuts_added": 25,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 25
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 17,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 1,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 152,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 109,
    "pricing_leaf_tasks_created": 204
  },
  "workers": {
    "pricing_idle_work_requests": 853,
    "pricing_parallel_calls": 17,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 17,
    "pricing_pool_shutdown_time": 0.13527536392211914,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.316208124160767,
    "pricing_process_cpu_time": 234.546875,
    "pricing_task_submission_time": 0.006001949310302734,
    "pricing_worker_busy_seconds": 245.71455192565918,
    "pricing_worker_idle_seconds": 691.4075026512146
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
  "heuristic_full_pool_time": 0.030874013900756836,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.030874013900756836,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.031939029693603516,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.19181752204895
}
```

## BPC Routes

- Route 650: truck `['Source', 'C12', 'C22', 'C21', 'C13', 'C8', 'H2', 'Sink']`; drones `{}`; return `142.5784215075944`; services `{'C12': 11.460452035384144, 'C22': 28.64366229922677, 'C21': 34.895143519727455, 'C13': 76.32387966072884, 'C8': 87.51226924893925}`
- Route 540: truck `['Source', 'C18', 'C16', 'H1', 'C25', 'Sink']`; drones `{'H1': ['C19', 'C24']}`; return `106.40154145621727`; services `{'C18': 31.472772962746212, 'C16': 39.794257034903545, 'C19': 57.861376421046174, 'C24': 49.832645556266456, 'C25': 78.36765208527817}`
- Route 680: truck `['Source', 'C23', 'H1', 'C11', 'H2', 'Sink']`; drones `{'H2': ['C2', 'C15', 'C17']}`; return `151.78927918426655`; services `{'C23': 6.915331804500895, 'C11': 56.74420011301795, 'C2': 78.30380212707283, 'C15': 87.79457194502699, 'C17': 85.71218564021547}`
- Route 133: truck `['Source', 'C3', 'C6', 'C5', 'Sink']`; drones `{}`; return `142.19268757918562`; services `{'C3': 45.63953428574409, 'C6': 73.87691285599149, 'C5': 97.27889462788467}`
- Route 646: truck `['Source', 'H3', 'C10', 'C20', 'C4', 'C1', 'C14', 'Sink']`; drones `{'H3': ['C7', 'C9']}`; return `124.60532313232824`; services `{'C7': 28.66483675775804, 'C9': 26.7087261449756, 'C10': 53.105103269032966, 'C20': 57.35880420857201, 'C4': 64.44763531935014, 'C1': 67.40707358988517, 'C14': 81.933584982305}`

## Compact Routes

- Route 0: truck `['Source', 'H3', 'C10', 'C20', 'C4', 'C6', 'H2', 'Sink']`; drones `{'H3': ['C7'], 'H2': ['C15']}`; return `161.7013352193668`; services `{'C7': 28.66483675775804, 'C10': 53.105103269032966, 'C20': 57.35880420857201, 'C4': 64.44763531935014, 'C6': 78.71989088462897, 'C15': 97.70662798012722}`
- Route 1: truck `['Source', 'C21', 'C13', 'C8', 'C14', 'Sink']`; drones `{}`; return `128.47144632811043`; services `{'C21': 20.78816834024346, 'C13': 62.216904481244846, 'C8': 73.40529406945525, 'C14': 85.7997081780872}`
- Route 2: truck `['Source', 'C23', 'H1', 'C25', 'H3', 'C5', 'Sink']`; drones `{}`; return `135.57039053591416`; services `{'C23': 6.915331804500895, 'C25': 46.35592571206203, 'C5': 90.65659758461321}`
- Route 3: truck `['Source', 'C12', 'C22', 'C11', 'H2', 'C3', 'Sink']`; drones `{'H2': ['C1', 'C2', 'C17']}`; return `154.29759731958887`; services `{'C12': 11.460452035384144, 'C22': 28.64366229922677, 'C11': 34.54834814571316, 'C1': 60.993039893058686, 'C2': 56.10795015976804, 'C17': 63.516333672910676, 'C3': 108.6580630338448}`
- Route 4: truck `['Source', 'C18', 'C16', 'H1', 'Sink']`; drones `{'H1': ['C9', 'C19', 'C24']}`; return `109.56155091068317`; services `{'C18': 31.472772962746212, 'C16': 39.794257034903545, 'C9': 59.44138114827912, 'C19': 57.861376421046174, 'C24': 49.832645556266456}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 591.6875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 3.646667278069772,
    "mean_cpu_percent_of_logical_machine": 30.38889398391477,
    "peak_interval_core_equivalent": 10.150460874946596,
    "peak_process_count": 13,
    "peak_rss_bytes": 983748608,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n25\\cases\\pilot25_PS_seed101\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 162.254314661026
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 3088.234375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.134359905928936,
    "mean_cpu_percent_of_logical_machine": 42.7863325494078,
    "peak_interval_core_equivalent": 6.0548366354445955,
    "peak_process_count": 1,
    "peak_rss_bytes": 268492800,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\medium_crossover_pilot_final_campaign\\pilot_n25\\cases\\pilot25_PS_seed101\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 601.4838132858276
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 21 artifacts are stored in the JSON report.
