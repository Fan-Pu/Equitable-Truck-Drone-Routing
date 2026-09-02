# large_PC_seed6 Solution Process Report

- Snapshot SHA-256: `e2a1da8bdbd667b02a19098ba3bc8e83efdafec7ef8f184ffb38b64bf3b06b2e`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.12156239554140308` / `0.12156239554140308` / `0.0`
- Compact incumbent/bound/gap: `0.12665643063601523` / `-0.05368124808308637` / `1.423833577288747`
- BPC runtime/nodes: `150.24021673202515` / `1`
- Compact runtime/nodes: `1800.371325969696` / `23290.0`
- Compact first incumbent: `172.38299989700317` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`
- Warm-start-filtered realized seed: `12000042`
- Warm-start screening attempt: `12`
- Screening incumbent/objective/time: `True` / `0.19043795183290585` / `28.515000104904175` s
- Selection disclosure: this instance was filtered for a validated unseeded 60-second compact incumbent.

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 149.95796060562134,
  "root_compact_accepted_columns": 6,
  "root_compact_bound_full": -0.05892357971862948,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 1042052.0,
  "root_compact_node_count": 1729.0,
  "root_compact_objective_full": 0.21874899676851198,
  "root_compact_route_paths": [
    [
      "Source",
      "C11",
      "C13",
      "Sink"
    ],
    [
      "Source",
      "C26",
      "C22",
      "C24",
      "C10",
      "H2",
      "Sink"
    ],
    [
      "Source",
      "H2",
      "DUP:C8",
      "DUP:C29",
      "C18",
      "C1",
      "C17",
      "H3",
      "Sink"
    ],
    [
      "Source",
      "C2",
      "C12",
      "C6",
      "C20",
      "H2",
      "DUP:C5",
      "DUP:C23",
      "C21",
      "Sink"
    ],
    [
      "Source",
      "C4",
      "C15",
      "H3",
      "DUP:C9",
      "DUP:C16",
      "DUP:C27",
      "DUP:C28",
      "C30",
      "Sink"
    ],
    [
      "Source",
      "C14",
      "C3",
      "C25",
      "C19",
      "C7",
      "H1",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.21874899676851145,
  "root_incumbent_at_fathom_full": 0.12156239554140308,
  "root_lower_bound_full": 0.12156239554140308,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 5,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.17890533918320803,
  "columns_added_farkas": 0,
  "columns_added_standard": 1295,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 1301,
  "pricing_complete_routes_generated": 36164,
  "pricing_cpu_core_equivalent_max": 7.565561784781875,
  "pricing_extensions_attempted": 255365,
  "pricing_extensions_rejected_by_deadline": 74411,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 53753,
  "pricing_labels_generated": 181242,
  "pricing_labels_pruned": 22787,
  "pricing_labels_purged": 694,
  "pricing_negative_routes_inserted": 1295,
  "pricing_negative_routes_verified": 2056,
  "pricing_process_cpu_time": 267.34375,
  "pricing_standard_bound_pruned": 22787,
  "standard_pricing_calls": 24,
  "total_routes": 1301
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
    "pricing_balanced_process_dynamic_calls": 24,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 314,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 130,
    "pricing_leaf_tasks_created": 288
  },
  "workers": {
    "pricing_idle_work_requests": 748,
    "pricing_parallel_calls": 24,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 24,
    "pricing_pool_shutdown_time": 0.16679167747497559,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.889724254608154,
    "pricing_process_cpu_time": 267.34375,
    "pricing_task_submission_time": 0.02566361427307129,
    "pricing_worker_busy_seconds": 286.411758184433,
    "pricing_worker_idle_seconds": 331.66185307502747
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
  "heuristic_full_pool_time": 0.0442500114440918,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.0442500114440918,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.04624795913696289,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.22857737541199
}
```

## BPC Routes

- Route 1192: truck `['Source', 'C14', 'H2', 'C18', 'C4', 'C15', 'C30', 'Sink']`; drones `{'H2': ['C5', 'C8', 'C23', 'C29']}`; return `130.8887186457912`; services `{'C14': 16.68181629722679, 'C5': 27.14284717701247, 'C8': 28.40535486764453, 'C23': 28.426221988956755, 'C29': 28.835076526398094, 'C18': 58.02505304552895, 'C4': 66.89601365660289, 'C15': 71.08615768347497, 'C30': 82.56574226165142}`
- Route 828: truck `['Source', 'C17', 'H3', 'C3', 'C25', 'Sink']`; drones `{'H3': ['C9', 'C16', 'C27', 'C28']}`; return `159.68615834289804`; services `{'C17': 29.986187737747674, 'C9': 58.275738869496394, 'C16': 56.27879740679059, 'C27': 60.98235687581426, 'C28': 55.396615994470615, 'C3': 71.59592323683458, 'C25': 106.58045305261862}`
- Route 473: truck `['Source', 'C19', 'C7', 'H1', 'C11', 'C13', 'Sink']`; drones `{'H1': ['C1']}`; return `152.26253631178537`; services `{'C19': 51.06665669981557, 'C7': 51.27010442735112, 'C1': 53.47282285741211, 'C11': 76.42573815271018, 'C13': 99.26200397412892}`
- Route 839: truck `['Source', 'C2', 'C20', 'H2', 'C21', 'C12', 'C6', 'Sink']`; drones `{}`; return `126.30270764600444`; services `{'C2': 18.600725066796322, 'C20': 20.85090628556669, 'C21': 63.719393688649994, 'C12': 76.53096687876985, 'C6': 77.52257620321413}`
- Route 1: truck `['Source', 'C26', 'C22', 'C24', 'C10', 'H2', 'Sink']`; drones `{}`; return `120.9081744518734`; services `{'C26': 15.450083793192302, 'C22': 53.10915503047953, 'C24': 58.36202564147504, 'C10': 65.32720283517449}`

## Compact Routes

- Route 0: truck `['Source', 'C14', 'H2', 'C18', 'C4', 'C15', 'H3', 'C10', 'Sink']`; drones `{'H2': ['C5', 'C8', 'C23', 'C29']}`; return `133.96613923531316`; services `{'C14': 16.68181629722679, 'C5': 27.14284717701247, 'C8': 28.40535486764453, 'C23': 28.426221988956755, 'C29': 28.835076526398094, 'C18': 58.02505304552895, 'C4': 66.89601365660289, 'C15': 71.08615768347497, 'C10': 78.38516761861426}`
- Route 1: truck `['Source', 'C25', 'C19', 'C7', 'H1', 'C24', 'Sink']`; drones `{'H1': ['C1']}`; return `134.69953351885977`; services `{'C25': 53.10570529027941, 'C19': 55.14475388074325, 'C7': 55.348201608278806, 'C1': 57.5509200383398, 'C24': 86.08373909586031}`
- Route 2: truck `['Source', 'C2', 'C11', 'C13', 'Sink']`; drones `{}`; return `106.00106467531288`; services `{'C2': 18.600725066796322, 'C11': 30.164266516237696, 'C13': 53.00053233765643}`
- Route 3: truck `['Source', 'C26', 'C22', 'H3', 'C30', 'Sink']`; drones `{'H3': ['C9', 'C16', 'C27', 'C28']}`; return `121.96471375162852`; services `{'C26': 15.450083793192302, 'C22': 53.10915503047953, 'C9': 58.2757388694964, 'C16': 56.278797406790595, 'C27': 60.98235687581426, 'C28': 55.39661599447062, 'C30': 73.64173736748874}`
- Route 4: truck `['Source', 'C20', 'H2', 'C21', 'C12', 'C6', 'C17', 'H3', 'C3', 'Sink']`; drones `{}`; return `180.80424644675065`; services `{'C20': 17.731607905382585, 'C21': 60.60009530846589, 'C12': 73.41166849858574, 'C6': 74.40327782303002, 'C17': 93.19722152807266, 'C3': 122.00764011853782}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 563.984375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 3.703952472274313,
    "mean_cpu_percent_of_logical_machine": 30.86627060228594,
    "peak_interval_core_equivalent": 9.780127574527073,
    "peak_process_count": 13,
    "peak_rss_bytes": 1420156928,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PC_seed6\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 152.26555395126343
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 8741.203125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.850620271203603,
    "mean_cpu_percent_of_logical_machine": 40.421835593363355,
    "peak_interval_core_equivalent": 6.086067110401017,
    "peak_process_count": 1,
    "peak_rss_bytes": 475713536,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PC_seed6\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1802.0794529914856
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
