# large_PC_seed8 Solution Process Report

- Snapshot SHA-256: `3b956fae3aedf69d05b536111b80b7ac4c3995a6578c6e5c3082251bb0aaa27a`
- BPC status: `time_limit`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.08838164300260085` / `0.0852138484506155` / `0.03584222293640918`
- Compact incumbent/bound/gap: `0.09982045896943964` / `-0.09627316580686447` / `1.9644632653546383`
- BPC runtime/nodes: `1802.1561069488525` / `3`
- Compact runtime/nodes: `1800.2984471321106` / `27763.0`
- Compact first incumbent: `35.85899996757507` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`
- Warm-start-filtered realized seed: `24000080`
- Warm-start screening attempt: `24`
- Screening incumbent/objective/time: `True` / `0.17762609578711777` / `35.51300001144409` s
- Selection disclosure: this instance was filtered for a validated unseeded 60-second compact incumbent.
- Performance-selection attempt/arcs: `24` / `438`
- Performance trial BPC gap: `0.0363101676991207`
- Performance trial compact status code: `9`
- Performance disclosure: PC8 was selected for low BPC gap and direct-Gurobi non-optimality.

## BPC Root

```json
{
  "root_branch_required": true,
  "root_closed": true,
  "root_closure_time": 1042.413376569748,
  "root_compact_accepted_columns": 6,
  "root_compact_bound_full": -0.09547817273900586,
  "root_compact_incumbent_validated": true,
  "root_compact_iteration_count": 1365674.0,
  "root_compact_node_count": 2745.0,
  "root_compact_objective_full": 0.1044629435538997,
  "root_compact_route_paths": [
    [
      "Source",
      "C6",
      "C26",
      "C18",
      "H1",
      "C20",
      "Sink"
    ],
    [
      "Source",
      "C30",
      "C24",
      "C19",
      "Sink"
    ],
    [
      "Source",
      "C1",
      "C29",
      "C13",
      "C8",
      "C3",
      "C9",
      "H1",
      "Sink"
    ],
    [
      "Source",
      "C11",
      "C5",
      "H1",
      "DUP:C4",
      "DUP:C22",
      "DUP:C23",
      "DUP:C28",
      "C25",
      "Sink"
    ],
    [
      "Source",
      "C14",
      "C2",
      "C12",
      "H2",
      "C7",
      "C15",
      "Sink"
    ],
    [
      "Source",
      "C16",
      "C10",
      "C17",
      "H3",
      "DUP:C21",
      "DUP:C27",
      "Sink"
    ]
  ],
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": null,
  "root_fractional_variable_count": 9,
  "root_incumbent_at_classification_full": 0.10446294355390004,
  "root_incumbent_at_fathom_full": null,
  "root_lower_bound_full": 0.0852138484506155,
  "root_max_integrality_violation": 0.5,
  "root_nonzero_variable_count": 10,
  "root_rmp_is_integer": false
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.2636677314121436,
  "columns_added_farkas": 0,
  "columns_added_standard": 1270,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 1276,
  "pricing_complete_routes_generated": 478102,
  "pricing_cpu_core_equivalent_max": 8.796850086232144,
  "pricing_extensions_attempted": 3665899,
  "pricing_extensions_rejected_by_deadline": 1686411,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 583178,
  "pricing_labels_generated": 1980016,
  "pricing_labels_pruned": 359857,
  "pricing_labels_purged": 7591,
  "pricing_negative_routes_inserted": 1270,
  "pricing_negative_routes_verified": 1833,
  "pricing_process_cpu_time": 9239.265625,
  "pricing_standard_bound_pruned": 322789,
  "standard_pricing_calls": 44,
  "total_routes": 1276
}
```

## Cuts and Branching

```json
{
  "branching_nodes": 2,
  "child_nodes_created": 4,
  "conditioned_arc_branches": 0,
  "customer_pair_branches": 2,
  "launch_pad_branches": 0,
  "open_nodes_at_termination": 3,
  "postroot_nodes_processed": 2,
  "postroot_open_nodes": 3,
  "queue_bound_fathoms": 0,
  "sr_cuts_added": 107,
  "sr_cuts_added_postroot": 33,
  "sr_cuts_added_root": 74
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 44,
    "pricing_dynamic_bytes_transferred": 89933815,
    "pricing_dynamic_child_tasks_created": 1030,
    "pricing_dynamic_labels_transferred": 122581,
    "pricing_dynamic_split_candidates": 607,
    "pricing_dynamic_split_control_time": 3.766829252243042,
    "pricing_dynamic_split_rejected_elapsed": 126,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 4904,
    "pricing_dynamic_splits_performed": 237,
    "pricing_leaf_tasks_closed": 1402,
    "pricing_leaf_tasks_created": 1558
  },
  "workers": {
    "pricing_idle_work_requests": 5760,
    "pricing_parallel_calls": 44,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 44,
    "pricing_pool_shutdown_time": 1.8111789226531982,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 10.049805402755737,
    "pricing_process_cpu_time": 9239.265625,
    "pricing_task_submission_time": 0.013995170593261719,
    "pricing_worker_busy_seconds": 9429.534791707993,
    "pricing_worker_idle_seconds": 9483.01630973816
  }
}
```

## Route Pool and Incumbent

```json
{
  "heuristic_calls": 2,
  "heuristic_full_pool_calls": 2,
  "heuristic_full_pool_feasible": 2,
  "heuristic_full_pool_incumbent_updates": 1,
  "heuristic_full_pool_time": 0.12208294868469238,
  "heuristic_hard_pool_feasible_solves": 2,
  "heuristic_hard_pool_solves": 2,
  "heuristic_hard_pool_time": 0.12208294868469238,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.126084566116333,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.25328850746155
}
```

## BPC Routes

- Route 2: truck `['Source', 'C1', 'C29', 'C13', 'C8', 'C3', 'C9', 'H1', 'Sink']`; drones `{}`; return `163.00888261244967`; services `{'C1': 17.30680685294745, 'C29': 26.356454551511476, 'C13': 36.65595483863811, 'C8': 49.18732372905661, 'C3': 93.52242128979327, 'C9': 93.94035065128887}`
- Route 5: truck `['Source', 'C16', 'C10', 'C17', 'H3', 'Sink']`; drones `{'H3': ['C21', 'C27']}`; return `155.60832065558532`; services `{'C16': 28.61271091298661, 'C10': 32.1935085917778, 'C17': 41.40521212141621, 'C21': 84.38943596129242, 'C27': 82.54939311269847}`
- Route 482: truck `['Source', 'C18', 'H1', 'Sink']`; drones `{'H1': ['C4', 'C22', 'C23', 'C28']}`; return `70.68399500985618`; services `{'C18': 13.63529989090573, 'C4': 37.74598684847081, 'C22': 38.46210690294764, 'C23': 32.239484883611276, 'C28': 36.63046797896234}`
- Route 713: truck `['Source', 'C20', 'C14', 'C2', 'C12', 'C19', 'Sink']`; drones `{}`; return `70.44403633220921`; services `{'C20': 26.37973905946153, 'C14': 29.773619761381745, 'C2': 30.516874904845324, 'C12': 41.380889944967095, 'C19': 52.25702320429727}`
- Route 390: truck `['Source', 'C30', 'C24', 'C11', 'C5', 'H1', 'C25', 'Sink']`; drones `{}`; return `69.35937314677642`; services `{'C30': 16.26220237328547, 'C24': 18.4912234335372, 'C11': 40.73567654403577, 'C5': 42.56643637020657, 'C25': 52.664324050250585}`
- Route 776: truck `['Source', 'C6', 'C26', 'C7', 'C15', 'Sink']`; drones `{}`; return `170.12558888680587`; services `{'C6': 13.389253174004892, 'C26': 26.532857517527965, 'C7': 41.883757700248516, 'C15': 97.36353605569893}`

## Compact Routes

- Route 0: truck `['Source', 'C30', 'C24', 'C11', 'H1', 'C25', 'Sink']`; drones `{}`; return `69.3593731467764`; services `{'C30': 16.26220237328547, 'C24': 18.4912234335372, 'C11': 40.73567654403577, 'C25': 52.66432405025058}`
- Route 1: truck `['Source', 'C20', 'C14', 'C2', 'C19', 'C3', 'C9', 'Sink']`; drones `{}`; return `163.37668677048669`; services `{'C20': 26.37973905946153, 'C14': 29.773619761381745, 'C2': 30.516874904845324, 'C19': 43.20222403059849, 'C3': 93.89022544783032, 'C9': 94.30815480932591}`
- Route 2: truck `['Source', 'C18', 'H1', 'C5', 'C16', 'C10', 'Sink']`; drones `{'H1': ['C23']}`; return `89.58195491191395`; services `{'C18': 13.63529989090573, 'C23': 32.239484883611276, 'C5': 40.24482196404026, 'C16': 56.79690658478479, 'C10': 60.37770426357598}`
- Route 3: truck `['Source', 'C17', 'C13', 'C8', 'H3', 'Sink']`; drones `{'H3': ['C21', 'C27']}`; return `164.97689336523098`; services `{'C17': 28.2346608544167, 'C13': 34.54767207063376, 'C8': 47.07904096105226, 'C21': 93.75800867093805, 'C27': 91.9179658223441}`
- Route 4: truck `['Source', 'C12', 'H2', 'C1', 'C29', 'Sink']`; drones `{'H2': ['C4', 'C22', 'C28']}`; return `85.41093552872633`; services `{'C12': 15.401407774274245, 'C4': 30.555272157650496, 'C22': 28.03403615705522, 'C28': 30.199296450265656, 'C1': 50.00483327865082, 'C29': 59.05448097721485}`
- Route 5: truck `['Source', 'C6', 'C26', 'C7', 'C15', 'H1', 'Sink']`; drones `{}`; return `170.12558888680587`; services `{'C6': 13.389253174004892, 'C26': 26.532857517527965, 'C7': 41.883757700248516, 'C15': 97.36353605569893}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 9801.328125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.431940179629729,
    "mean_cpu_percent_of_logical_machine": 45.26616816358108,
    "peak_interval_core_equivalent": 11.160165547770017,
    "peak_process_count": 13,
    "peak_rss_bytes": 1840033792,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PC_seed8\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1804.3880825042725
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 8596.28125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.770864838262062,
    "mean_cpu_percent_of_logical_machine": 39.75720698551718,
    "peak_interval_core_equivalent": 6.0987557094511775,
    "peak_process_count": 1,
    "peak_rss_bytes": 501563392,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PC_seed8\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.8287127017975
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 29 artifacts are stored in the JSON report.
