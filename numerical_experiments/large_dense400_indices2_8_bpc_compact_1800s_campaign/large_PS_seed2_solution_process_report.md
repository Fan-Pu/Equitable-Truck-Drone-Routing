# large_PS_seed2 Solution Process Report

- Snapshot SHA-256: `a9d7720f4fd4173ac5d0c3c8b25a743916b2685646bdda71194e46b1861ba6a9`
- BPC status: `time_limit`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.11114377959701133` / `0.10913757591533318` / `0.018050525984920615`
- Compact incumbent/bound/gap: `0.11114377959701441` / `-0.09470982422080572` / `1.8521378754996907`
- BPC runtime/nodes: `1800.47678565979` / `7`
- Compact runtime/nodes: `1800.3193762302399` / `25650.0`
- Compact first incumbent: `91.8309998512268` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": true,
  "root_closed": true,
  "root_closure_time": 469.19060921669006,
  "root_compact_accepted_columns": 0,
  "root_compact_bound_full": -0.10017770696647402,
  "root_compact_iteration_count": 446853.0,
  "root_compact_node_count": 1554.0,
  "root_compact_objective_full": null,
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "timeout",
  "root_fathom_reason": null,
  "root_fractional_variable_count": 12,
  "root_incumbent_at_classification_full": null,
  "root_incumbent_at_fathom_full": null,
  "root_lower_bound_full": 0.10585283047762667,
  "root_max_integrality_violation": 0.3333333333333334,
  "root_nonzero_variable_count": 12,
  "root_rmp_is_integer": false
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -31.99999999999997,
  "columns_added_farkas": 380,
  "columns_added_standard": 1259,
  "farkas_pricing_calls": 24,
  "global_pool_routes": 1639,
  "pricing_complete_routes_generated": 382473,
  "pricing_cpu_core_equivalent_max": 8.676031940237106,
  "pricing_extensions_attempted": 3248665,
  "pricing_extensions_rejected_by_deadline": 1262510,
  "pricing_farkas_bound_pruned": 13773,
  "pricing_labels_dominated": 555741,
  "pricing_labels_generated": 1987127,
  "pricing_labels_pruned": 497015,
  "pricing_labels_purged": 9303,
  "pricing_negative_routes_inserted": 1639,
  "pricing_negative_routes_verified": 2305,
  "pricing_process_cpu_time": 6595.453125,
  "pricing_standard_bound_pruned": 360813,
  "standard_pricing_calls": 57,
  "total_routes": 1639
}
```

## Cuts and Branching

```json
{
  "branching_nodes": 5,
  "child_nodes_created": 10,
  "conditioned_arc_branches": 0,
  "customer_pair_branches": 5,
  "launch_pad_branches": 0,
  "open_nodes_at_termination": 5,
  "postroot_nodes_processed": 6,
  "postroot_open_nodes": 5,
  "queue_bound_fathoms": 0,
  "sr_cuts_added": 117,
  "sr_cuts_added_postroot": 80,
  "sr_cuts_added_root": 37
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 81,
    "pricing_dynamic_bytes_transferred": 27428367,
    "pricing_dynamic_child_tasks_created": 388,
    "pricing_dynamic_labels_transferred": 42811,
    "pricing_dynamic_split_candidates": 94,
    "pricing_dynamic_split_control_time": 0.9975385665893555,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 8410,
    "pricing_dynamic_splits_performed": 73,
    "pricing_leaf_tasks_closed": 1015,
    "pricing_leaf_tasks_created": 1360
  },
  "workers": {
    "pricing_idle_work_requests": 10151,
    "pricing_parallel_calls": 81,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 81,
    "pricing_pool_shutdown_time": 0.1537947654724121,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 10.273591041564941,
    "pricing_process_cpu_time": 6595.453125,
    "pricing_task_submission_time": 0.01251363754272461,
    "pricing_worker_busy_seconds": 6997.709321975708,
    "pricing_worker_idle_seconds": 9740.403007745743
  }
}
```

## Route Pool and Incumbent

```json
{
  "heuristic_calls": 5,
  "heuristic_full_pool_calls": 5,
  "heuristic_full_pool_feasible": 5,
  "heuristic_full_pool_incumbent_updates": 2,
  "heuristic_full_pool_time": 0.35762810707092285,
  "heuristic_hard_pool_feasible_solves": 5,
  "heuristic_hard_pool_solves": 5,
  "heuristic_hard_pool_time": 0.35762810707092285,
  "heuristic_incumbent_updates": 2,
  "heuristic_time": 0.3686213493347168,
  "incumbent_source": "route_pool_after_node_close",
  "time_to_first_incumbent": 469.3109209537506
}
```

## BPC Routes

- Route 1072: truck `['Source', 'C1', 'C23', 'H3', 'C20', 'Sink']`; drones `{'H3': ['C2', 'C6', 'C9', 'C30']}`; return `126.96412663179433`; services `{'C1': 21.003872912378988, 'C23': 38.68288149297313, 'C2': 54.103056109068206, 'C6': 54.064725538731345, 'C9': 56.413339695107915, 'C30': 57.40539988595236, 'C20': 78.20272096679108}`
- Route 28: truck `['Source', 'C11', 'C26', 'C13', 'C27', 'Sink']`; drones `{}`; return `169.52919540439768`; services `{'C11': 32.13849896779914, 'C26': 47.53690509913454, 'C13': 79.47149827648016, 'C27': 104.54479471244312}`
- Route 18: truck `['Source', 'C15', 'C10', 'Sink']`; drones `{}`; return `43.41580521000932`; services `{'C15': 7.9406575082448585, 'C10': 21.70790260500466}`
- Route 67: truck `['Source', 'C25', 'C16', 'C7', 'C14', 'C5', 'C28', 'Sink']`; drones `{}`; return `135.46444681021268`; services `{'C25': 5.224039947729214, 'C16': 40.73839204566539, 'C7': 57.45873148525702, 'C14': 59.81150541920752, 'C5': 68.52149335057089, 'C28': 84.51042540323621}`
- Route 1320: truck `['Source', 'C4', 'C24', 'C18', 'H3', 'C21', 'Sink']`; drones `{'H3': ['C22']}`; return `124.38136819400742`; services `{'C4': 9.114288776237686, 'C24': 27.714446060354188, 'C18': 39.411700008039524, 'C22': 49.04778703809451, 'C21': 67.9079788801613}`
- Route 1075: truck `['Source', 'C8', 'H1', 'C19', 'C17', 'C12', 'Sink']`; drones `{'H1': ['C3', 'C29']}`; return `149.76366372958125`; services `{'C8': 39.59884022040916, 'C3': 50.69933891786366, 'C29': 50.3683872168678, 'C19': 60.85604794148908, 'C17': 76.56952568716841, 'C12': 87.25966303058368}`

## Compact Routes

- Route 0: truck `['Source', 'C4', 'C24', 'C18', 'H3', 'C20', 'Sink']`; drones `{'H3': ['C22']}`; return `108.95740089632166`; services `{'C4': 9.114288776237686, 'C24': 27.714446060354188, 'C18': 39.411700008039524, 'C22': 49.04778703809451, 'C20': 60.19599523131841}`
- Route 1: truck `['Source', 'C8', 'H1', 'C19', 'C17', 'C12', 'Sink']`; drones `{'H1': ['C3', 'C29']}`; return `149.76366372958125`; services `{'C8': 39.59884022040916, 'C3': 50.69933891786366, 'C29': 50.3683872168678, 'C19': 60.85604794148908, 'C17': 76.56952568716841, 'C12': 87.25966303058368}`
- Route 2: truck `['Source', 'C15', 'C10', 'Sink']`; drones `{}`; return `43.41580521000932`; services `{'C15': 7.9406575082448585, 'C10': 21.70790260500466}`
- Route 3: truck `['Source', 'C25', 'C16', 'C7', 'C14', 'C5', 'C28', 'Sink']`; drones `{}`; return `135.46444681021268`; services `{'C25': 5.224039947729214, 'C16': 40.73839204566539, 'C7': 57.45873148525702, 'C14': 59.81150541920752, 'C5': 68.52149335057089, 'C28': 84.51042540323621}`
- Route 4: truck `['Source', 'C11', 'C26', 'C13', 'C27', 'Sink']`; drones `{}`; return `169.52919540439768`; services `{'C11': 32.13849896779914, 'C26': 47.53690509913454, 'C13': 79.47149827648016, 'C27': 104.54479471244312}`
- Route 5: truck `['Source', 'C1', 'C23', 'H3', 'C21', 'Sink']`; drones `{'H3': ['C2', 'C6', 'C9', 'C30']}`; return `142.38809392948008`; services `{'C1': 21.003872912378988, 'C23': 38.68288149297313, 'C2': 54.103056109068206, 'C6': 54.064725538731345, 'C9': 56.413339695107915, 'C30': 57.40539988595236, 'C21': 85.91470461563395}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 7201.375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 3.995354295443572,
    "mean_cpu_percent_of_logical_machine": 33.29461912869643,
    "peak_interval_core_equivalent": 11.121666852040624,
    "peak_process_count": 13,
    "peak_rss_bytes": 1691746304,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PS_seed2\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1802.43714761734
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 8691.984375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.823868939036912,
    "mean_cpu_percent_of_logical_machine": 40.19890782530761,
    "peak_interval_core_equivalent": 6.122376559772954,
    "peak_process_count": 1,
    "peak_rss_bytes": 484302848,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PS_seed2\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.869927406311
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 42 artifacts are stored in the JSON report.
