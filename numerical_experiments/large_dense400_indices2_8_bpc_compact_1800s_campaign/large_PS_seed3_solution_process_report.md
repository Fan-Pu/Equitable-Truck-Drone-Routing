# large_PS_seed3 Solution Process Report

- Snapshot SHA-256: `00baf6b5728220336f959d34439b30f135500ec874f82add80996191dfbaade9`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.12710178389894503` / `0.12710178389894503` / `0.0`
- Compact incumbent/bound/gap: `0.13362235679159676` / `-0.06898869143845564` / `1.5162960233223053`
- BPC runtime/nodes: `254.87317895889282` / `1`
- Compact runtime/nodes: `1800.325535774231` / `52155.0`
- Compact first incumbent: `25.450000047683716` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 254.6526849269867,
  "root_compact_accepted_columns": 6,
  "root_compact_bound_full": -0.10150175454616672,
  "root_compact_iteration_count": 668123.0,
  "root_compact_node_count": 2106.0,
  "root_compact_objective_full": 0.13465492070684654,
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.13465492070684795,
  "root_incumbent_at_fathom_full": 0.12710178389894503,
  "root_lower_bound_full": 0.12710178389894503,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 6,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.18694556132566842,
  "columns_added_farkas": 0,
  "columns_added_standard": 858,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 864,
  "pricing_complete_routes_generated": 35647,
  "pricing_cpu_core_equivalent_max": 7.428652659921898,
  "pricing_extensions_attempted": 185674,
  "pricing_extensions_rejected_by_deadline": 47148,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 37567,
  "pricing_labels_generated": 138730,
  "pricing_labels_pruned": 7015,
  "pricing_labels_purged": 442,
  "pricing_negative_routes_inserted": 858,
  "pricing_negative_routes_verified": 1233,
  "pricing_process_cpu_time": 366.046875,
  "pricing_standard_bound_pruned": 7015,
  "standard_pricing_calls": 17,
  "total_routes": 864
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
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 3,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 152,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 117,
    "pricing_leaf_tasks_created": 204
  },
  "workers": {
    "pricing_idle_work_requests": 521,
    "pricing_parallel_calls": 17,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 17,
    "pricing_pool_shutdown_time": 0.14057087898254395,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.465373754501343,
    "pricing_process_cpu_time": 366.046875,
    "pricing_task_submission_time": 0.0050029754638671875,
    "pricing_worker_busy_seconds": 374.7847695350647,
    "pricing_worker_idle_seconds": 1504.9571685791016
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
  "heuristic_full_pool_time": 0.027200937271118164,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.027200937271118164,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.028200626373291016,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.25084114074707
}
```

## BPC Routes

- Route 775: truck `['Source', 'C1', 'C6', 'C13', 'H3', 'Sink']`; drones `{'H3': ['C2', 'C8', 'C14', 'C17']}`; return `140.0261146897151`; services `{'C1': 12.092237765239652, 'C6': 34.048813991888466, 'C13': 55.38277538760245, 'C2': 77.35442248782275, 'C8': 77.17038616094615, 'C14': 77.65272994598892, 'C17': 76.08993382759249}`
- Route 417: truck `['Source', 'C15', 'H1', 'C18', 'C11', 'Sink']`; drones `{}`; return `98.53049100680008`; services `{'C15': 27.649166374428553, 'C18': 41.37968408823194, 'C11': 61.96985257147824}`
- Route 739: truck `['Source', 'C16', 'H2', 'C3', 'Sink']`; drones `{'H2': ['C25', 'C27', 'C28']}`; return `101.17682610285512`; services `{'C16': 17.441120511852322, 'C25': 42.72718435441961, 'C27': 36.13914105532694, 'C28': 41.28779163624619, 'C3': 81.40474175997238}`
- Route 2: truck `['Source', 'C23', 'C9', 'C21', 'C24', 'Sink']`; drones `{}`; return `134.29967926231961`; services `{'C23': 44.52293870450179, 'C9': 54.067813502930456, 'C21': 74.87246528989067, 'C24': 97.5196752652573}`
- Route 641: truck `['Source', 'C26', 'C4', 'C20', 'C12', 'H1', 'C19', 'C7', 'Sink']`; drones `{}`; return `126.22795937251053`; services `{'C26': 23.47569349764275, 'C4': 24.00207988202979, 'C20': 35.330455629517644, 'C12': 54.79423672667167, 'C19': 77.29609823850072, 'C7': 88.07446217804858}`
- Route 512: truck `['Source', 'C5', 'C30', 'C10', 'C29', 'C22', 'H1', 'Sink']`; drones `{}`; return `131.92114466076947`; services `{'C5': 31.80934392489817, 'C30': 39.48315190080426, 'C10': 46.210202426107244, 'C29': 55.26051805325341, 'C22': 75.56334498461126}`

## Compact Routes

- Route 0: truck `['Source', 'C26', 'C4', 'C20', 'C12', 'C6', 'C13', 'H1', 'Sink']`; drones `{}`; return `142.59526656245612`; services `{'C26': 23.47569349764275, 'C4': 24.00207988202979, 'C20': 35.330455629517644, 'C12': 54.79423672667167, 'C6': 65.87852977913967, 'C13': 87.21249117485365}`
- Route 1: truck `['Source', 'C15', 'H1', 'C18', 'C11', 'Sink']`; drones `{'H1': ['C3']}`; return `112.59297286477188`; services `{'C15': 27.649166374428553, 'C3': 46.30291721573135, 'C18': 55.442165946203744, 'C11': 76.03233442945005}`
- Route 2: truck `['Source', 'C5', 'C30', 'C10', 'C27', 'H3', 'Sink']`; drones `{'H3': ['C2', 'C8', 'C14', 'C25']}`; return `148.30643717518697`; services `{'C5': 31.80934392489817, 'C30': 39.48315190080426, 'C10': 46.210202426107244, 'C27': 46.32092047491628, 'C2': 72.03880251532382, 'C8': 71.85476618844723, 'C14': 72.33710997349, 'C25': 79.1350812024754}`
- Route 3: truck `['Source', 'C23', 'C9', 'C21', 'C24', 'Sink']`; drones `{}`; return `134.29967926231961`; services `{'C23': 44.52293870450179, 'C9': 54.067813502930456, 'C21': 74.87246528989067, 'C24': 97.5196752652573}`
- Route 4: truck `['Source', 'C19', 'C7', 'C16', 'H2', 'Sink']`; drones `{'H2': ['C28']}`; return `135.62219478505085`; services `{'C19': 38.996991402253315, 'C7': 49.775355341801166, 'C16': 70.48773202441079, 'C28': 94.33440314880465}`
- Route 5: truck `['Source', 'C1', 'C29', 'C22', 'H1', 'C17', 'Sink']`; drones `{}`; return `172.79008038561852`; services `{'C1': 12.092237765239652, 'C29': 36.05497274480035, 'C22': 56.3577996761582, 'C17': 113.09143330427166}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 643.203125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.5110166984083144,
    "mean_cpu_percent_of_logical_machine": 20.92513915340262,
    "peak_interval_core_equivalent": 8.96436291020842,
    "peak_process_count": 13,
    "peak_rss_bytes": 1341890560,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PS_seed3\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 256.1524682044983
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 9417.765625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.227840671826777,
    "mean_cpu_percent_of_logical_machine": 43.56533893188981,
    "peak_interval_core_equivalent": 6.114475938105622,
    "peak_process_count": 1,
    "peak_rss_bytes": 516984832,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PS_seed3\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.4637813568115
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 21 artifacts are stored in the JSON report.
