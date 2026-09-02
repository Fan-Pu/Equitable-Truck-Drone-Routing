# large_PC_seed5 Solution Process Report

- Snapshot SHA-256: `fa61e4218317d43b6554f8e04dfa4f247eb4345e4dc2b6510193627ce56d05b3`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.06228878798169343` / `0.06228878798169343` / `0.0`
- Compact incumbent/bound/gap: `0.062288787981695695` / `-0.09995049162767818` / `2.604630542129826`
- BPC runtime/nodes: `1086.7914590835571` / `1`
- Compact runtime/nodes: `1800.397703409195` / `54534.0`
- Compact first incumbent: `17.94599986076355` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 1086.4480199813843,
  "root_compact_accepted_columns": 6,
  "root_compact_bound_full": -0.10529795969426217,
  "root_compact_iteration_count": 414871.0,
  "root_compact_node_count": 1626.0,
  "root_compact_objective_full": 0.07995957187672237,
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.0799595718767461,
  "root_incumbent_at_fathom_full": 0.06228878798169343,
  "root_lower_bound_full": 0.06228878798169343,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 6,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.10595516774291287,
  "columns_added_farkas": 0,
  "columns_added_standard": 1503,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 1509,
  "pricing_complete_routes_generated": 213700,
  "pricing_cpu_core_equivalent_max": 7.020718647111253,
  "pricing_extensions_attempted": 1577949,
  "pricing_extensions_rejected_by_deadline": 608555,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 276138,
  "pricing_labels_generated": 969730,
  "pricing_labels_pruned": 149943,
  "pricing_labels_purged": 5069,
  "pricing_negative_routes_inserted": 1503,
  "pricing_negative_routes_verified": 1994,
  "pricing_process_cpu_time": 4841.390625,
  "pricing_standard_bound_pruned": 149943,
  "standard_pricing_calls": 28,
  "total_routes": 1509
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
  "sr_cuts_added": 8,
  "sr_cuts_added_postroot": 0,
  "sr_cuts_added_root": 8
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 28,
    "pricing_dynamic_bytes_transferred": 21624136,
    "pricing_dynamic_child_tasks_created": 348,
    "pricing_dynamic_labels_transferred": 68871,
    "pricing_dynamic_split_candidates": 314,
    "pricing_dynamic_split_control_time": 0.811323881149292,
    "pricing_dynamic_split_rejected_elapsed": 246,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 1592,
    "pricing_dynamic_splits_performed": 74,
    "pricing_leaf_tasks_closed": 511,
    "pricing_leaf_tasks_created": 684
  },
  "workers": {
    "pricing_idle_work_requests": 2534,
    "pricing_parallel_calls": 28,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 28,
    "pricing_pool_shutdown_time": 0.18205571174621582,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.958173990249634,
    "pricing_process_cpu_time": 4841.390625,
    "pricing_task_submission_time": 0.006032466888427734,
    "pricing_worker_busy_seconds": 5016.895286560059,
    "pricing_worker_idle_seconds": 6503.275073051453
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
  "heuristic_full_pool_time": 0.08107447624206543,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.08107447624206543,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.08459043502807617,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.309582233428955
}
```

## BPC Routes

- Route 646: truck `['Source', 'C15', 'C3', 'C16', 'H1', 'C30', 'Sink']`; drones `{}`; return `89.57529608097771`; services `{'C15': 15.276641990975135, 'C3': 17.601660800760524, 'C16': 42.46549753616979, 'C30': 58.92510954453096}`
- Route 4: truck `['Source', 'C21', 'C5', 'C11', 'C23', 'Sink']`; drones `{}`; return `84.8693512618814`; services `{'C21': 19.62606569479296, 'C5': 44.23946854685788, 'C11': 45.47645408065007, 'C23': 46.28230477634775}`
- Route 1204: truck `['Source', 'C22', 'C2', 'H1', 'C24', 'Sink']`; drones `{'H1': ['C6', 'C8', 'C14', 'C26']}`; return `96.88781118977191`; services `{'C22': 18.785524365075382, 'C2': 28.436545270358106, 'C6': 42.395526059630434, 'C8': 36.32528597554823, 'C14': 38.741206526767634, 'C26': 37.72480850221382, 'C24': 62.62830693490374}`
- Route 5: truck `['Source', 'C28', 'C10', 'H1', 'C20', 'C12', 'C18', 'C19', 'Sink']`; drones `{}`; return `126.69200688775811`; services `{'C28': 16.21551788640688, 'C10': 16.952138955003797, 'C20': 32.45596054754594, 'C12': 38.8710426218465, 'C18': 41.060929743691716, 'C19': 67.43696989636607}`
- Route 2: truck `['Source', 'C4', 'C13', 'C25', 'C7', 'C1', 'H1', 'Sink']`; drones `{}`; return `126.73542478323184`; services `{'C4': 15.842406361386308, 'C13': 59.556596839867225, 'C25': 60.73691515244224, 'C7': 62.90761119951948, 'C1': 65.57619394992038}`
- Route 839: truck `['Source', 'C9', 'C27', 'C29', 'H3', 'Sink']`; drones `{'H3': ['C17']}`; return `81.19101983500587`; services `{'C9': 15.066569598416965, 'C27': 16.493254885680116, 'C29': 38.87468539389864, 'C17': 41.50664860481559}`

## Compact Routes

- Route 0: truck `['Source', 'C28', 'C10', 'C20', 'C12', 'C18', 'C19', 'H1', 'Sink']`; drones `{}`; return `126.69200688775811`; services `{'C28': 16.21551788640688, 'C10': 16.952138955003797, 'C20': 32.45596054754594, 'C12': 38.8710426218465, 'C18': 41.060929743691716, 'C19': 67.43696989636607}`
- Route 1: truck `['Source', 'C4', 'C13', 'C25', 'C7', 'C1', 'Sink']`; drones `{}`; return `126.73542478323182`; services `{'C4': 15.842406361386308, 'C13': 59.556596839867225, 'C25': 60.73691515244224, 'C7': 62.90761119951948, 'C1': 65.57619394992038}`
- Route 2: truck `['Source', 'C9', 'C27', 'C29', 'H3', 'Sink']`; drones `{'H3': ['C17']}`; return `81.19101983500587`; services `{'C9': 15.066569598416965, 'C27': 16.493254885680116, 'C29': 38.87468539389864, 'C17': 41.50664860481559}`
- Route 3: truck `['Source', 'C22', 'C2', 'H1', 'C24', 'Sink']`; drones `{'H1': ['C6', 'C8', 'C14', 'C26']}`; return `96.88781118977191`; services `{'C22': 18.785524365075382, 'C2': 28.436545270358106, 'C6': 42.395526059630434, 'C8': 36.32528597554823, 'C14': 38.741206526767634, 'C26': 37.72480850221382, 'C24': 62.62830693490374}`
- Route 4: truck `['Source', 'C15', 'C3', 'C16', 'H1', 'C30', 'Sink']`; drones `{}`; return `89.57529608097771`; services `{'C15': 15.276641990975135, 'C3': 17.601660800760524, 'C16': 42.46549753616979, 'C30': 58.92510954453096}`
- Route 5: truck `['Source', 'C21', 'C5', 'C11', 'C23', 'Sink']`; drones `{}`; return `84.8693512618814`; services `{'C21': 19.62606569479296, 'C5': 44.23946854685788, 'C11': 45.47645408065007, 'C23': 46.28230477634775}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 5142.328125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.721725337329829,
    "mean_cpu_percent_of_logical_machine": 39.34771114441524,
    "peak_interval_core_equivalent": 10.54750406711333,
    "peak_process_count": 13,
    "peak_rss_bytes": 1643286528,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PC_seed5\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1089.078198671341
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 9211.828125,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.112502545746163,
    "mean_cpu_percent_of_logical_machine": 42.60418788121803,
    "peak_interval_core_equivalent": 6.062114099391809,
    "peak_process_count": 1,
    "peak_rss_bytes": 529248256,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PC_seed5\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.823674917221
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 21 artifacts are stored in the JSON report.
