# large_PC_seed7 Solution Process Report

- Snapshot SHA-256: `b659137b599d137b8fa1ba433614b55353acdd5da4d47449a2b562c8a0f3fb2d`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.15750243863113086` / `0.15750243863113086` / `0.0`
- Compact incumbent/bound/gap: `0.16734472603608233` / `0.02642547730348802` / `0.8420895720502705`
- BPC runtime/nodes: `128.72548174858093` / `1`
- Compact runtime/nodes: `1800.3579981327057` / `46923.0`
- Compact first incumbent: `80.85199999809265` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 128.45440006256104,
  "root_compact_accepted_columns": 0,
  "root_compact_bound_full": 0.020628187660087206,
  "root_compact_iteration_count": 1053548.0,
  "root_compact_node_count": 3020.0,
  "root_compact_objective_full": null,
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "timeout",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": null,
  "root_incumbent_at_fathom_full": 0.15750243863113086,
  "root_lower_bound_full": 0.15750243863113092,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 6,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -12.0,
  "columns_added_farkas": 275,
  "columns_added_standard": 440,
  "farkas_pricing_calls": 18,
  "global_pool_routes": 715,
  "pricing_complete_routes_generated": 16197,
  "pricing_cpu_core_equivalent_max": 5.148668046195541,
  "pricing_extensions_attempted": 146670,
  "pricing_extensions_rejected_by_deadline": 54035,
  "pricing_farkas_bound_pruned": 4379,
  "pricing_labels_dominated": 30774,
  "pricing_labels_generated": 92971,
  "pricing_labels_pruned": 16857,
  "pricing_labels_purged": 580,
  "pricing_negative_routes_inserted": 715,
  "pricing_negative_routes_verified": 1174,
  "pricing_process_cpu_time": 137.3125,
  "pricing_standard_bound_pruned": 12478,
  "standard_pricing_calls": 10,
  "total_routes": 715
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
    "pricing_balanced_process_dynamic_calls": 28,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 99,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 139,
    "pricing_leaf_tasks_created": 336
  },
  "workers": {
    "pricing_idle_work_requests": 375,
    "pricing_parallel_calls": 28,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 28,
    "pricing_pool_shutdown_time": 0.16472411155700684,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 10.189480304718018,
    "pricing_process_cpu_time": 137.3125,
    "pricing_task_submission_time": 0.006516933441162109,
    "pricing_worker_busy_seconds": 144.27984309196472,
    "pricing_worker_idle_seconds": 363.1251516342163
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
  "heuristic_full_pool_time": 0.0255277156829834,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.0255277156829834,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.0270388126373291,
  "incumbent_source": "route_pool_after_node_close",
  "time_to_first_incumbent": 128.5092806816101
}
```

## BPC Routes

- Route 671: truck `['Source', 'C10', 'H1', 'C22', 'C7', 'C28', 'Sink']`; drones `{'H1': ['C3', 'C9', 'C15']}`; return `138.41245489550622`; services `{'C10': 31.887976181122323, 'C3': 41.880365459681045, 'C9': 44.34015190924097, 'C15': 44.26990539337001, 'C22': 58.33641900277787, 'C7': 83.4341408550884, 'C28': 105.37520046319297}`
- Route 48: truck `['Source', 'C14', 'C24', 'C18', 'Sink']`; drones `{}`; return `84.31684307725843`; services `{'C14': 37.33574568963566, 'C24': 60.365981444222896, 'C18': 61.99333206881848}`
- Route 620: truck `['Source', 'C16', 'C4', 'C21', 'C13', 'H1', 'C19', 'Sink']`; drones `{}`; return `153.4837972470754`; services `{'C16': 30.53463396104125, 'C4': 32.233393873123795, 'C21': 45.01288058554155, 'C13': 57.1518042716584, 'C19': 95.76643296032215}`
- Route 518: truck `['Source', 'C26', 'C20', 'C11', 'C17', 'H2', 'Sink']`; drones `{'H2': ['C2', 'C25']}`; return `130.03293250601698`; services `{'C26': 36.12069082916082, 'C20': 38.33448636652498, 'C11': 49.35027477162234, 'C17': 50.76410837612991, 'C2': 65.36542825782104, 'C25': 69.22051773262264}`
- Route 379: truck `['Source', 'C8', 'C29', 'C23', 'C1', 'C27', 'Sink']`; drones `{}`; return `124.4301111527678`; services `{'C8': 35.6567588754856, 'C29': 46.707085529803, 'C23': 47.15277703225774, 'C1': 68.20982010422952, 'C27': 81.0041008963906}`
- Route 435: truck `['Source', 'H3', 'C5', 'Sink']`; drones `{'H3': ['C6', 'C12', 'C30']}`; return `90.33066591369219`; services `{'C6': 23.297715703060113, 'C12': 23.329379595357505, 'C30': 23.591175759191845, 'C5': 46.10122684085144}`

## Compact Routes

- Route 0: truck `['Source', 'C8', 'C29', 'C23', 'C1', 'C19', 'Sink']`; drones `{}`; return `127.78531392440712`; services `{'C8': 35.6567588754856, 'C29': 46.707085529803, 'C23': 47.15277703225774, 'C1': 68.20982010422952, 'C19': 70.06794963765387}`
- Route 1: truck `['Source', 'C6', 'H1', 'C27', 'Sink']`; drones `{'H1': ['C3', 'C9', 'C15']}`; return `110.25143306220258`; services `{'C6': 21.872724902135605, 'C3': 52.045738574524954, 'C9': 54.50552502408489, 'C15': 54.43527850821392, 'C27': 66.82542280582538}`
- Route 2: truck `['Source', 'C16', 'C4', 'C21', 'C13', 'H2', 'Sink']`; drones `{'H2': ['C2', 'C25']}`; return `151.48816855121265`; services `{'C16': 30.53463396104125, 'C4': 32.233393873123795, 'C21': 45.01288058554155, 'C13': 57.1518042716584, 'C2': 86.8206643030167, 'C25': 90.6757537778183}`
- Route 3: truck `['Source', 'C10', 'C22', 'C7', 'C28', 'Sink']`; drones `{}`; return `114.30695531082554`; services `{'C10': 31.887976181122323, 'C22': 34.230919418097216, 'C7': 59.32864127040775, 'C28': 81.26970087851231}`
- Route 4: truck `['Source', 'C26', 'C20', 'C11', 'C17', 'H3', 'C5', 'Sink']`; drones `{'H3': ['C12', 'C30']}`; return `143.90564095235408`; services `{'C26': 36.12069082916082, 'C20': 38.33448636652498, 'C11': 49.35027477162234, 'C17': 50.76410837612991, 'C12': 76.90435463401941, 'C30': 77.16615079785375, 'C5': 99.67620187951334}`
- Route 5: truck `['Source', 'C14', 'C24', 'C18', 'Sink']`; drones `{}`; return `84.31684307725843`; services `{'C14': 37.33574568963566, 'C24': 60.365981444222896, 'C18': 61.99333206881848}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 451.5625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 3.4605907535352873,
    "mean_cpu_percent_of_logical_machine": 28.838256279460726,
    "peak_interval_core_equivalent": 6.029419094694603,
    "peak_process_count": 13,
    "peak_rss_bytes": 1116332032,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PC_seed7\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 130.48711395263672
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 9454.390625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 5.2469399302129744,
    "mean_cpu_percent_of_logical_machine": 43.72449941844145,
    "peak_interval_core_equivalent": 6.065501511493249,
    "peak_process_count": 1,
    "peak_rss_bytes": 523636736,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PC_seed7\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.8865759372711
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
