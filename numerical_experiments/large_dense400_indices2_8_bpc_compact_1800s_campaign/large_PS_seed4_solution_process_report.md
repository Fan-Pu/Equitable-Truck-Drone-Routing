# large_PS_seed4 Solution Process Report

- Snapshot SHA-256: `09d7ed21ca986b2718a8c1f7c87753c3e71bf437581d531c91b389a1d2129c04`
- BPC status: `optimal`
- Compact status: `success`
- Compact termination: `time_limit_with_incumbent`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.1403486137898247` / `0.1403486137898247` / `0.0`
- Compact incumbent/bound/gap: `0.1420731662624602` / `-0.05481094661820141` / `1.3857937994916358`
- BPC runtime/nodes: `114.08121418952942` / `1`
- Compact runtime/nodes: `1800.3158695697784` / `36401.0`
- Compact first incumbent: `26.84999990463257` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": false,
  "root_closed": true,
  "root_closure_time": 113.83708095550537,
  "root_compact_accepted_columns": 6,
  "root_compact_bound_full": -0.0644267599425305,
  "root_compact_iteration_count": 1167197.0,
  "root_compact_node_count": 3031.0,
  "root_compact_objective_full": 0.22049704100088044,
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "success",
  "root_fathom_reason": "integral_rmp_bound_matches_route_pool_incumbent",
  "root_fractional_variable_count": 0,
  "root_incumbent_at_classification_full": 0.22049704100082926,
  "root_incumbent_at_fathom_full": 0.1403486137898247,
  "root_lower_bound_full": 0.14034861378982463,
  "root_max_integrality_violation": 0.0,
  "root_nonzero_variable_count": 6,
  "root_rmp_is_integer": true
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -0.1517004216125316,
  "columns_added_farkas": 0,
  "columns_added_standard": 836,
  "farkas_pricing_calls": 0,
  "global_pool_routes": 842,
  "pricing_complete_routes_generated": 18688,
  "pricing_cpu_core_equivalent_max": 7.203972028425814,
  "pricing_extensions_attempted": 162499,
  "pricing_extensions_rejected_by_deadline": 63217,
  "pricing_farkas_bound_pruned": 0,
  "pricing_labels_dominated": 29323,
  "pricing_labels_generated": 99462,
  "pricing_labels_pruned": 16703,
  "pricing_labels_purged": 177,
  "pricing_negative_routes_inserted": 836,
  "pricing_negative_routes_verified": 1339,
  "pricing_process_cpu_time": 155.015625,
  "pricing_standard_bound_pruned": 16703,
  "standard_pricing_calls": 15,
  "total_routes": 842
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
    "pricing_balanced_process_dynamic_calls": 15,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 140,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 73,
    "pricing_leaf_tasks_created": 180
  },
  "workers": {
    "pricing_idle_work_requests": 341,
    "pricing_parallel_calls": 15,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 15,
    "pricing_pool_shutdown_time": 0.1495962142944336,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 9.621828556060791,
    "pricing_process_cpu_time": 155.015625,
    "pricing_task_submission_time": 0.005003929138183594,
    "pricing_worker_busy_seconds": 162.66488194465637,
    "pricing_worker_idle_seconds": 180.81875729560852
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
  "heuristic_full_pool_time": 0.04702568054199219,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.04702568054199219,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.048024892807006836,
  "incumbent_source": "compact_root",
  "time_to_first_incumbent": 60.25961661338806
}
```

## BPC Routes

- Route 710: truck `['Source', 'C10', 'C24', 'C18', 'C16', 'H1', 'C19', 'Sink']`; drones `{}`; return `124.23924379679997`; services `{'C10': 29.495407597744826, 'C24': 44.394715169331214, 'C18': 47.36525691136045, 'C16': 67.72240878371912, 'C19': 80.45701318882489}`
- Route 660: truck `['Source', 'C12', 'C25', 'H2', 'C9', 'C28', 'Sink']`; drones `{'H2': ['C5', 'C6']}`; return `146.95061097541085`; services `{'C12': 29.964493965654245, 'C25': 47.49430791849601, 'C5': 57.0640513535774, 'C6': 56.993750310616235, 'C9': 75.77713883484883, 'C28': 105.84288368479396}`
- Route 832: truck `['Source', 'C2', 'C26', 'C15', 'Sink']`; drones `{}`; return `92.91621294001888`; services `{'C2': 39.640489860123985, 'C26': 49.976931708386175, 'C15': 71.40214244250123}`
- Route 686: truck `['Source', 'C20', 'H3', 'C27', 'C30', 'C8', 'Sink']`; drones `{'H3': ['C3', 'C4']}`; return `129.06288708627525`; services `{'C20': 25.988210133652387, 'C3': 38.238376869642295, 'C4': 38.77410408483874, 'C27': 69.42215021741879, 'C30': 75.61315402101073, 'C8': 78.29243236439213}`
- Route 762: truck `['Source', 'C22', 'C29', 'H2', 'C13', 'H1', 'C7', 'Sink']`; drones `{}`; return `140.32328639400262`; services `{'C22': 24.401419425263217, 'C29': 35.54670449115751, 'C13': 80.73407932281098, 'C7': 94.59417407123728}`
- Route 585: truck `['Source', 'C23', 'H1', 'C1', 'C17', 'Sink']`; drones `{'H1': ['C11', 'C14', 'C21']}`; return `157.59262566721085`; services `{'C23': 19.970432975556903, 'C11': 56.79505395863413, 'C14': 52.42513043502861, 'C21': 54.07117950866543, 'C1': 97.75161989025719, 'C17': 100.55051863954631}`

## Compact Routes

- Route 0: truck `['Source', 'C20', 'H3', 'C27', 'C30', 'C8', 'Sink']`; drones `{'H3': ['C3', 'C4']}`; return `129.06288708627525`; services `{'C20': 25.988210133652387, 'C3': 38.238376869642295, 'C4': 38.77410408483874, 'C27': 69.42215021741879, 'C30': 75.61315402101073, 'C8': 78.29243236439213}`
- Route 1: truck `['Source', 'C2', 'C26', 'C15', 'H1', 'C7', 'Sink']`; drones `{}`; return `141.34629659051427`; services `{'C2': 39.640489860123985, 'C26': 49.976931708386175, 'C15': 71.40214244250123, 'C7': 95.61718426774893}`
- Route 2: truck `['Source', 'C12', 'C1', 'C25', 'H2', 'C13', 'Sink']`; drones `{}`; return `123.61459455488067`; services `{'C12': 29.964493965654245, 'C1': 54.53938718950235, 'C25': 61.58446646050868, 'C13': 86.43381582707482}`
- Route 3: truck `['Source', 'C23', 'H1', 'C17', 'Sink']`; drones `{'H1': ['C11', 'C14', 'C21']}`; return `151.9948281686326`; services `{'C23': 19.970432975556903, 'C11': 56.79505395863413, 'C14': 52.42513043502861, 'C21': 54.07117950866543, 'C17': 94.95272114096807}`
- Route 4: truck `['Source', 'C22', 'C29', 'H2', 'C9', 'C28', 'Sink']`; drones `{'H2': ['C5', 'C6']}`; return `155.3410330131597`; services `{'C22': 24.401419425263217, 'C29': 35.54670449115751, 'C5': 65.45447339132622, 'C6': 65.38417234836507, 'C9': 84.16756087259765, 'C28': 114.23330572254278}`
- Route 5: truck `['Source', 'C10', 'C24', 'C18', 'C16', 'H1', 'C19', 'Sink']`; drones `{}`; return `124.23924379679997`; services `{'C10': 29.495407597744826, 'C24': 44.394715169331214, 'C18': 47.36525691136045, 'C16': 67.72240878371912, 'C19': 80.45701318882489}`

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 465.90625,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.016945228342592,
    "mean_cpu_percent_of_logical_machine": 33.4745435695216,
    "peak_interval_core_equivalent": 9.552313981447915,
    "peak_process_count": 13,
    "peak_rss_bytes": 1142153216,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PS_seed4\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 115.9852135181427
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 8825.421875,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.897796107463072,
    "mean_cpu_percent_of_logical_machine": 40.81496756219226,
    "peak_interval_core_equivalent": 6.036130303319249,
    "peak_process_count": 1,
    "peak_rss_bytes": 461139968,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_dense400_indices2_8_bpc_compact_1800s_campaign\\cases\\large_PS_seed4\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.916960477829
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 20 artifacts are stored in the JSON report.
