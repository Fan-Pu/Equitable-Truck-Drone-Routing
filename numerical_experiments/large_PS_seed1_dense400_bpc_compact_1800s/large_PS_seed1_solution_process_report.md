# Large PS Seed 1 Solution Process Report

- Snapshot SHA-256: `170a5effe33a12985d9ccf161688c672b515308969514a1aa28f767e75c7ce54`
- BPC status: `optimal`
- Compact status: `timeout`
- Compact termination: `timeout`
- Same canonical route set: `False`
- BPC incumbent/bound/gap: `0.1633362825374107` / `0.1633362825374107` / `0.0`
- Compact incumbent/bound/gap: `None` / `-0.05928021115308084` / `None`
- BPC runtime/nodes: `395.87407994270325` / `3`
- Compact runtime/nodes: `1800.3490743637085` / `46742.0`
- Compact first incumbent: `None` s
- BPC root/compact Gurobi versions: `12.0.1` / `12.0.1`

## BPC Root

```json
{
  "root_branch_required": true,
  "root_closed": true,
  "root_closure_time": 250.45568013191223,
  "root_compact_accepted_columns": 0,
  "root_compact_bound_full": -0.08863243675761576,
  "root_compact_iteration_count": 834125.0,
  "root_compact_node_count": 1728.0,
  "root_compact_objective_full": null,
  "root_compact_solve_budget_seconds": 60.0,
  "root_compact_status": "timeout",
  "root_fathom_reason": null,
  "root_fractional_variable_count": 15,
  "root_incumbent_at_classification_full": null,
  "root_incumbent_at_fathom_full": null,
  "root_lower_bound_full": 0.1624020728300066,
  "root_max_integrality_violation": 0.5,
  "root_nonzero_variable_count": 16,
  "root_rmp_is_integer": false
}
```

## Pricing and Columns

```json
{
  "best_reduced_cost_at_stop": -13.000000000000004,
  "columns_added_farkas": 262,
  "columns_added_standard": 625,
  "farkas_pricing_calls": 17,
  "global_pool_routes": 887,
  "pricing_complete_routes_generated": 115865,
  "pricing_cpu_core_equivalent_max": 7.086815777239372,
  "pricing_extensions_attempted": 816836,
  "pricing_extensions_rejected_by_deadline": 363782,
  "pricing_farkas_bound_pruned": 2885,
  "pricing_labels_dominated": 129960,
  "pricing_labels_generated": 453714,
  "pricing_labels_pruned": 67569,
  "pricing_labels_purged": 1940,
  "pricing_negative_routes_inserted": 887,
  "pricing_negative_routes_verified": 1308,
  "pricing_process_cpu_time": 653.4375,
  "pricing_standard_bound_pruned": 60900,
  "standard_pricing_calls": 38,
  "total_routes": 887
}
```

## Cuts and Branching

```json
{
  "branching_nodes": 1,
  "child_nodes_created": 2,
  "conditioned_arc_branches": 0,
  "customer_pair_branches": 1,
  "launch_pad_branches": 0,
  "open_nodes_at_termination": 0,
  "postroot_nodes_processed": 2,
  "postroot_open_nodes": 0,
  "queue_bound_fathoms": 0,
  "sr_cuts_added": 74,
  "sr_cuts_added_postroot": 33,
  "sr_cuts_added_root": 41
}
```

## Dynamic Splitting and Workers

```json
{
  "dynamic_splitting": {
    "pricing_balanced_process_dynamic_calls": 55,
    "pricing_dynamic_bytes_transferred": 0,
    "pricing_dynamic_child_tasks_created": 0,
    "pricing_dynamic_labels_transferred": 0,
    "pricing_dynamic_split_candidates": 0,
    "pricing_dynamic_split_control_time": 0.0,
    "pricing_dynamic_split_rejected_elapsed": 0,
    "pricing_dynamic_split_rejected_low_work": 0,
    "pricing_dynamic_split_rejected_near_closure": 0,
    "pricing_dynamic_split_rejected_small_frontier": 1404,
    "pricing_dynamic_splits_performed": 0,
    "pricing_leaf_tasks_closed": 450,
    "pricing_leaf_tasks_created": 660
  },
  "workers": {
    "pricing_idle_work_requests": 3745,
    "pricing_parallel_calls": 55,
    "pricing_parallel_workers_max": 12,
    "pricing_pool_reused_calls": 55,
    "pricing_pool_shutdown_time": 0.1620643138885498,
    "pricing_pool_startup_count": 1,
    "pricing_pool_startup_time": 11.696508646011353,
    "pricing_process_cpu_time": 653.4375,
    "pricing_task_submission_time": 0.014011383056640625,
    "pricing_worker_busy_seconds": 697.2108016014099,
    "pricing_worker_idle_seconds": 1301.5034019947052
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
  "heuristic_full_pool_time": 0.0400395393371582,
  "heuristic_hard_pool_feasible_solves": 1,
  "heuristic_hard_pool_solves": 1,
  "heuristic_hard_pool_time": 0.0400395393371582,
  "heuristic_incumbent_updates": 1,
  "heuristic_time": 0.041039466857910156,
  "incumbent_source": "route_pool_after_node_close",
  "time_to_first_incumbent": 250.53023147583008
}
```

## BPC Routes

- Route 80: truck `['Source', 'C19', 'C5', 'C14', 'Sink']`; drones `{}`; return `99.56362813015377`; services `{'C19': 21.30683024715842, 'C5': 59.220491285626295, 'C14': 68.88420089542359}`
- Route 518: truck `['Source', 'C28', 'C6', 'C23', 'C4', 'C30', 'Sink']`; drones `{}`; return `111.71774398773411`; services `{'C28': 10.232870378276736, 'C6': 48.43711582102818, 'C23': 52.75020997306709, 'C4': 56.96804768669141, 'C30': 61.1509883961577}`
- Route 18: truck `['Source', 'C3', 'C10', 'C16', 'Sink']`; drones `{}`; return `78.58284343910925`; services `{'C3': 27.56842128686479, 'C10': 37.67052078180461, 'C16': 54.88531101340992}`
- Route 698: truck `['Source', 'C8', 'C26', 'H3', 'C29', 'Sink']`; drones `{'H3': ['C13', 'C17']}`; return `151.2686114661974`; services `{'C8': 28.375976953961118, 'C26': 55.13939428342718, 'C13': 69.56489822257828, 'C17': 68.54075137139182, 'C29': 86.89946882376968}`
- Route 655: truck `['Source', 'C9', 'C7', 'H1', 'C15', 'C18', 'C21', 'Sink']`; drones `{'H1': ['C1', 'C12', 'C25', 'C27']}`; return `152.8384534202847`; services `{'C9': 20.143300638536022, 'C7': 41.931015747281116, 'C1': 55.30426881152482, 'C12': 55.01588465936477, 'C25': 50.654555517236794, 'C27': 49.22699691325522, 'C15': 74.39108806829387, 'C18': 93.32780059466175, 'C21': 96.81491563453352}`
- Route 585: truck `['Source', 'H2', 'C24', 'C2', 'Sink']`; drones `{'H2': ['C11', 'C20', 'C22']}`; return `134.8726994182741`; services `{'C11': 37.25031970016186, 'C20': 31.445622109505912, 'C22': 31.298631615176166, 'C24': 72.80429050431749, 'C2': 93.8923596736412}`

## Compact Routes

No compact incumbent route was returned.

## Resource Usage

```json
{
  "bpc": {
    "aggregate_process_tree_cpu_seconds": 1045.34375,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 2.6268428718233165,
    "mean_cpu_percent_of_logical_machine": 21.8903572651943,
    "peak_interval_core_equivalent": 9.732953697711496,
    "peak_process_count": 13,
    "peak_rss_bytes": 1325076480,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_PS_seed1_dense400_bpc_compact_1800s\\cases\\large_PS_seed1\\bpc\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 397.94681334495544
  },
  "compact": {
    "aggregate_process_tree_cpu_seconds": 7765.874999999999,
    "logical_cpu_count": 12,
    "mean_core_equivalent": 4.310624259622133,
    "mean_cpu_percent_of_logical_machine": 35.921868830184444,
    "peak_interval_core_equivalent": 6.063629767786567,
    "peak_process_count": 1,
    "peak_rss_bytes": 429199360,
    "sample_period_seconds": 1.0,
    "samples_file": "D:\\GitHub\\Equitable-Truck-Drone-Routing\\numerical_experiments\\large_PS_seed1_dense400_bpc_compact_1800s\\cases\\large_PS_seed1\\compact\\attempt_001\\resource_samples.jsonl",
    "wall_runtime_seconds": 1801.5662076473236
  }
}
```

Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for 32 artifacts are stored in the JSON report.
