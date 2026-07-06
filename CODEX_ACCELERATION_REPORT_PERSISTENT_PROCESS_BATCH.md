# Implementation Report: Yield-Aware Productive Pricing And Primal Discovery

## 1. Purpose Of This Revision

This revision builds on the persistent process-pool and batch-oriented productive-pricing implementation. The previous run fixed process startup and first-hit interruption, but the medium instance still spent most of its time in a few large productive pricing calls under stale duals and did not find a feasible incumbent.

The current revision adds exact scheduling and primal-discovery controls:

- productive pricing time slices;
- bounded worker candidate quotas;
- dynamic one-source-neighbor process tasks;
- deterministic diversity-aware batch selection after direct verification;
- first-incumbent route-pool search as a primal-only mechanism.

These are execution and scheduling components, not heuristic closure rules. Pricing closure remains exact: only an exhaustive certification call with all source-neighbor tasks closed and no negative route can certify closure. Productive calls never certify closure.

No label cap, DSS relaxation, fallback solver, heuristic closure, skipped route verification, silent repair, or approximate certificate was introduced.

## 2. Code-Level Changes

### 2.1 New solver configuration fields

Implemented in `thvrpd/config.py`, `thvrpd/solve.py`, and `thvrpd/experiments.py`.

New defaults:

```text
productive_pricing_slice_seconds = 30.0
productive_candidate_multiplier = 1.5
source_neighbor_task_size = 1
pricing_diversity_batch_fraction = 0.5
first_incumbent_route_pool_time_limit = 10.0
```

New CLI flags:

```text
--productive-pricing-slice-seconds
--productive-candidate-multiplier
--source-neighbor-task-size
--pricing-diversity-batch-fraction
--first-incumbent-route-pool-time-limit
```

Strict validation is used. Invalid nonpositive task sizes, nonpositive candidate multipliers, and diversity fractions outside `[0,1]` raise errors.

### 2.2 Productive pricing time slices

Implemented in `thvrpd/bpc.py` and `thvrpd/pricing.py`.

At the root, productive standard-pricing calls now use:

```text
pricing_deadline = min(global_deadline, now + productive_pricing_slice_seconds)
```

If a productive slice returns verified negative columns, those columns are inserted and the RMP is reoptimized. If the slice times out without columns, the pricing state is recorded as unresolved and the algorithm moves toward closure/heuristic scheduling. It does not certify closure.

The following diagnostic fields were added:

```text
productive_slice_seconds
productive_slice_deadline_used
productive_time_limit_with_columns
productive_time_limit_no_columns
```

### 2.3 Bounded worker candidate quotas

Implemented in `thvrpd/pricing.py`.

For process-backend productive pricing, each exact source-neighbor task receives a local candidate quota:

```text
ceil(batch_size * productive_candidate_multiplier / task_count)
```

The main process still inserts at most the global batch size. The worker quota only limits how many negative candidate paths each exact task returns in a productive call. It is not a label cap and is not used for certification. Closure mode still requires exhaustive task completion.

Diagnostics:

```text
local_worker_candidate_quota
pricing_batch_target
pricing_returned_batch_size
```

### 2.4 Dynamic source-neighbor task assignment

Implemented in `thvrpd/pricing.py`.

The previous persistent process pool used one static block per worker. The new process backend partitions source neighbors into smaller exact chunks:

```text
source_neighbor_task_size = 1
```

These chunks are disjoint and exhaustive. The same exact forward label-setting pricing logic is used inside each task. The persistent `ProcessPoolExecutor` schedules many small tasks over four worker processes, improving load balance while preserving partition coverage.

New helper:

```text
_chunk_source_neighbors(...)
```

Diagnostics:

```text
source_neighbor_task_count
source_neighbor_task_sizes
pricing_worker_payload_count
pricing_worker_response_count
```

### 2.5 Diversity-aware verified batch selection

Implemented in `thvrpd/pricing.py`.

Worker outputs are compact paths and worker-reported reduced costs. The main process still performs the authoritative sequence:

1. deterministic path ordering;
2. full `route_from_path` decoding;
3. branch-feasibility check;
4. direct `route_reduced_cost` or `route_farkas_reduced_cost` recomputation;
5. worker/direct reduced-cost equality check;
6. duplicate-equivalent and cost-dominated filtering;
7. deterministic batch selection.

After verification, batch selection reserves up to:

```text
ceil(batch_size * pricing_diversity_batch_fraction)
```

for stable round-robin selection by source neighbor and residual-customer coverage. The remaining batch is filled by `(reduced_cost, path)` order.

This is exact because it only chooses which already verified negative columns to insert now. It never inserts nonnegative routes and never certifies closure.

New helpers:

```text
_candidate_source_neighbor(...)
_candidate_counts_by_source(...)
_select_diverse_pricing_candidates(...)
```

Diagnostics:

```text
verified_candidates_by_source_neighbor
selected_candidates_by_source_neighbor
diversity_quota
diversity_selected_routes
diversity_selected_customers
```

### 2.6 First-incumbent route-pool search

Implemented in `thvrpd/bpc.py` and integrated with existing `thvrpd/heuristics.py`.

While no incumbent exists, the root node now invokes the route-pool combination heuristic after a productive batch with at least `32` newly verified columns. This uses:

```text
first_incumbent_route_pool_time_limit = 10.0
```

The route-pool search remains primal-only:

- it may update the incumbent;
- it may generate valid repair/primal columns;
- it cannot certify pricing closure;
- it cannot change the lower bound;
- it cannot fathom a node.

Diagnostics:

```text
time_to_first_incumbent
first_incumbent_route_pool_calls
first_incumbent_route_pool_time
first_incumbent_route_pool_feasible_solves
first_incumbent_route_pool_routes_max
incumbent_source
```

### 2.7 Corrected persistent-process CPU diagnostics

Implemented in `thvrpd/pricing.py`.

Dynamic one-neighbor tasks create many task results per pricing call. The diagnostic elapsed time now records actual pricing-call wall time rather than the maximum single-task duration. This makes core-equivalent CPU accounting meaningful for the process backend.

The corrected medium run reports:

```text
weighted core-equivalent CPU = 3.492
maximum per-call core-equivalent CPU = 3.924
```

This is consistent with four process workers.

## 3. Tests Added Or Updated

Implemented mainly in `tests/test_pricing_rmp_bpc.py`.

Coverage added or updated:

- dynamic source-neighbor chunks are disjoint and exhaustive;
- persistent process pool still starts once and is reused;
- process backend uses dynamic tasks and returns verified batches;
- productive slice timeout without columns is unresolved and not closure;
- bounded worker quotas return only directly verified negative routes;
- diversity-aware selection is deterministic and uses verified routes only;
- first-incumbent route-pool logic can update incumbents without changing lower-bound or closure logic;
- batch size `1` preserves first-hit compatibility;
- stale process responses still raise;
- production pricing diagnostics continue to report zero backward labels and zero joins.

Verification:

```text
python -m compileall thvrpd tests
success

pytest tests -q
99 passed
```

## 4. Numerical Evidence

Run:

```text
numerical_experiments\run_after_yield_aware_productive_pricing
```

Scope:

```text
PS, seed=1, small and medium, 300 seconds per case, 4 process workers
```

Baseline:

```text
numerical_experiments\run_after_persistent_process_batch
```

### 4.1 Small instance

Small remains optimal:

```text
objective = 0.0510500097
runtime = 3.939s
root closed = true
standard pricing time = 0.317s
time to first incumbent = 1.275s
incumbent source = compact_root
```

The revision adds small scheduling overhead, so small is slightly slower than the previous persistent-process run (`3.939s` versus `3.587s`). This is expected because the case is too small to benefit from the new scheduling controls.

### 4.2 Medium instance

Medium still times out before root closure, but it now finds a feasible incumbent:

```text
status = time_limit
incumbent = 0.0685444418
lower bound = -0.0842303948
gap = 2.2288
root_closed = false
best_reduced_cost_at_stop = -4.9448477262
time_to_first_incumbent = 19.947s
incumbent_source = first_incumbent_after_standard_batch
```

The previous persistent-process run had:

```text
status = time_limited_internal
incumbent = none
time_to_first_incumbent = null
```

The first-incumbent route-pool mechanism therefore fixed the prior no-incumbent failure.

## 5. Performance Comparison

| Metric | Persistent batch baseline | Yield-aware revision |
|---|---:|---:|
| Medium runtime | 301.056s | 300.896s |
| Medium incumbent | none | 0.0685444418 |
| Time to first incumbent | none | 19.947s |
| Root closed | false | false |
| Best reduced cost at stop | -0.0016229507 | -4.9448477262 |
| Standard pricing time | 278.878s | 258.132s |
| Pricing calls | 26 | 52 |
| Max pricing-call elapsed | 121.938s | 30.421s |
| RMP solves | 13 | 26 |
| Worker payloads / responses | 104 / 104 | 884 / 884 |
| Weighted core-equivalent CPU | 3.186 | 3.492 |
| Negative routes verified | 953 | 2,064 |
| Negative routes inserted | 953 | 1,400 |
| Standard columns added | 821 | 1,223 |
| Main-process merge time | 1.997s | 7.698s |
| Heuristic time | 2.051s | 8.239s |
| Repair pricing time | 0.782s | 7.051s |
| SR cuts added | 0 | 0 |
| Branching nodes | 0 | 0 |

The scheduling revision changes the shape of the computation:

- It removes very long productive calls. The maximum pricing-call elapsed time falls from `121.938s` to `30.421s`.
- It reoptimizes more often. RMP solves increase from `13` to `26`, but RMP time is still only `1.150s`.
- It uses process workers more effectively. Weighted core-equivalent CPU rises from `3.186` to `3.492`.
- It verifies and inserts more negative columns. Medium inserted `1,400` verified negative routes versus `953`.
- It finds a feasible incumbent early.

## 6. Exactness Statement For The Paper

The paper should describe these changes as exact acceleration and scheduling rules:

1. Productive pricing slices are not certificates. They may return columns or be marked unresolved.
2. Candidate quotas bound returned productive candidates per task. They do not truncate certification search.
3. Diversity-aware selection is applied only after full route decoding and direct reduced-cost verification.
4. Route-pool search is primal-only. It can improve the incumbent but cannot close pricing, change lower bounds, or fathom nodes.
5. Closure remains certified only by exhaustive source-neighbor pricing under the current dual solution.

Suggested exactness paragraph:

> Productive pricing and primal discovery are separated from certification. During productive pricing, worker processes may return a bounded batch of negative candidate paths within a time slice. The master decodes and directly verifies every candidate route before insertion. A productive call, including a time-limited call that returns no columns, never certifies closure. Certification is declared only in closure mode when all source-neighbor tasks exhaust their exact label search under the current dual solution and no negative reduced-cost route exists.

## 7. Paper-Ready Algorithm Description

### Yield-aware productive pricing

Suggested method text:

> We use a yield-aware productive-pricing schedule at the root node. Productive pricing is run in bounded time slices. If a slice returns a verified negative batch, the restricted master problem is reoptimized immediately. If a slice expires without a verified negative route, the pricing state is marked unresolved and the algorithm shifts toward closure or primal scheduling. This prevents long searches under stale dual prices while preserving exactness, since productive pricing is never used to certify closure.

### Dynamic source-neighbor tasks

Suggested method text:

> For process-based parallel pricing, source-neighbor sets are split into small disjoint chunks. Each chunk defines an exact subproblem over routes whose first transformed node belongs to that chunk. The chunks are exhaustive over all admissible source neighbors, and worker processes pull them through a persistent executor. This improves load balance without changing the route space or the pricing certificate.

### Diversity-aware batch insertion

Suggested method text:

> After collecting candidate paths from workers, the master reconstructs every candidate route and recomputes its reduced cost directly. A deterministic portion of the insertion batch is selected by source-neighbor and residual-customer coverage diversity, and the remainder is selected by reduced cost. Since the selection operates only on verified negative routes, it affects only column scheduling, not correctness.

### First-incumbent route-pool search

Suggested method text:

> Before a feasible incumbent is available, the algorithm periodically solves a primal route-pool combination problem using the verified route pool. This search is primal-only: it can produce an incumbent or additional valid columns, but it does not alter the lower bound, certify pricing closure, or fathom nodes.

## 8. Interpretation For Paper Revision

The evidence supports the following narrative:

- Persistent process pools and batch productive pricing solved the execution-level bottleneck from the earlier implementation.
- Yield-aware slices and dynamic source-neighbor tasks prevent oversized stale-dual productive calls and improve CPU usage.
- The first-incumbent route-pool search solves the no-incumbent failure on the medium case.
- The remaining unsolved issue is root pricing closure throughput.

The paper should not claim full medium-scale BPC tree efficiency from this run:

```text
sr_cuts_added = 0
branching_nodes = 0
root_closed = false
```

The experiment diagnoses root-node pricing and primal discovery only. It does not evaluate SR-cut separation efficiency or branch-and-bound tree scalability.

## 9. Recommended Next Paper/Code Direction

The next exact research direction should target closure throughput:

1. Add a formal closure-entry policy based on recent productive yield.
2. Add dual-staleness-aware reoptimization triggers after enough diverse columns are collected.
3. Strengthen forward-label dominance because worker label search is now the main cost.
4. Keep route-pool search primal-only, but improve its objective to reduce the upper bound after the first incumbent.
5. Add an ablation table for slice length, diversity fraction, and source-neighbor task size.

These should be framed as exact scheduling and acceleration components. They are not fallback heuristics and should not be used to certify closure unless exhaustive pricing conditions hold.

## 10. Summary For GPT Paper Revision

The code now implements an exact yield-aware productive-pricing schedule for the forward source-neighbor pricing algorithm. It uses a persistent four-process pool, dynamically scheduled one-source-neighbor tasks, bounded worker candidate quotas, deterministic diversity-aware selection of fully verified negative routes, and a primal-only first-incumbent route-pool search.

Numerically, the small case remains optimal with the same objective. On the medium case, the revision changes the outcome from no feasible incumbent to a feasible incumbent at `19.947s`, reduces the maximum pricing-call duration from `121.938s` to `30.421s`, raises weighted process CPU usage from `3.186` to `3.492` core-equivalents, and inserts more verified negative routes. Medium still times out before root closure, so the paper should state that the remaining bottleneck is exact root pricing closure, not process startup, first-hit interruption, or lack of an incumbent.
