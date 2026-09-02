# Current-Paper Numerical Test Report

## Authoritative Large-Scale Crossover Validation

This section is the current authoritative evidence for the large-scale comparison. It uses two immutable 35-customer snapshots, one purely sparse (PS) and one purely clustered (PC), with identical physical data supplied to BPC and direct Gurobi in each pair. All older sections below are retained only as historical development evidence.

The user-fixed settings were unchanged:

```text
region=[0,25]^2 km
2 synchronization pads
4 drones/truck
12 BPC pricing processes
Gurobi Threads=0 (automatic; 12 logical processors used)
3600-second limit per solver
```

The selected scalable dimensions and BPC proof setting were:

```text
35 customers, 7 trucks
pricing_tolerance=0.002
service-window offsets=30-90 minutes
truck_arc_probability=0.05
hub_arc_probability=0.18
mandatory_drone_customer_fraction=0.16
```

### Frozen Instance Identity

| Case | Requested/realized seed | Snapshot SHA-256 | Truck arcs | Drone arcs | Mandatory drone customers | Generation feasibility time (s) |
|---|---|---|---:|---:|---:|---:|
| PS35-101 | 101 / 5,000,116 | `0ae46a5d37aacf2db67cda3ec52284be64b0b4ca3b4a2d757991cb7e5a4d8276` | 136 | 15 | 6 | 990.95 |
| PC35-105 | 105 / 1,000,108 | `fdff3612da8c3cfcf4b6f919ad3f812a29eafa7851c0e5be9633dbc4ca942414` | 140 | 20 | 6 | 977.16 |

Instance-generation time is reported separately and is excluded from both 3,600-second optimization limits. Each BPC/Gurobi pair records the same snapshot hash.

### BPC Versus Direct Gurobi

| Case | Solver | Status | Incumbent | Lower bound | Gap | Runtime (s) | Nodes | Drone sorties |
|---|---|---|---:|---:|---:|---:|---:|---:|
| PS35-101 | BPC | optimal | 0.2138373376 | 0.2138373376 | 0.00% | 1,255.16 | 1 | 14 |
| PS35-101 | Gurobi | time limit | 0.2138373376 | -0.0070612016 | 103.30% | 3,600.34 | 115,442 | 14 |
| PC35-105 | BPC | optimal | 0.1763743003 | 0.1763743003 | 0.00% | 1,578.05 | 1 | 11 |
| PC35-105 | Gurobi | time limit, no incumbent | none | -0.0496071263 | not defined | 3,600.40 | 199,282 | none |

BPC closed both instances entirely at the root. On PS35-101, direct Gurobi found the same incumbent but could not establish a useful lower bound within one hour. On PC35-105, direct Gurobi did not find any feasible incumbent, while BPC produced and certified an optimal solution in 26.3 minutes. Thus the separation is visible in both proof performance and, for PC35-105, primal feasibility.

### BPC Proof Work

| Metric | PS35-101 | PC35-105 |
|---|---:|---:|
| Forward labels generated | 959,794 | 1,563,760 |
| Verified route columns retained | 1,066 | 1,303 |
| SR cuts added | 64 | 32 |
| Dynamic subspace splits | 108 | 81 |
| BPC branch nodes | 0 | 0 |

The BPC advantage comes from root-node route-space proof rather than a smaller branch-and-bound tree after branching. Direct compact Gurobi instead explored large truck/drone-indexed MIP trees and retained weak bounds at termination.

### Tolerance Audit

The initial PS35 pilot at `pricing_tolerance=0.01` terminated with objective `0.2161292123`, which was worse than Gurobi's later incumbent. Tightening the computational tolerance to `0.001` recovered objective `0.2138373376`; the common reported setting `0.002` reproduced that objective and certified root closure in less time. The reported crossover therefore uses `0.002` consistently for both PS and PC, rather than relying on the weaker `0.01` threshold-relative solution.

### Implementation Validation

The XL pilot exposed a race in which a candidate-paused pricing worker could receive an already queued split request. The worker state machine now rejects that split transaction and waits for the master resume/cancel command without losing or duplicating labels. A multi-process regression test covers the race. Static and Gurobi-backed validation after the correction reports:

```text
python -m compileall thvrpd tests
94 passed
```

## Historical Development Results

# Forced Compact Warm-Start Large20 Audit

## Large25 PC Seed-5 Bound-Control Correction and Replacement Run

The prior `large_PC_seed5` BPC artifact was invalid because its stored full lower bound (`0.1647963306`) exceeded its incumbent (`0.1608729579`), while the reported gap was clamped to zero. The error was in branch-and-bound control and reporting, not in the BPC mathematical framework. After a route-pool incumbent improvement, the implementation could continue solving queued nodes whose valid queue bounds already met the incumbent. Final gap computation then hid the invalid bound order by applying a nonnegative clamp.

The production control flow was corrected in three places:

1. The minimum queued bound is tested against the incumbent before the wall-clock termination check. If it meets the incumbent within the integrality tolerance, every remaining queued node is bound-fathomed and the queue closes.
2. Each popped node is tested before `_solve_node`, and a node is tested again immediately after a route-pool incumbent improvement before integer processing or branching.
3. Relative-gap computation now raises an error if `lower_bound > upper_bound`; it no longer converts an invalid bound order into a zero gap.

The audit snapshot writer was also made genuinely atomic by writing a process-specific temporary file and replacing the current snapshot. This prevents a progress-write failure from leaving persistent pricing workers attached to an exited solver. This logging correction does not change pricing, bounds, incumbents, or branching.

Validation after the correction:

```text
python -m compileall thvrpd tests
python -c "import sys, pytest; sys.path.append(r'D:\gurobi1201\win64\python311\lib'); raise SystemExit(pytest.main(['tests','-q']))"

compileall succeeded
43 passed in 8.97s
```

The invalid seed-5 BPC attempt and the failed partial replacement attempt were removed. The authoritative replacement directory contains exactly one attempt:

```text
numerical_experiments\ps_pc_large25_bpc_compact_8case_1h_campaign\cases\large_PC_seed5\bpc\attempt_001
```

### Replacement Result

| Metric | Corrected seed-5 result |
|---|---:|
| Status | time_limit |
| Runtime (s) | 3600.7699 |
| Nodes processed | 29 |
| Root closed | true |
| Root closure time (s) | 382.3752 |
| Full incumbent | 0.1658971648 |
| Full lower bound | 0.1619182503 |
| Full relative gap | 2.3984% |
| Open nodes at termination | 17 |
| Branching nodes | 22 |
| Child nodes created | 44 |
| Bound fathoms immediately after incumbent improvement | 1 |
| Selected routes/trucks | 4 |
| Drone sorties | 4 |
| Service feasible | true |
| Payload feasible | true |

The replacement has the required ordering `lower_bound <= incumbent`. It is not globally closed: the one-hour limit was reached with 17 open nodes and a valid positive gap. The campaign-wide verification now reports `all_bpc_bounds_not_above_incumbents=true`.

The append-only history records three route-pool incumbent improvements. At event 1111, the shifted incumbent improved to `0.4841657699`; event 1112 then fathomed the current node because its bound equaled the new incumbent within tolerance. A complete scan found zero later `node_started` events whose queue bound met or exceeded the incumbent then available. Thus, the redundant-node-start pattern from the deleted run is absent.

### Replacement Incumbent Structure

| Route | Truck path | Drone block | Sorties |
|---:|---|---|---:|
| 1 | Source-C13-H2-C14-H1-C1-Sink | H2: C16, C6; H1: C20, C25 | 4 |
| 2 | Source-C22-C7-H1-C11-C21-C12-C19-Sink | none | 0 |
| 3 | Source-C3-C18-C23-C17-C10-C15-H2-Sink | none | 0 |
| 4 | Source-C9-C2-C24-C4-C8-H2-C5-Sink | none | 0 |

The routes cover all 25 customers exactly once, use four of five available trucks, and have maximum route payload `43.70` under truck capacity `50`. The incumbent contains 21 truck-served customers and four drone-served customers.

### Measured Proof Work

| Metric | Value |
|---|---:|
| Standard pricing time (s) | 1892.8028 |
| Post-root standard pricing time (s) | 1704.1862 |
| RMP time (s) | 629.7332 |
| SR separation time (s) | 619.3819 |
| Forward labels generated | 3,937,857 |
| Labels dominated | 155,204 |
| Labels pruned | 532,227 |
| Complete routes generated | 1,071,841 |
| Route-pool incumbent updates | 3 |

The corrected rerun did not reproduce the deleted run's stronger incumbent. Process-parallel productive pricing can change column discovery and primal route-pool search order, so equal inputs and time limits do not guarantee identical time-limited incumbents. The replacement result is nevertheless internally valid and is now the sole seed-5 BPC artifact used by all regenerated campaign tables.

## Scope

This report documents the requested forced compact Gurobi warm-start revision and the large-only numerical audit. No medium run and no direct standalone compact Gurobi benchmark were run.

Fresh output folders:

```text
numerical_experiments\run_force_compact_large20_seed1_15min
numerical_experiments\run_force_compact_large20_seed2_15min
numerical_experiments\run_force_compact_large20_seed3_15min
```

Common instance and algorithm settings:

```text
20 customers, 5 trucks, 2 hubs, 4 drones/truck
PS distribution, seeds 1, 2, 3
random absolute promised windows, offset 30-90, witness slack 3
truck_speed=40, truck_payload=50, truck_cost=20
drone_payload=6, drone_speed=100, drone_endurance=75, drone_cost=1
pricing_tolerance=0.05
6 process pricing workers
900-second time limit per seed
```

## Code Revision

The production default now forces compact Gurobi after a constructive incumbent:

```text
root_compact_after_constructive = full_budget
root_compact_solve_time_limit = 60.0
root_compact_wall_time_limit = 0.0
```

The compact warm start uses a fixed Gurobi solve budget, not a wall-clock cap for build, solve, extraction, verification, and insertion. If Gurobi returns an incumbent, BPC decodes the returned route paths through `route_from_path` and inserts only verified routes.

During the seed-2 audit, the initial run exposed an objective-consistency bug: the compact model rebuilt `ObjectiveData` internally, including time-limited promised-window witness construction, while BPC verified extracted routes against the root `ObjectiveData`. The code now passes the root objective into `solve_compact_solution`, so compact optimization and BPC route verification use the same service envelopes and objective scaling.

## Validation

Commands:

```powershell
python -m compileall thvrpd tests
python -c "import sys, pytest; sys.path.append(r'D:\gurobi1201\win64\python311\lib'); raise SystemExit(pytest.main(['tests','-q']))"
```

Result:

```text
compileall succeeded
151 passed in 11.06s
```

The Gurobi-backed commands were run under the licensed Windows user.

## Forced-Compact Results

| Metric | Seed 1 | Seed 2 | Seed 3 |
|---|---:|---:|---:|
| Status | time_limit | time_limit | time_limit |
| Runtime (s) | 902.5646 | 900.4643 | 900.5657 |
| Nodes processed | 1 | 1 | 1 |
| Root closed | false | false | false |
| Constructive incumbent found | false | false | false |
| Compact attempted | true | true | true |
| Compact status | success | success | success |
| Compact accepted columns | 4 | 5 | 3 |
| Incumbent source | compact_root | compact_root | compact_root |
| Final incumbent | 0.1656331182 | 0.1402907417 | 0.0986189338 |
| Full lower bound | -0.3053709357 | -0.6032600744 | -0.3039501371 |
| Full gap | 284.37% | 530.01% | 408.21% |
| Best reduced cost at stop | -0.0957795735 | -0.0639856542 | -0.0594941881 |
| Pricing time (s) | 777.7618 | 806.4517 | 805.9902 |
| Selected routes | 4 | 4 | 3 |
| Drone sorties | 5 | 5 | 8 |

The forced-after-constructive policy was validated by regression tests. In the three fresh large20 runs, constructive did not produce an incumbent, so compact was called through the no-constructive root path. The policy change is still active in code and will apply when a constructive incumbent exists.

## Compact Warm-Start Accounting

| Metric | Seed 1 | Seed 2 | Seed 3 |
|---|---:|---:|---:|
| Compact build time (s) | 30.4484 | 0.1075 | 0.1526 |
| Compact Gurobi solve time (s) | 60.0037 | 60.0053 | 60.0045 |
| Compact route decode time (s) | 0.0040 | 0.0030 | 0.0022 |
| BPC `route_from_path` verification time (s) | 0.0010 | 0.0000 | 0.0010 |
| Compact wall budget hit | false | false | false |

The results confirm the intended semantics: Gurobi receives the full 60-second solve budget, and returned incumbents are decoded and inserted afterward. Compact extraction and route verification are negligible compared with the compact solve and root pricing.

## Selected Route Structures

### Seed 1

| Route | Truck path | Drone block | Drone sorties |
|---:|---|---|---:|
| 1 | Source-C10-C16-H2-C2-Sink | H2: C1, C20 | 2 |
| 2 | Source-C14-C8-C4-C11-Sink | none | 0 |
| 3 | Source-C19-H1-C13-C6-H2-Sink | H1: C12, C15, C9 | 3 |
| 4 | Source-C5-C18-C17-C7-C3-Sink | none | 0 |

### Seed 2

| Route | Truck path | Drone block | Drone sorties |
|---:|---|---|---:|
| 1 | Source-C13-C14-H2-C17-H1-Sink | H2: C1, C18, C3 | 3 |
| 2 | Source-C16-C8-H2-C5-Sink | H2: C11, C9 | 2 |
| 3 | Source-C19-C6-C4-C20-H2-C15-Sink | none | 0 |
| 4 | Source-C2-C10-C7-C12-Sink | none | 0 |

### Seed 3

| Route | Truck path | Drone block | Drone sorties |
|---:|---|---|---:|
| 1 | Source-C16-C5-C2-C1-C8-Sink | none | 0 |
| 2 | Source-C19-H1-C13-C14-C10-Sink | H1: C12, C15, C6, C9 | 4 |
| 3 | Source-H2-C7-C17-C20-Sink | H2: C11, C18, C3, C4 | 4 |

All reported incumbents are compact-root incumbents and include drone sorties.

## Pricing Workload

| Metric | Seed 1 | Seed 2 | Seed 3 |
|---|---:|---:|---:|
| Standard pricing calls | 5 | 15 | 5 |
| Forward labels generated | 95,563 | 334,066 | 146,846 |
| Complete routes generated | 12,672 | 70,264 | 37,135 |
| Dominance candidate pairs materialized | 793,057 | 2,105,066 | 1,622,107 |
| Dominance pairs avoided before materialization | 50,961,913 | 58,353,653 | 47,366,067 |
| Weighted effective CPU cores | 4.9700 | 3.8689 | 3.7719 |
| Main-process merge time (s) | 0.0859 | 0.3117 | 0.0819 |

The remaining limitation in all three runs is root-pricing proof throughput. Each run stayed at node 1, root closure remained false, and the best reduced cost at termination was still below the configured tolerance threshold of `-0.05`.

## Interpretation

The forced compact warm start is working as intended. Compact Gurobi returned incumbents for all three large20 seeds, BPC accepted verified compact columns, and each final incumbent came from the compact root warm start.

Across these different random instances, seed 3 has the lowest normalized incumbent value and the largest drone use. Seed 1 has the smallest reported full gap. Because the seeds define different instances and different promised-window realizations, these objective values should not be interpreted as a same-instance method comparison.

The compact warm start improves primal feasibility and drone-using incumbent construction, but it does not address lower-bound proof throughput. The proof bottleneck remains exact forward pricing closure at the root.

## Large-25 Drone-Required Scale Update: 40% Baseline

This baseline used `25 customers, 5 trucks, 2 hubs, 4 drones/truck` with `truck_arc_probability=0.08`, `hub_arc_probability=0.25`, `mandatory_drone_customer_fraction=0.40`, `max_drone_access_customers_per_hub=8`, `max_drone_launch_hubs_per_customer=2`, and `min_drone_service_time_saving=0.0`. The drone-friendly parameters were `drone_payload=6`, `drone_speed=100`, `drone_endurance=75`, and `drone_cost=1`.

Validation after this change:

```text
python -m compileall thvrpd tests
python -c "import sys, pytest; sys.path.append(r'D:\gurobi1201\win64\python311\lib'); raise SystemExit(pytest.main(['tests','-q']))"

compileall succeeded
152 passed in 11.13s
```

Fresh large-25 run:

```text
numerical_experiments\run_large25_drone_required_40pct_lite_p6_s100_e75_c1_mid_tol005_w6_15min
```

Command used `pricing_tolerance=0.05`, six process workers, service windows `30-90` with slack `3`, and a forced 60-second compact warm start.

### Instance Structure

| Metric | Value |
|---|---:|
| Customers | 25 |
| Trucks | 5 |
| Hubs | 2 |
| Drones/truck | 4 |
| Mandatory drone customers | 10 |
| Truck arcs | 81 |
| Drone arcs | 16 |
| Transformed arcs | 265 |
| Witness routes | 4 |
| Witness drone sorties | 10 |

Mandatory drone customers: `C12, C13, C17, C18, C21, C22, C24, C4, C6, C7`. These customers have no truck-service representation and each has at least one retained drone arc, so every feasible solution must use at least 10 drone sorties.

### BPC Result

| Metric | Value |
|---|---:|
| Status | time_limit |
| Runtime (s) | 900.4657 |
| Nodes processed | 1 |
| Root closed | false |
| Best reduced cost at stop | -0.1148513080 |
| Incumbent objective | 0.0861753092 |
| Full lower bound | -0.7784979414 |
| Full gap | 1003.39% |
| Selected trucks | 4 |
| Drone sorties | 10 |
| Truck-served customers | 15 |
| Drone-served customers | 10 |
| Payload feasible | true |
| Service feasible | true |

The incumbent uses exactly the structurally required 10 drone sorties. The selected routes are:

| Route | Truck path | Drone block | Drone sorties |
|---:|---|---|---:|
| 1 | Source-C11-C3-H2-C14-Sink | H2: C13, C18, C21, C4 | 4 |
| 2 | Source-C16-C15-C25-C19-Sink | none | 0 |
| 3 | Source-C20-C10-C5-H1-C2-C1-Sink | H1: C12, C7 | 2 |
| 4 | Source-H2-C8-C23-C9-Sink | H2: C17, C22, C24, C6 | 4 |

### Proof-Throughput Evidence

| Metric | Value |
|---|---:|
| Standard pricing time (s) | 806.7381 |
| Standard pricing calls | 9 |
| Labels generated | 128,020 |
| Labels dominated | 12,397 |
| Complete routes generated | 23,205 |
| Extensions attempted | 145,207 |
| Extensions rejected by deadline | 17,241 |
| Dominance pairs avoided before materialization | 38,146,182 |
| Dominance candidate pairs materialized | 914,369 |
| Same-node dominance tests/deletions | 8,269 |
| Physical-location dominance tests/deletions | 4,128 |
| K-core subspaces | 6 |
| Empty core blocks | 0 |
| Certification calls | 2 |
| Certification unresolved-core count | 8 |
| Minimum core reduced cost observed | -0.3603988152 |
| RMP build time (s) | 0.0160 |
| RMP solve time (s) | 0.0150 |
| SR separation time (s) | 0.0000 |

The run is not limited by RMP construction, LP solve time, or SR separation. The active bottleneck is still exact root-pricing proof throughput: pricing consumed almost all post-warm-start runtime, root closure remained false, and the best reduced cost at termination was below the configured tolerance threshold.

### Compact Warm Start

| Metric | Value |
|---|---:|
| Compact status | success |
| Compact accepted columns | 4 |
| Compact total extraction time (s) | 60.1407 |
| Compact Gurobi solve time (s) | 60.0037 |

The compact warm start produced the accepted incumbent routes and all route decoding/verification passed. Compact is still primal-only; it did not certify the lower bound or root closure.

## Large-25 Protected-Only Drone-Access Update

To reduce pricing state space, the default drone-access policy was tightened after the 40% baseline:

```text
mandatory_drone_customer_fraction = 0.30
max_drone_access_customers_per_hub = 4
max_drone_launch_hubs_per_customer = 1
retain_optional_drone_arcs = false
```

The generator now keeps only one protected launch hub for each mandatory drone customer by default. Nonmandatory optional drone arcs are not retained unless explicitly enabled. This preserves structural drone use while shrinking the duplicated drone-node layer in the transformed pricing network.

Validation after this change:

```text
python -m compileall thvrpd tests
python -c "import sys, pytest; sys.path.append(r'D:\gurobi1201\win64\python311\lib'); raise SystemExit(pytest.main(['tests','-q']))"

compileall succeeded
152 passed in 14.33s
```

Fresh protected-only run:

```text
numerical_experiments\run_large25_drone_required_30pct_protected_only_p6_s100_e75_c1_mid_tol005_w6_15min
```

### Direct Comparison

| Metric | 40% baseline | 30% protected-only |
|---|---:|---:|
| Mandatory drone customers | 10 | 8 |
| Drone arcs | 16 | 8 |
| Transformed arcs | 265 | 177 |
| Witness drone sorties | 10 | 8 |
| Status | time_limit | time_limit |
| Runtime (s) | 900.4657 | 900.4831 |
| Nodes processed | 1 | 1 |
| Root closed | false | false |
| Best reduced cost at stop | -0.1148513080 | -0.0681004403 |
| Incumbent objective | 0.0861753092 | 0.0731124504 |
| Full lower bound | -0.7784979414 | -0.7204886518 |
| Full gap | 1003.39% | 1085.45% |
| Selected trucks | 4 | 4 |
| Drone sorties in incumbent | 10 | 8 |
| Service feasible | true | true |

### Pricing Workload Comparison

| Metric | 40% baseline | 30% protected-only |
|---|---:|---:|
| Standard pricing time (s) | 806.7381 | 801.4583 |
| Standard pricing calls | 9 | 16 |
| Labels generated | 128,020 | 180,892 |
| Labels dominated | 12,397 | 15,741 |
| Complete routes generated | 23,205 | 52,769 |
| Extensions attempted | 145,207 | 244,196 |
| Extensions rejected by deadline | 17,241 | 63,400 |
| Dominance pairs avoided before materialization | 38,146,182 | 25,389,590 |
| Dominance candidate pairs materialized | 914,369 | 896,357 |
| Same-node dominance deletions | 8,269 | 10,282 |
| Physical-location dominance deletions | 4,128 | 5,459 |
| Certification unresolved-core count | 8 | 8 |

The protected-only policy substantially reduced the transformed network size and moved the best reduced cost much closer to the configured closure threshold: from `-0.1148513080` to `-0.0681004403`. This is evidence that reducing drone accessibility helps the proof side. However, root closure still failed because the best reduced cost remained below `-0.05`.

The pricing workload changed rather than uniformly decreasing. Fewer drone duplicate nodes reduced dominance pair pressure, but the solver completed more pricing calls and generated more complete routes within the same runtime. The remaining proof bottleneck is still certification of all source-neighbor core subspaces at the root, not compact warm start, RMP solve, or SR separation.

## Large-25 Medium-Like Setting And Compact Arc Benchmark

The retained large-lite target keeps the named large instance at `25 customers, 5 trucks, 2 hubs, 4 drones/truck`, keeps the structurally non-truck-only policy, and uses the original root-closed `425.017s` sparse graph. The production defaults are:

```text
truck_arc_probability = 0.05
hub_arc_probability = 0.18
mandatory_drone_customer_fraction = 0.16
max_drone_access_customers_per_hub = 2
max_drone_launch_hubs_per_customer = 1
retain_optional_drone_arcs = false
drone_payload = 6
drone_speed = 100
drone_endurance = 75
drone_cost = 1
```

This gives four mandatory drone customers for the 25-customer instance. Because those customers have no truck-service representation, every feasible solution must use drones; hence the optimal solution cannot be truck-only.

### Density Tuning Evidence

The table compares the verified sparse-density probes. The user-selected retained setting is `0.05/0.18`, which is the original `425.017s` root-closed case. The denser `0.06/0.20` case was tested and is reported for context, but it is not retained as the default.

| Truck arc prob. | Hub arc prob. | Truck arcs | Drone arcs | Transformed arcs | Runtime (s) | Nodes | Root closed | Root standard pricing (s) | Final best reduced cost | Comment |
|---:|---:|---:|---:|---:|---:|---:|:---:|---:|---:|---|
| 0.05 | 0.18 | 111 | 4 | 145 | 600.332 | 2 | true | 425.017 | -0.071492298 | Retained default; root closes quickly and remains structurally non-truck-only. |
| 0.05 | 0.20 | 112 | 4 | 146 | 208.023 | 1 | true | 101.528 | -0.041575440 | Too easy; full run closed quickly. |
| 0.058 | 0.19 | 113 | 4 | 147 | 900.282 | 3 | true | 388.921 | -0.035092116 | Intermediate probe; farther from 600s than the retained default. |
| 0.058 | 0.20 | 114 | 4 | 148 | 900.423 | 3 | true | 251.807 | -0.073206513 | Nonmonotone effect; root became easier, post-root became harder. |
| 0.06 | 0.19 | 114 | 4 | 148 | 900.507 | 1 | false | 785.415 | -0.058361059 | Root did not close under the 900s run, so not selected. |
| 0.065 | 0.18 | 115 | 4 | 149 | 900.281 | 2 | true | 263.123 | -0.028322143 | Denser graph but easier at the root. |
| 0.06 | 0.20 | 115 | 4 | 149 | 900.359 | 2 | true | 754.264 | -0.109343882 | Denser alternative; not retained after the user selected the 425.017s setting. |

The pricing difficulty is not monotone in arc count because the specific added arcs change which source-neighbor subproblems, SR cuts, dominance frontiers, and post-root branches become active. Therefore the retained setting is based on the user's selected 425.017s run, with the denser probes kept only as sensitivity evidence.

### Selected Instance Structure

| Metric | Value |
|---|---:|
| Customers | 25 |
| Trucks | 5 |
| Hubs | 2 |
| Drones/truck | 4 |
| Mandatory drone customers | 4 |
| Truck arcs | 111 |
| Drone arcs | 4 |
| Transformed nodes | 33 |
| Transformed arcs | 145 |
| Witness routes | 4 |
| Witness drone sorties | 4 |

Mandatory drone customers: `C12, C13, C18, C21`.

### Selected Run Result

```text
numerical_experiments\run_large25_mediumlike_drone_required_16pct_root600_p6_s100_e75_c1_tol005_w6
```

| Metric | Value |
|---|---:|
| Status | time_limit |
| Runtime (s) | 600.3323 |
| Nodes processed | 2 |
| Root closed | true |
| Root standard pricing time (s) | 425.0171 |
| Post-root standard pricing time (s) | 45.6460 |
| Total standard pricing time (s) | 470.6631 |
| Standard pricing calls | 31 |
| Labels generated | 252,308 |
| Completed routes generated | 71,325 |
| Dominance candidate pairs materialized | 575,051 |

The root pricing proof closed under the configured `pricing_tolerance=0.05`. The final negative `best_reduced_cost_at_stop = -0.071492298` is post-root evidence after branching, not a root-closure failure.

### Incumbent And Feasibility

| Metric | Value |
|---|---:|
| Incumbent objective | 0.1094226658 |
| Full lower bound | 0.0878163258 |
| Full gap | 19.75% |
| Selected trucks | 4 |
| Drone sorties | 4 |
| Truck-served customers | 21 |
| Drone-served customers | 4 |
| Payload feasible | true |
| Service feasible | true |

### Direct Compact Arc-Based Gurobi Benchmark

The direct compact MIQP benchmark uses the same retained `0.05/0.18` promised-window instance and calls the existing arc-based compact solver for a 20-minute Gurobi solve limit:

```text
numerical_experiments\run_compact_arc_large25_mediumlike_425param_20min
```

| Metric | Compact arc-based Gurobi |
|---|---:|
| Gurobi status | time limit |
| Extracted incumbent status | success |
| Runtime (s) | 1200.1017 |
| Model build time (s) | 0.0754 |
| Gurobi solve time (s) | 1200.0042 |
| Route decode time (s) | 0.0020 |
| Incumbent objective | 0.0727504027 |
| Best bound | -0.2940004425 |
| MIP gap | 504.12% |
| Explored nodes | 14,021 |
| Simplex iterations | 12,757,263 |
| Route count | 4 |
| Drone sorties | 4 |
| Payload feasible | true |
| Service feasible | true |

Compact route structure:

| Route | Truck path | Drone sorties | Served customers |
|---:|---|---:|---|
| 0 | Source-C5-C20-C8-C22-C3-C4-Sink | 0 | C20, C22, C3, C4, C5, C8 |
| 1 | Source-C10-H2-C14-C9-C6-C23-Sink | 2 | C10, C13, C14, C18, C23, C6, C9 |
| 2 | Source-C16-C15-C2-C1-Sink | 0 | C1, C15, C16, C2 |
| 3 | Source-H1-C24-C7-C25-C19-C17-H2-C11-Sink | 2 | C11, C12, C17, C19, C21, C24, C25, C7 |

The compact model found a substantially stronger feasible incumbent than the BPC 600-second run on the same instance (`0.0727504027` versus `0.1094226658`). Its proof bound is much weaker than the BPC root lower bound (`-0.2940004425` versus `0.0878163258`). Therefore, within 20 minutes the compact arc-based model is useful for primal incumbent discovery, but not for proving a strong lower bound on this instance.

### One-Hour BPC Comparison Against Compact Incumbent

The retained large-25 case was rerun with BPC for a one-hour time limit:

```text
numerical_experiments\run_bpc_large25_425param_1h
```

The run terminated early because the BPC root closed under the configured `pricing_tolerance=0.05`.

| Metric | BPC 1-hour limit | Compact arc 20-minute |
|---|---:|---:|
| Status | optimal under pricing tolerance | time limit |
| Runtime (s) | 200.5375 | 1200.1017 |
| Nodes processed / explored | 1 | 14,021 |
| Incumbent objective | 0.1056060693 | 0.0727504027 |
| Lower/best bound | 0.1056060693 | -0.2940004425 |
| Reported gap | 0.00% | 504.12% |
| Drone sorties | 4 | 4 |
| Service feasible | true | true |
| Payload feasible | true | true |

BPC did not beat the compact incumbent. The compact incumbent is better by `0.0328556666` in full normalized objective value.

BPC route structure:

| Route | Truck path | Drone sorties | Served customers |
|---:|---|---:|---|
| 33 | Source-C19-C17-C24-C7-C25-C14-H2-C4-Sink | 0 | C14, C17, C19, C24, C25, C4, C7 |
| 31 | Source-C20-C10-H2-C9-C6-H1-C1-Sink | 2 | C1, C10, C13, C18, C20, C6, C9 |
| 32 | Source-C5-H1-C16-C15-C2-Sink | 2 | C12, C15, C16, C2, C21, C5 |
| 30 | Source-C8-C22-C3-H2-C11-C23-Sink | 0 | C11, C22, C23, C3, C8 |

The compact route set was checked against the BPC run's service upper bounds and had no service-window violations. Its route-cost sum plus the BPC objective shift equals `0.0727504027`, so the objective comparison is on the same scale.

The reason BPC can report `optimal` while the compact incumbent is better is the configured pricing tolerance. The BPC run stopped with `best_reduced_cost_at_stop=-0.0496311555`, which is inside the `0.05` tolerance and is therefore treated as nonnegative by the production pricing logic. This is a tolerance-closed solution, not an exact zero-reduced-cost closure.

Validation after the retained default update:

```text
python -m compileall thvrpd tests
python -c "import sys, pytest; sys.path.append(r'D:\gurobi1201\win64\python311\lib'); raise SystemExit(pytest.main(['tests','-q']))"

compileall succeeded
152 passed in 10.47s
```

This is the retained large-lite setting. It remains a sparse 25-customer extension of the 15-customer V4 medium case, structurally prevents truck-only solutions, and has a verified root pricing proof time of `425.017s`.

## Balanced Dynamic K-Core Large25 Audit

This section reports the resumed large-only efficiency audit after implementing balanced dynamic K-core pricing. No medium run was performed, and no direct compact Gurobi benchmark was rerun. The compact arc result below is reused only as existing context.

Fresh output folder:

```text
numerical_experiments\run_balanced_dynamic_kcore_large25_tol001_w6_1h_resumed
```

Run setting:

```text
25 customers, 5 trucks, 2 hubs, 4 drones/truck
PS distribution, seed 1
pricing_tolerance=0.01
6 process pricing workers
time_limit=3600
random absolute promised windows, offset 30-90, witness slack 3
drone_payload=6, drone_speed=100, drone_endurance=75, drone_cost=1
truck_arc_probability=0.05, hub_arc_probability=0.18
```

Validation after the balanced dynamic K-core implementation:

```text
python -m compileall thvrpd tests
python -c "import sys, pytest; sys.path.append(r'D:\gurobi1201\win64\python311\lib'); raise SystemExit(pytest.main(['tests','-q']))"

compileall succeeded
154 passed in 11.12s
```

### Main Result

| Metric | Balanced dynamic K-core BPC |
|---|---:|
| Status | optimal under `pricing_tolerance=0.01` |
| Runtime (s) | 2036.1661 |
| Nodes processed | 7 |
| Post-root nodes processed | 6 |
| Root closed | true |
| Full incumbent | 0.0695884199 |
| Full lower bound | 0.0695884199 |
| Full gap | 0.00% |
| Truck arcs | 111 |
| Drone arcs | 4 |
| Mandatory drone customers | 4: C12, C13, C18, C21 |
| Selected routes | 4 |
| Drone sorties | 4 |

Selected route structure:

| Route | Path | Drone sorties | Served customers |
|---:|---|---:|---|
| 450 | Source-C10-H2-C14-C9-C6-C23-Sink | 2 | C10, C13, C14, C18, C23, C6, C9 |
| 444 | Source-C16-C15-C2-C1-Sink | 0 | C1, C15, C16, C2 |
| 449 | Source-C5-C20-C8-C22-C3-C4-Sink | 0 | C20, C22, C3, C4, C5, C8 |
| 427 | Source-H1-C24-C7-C25-C19-C17-H2-C11-Sink | 2 | C11, C12, C17, C19, C21, C24, C25, C7 |

The four drone sorties are exactly the structurally mandatory drone customers: C13 and C18 from H2, and C12 and C21 from H1. The final solution is therefore not truck-only.

### Comparison

| Metric | Balanced dynamic K-core BPC | Previous BPC, `tol=0.05` | Existing compact arc 20-min |
|---|---:|---:|---:|
| Output folder | `run_balanced_dynamic_kcore_large25_tol001_w6_1h_resumed` | `run_bpc_large25_425param_1h` | `run_compact_arc_large25_mediumlike_425param_20min` |
| Runtime (s) | 2036.1661 | 200.5375 | 1200.1017 |
| Status | optimal | optimal under `tol=0.05` | time limit, incumbent extracted |
| Nodes processed / explored | 7 | 1 | 14,021 |
| Incumbent objective | 0.0695884199 | 0.1056060693 | 0.0727504027 |
| Lower/best bound | 0.0695884199 | 0.1056060693 | -0.2940004425 |
| Gap | 0.00% | 0.00% | 504.12% |
| Drone sorties | 4 | 4 | 4 |

The balanced dynamic BPC run improves the previous BPC incumbent by `0.0360176494` and improves the existing compact arc incumbent by `0.0031619828`. Unlike the compact arc run, it also proves the matching lower bound under the configured pricing tolerance.

The earlier `run_bpc_large25_425param_tol001_1h` folder contains a progress snapshot with root closure and no final solve JSON. It is therefore used only for pricing-throughput context, not for incumbent comparison.

### Pricing And Parallel Workload

| Metric | Balanced dynamic K-core BPC |
|---|---:|
| Standard pricing time (s) | 1550.3974 |
| Share of total runtime | 76.14% |
| RMP time (s) | 146.3795 |
| RMP share | 7.19% |
| SR separation time (s) | 176.4038 |
| SR share | 8.66% |
| Compact warm-start solve time (s) | 60.0050 |
| Compact share | 2.95% |
| Heuristic time (s) | 1.7377 |
| Pricing labels generated | 2,777,844 |
| Labels dominated | 124,201 |
| Labels pruned | 616,818 |
| Complete routes generated | 803,514 |
| Maximum queue size | 335 |
| RMP solves | 229 |
| SR cuts added | 387 |

Pricing remains the dominant proof cost. RMP and SR work are material but secondary. The compact warm start is a small part of total runtime and remained primal-only.

### Balanced K-Core And Dynamic Refinement Diagnostics

| Metric | Value |
|---|---:|
| Process workers | 6 |
| Maximum CPU-core equivalent | 5.7235 |
| Weighted CPU-core equivalent | 4.7207 |
| Pricing process CPU time (s) | 7309.5000 |
| Worker CPU time (s) | 7304.1875 |
| Main-process CPU time (s) | 5.3125 |
| Main merge time (s) | 5.5194 |
| Idle worker seconds | 1848.0260 |
| Initial load imbalance, max/mean | 1.0565 |
| Empty core blocks | 0 |
| Split candidates | 1,150 |
| Splits performed | 950 |
| Dynamic child tasks created | 13,895 |
| Labels transferred to child tasks | 13,895 |
| Leaf tasks closed | 13,152 |
| Max open labels by task | 335 |

The balanced assignment produced a small initial load imbalance (`1.0565` max/mean) and no empty core blocks. Dynamic refinement was active: 950 of 1,150 split candidates were split, creating 13,895 child tasks. CPU utilization improved relative to the prior static-tolerance progress context (`4.7207` weighted core-equivalent versus `2.7265` in `run_bpc_large25_425param_tol001_1h`). Main-process merge and control time was only `5.5194s`, so serial merge/control is not the limiting factor in this run.

Idle worker time is still measurable: `1848.0260` worker-seconds, about 19.9% of the theoretical six-worker pricing wall-clock capacity over standard pricing time. The remaining idle time occurs despite dynamic refinement, so the residual imbalance is in the tail of task exhaustion rather than in the initial source-neighbor assignment.

### Dominance And Closure-Frontier Workload

| Metric | Value |
|---|---:|
| Dominance frontier queries | 3,390,320 |
| Frontier keys scanned | 4,314,525 |
| Frontier keys skipped by mask | 41,510,177 |
| Bucket pairs considered | 3,462,208 |
| Candidate pairs materialized | 3,799,965 |
| Pairs avoided before materialization | 56,113,954 |
| Full same-node tests | 103,356 |
| Full physical-location tests | 20,845 |
| Labels deleted by same-node dominance | 103,356 |
| Labels deleted by physical-location dominance | 20,845 |
| Physical-location dominance time (s) | 28.2145 |
| Resource reward-bound calls | 1,633,679 |
| Resource reward-bound time (s) | 125.1060 |
| Labels certified by cell lower bounds | 780,388 |

The dominance-frontier gates avoided a large number of impossible comparisons before label-pair materialization. Full theorem checks were still substantial but much smaller than the avoided-pair count. Physical-location dominance consumed `28.2145s`, which is not negligible but is not the dominant bottleneck relative to total standard pricing time. Resource reward bounds consumed `125.1060s`, making reduced-cost pruning and closure-bound evaluation a larger measured pricing-side subcomponent than physical-location dominance.

### Bottleneck Diagnosis

The measured bottleneck is still lower-bound proof throughput in forward pricing, not primal incumbent discovery. The incumbent is strong enough to beat the compact arc incumbent, and the proof eventually closes. The dominant cost is the volume of exact forward-pricing work required across root and post-root certification:

```text
standard pricing time = 1550.3974s
labels generated = 2,777,844
complete routes generated = 803,514
resource reward-bound calls = 1,633,679
RMP solves = 229
SR cuts added = 387
```

The balanced dynamic K-core scheduler improved processor use and reduced the original idle-worker problem. It did not eliminate tail imbalance completely. The evidence is:

```text
weighted CPU-core equivalent = 4.7207 / 6
initial load imbalance max/mean = 1.0565
idle worker seconds = 1848.0260
dynamic splits performed = 950
```

The proof-side secondary burden is active RMP/SR cycling after root closure:

```text
RMP time = 146.3795s
SR separation time = 176.4038s
RMP solves = 229
SR cuts added = 387
nodes processed = 7
```

Thus, after balanced dynamic K-core scheduling, the bottleneck is no longer gross CPU underuse or serial merge/control. It is exact pricing volume plus repeated RMP/SR proof maintenance.

### Future Revision Directions From This Run

1. Dynamic refinement tail scheduling should focus on late certification tails. Initial load balancing is already effective (`1.0565` max/mean), but `1848.0260` idle worker-seconds remain. The next revision should measure whether late-stage leaf tasks with small open queues but long dominance/frontier histories are causing tail idle time.

2. Resource reward-bound evaluation is a larger measured subcomponent than physical-location dominance. The run spent `125.1060s` in resource reward-bound calls versus `28.2145s` in physical-location dominance. Any future pruning revision should separately report reward-bound cache hits, repeated location states, and the distribution of reachable-customer set sizes.

3. Active SR and RMP proof maintenance needs its own post-root budget accounting. The run added 387 SR cuts and solved 229 RMPs, consuming `322.7833s` combined RMP and SR time. Future revisions should report active-row density by node and distinguish coefficient materialization from LP solve time in the post-root nodes.

4. Dominance-frontier gates are effective but still materialize millions of pairs. The run avoided `56,113,954` pairs before materialization, but still materialized `3,799,965` candidate pairs. The next revision should report materialized-pair distribution by task and by frontier cell, because the bottleneck may be concentrated in a small number of high-density cells.

5. Serial merge/control is not the current limiting component. Main merge time was `5.5194s`, and main-process CPU time was `5.3125s`, compared with `7304.1875s` of worker CPU time. Future parallel work should therefore target task splitting, closure-bound reuse, and dominance-frontier density rather than moving merge logic into separate processors.

6. The compact warm start remains useful but is not the proof bottleneck. It accepted 5 root columns after a 60-second solve and contributed to the final route pool, but total proof time was driven by pricing and RMP/SR cycling. Future warm-start revisions should be evaluated by incumbent quality and column diversity, not by lower-bound proof time.

## Direct Compact Gurobi Same-Case 1-Hour Benchmark

This section reports a direct compact arc-based MIQP run on the same 25-customer case as the balanced dynamic K-core BPC audit. This was a standalone compact benchmark, not a BPC warm start and not a new BPC run.

Output folder:

```text
numerical_experiments\run_compact_arc_large25_samecase_1h
```

Run setting:

```text
25 customers, 5 trucks, 2 hubs, 4 drones/truck
PS distribution, seed 1
time_limit=3600
threads=1
random absolute promised windows, offset 30-90, witness slack 3
drone_payload=6, drone_speed=100, drone_endurance=75, drone_cost=1
truck_arc_probability=0.05, hub_arc_probability=0.18
```

The compact benchmark JSON reports `status=success` because a feasible incumbent was extracted and decoded. Gurobi itself terminated by time limit: `status_code=9`, with the log line `Time limit reached`.

### Compact Gurobi Result

| Metric | Direct compact Gurobi |
|---|---:|
| Runtime (s) | 3600.1241 |
| Gurobi status | time limit |
| Extracted incumbent objective | 0.0892879979 |
| Best bound | -0.2941049923 |
| MIP gap | 429.3892% |
| Explored nodes | 48,936 |
| Simplex iterations | 41,816,339 |
| Transformed nodes | 33 |
| Transformed arcs | 145 |
| Selected routes | 4 |
| Drone sorties | 4 |
| Model build time (s) | 0.0940 |
| Solve time (s) | 3600.0026 |
| Route decode time (s) | 0.0015 |

Selected compact route structure:

| Route | Truck path | Drone block | Drone sorties | Served customers |
|---:|---|---|---:|---|
| 1 | Source-C10-C25-C19-C17-C24-C7-H2-C11-Sink | H2: C13, C18 | 2 | C10, C11, C13, C17, C18, C19, C24, C25, C7 |
| 2 | Source-C5-C20-C8-C22-C3-C4-Sink | none | 0 | C20, C22, C3, C4, C5, C8 |
| 3 | Source-C16-C23-C15-C2-Sink | none | 0 | C15, C16, C2, C23 |
| 4 | Source-C14-C9-C6-H1-C1-Sink | H1: C12, C21 | 2 | C1, C12, C14, C21, C6, C9 |

### Direct Comparison With Balanced Dynamic K-Core BPC

| Metric | Balanced dynamic K-core BPC | Direct compact Gurobi |
|---|---:|---:|
| Output folder | `run_balanced_dynamic_kcore_large25_tol001_w6_1h_resumed` | `run_compact_arc_large25_samecase_1h` |
| Runtime (s) | 2036.1661 | 3600.1241 |
| Termination | proven optimal under `pricing_tolerance=0.01` | time limit with incumbent |
| Incumbent objective | 0.0695884199 | 0.0892879979 |
| Lower/best bound | 0.0695884199 | -0.2941049923 |
| Gap | 0.00% | 429.3892% |
| Search nodes | 7 BPC nodes | 48,936 compact MIP nodes |
| Drone sorties | 4 | 4 |

The BPC run beats the direct compact MIQP on both primal quality and proof quality for this case. Its incumbent is lower by `0.0196995780`, and it proves optimality under the configured pricing tolerance in `2036.1661s`. The compact MIQP still has a very weak proof bound after the full hour, despite exploring 48,936 branch-and-bound nodes and 41.8 million simplex iterations.

The direct compact result reinforces the same bottleneck interpretation: the compact arc-based model can find drone-using incumbents, but its lower-bound proof is substantially weaker than the route-space BPC proof on this sparse, promised-window, drone-required instance.

## Consistent Scale Settings And Medium 30-Minute Comparison

The large 25-customer setting used above implies the following scale-consistent instance family. The graph-generation policy, drone-required policy, service-window policy, objective weights, and drone-friendly physical parameters are held fixed across scales; only the customer and truck counts change.

| Scale | Customers | Trucks | Hubs | Drones/truck | Truck arc probability | Hub arc probability | Mandatory drone fraction | Max drone customers/hub | Max launch hubs/customer |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Small | 5 | 2 | 2 | 4 | 0.05 | 0.18 | 0.16 | 2 | 1 |
| Medium | 15 | 5 | 2 | 4 | 0.05 | 0.18 | 0.16 | 2 | 1 |
| Large | 25 | 5 | 2 | 4 | 0.05 | 0.18 | 0.16 | 2 | 1 |

Common physical and algorithmic settings:

```text
distribution=PS, seed=1
weights=(0.4, 0.3, 0.3)
truck_speed=40, drone_speed=100
truck_payload=50, drone_payload=6
drone_endurance=75
truck_cost=20, drone_cost=1
service_deadline_mode=random_absolute
service_deadline_offset_min=30
service_deadline_offset_max=90
service_deadline_witness_slack=3
service_deadline_witness_method=constructive_then_compact
pricing_tolerance=0.01 for BPC
pricing_parallel_workers=6 for BPC
root_compact_after_constructive=full_budget
root_compact_solve_time_limit=60
```

Fresh medium output folders:

```text
numerical_experiments\run_medium15_samecase_bpc_tol001_w6_30min
numerical_experiments\run_compact_arc_medium15_samecase_30min
```

Both runs used a 30-minute time limit. Both terminated before the limit.

### Medium Result Summary

| Metric | BPC | Direct compact Gurobi |
|---|---:|---:|
| Runtime (s) | 63.3401 | 97.5358 |
| Termination | optimal | optimal |
| Objective | 0.1629495288 | 0.1629495288 |
| Lower/best bound | 0.1629495288 | 0.1629495288 |
| Gap | 0.00% | 0.00% |
| BPC/root or MIP nodes | 1 BPC node | 25,008 MIP nodes |
| Simplex iterations | not applicable | 2,125,807 |
| Selected routes | 3 | 3 |
| Drone sorties | 3 | 3 |
| Mandatory drone customers | 3 | 3: C12, C15, C7 |
| Transformed nodes | 22 | 22 |
| Transformed arcs | 75 | 75 |

The medium BPC run closed at the root. Its total runtime includes the forced compact warm start:

| BPC component | Value |
|---|---:|
| Root compact status | success |
| Root compact accepted columns | 3 |
| Standard pricing time (s) | 1.2326 |
| RMP time (s) | 0.0216 |
| SR separation time (s) | 0.1690 |
| Labels generated | 3,974 |
| Complete routes generated | 1,019 |
| Weighted CPU-core equivalent | 1.3365 |

The direct compact MIQP also solved the medium instance, but needed a substantially larger MIP tree:

```text
Explored 25,008 compact MIP nodes
2,125,807 simplex iterations
Best objective = 0.1629495288438
Best bound = 0.1629495288438
```

### Medium Route Structure

The BPC and compact Gurobi solutions select the same route set, up to route ordering:

| Route | Path | Drone sorties | Served customers |
|---:|---|---:|---|
| 1 | Source-C10-C8-C14-C2-C9-Sink | 0 | C10, C14, C2, C8, C9 |
| 2 | Source-H1-C15 drone block-C11-C6-C1-Sink | 1 | C1, C11, C15, C6 |
| 3 | Source-H2-C12/C7 drone block-C3-C5-C4-C13-Sink | 2 | C12, C13, C3, C4, C5, C7 |

The three drone sorties are exactly the mandatory drone customers in this medium instance: C12, C15, and C7. Thus the medium result is also not truck-only.

### Medium Interpretation

On the medium scale, both approaches prove optimality within the 30-minute budget. BPC is faster in wall-clock time (`63.3401s` versus `97.5358s`) and proves the same solution with one root node, while compact Gurobi explores 25,008 MIP nodes. The BPC runtime is dominated by the forced 60-second compact warm start; the route-space proof itself is very short, with only `1.2326s` of standard pricing time and `3,974` labels generated.
