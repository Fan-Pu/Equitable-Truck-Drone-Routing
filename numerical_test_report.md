# Revised V4 Proof-Aligned TH-VRPD Numerical Test Report

## Scope

This report documents the revised V4 proof-aligned solver audits generated in:

```text
numerical_experiments\run_restored_v4_proof_revised_medium_10min
numerical_experiments\run_restored_v4_proof_revised_large_20min
```

The medium run is compared with the available medium 10-minute folders:

```text
numerical_experiments\run_after_postroot_v4_medium_10min
numerical_experiments\run_restored_postroot_v4_medium_10min
numerical_experiments\run_after_postroot_v5_medium_10min
numerical_experiments\run_after_postroot_v6_medium_10min
```

The tested solver is the promised-service-window restricted TH-VRPD solver with forward-only source-neighbor process pricing, physical-location dominance with return-time credit, verified route-pool incumbent search, branch-indexed inheritance, incremental RMP updates, active-SR coefficient caching, active-SR lifecycle metadata, inactive-column hydration, and structured child-certification epochs. The large run uses the same promised-window policy as the medium run and extends the time limit to 20 minutes.

## Verification

The requested validation commands were run.

```text
python -m compileall thvrpd tests
python -c "import sys, pytest; sys.path.append(r'D:\gurobi1201\win64\python311\lib'); raise SystemExit(pytest.main(['tests','-q']))"
```

Result:

```text
compileall succeeded
140 passed in 12.42s
```

The Gurobi-backed pytest command was executed outside the sandbox because the sandbox user did not match the installed Gurobi license user. The same command passed under the licensed Windows user.

## Audit Command

```powershell
python -m thvrpd.experiments --output-dir numerical_experiments\run_restored_v4_proof_revised_medium_10min --seeds 1 --distributions PS --scales medium --threads 1 --case-time-limit 600 --external-timeout-grace 60 --pricing-parallel-workers 4 --pricing-worker-backend process --service-deadline-mode random_absolute --service-deadline-offset-min 45 --service-deadline-offset-max 120 --service-deadline-witness-slack 5 --service-deadline-witness-method constructive_then_compact --root-compact-after-constructive conditional_wall_budget --prefix-task-depth-child 2 --logging-mode light --gurobi-python-path D:\gurobi1201\win64\python311\lib
```

## Main Results

| Run | Status | Incumbent | Lower bound | Full gap | Root closed | Nodes | Post-root open nodes |
|---|---:|---:|---:|---:|---|---:|---:|
| Recorded V4 | time limit | 0.0846704491 | 0.0796601339 | 5.92% | yes | 7 | 5 |
| Restored V4 | time limit | 0.0811406255 | 0.0783032343 | 3.50% | yes | 7 | 5 |
| V5 | time limit | 0.2257310146 | 0.0783032343 | 65.31% | yes | 7 | 7 |
| V6 | time limit | 0.2257310146 | 0.0798413484 | 64.63% | yes | 10 | 10 |
| Revised proof V4 | time limit | 0.0811406255 | 0.0800923520 | 1.29% | yes | 11 | 7 |

The revised proof-aligned implementation preserves the strong restored V4 incumbent value, improves the lower bound beyond both recorded V4 and restored V4, and reduces the full gap to 1.29%. The run remains time-limited, so this is not an optimality proof.

## Incumbent Structure

The revised proof V4 incumbent uses three truck-only routes and no drone sorties.

| Route | Truck path | Customers | Return time | Shifted route cost |
|---:|---|---|---:|---:|
| 366 | Source-C5-C10-C7-H2-C11-Sink | C5, C7, C10, C11 | 79.1671 | 0.0546946 |
| 560 | Source-C8-C3-H2-C6-C13-C4-C14-Sink | C3, C4, C6, C8, C13, C14 | 75.5302 | 0.0564077 |
| 336 | Source-C9-C1-C2-C12-H1-C15-Sink | C1, C2, C9, C12, C15 | 80.2829 | 0.0542687 |

Objective components for this incumbent:

| Component | Value |
|---|---:|
| Raw delay-square sum | 5,563.7747 |
| Raw return-time sum | 234.9803 |
| Raw operating cost | 60.0000 |
| Weighted delay contribution | 0.0199635 |
| Weighted return contribution | 0.0211772 |
| Weighted cost contribution | 0.0400000 |
| Full normalized objective | 0.0811406255 |

The good incumbent was produced by the V4 primal path: root constructive incumbent, verified route-pool construction, support-pool hard solves, full node-admissible hard-pool solves, and true route-cost incumbent evaluation. The route-pool layer made five incumbent updates; two came from support-pool hard solves and three came from full-pool hard solves. This confirms that the full node-admissible hard-pool step is active and materially contributes to incumbent quality.

No drone sortie is used in the final solution. Therefore, the stronger incumbent in this case is not explained by drone-service insertion. It is explained by better verified truck-route composition from the route-pool incumbent mechanism.

## Route-Pool Behavior

| Diagnostic | Revised proof V4 |
|---|---:|
| Heuristic incumbent updates | 5 |
| Support-pool hard calls | 18 |
| Support-pool feasible solves | 2 |
| Support-pool incumbent updates | 2 |
| Full-pool hard calls | 12 |
| Full-pool feasible solves | 12 |
| Full-pool incumbent updates | 3 |
| Hard-pool solve time | 0.5792s |
| Soft-pool calls | 9 |
| Repair-pricing time | 1.3587s |
| Maximum node pool size | 614 |
| Maximum support size | 29 |
| Maximum node/support ratio | 33.47 |

The measured bottleneck is not the route-pool solve time. Hard-pool solving consumed less than one second, while the full-pool calls generated most of the incumbent improvements. The large node/support ratio shows why the full node-admissible hard-pool phase matters: the support pool can be very small relative to the verified node pool, and support-only search would miss some high-quality integer combinations.

## Dominance-Throughput Diagnostics

| Diagnostic | Revised proof V4 |
|---|---:|
| Forward labels generated | 5,438,809 |
| Dominance gate pairs seen | 107,643,334 |
| Gate mask failures | 90,583,850 |
| Gate scalar failures | 11,962,108 |
| Gate branch failures | 2,807,429 |
| Gate deadline failures | 31,908 |
| Return-credit checks executed | 7,348,349 |
| Return-credit checks skipped | 100,294,985 |
| Labels dominated | 2,258,011 |
| Same-node dominated labels | 1,873,372 |
| Physical-location dominated labels | 384,639 |

The dominance gates are highly selective. Only 7.35 million of 107.64 million candidate pairs reached the return-credit stage, so 93.17% of candidate pairs were filtered before the expensive full dominance evaluation. The largest screening source is customer/physical-node mask containment, which accounts for 90.58 million failures. Physical-location dominance is also productive: it removes 384,639 labels, approximately 17.03% of all dominated labels.

The remaining pricing bottleneck is still proof throughput. Standard pricing consumed 468.47s, or 77.88% of the BPC wall time. Worker CPU time was 1,628.92s across process workers, and forward-labeling CPU accounting was 1,633.33s. The wall-clock bottleneck is therefore parallel forward-pricing certification and dominance work, not LP solving.

## Child-Certification Epochs

| Diagnostic | Revised proof V4 |
|---|---:|
| Child certification calls | 94 |
| Calls with columns | 71 |
| Exhausted calls | 23 |
| Time-limited child calls | 0 |
| Epochs started | 94 |
| Epochs completed | 23 |
| Saved states | 94 |
| Resumed states | 0 |
| State discards | 85 |
| Discards by active columns | 74 |
| Discards by dual change | 14 |
| Discards by SR change | 11 |
| Exhausted source-neighbor tasks | 1,527 |
| Unresolved source-neighbor tasks | 0 |

Certification is exact when it closes, but the measured epoch reuse is poor. No saved child-certification state was resumed. Most discards are caused by active-column changes after productive child certification returns columns. Dual and SR changes also invalidate saved epochs. This identifies child-node proof throughput as a real bottleneck: the solver repeatedly starts valid certification epochs, but the changing RMP state prevents reuse of partially exhausted work.

## RMP And Active-SR Burden

| Diagnostic | Revised proof V4 |
|---|---:|
| RMP solves | 130 |
| RMP build time | 49.8614s |
| RMP solve time | 0.6655s |
| Incremental RMP updates | 119 |
| Incremental update time | 35.9648s |
| Full rebuilds | 11 |
| Active-SR coefficient count | 5,228,110 |
| Active-SR nonzeros | 1,058,978 |
| Active-SR max density | 0.2292 |
| SR cuts added | 265 |
| Root SR cuts added | 109 |
| Post-root SR cuts added | 156 |
| SR cuts removed | 0 |
| SR removal candidates marked | 424 |
| SR metadata update time | 10.7914s |
| SR separation time | 28.0412s |

LP optimization itself is not the bottleneck: RMP solve time is only 0.6655s. Model construction and coefficient maintenance dominate the RMP side. Incremental RMP update time alone is 35.96s, and active-SR metadata updates add 10.79s. SR removal did not remove cuts in this run, but 424 candidate marks and multiple trigger failures indicate that the burden logic was active and conservative. The post-root active-SR set reached 166 cuts, and the active-SR coefficient matrix exceeded one million nonzeros.

## Compact Warm Start And Drone Diversification

| Diagnostic | Revised proof V4 |
|---|---:|
| Root constructive incumbent found | true |
| Root constructive value | 0.3724493585 |
| Root constructive routes | 4 |
| Root constructive time | 0.0260s |
| Drone diversification attempted | true |
| Root compact attempted | true |
| Root compact conditional reason | low_diversity,no_drone_sorties |
| Root compact status | budget_exhausted_build |
| Root compact wall budget | 1.0000s |
| Root compact non-solver/build accounting | 6.2243s |
| Root compact warm-start only | true |

The compact warm start remained primal-only. The wall budget was exhausted during build/accounting, so no proof-side state was changed by compact extraction. This is consistent with the revised paper requirement that compact warm starts can add verified primal columns but cannot certify closure or change a lower bound.

## Timing Shares

| Work category | Time | Share of BPC wall time |
|---|---:|---:|
| Standard pricing | 468.4678s | 77.88% |
| RMP build/update | 49.8614s | 8.29% |
| SR separation | 28.0412s | 4.66% |
| Progress serialization | 19.7598s | 3.28% |
| Root compact extraction/build accounting | 6.2243s | 1.03% |
| Heuristic route-pool work | 2.7313s | 0.45% |
| RMP LP solve | 0.6655s | 0.11% |

The true bottlenecks are lower-bound proof throughput and model/coefficient construction, not incumbent discovery. The incumbent mechanism works; it reaches the strong restored objective and contributes several improvements at negligible wall time. The limiting factors are the exact certification workload, dominance-comparison volume, active-SR coefficient maintenance, and repeated child certification after state changes.

## Future Revision Directions From Measured Bottlenecks

1. Dominance-gate selectivity should be improved where the data show the largest comparison volume. Mask containment rejects 90.58 million pairs, so the next revision should prioritize bucket keys and frontier organization that avoid generating mask-incompatible comparisons in the first place. This is a data-structure target, not a change to the dominance theorem.

2. Physical-location dominance should remain enabled. It removed 384,639 labels and accounts for 17.03% of dominated labels. The measured future work is to reduce the cost of reaching the physical-location full test, because return-credit checks were still executed 7.35 million times after cheap gates.

3. Child-certification reuse needs an epoch-stability improvement. The run started 94 child-certification epochs, completed 23, and resumed zero. The main invalidation source is active-column changes. Future revisions should focus on scheduling and column-insertion timing that reduces avoidable epoch churn while preserving the rule that a saved certification state is reusable only under the same dual, SR set/version, residual mask, branch state, fixed-route signature, and active-column version.

4. Active-SR coefficient burden is a proof-side bottleneck. The run maintained 5.23 million active-SR coefficients and 1.06 million active-SR nonzeros. Incremental RMP updates consumed 35.96s, and SR metadata updates consumed 10.79s. Future revisions should target row-local coefficient construction and active-SR density control, while keeping the current requirement that any SR removal is followed by RMP reoptimization and exact repricing before bound use.

5. RMP LP solving is not the next bottleneck. Solver time was 0.6655s over 130 solves, whereas RMP build/update time was 49.86s. Future work on simplex tuning or basis reuse is unlikely to move this run unless it also reduces coefficient construction and model-update overhead.

6. Route-pool incumbent search should be preserved. Full-pool hard solves produced three incumbent updates and support-pool solves produced two, with total hard-pool time under one second. The measured direction is to keep the support-plus-full-pool sequence and improve when it is invoked, not to replace it with compact fallback or biased heuristic-objective incumbents.

7. Compact warm-start accounting should stay wall-clock bounded. The compact attempt exhausted the wall budget during build/accounting and produced no proof-side effect. Future revisions should either reduce compact model-build overhead or skip compact earlier under the same wall-budget rule; it should not become a fallback proof device.

## Large-Scale BPC Audit

The large-scale audit was run on the same PS seed-1 promised-window policy as the medium run, with a 20-minute case time limit. The output folder is:

```text
numerical_experiments\run_restored_v4_proof_revised_large_20min
```

The command was:

```powershell
python -m thvrpd.experiments --output-dir numerical_experiments\run_restored_v4_proof_revised_large_20min --seeds 1 --distributions PS --scales large --threads 1 --case-time-limit 1200 --external-timeout-grace 60 --pricing-parallel-workers 4 --pricing-worker-backend process --service-deadline-mode random_absolute --service-deadline-offset-min 45 --service-deadline-offset-max 120 --service-deadline-witness-slack 5 --service-deadline-witness-method constructive_then_compact --root-compact-after-constructive conditional_wall_budget --prefix-task-depth-child 2 --logging-mode light --gurobi-python-path D:\gurobi1201\win64\python311\lib
```

### Large Instance Setup

| Field | Value |
|---|---:|
| Scale | large |
| Distribution | PS |
| Seed | 1 |
| Customers | 25 |
| Trucks | 8 |
| Hubs | 2 |
| Drones per truck | 4 |
| Total available drones | 32 |
| Service-deadline mode | random_absolute |
| Service-deadline file | none |
| Manual deadline bound count | none |
| Deadline offset range | 45 to 120 |
| Witness slack | 5 |
| Witness method | constructive_then_compact |
| Active deadline count | 25 |
| Minimum deadline slack | 48.8007 |
| Witness status | success |
| Witness lifts | 1 |
| Maximum witness lift | 27.8564 |

The large run therefore uses the same automatically generated random absolute promised-service windows as the medium run. It does not use a manually supplied deadline file or manual customer-specific upper-bound table. All 25 customers have active promised-service windows.

### Large Main Result

| Metric | Value |
|---|---:|
| Solver status | time limit |
| Full incumbent objective | 0.0306734659 |
| Shifted incumbent objective | 0.1385497082 |
| Full lower bound | -0.1078762423 |
| Shifted lower bound | 0.0000000000 |
| Full gap | 451.69% |
| Shifted gap | 100.00% |
| BPC wall time | 1200.9928s |
| Nodes processed | 1 |
| Root closed | false |
| Post-root nodes processed | 0 |
| Post-root open nodes | 0 |
| Best reduced cost at stop | -0.0009800377 |
| RMP solves | 45 |
| Standard pricing calls | 45 |
| Productive pricing calls | 32 |
| Closure-mode pricing calls | 13 |
| Certification pricing passes | 13 |

The 20-minute large run did not close the root node. The algorithm processed only the root node, and no post-root branching was reached. The root lower bound remains at the shifted value 0, which corresponds to the full-scale lower bound -0.1078762423 after adding the objective shift. The final best reduced cost is still negative, so the root was stopped during unresolved pricing rather than after exact column-generation closure.

### Large Incumbent Structure

The incumbent covers all 25 customers with four truck-only routes and no drone sorties.

| Route | Truck path | Customers | Return time | Delay-square contribution | Shifted route cost |
|---:|---|---|---:|---:|---:|
| 1083 | Source-C16-C17-C7-C25-H1-C1-C21-Sink | C1, C16, C17, C21, C25, C7 | 89.5554 | 1880.3180 | 0.0343965 |
| 1238 | Source-C19-C24-C12-C2-C15-C18-C13-H2-Sink | C12, C13, C15, C18, C19, C2, C24 | 97.8768 | 1661.7008 | 0.0342752 |
| 1345 | Source-C20-C10-C3-C6-C23-C9-Sink | C10, C20, C23, C3, C6, C9 | 68.6240 | 2611.1491 | 0.0351129 |
| 1400 | Source-C8-C4-H2-C11-C22-C14-C5-Sink | C11, C14, C22, C4, C5, C8 | 59.6715 | 2635.8020 | 0.0347652 |

The selected route set satisfies the decoded service checks.

| Service metric | Value |
|---|---:|
| Service feasible | true |
| Selected trucks | 4 of 8 |
| Covered customers | 25 of 25 |
| Truck-served customers | 25 |
| Drone-served customers | 0 |
| Drone sorties | 0 |
| Waiting blocks | 0 |
| Total wait time | 0.0000 |
| Payload feasible | true |
| Route payloads | 30.25, 30.20, 46.71, 49.87 |
| Maximum route payload | 49.87 |
| Total customer demand | 157.03 |
| Available truck payload | 400.00 |
| Mean delivery delay | 14.2877 |
| Maximum delivery delay | 46.0635 |

Objective components for the large incumbent are:

| Component | Value |
|---|---:|
| Raw delay-square sum | 8788.9698 |
| Raw return-time sum | 315.7277 |
| Raw operating cost | 80.0000 |
| Weighted normalized delay | 0.0199651 |
| Weighted normalized return | 0.0107084 |
| Weighted normalized cost | 0.0000000 |
| Route-sum value before shift | 0.1385497 |
| Objective shift | -0.1078762 |
| Full normalized objective | 0.0306735 |

The incumbent is strong on the primal side relative to the initial constructive solution. The constructive incumbent used all 8 trucks, had value 0.3391510450, and had no drone sorties. The final route-pool incumbent uses 4 trucks and reduces the full objective to 0.0306734659. Drone diversification was attempted, generated 5 verified columns, and improved an intermediate incumbent, but the final best solution remains truck-only.

### Large Route-Pool Behavior

| Diagnostic | Value |
|---|---:|
| Heuristic calls | 12 |
| Heuristic incumbent updates | 10 |
| Heuristic time | 8.9412s |
| Hard-pool solves | 25 |
| Hard-pool time | 1.7814s |
| Hard-pool feasible solves | 12 |
| Support-pool calls | 14 |
| Support-pool feasible solves | 1 |
| Support-pool incumbent updates | 1 |
| Full-pool calls | 11 |
| Full-pool feasible solves | 11 |
| Full-pool incumbent updates | 9 |
| Soft-pool solves | 2 |
| Repair-pricing calls | 14 |
| Repair-pricing time | 6.1572s |
| Repair customers | 6 |
| Repair columns generated | 36 |
| Maximum node-pool size | 1508 |
| Maximum support-pool size | 54 |
| Maximum node/support ratio | 35.9048 |

The route-pool mechanism is the main source of the high-quality large incumbent. Of the 10 incumbent updates, 9 came from the full node-admissible hard-pool solve and only 1 came from the support-pool hard solve. The maximum node/support ratio is 35.90, so a support-only incumbent search would inspect only a small fraction of the verified pool. The measured large-scale result therefore reinforces the medium-run conclusion: the full node-admissible hard-pool solve is important for primal performance, and its wall time is small relative to pricing.

### Large Pricing And Dominance Throughput

| Diagnostic | Value |
|---|---:|
| Forward labels generated | 10,011,682 |
| Complete routes generated | 1,811,777 |
| Verified negative routes | 1,955 |
| Inserted negative routes | 1,561 |
| Labels dominated | 5,873,098 |
| Labels pruned by reduced-cost bound | 58,167 |
| Labels purged | 10,832 |
| Stale labels skipped | 7,458 |
| Extensions attempted | 12,619,975 |
| Extensions rejected by deadline | 2,609,886 |
| Deadline reachability removals | 10,898,732 |
| Maximum queue size | 4,653 |
| Maximum pricing-call elapsed time | 72.6211s |

The promised-service-window machinery is active at large scale. Deadline extension checks reject 2.61 million attempted extensions, and deadline reachability removes 10.90 million customer candidates from reduced-cost reward sets. No route was rejected by deadline at master insertion, which means the route decoder and verification layer accepted only deadline-feasible generated routes.

Dominance statistics are:

| Diagnostic | Value |
|---|---:|
| Dominance gate pairs seen | 641,619,664 |
| Gate mask failures | 617,593,068 |
| Gate scalar failures | 18,075,975 |
| Gate branch failures | 0 |
| Gate deadline failures | 77,399 |
| Same-node full dominance tests | 4,314,950 |
| Physical-location full dominance tests | 1,558,272 |
| Return-credit checks executed | 20,518,149 |
| Return-credit checks skipped | 621,101,515 |
| Same-node dominated labels | 4,314,826 |
| Physical-location dominated labels | 1,558,272 |

The dominance gates are essential on the large instance. The solver observes 641.62 million candidate dominance pairs, but only 5.87 million reach full dominance tests. Thus, approximately 99.08% of candidate pairs are screened before full dominance. The dominant rejection source is mask containment: 617.59 million pair checks fail the customer or physical-node mask gate. Physical-location dominance removes 1.56 million labels, or 26.53% of all dominated labels, so it remains productive at large scale.

The bottleneck is the sheer volume of forward-pricing proof work. Standard pricing consumes 1098.49s of the 1200.99s BPC wall time. Worker CPU time is 4043.48s, and the maximum observed core equivalent is 3.91 across the 4 process workers. The large run is therefore pricing-bound, not LP-bound.

### Large RMP And Active-SR Burden

| Diagnostic | Value |
|---|---:|
| RMP solves | 45 |
| RMP build/update time | 16.5178s |
| RMP solve time | 0.4142s |
| RMP column insertion time | 14.4981s |
| Incremental RMP updates | 44 |
| Incremental update time | 16.5108s |
| Full rebuilds | 1 |
| Basis reuse attempts | 44 |
| Basis reuse successes | 8 |
| Active SR cuts added | 73 |
| Maximum active SR cuts | 73 |
| Active-SR coefficient count | 1,613,227 |
| Active-SR nonzeros | 212,396 |
| Active-SR maximum density | 0.1324 |
| SR separation time | 32.3488s |
| SR cuts removed | 0 |
| SR cuts reactivated | 0 |

The RMP solve time is negligible compared with pricing and model update time. LP solves take only 0.4142s over 45 RMP solves, while RMP construction and incremental updates take 16.52s. SR separation takes 32.35s and adds 73 root cuts. Because the root is not closed, no post-root active-SR lifecycle, child inheritance, child hydration, or child certification behavior is exercised in this large audit.

### Large Warm-Start Accounting

| Diagnostic | Value |
|---|---:|
| Root constructive status | success |
| Root constructive routes | 8 |
| Root constructive value | 0.3391510 |
| Root constructive drone sorties | 0 |
| Root constructive time | 0.0665s |
| Drone diversification attempted | true |
| Drone-diversification routes generated | 5 |
| Drone-diversification columns accepted | 5 |
| Drone-diversification incumbent improved | true |
| Root compact attempted | true |
| Compact trigger reason | low_diversity,all_trucks_used,no_drone_sorties |
| Compact status | budget_exhausted_build |
| Compact wall budget | 1.0000s |
| Compact model-build time | 20.8409s |
| Compact solve time | 0.0000s |
| Compact accepted columns | 0 |

The compact warm start remains a primal-only device. In this large run, compact extraction was triggered by the weak constructive structure, but the model-build phase itself exceeded the wall budget, so no compact solve was performed and no compact columns were accepted. This large run therefore obtains its final incumbent through verified route-pool search and pricing-generated columns, not through compact model extraction.

### Large Timing Shares

| Work category | Time | Share of BPC wall time |
|---|---:|---:|
| Standard pricing | 1098.4932s | 91.47% |
| SR separation | 32.3488s | 2.69% |
| Compact model build/extraction accounting | 20.8519s | 1.74% |
| RMP build/update | 16.5178s | 1.38% |
| RMP column insertion | 14.4981s | 1.21% |
| Heuristic route-pool work | 8.9412s | 0.74% |
| Progress serialization | 4.3170s | 0.36% |
| RMP LP solve | 0.4142s | 0.03% |

The large-scale bottleneck is root pricing closure. The route-pool incumbent mechanism is effective and inexpensive, but the exact proof side does not complete root certification within 20 minutes. The unresolved best reduced cost at termination, the absence of post-root nodes, and the negative full lower bound all indicate that the large run is limited by lower-bound proof throughput rather than by incumbent discovery.

## Direct Arc-Based Gurobi Benchmark

A direct Gurobi run was performed on the compact arc-based MIQP for the same medium PS seed-1 promised-window instance. The output is stored in:

```text
numerical_experiments\run_gurobi_arc_medium_10min
```

The benchmark used `solve_compact_solution` with a 600-second time limit, one thread, and `require_optimal=False`. The Gurobi log is:

```text
numerical_experiments\run_gurobi_arc_medium_10min\compact_gurobi.log
```

### Direct Comparison

| Method | Status | Incumbent | Lower bound | Gap | Nodes explored | Routes | Drone sorties |
|---|---:|---:|---:|---:|---:|---:|---:|
| Proposed BPC | time limit | 0.0811406255 | 0.0800923520 | 1.29% | 11 BPC nodes | 3 | 0 |
| Direct Gurobi arc MIQP | time limit | 0.0811982680 | -0.0838480176 | 203.26% | 20,837 MIP nodes | 3 | 0 |

The proposed BPC beats direct Gurobi on incumbent quality and lower-bound proof quality in this 10-minute run. The incumbent advantage is small but measurable:

```text
0.0811982680 - 0.0811406255 = 0.0000576425
```

The proof-bound advantage is much larger. Direct Gurobi's compact MIQP lower bound remains negative after 600 seconds, while the proposed BPC lower bound is 0.0800923520. Thus, the proposed algorithm is not only finding a slightly better feasible solution, but also proving a much tighter lower bound for this promised-window instance.

### Gurobi Incumbent Structure

The direct Gurobi incumbent is also a three-truck, truck-only solution.

| Route | Truck path | Customers | Return time | Shifted route cost |
|---:|---|---|---:|---:|
| 0 | Source-C8-C1-C2-C12-H1-C15-Sink | C1, C2, C8, C12, C15 | 80.2829 | 0.0542687 |
| 1 | Source-C3-C9-C14-H2-C6-C13-C4-Sink | C3, C4, C6, C9, C13, C14 | 86.7634 | 0.0564653 |
| 2 | Source-C5-C10-C7-H2-C11-Sink | C5, C7, C10, C11 | 79.1671 | 0.0546946 |

Gurobi service metrics:

| Metric | Value |
|---|---:|
| Service feasible | true |
| Selected trucks | 3 |
| Truck-served customers | 15 |
| Drone-served customers | 0 |
| Payload feasible | true |
| Maximum route payload | 41.85 |
| Mean delay | 14.4932 |
| Maximum delay | 35.8747 |
| Delay-square sum | 5,241.3320 |
| Waiting blocks | 0 |

The Gurobi incumbent has a lower delay-square sum than the BPC incumbent, but it has a longer total return time. Its route return-time sum is approximately 246.2135, compared with 234.9803 for the BPC incumbent. Because the objective weights include both delay and return time, the BPC route set has the better full normalized objective.

### Gurobi Search Diagnosis

The compact MIQP explored 20,837 MIP nodes and 6,225,979 simplex iterations in 600 seconds. It found 10 feasible solutions. The final Gurobi log reports:

```text
Best objective 8.119826802661e-02
Best bound    -8.384801762822e-02
Gap           203.2633%
```

This confirms that the compact arc-based formulation is a weak proof vehicle for the medium promised-window instance. Direct Gurobi can find a competitive feasible incumbent, but it does not approach the proof quality of the branch-price-and-cut formulation within the same time budget.
