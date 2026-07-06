# Restored V4 Paper-Faithful TH-VRPD Numerical Report

## 1. Scope

Fresh restored output folder:

```text
numerical_experiments\run_restored_postroot_v4_medium_10min
```

Comparison folders:

```text
numerical_experiments\run_after_postroot_v4_medium_10min
numerical_experiments\run_after_postroot_v5_medium_10min
numerical_experiments\run_after_postroot_v6_medium_10min
```

The restored code targets the promised-service-window restricted TH-VRPD in the supplied V4 LaTeX design. It keeps the exact forward-only production path: source-neighbor process pricing, promised windows, physical-location dominance with return-time credit, route signatures, branch-indexed inheritance, incremental RMP updates, active-SR lifecycle, delayed hydration, resumable child certification, true wall-clock compact warm start, and primal-only constructive/route-pool incumbent search.

Removed or neutralized V5/V6 regression paths:

- no `skip_or_tiny` no-drone compact policy;
- no promised-drone constructor gate or promised-drone local constructor controls in warm-start diversification;
- no child useful/no-route dual-stability batch growth/shrink scheduling;
- no child closure-batch adaptation during certification returns;
- no V6 weighted SR removal-score policy in the active removal decision;
- active-SR changes no longer force full RMP rebuilds when the existing incremental RMP can synchronize row changes exactly.

Verification:

```text
python -m compileall thvrpd tests
python -c "import sys, pytest; sys.path.append(r'D:\gurobi1201\win64\python311\lib'); raise SystemExit(pytest.main(['tests','-q']))"
```

Result:

```text
compileall succeeded
137 passed in 14.52s
```

## 2. Restored Run Command

```powershell
python -m thvrpd.experiments --output-dir numerical_experiments\run_restored_postroot_v4_medium_10min --seeds 1 --distributions PS --scales medium --threads 1 --case-time-limit 600 --external-timeout-grace 60 --pricing-parallel-workers 4 --pricing-worker-backend process --service-deadline-mode random_absolute --service-deadline-offset-min 45 --service-deadline-offset-max 120 --service-deadline-witness-slack 5 --service-deadline-witness-method constructive_then_compact --root-compact-after-constructive conditional_wall_budget --prefix-task-depth-child 2 --logging-mode light --gurobi-python-path D:\gurobi1201\win64\python311\lib
```

The earlier restored attempts are preserved as:

```text
numerical_experiments\run_restored_postroot_v4_medium_10min_before_rmpfix
numerical_experiments\run_restored_postroot_v4_medium_10min_before_fullpool
```

## 3. Medium Outcome

| Run | Status | Incumbent | Lower bound | Full gap | Runtime | Root closed | Nodes | Post-root nodes | Open nodes |
|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| Restored V4 | time limit | 0.0811406255 | 0.0783032343 | 3.50% | 603.12s | yes | 7 | 6 | 5 |
| Recorded V4 | time limit | 0.0846704491 | 0.0796601339 | 5.92% | 601.73s | yes | 7 | 6 | 5 |
| V5 | time limit | 0.2257310146 | 0.0783032343 | 65.31% | 602.54s | yes | 7 | 6 | 7 |
| V6 | time limit | 0.2257310146 | 0.0798413484 | 64.63% | 603.31s | yes | 10 | 9 | 10 |

The restored run recovers the strong-incumbent behavior. Its incumbent is better than the recorded V4 target (`0.0811406255` versus `0.0846704491`) and far better than V5/V6 (`0.2257310146`). The root closes exactly. The lower bound is not as high as recorded V4, but the stronger incumbent more than offsets that, reducing the full gap to `3.50%`.

## 4. Recovery Checks

| Check | Target | Restored result | Status |
|---|---:|---:|---|
| Recover incumbent near recorded V4 | 0.0846704491 | 0.0811406255 | recovered and improved |
| Root closure | true | true | recovered |
| Lower bound near recorded V4 | 0.0796601339 | 0.0783032343 | below recorded V4 |
| Improve full gap versus V5/V6 | < 64% | 3.50% | recovered |
| Explain stronger incumbent | route-pool/primal evidence | 3 truck-only routes, 0 drone sorties | explained |

The strong incumbent is not caused by drone sortie use. Like recorded V4, the final solution is truck-only with zero waiting blocks. The improvement comes from selecting a better three-route truck solution from the verified route pool.

## 5. Final Incumbent Mix

| Run | Selected trucks | Truck-served customers | Drone-served customers | Drone sorties | Mean delay | Max delay | Delay square sum |
|---|---:|---:|---:|---:|---:|---:|---:|
| Restored V4 | 3 | 15 | 0 | 0 | 13.613 | 41.169 | 5,563.775 |
| Recorded V4 | 3 | 15 | 0 | 0 | 14.496 | 41.355 | 6,714.369 |
| V5 | 4 | 15 | 0 | 0 | 34.752 | 105.148 | 30,652.039 |
| V6 | 4 | 15 | 0 | 0 | 34.752 | 105.148 | 30,652.039 |

Restored selected routes:

```text
R1: Source -> C5 -> C10 -> C7 -> H2 -> C11 -> Sink
R2: Source -> C8 -> C3 -> H2 -> C6 -> C13 -> C4 -> C14 -> Sink
R3: Source -> C9 -> C1 -> C2 -> C12 -> H1 -> C15 -> Sink
```

Recorded V4 selected routes:

```text
R1: Source -> C10 -> C8 -> C1 -> C2 -> C12 -> H1 -> C15 -> Sink
R2: Source -> C3 -> C9 -> C7 -> C5 -> Sink
R3: Source -> C6 -> C13 -> C4 -> C14 -> H2 -> C11 -> Sink
```

## 6. Post-Root Throughput

| Metric | Restored V4 | Recorded V4 | V5 | V6 |
|---|---:|---:|---:|---:|
| Standard pricing time | 477.82s | 499.89s | 479.59s | 438.67s |
| Root standard pricing time | 91.93s | 58.51s | 106.45s | 97.54s |
| Post-root standard pricing time | 385.89s | 441.38s | 373.14s | 341.13s |
| RMP total time | 52.41s | 16.45s | 55.71s | 70.61s |
| RMP build/update time | 51.21s | 15.12s | 54.60s | 69.35s |
| RMP solve time | 0.70s | 0.82s | 0.68s | 0.74s |
| SR separation time | 28.12s | 37.03s | 24.73s | 36.65s |
| Child certification calls | 62 | 82 | 62 | 64 |
| Child calls with columns | 50 | 59 | 50 | 46 |
| Child exhausted calls | 11 | 23 | 12 | 18 |
| Child time-limited calls | 1 | 0 | 0 | 0 |
| Negative routes inserted | 872 | 986 | 833 | 979 |
| Labels generated | 3,979,400 | 4,657,794 | 4,188,712 | 3,946,220 |

The restored run recovers the incumbent but still differs from recorded V4 in proof throughput. RMP construction remains higher than recorded V4, and one child certification slice reaches the time limit. This is why the lower bound remains at the V5 level even though the incumbent is strong.

## 7. Primal Heuristic Diagnostics

| Metric | Restored V4 | Recorded V4 | V5 | V6 |
|---|---:|---:|---:|---:|
| Heuristic calls | 11 | 12 | 12 | 17 |
| Hard pool solves | 24 | 16 | 17 | 23 |
| Hard pool feasible solves | 11 | 2 | 1 | 1 |
| Max node-pool routes | 628 | 802 | 607 | 695 |
| Max support routes | 29 | 31 | 28 | 31 |
| Incumbent updates | 6 | 2 | 1 | 1 |
| Post-root incumbent updates | 0 | 1 | 0 | 0 |
| Repair pricing time | 1.33s | 1.10s | 1.73s | 2.79s |

The restored code now solves a hard route-pool set-partitioning IP over the full node-admissible verified route pool after the smaller LP-support pool is tried. This is a primal-only search over verified columns. It does not change lower bounds, pricing closure, or fathoming. It is the mechanism that recovered the strong three-route incumbent.

## 8. Compact And Drone Warm Starts

| Mechanism | Restored result | Interpretation |
|---|---:|---|
| Root compact status | `budget_exhausted_build` | True wall-clock budget counted build time and was exhausted before solve/decode. |
| Root compact model build | 8.593s | Build alone exceeded the 1s wall budget. |
| Root compact accepted columns | 0 | Compact did not seed the strong incumbent. |
| Drone diversification attempted | true | V4 warm-start diversification ran. |
| Drone diversification generated/accepted | 0 / 0 | No verified drone variants were accepted. |
| Final drone sorties | 0 | Strong incumbent is truck-only. |

The restored strong incumbent therefore comes from route-pool selection, not compact extraction or drone diversification.

## 9. Code Changes That Matter

1. `SolverConfig` and CLI defaults now use V4 behavior:
   `root_compact_after_constructive=conditional_wall_budget`, fixed certification closure batches, process backend compatibility, and no V6 no-drone compact skip/tiny default.

2. Child certification no longer observes useful/no-route dual-stability histories to grow or shrink child closure batches. Productive calls and certification calls remain separated; only exhaustive certification can close pricing.

3. Active-SR changes are synchronized through the incremental RMP instead of invalidating the whole model. This restores the V4-compatible row-update intent.

4. SR cut removal uses V4-style inactivity/build-burden logic instead of V6 weighted score controls.

5. Constructive drone diversification is route-decoder verified and no longer gated by the V6 promised-drone constructor.

6. The primal route-pool heuristic now includes a hard full-pool solve over the node-admissible verified route pool after the smaller LP-support solve. This is a general primal-only route-pool search and is consistent with the V4 paper's route-pool incumbent-search role.

## 10. Bottom Line

The restored implementation recovers the strong-incumbent version. The final medium incumbent is `0.0811406255`, better than the recorded V4 value `0.0846704491`, with root closure and a `3.50%` full gap. The remaining discrepancy is lower-bound/proof throughput: the restored lower bound is `0.0783032343`, below recorded V4's `0.0796601339`, because fewer child certifications exhaust and RMP build/update time remains higher than recorded V4.

The acceptance-critical result is recovered: V5/V6's weak four-truck incumbent is gone, and the restored code again finds a strong three-truck promised-window incumbent within the 10-minute audit.
