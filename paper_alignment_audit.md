# Paper-Faithful TH-VRPD Code Alignment Audit

Audit date: 2026-07-03  
Paper source: `C:/Users/77270/.codex/attachments/0de8e89b-cd5f-456e-aa55-4694657fa536/pasted-text.txt`  
Strictness: paper-exact

## Summary

The implementation matches the paper's exactness-critical design: compact MIQP validation, route-space RMP, hub-specific transformed pricing network, exact branch-price-and-cut loop, Farkas pricing, SR cuts, paper branching order, bidirectional same-node joining, full route decoding, direct reduced-cost verification, and conservative non-identical backward suffix dominance.

This revision fixed the previous backward-dominance efficiency gap. Backward labels now carry suffix timing profiles, service-mode resources, SR counts, branch-state metadata, physical-node sets, payload, and leading drone-block interface state. The bidirectional engine discards a non-identical backward suffix only when prefix-compatibility inclusion and standard/Farkas reduced-cost dominance are certified. If any comparison cannot be certified exactly, both suffix labels are retained.

## Alignment Matrix

| Paper component | Status | Evidence | Notes |
|---|---:|---|---|
| Instance and network construction | implemented | `thvrpd/instance.py`, `thvrpd/transform.py` | Source/sink depots, truck arcs, drone arcs, one-way and round-trip drone times. |
| Objective and normalization | implemented | `thvrpd/objective.py`, `thvrpd/routes.py` | Weighted normalized objective and shifted route costs. |
| Compact MIQP | implemented | `thvrpd/compact.py`, `thvrpd/validate.py` | Used for tiny deterministic validation. |
| Route-space RMP | implemented | `thvrpd/rmp.py`, `thvrpd/bpc.py` | Residual customers/fleet, fixed routes, SR cuts, paper dual signs. |
| Transformed pricing network | implemented | `thvrpd/transform.py`, `thvrpd/routes.py` | Hub-specific duplicates and canonical block ordering. |
| BPC node loop | implemented | `thvrpd/bpc.py` | RMP, Farkas pricing, standard pricing, SR separation, branching. |
| Branching | implemented | `thvrpd/branching.py`, `thvrpd/bpc.py` | Customer-pair, service-mode, launch-pad, transformed-arc, merge, route-variable fallback. |
| Forward pricing | implemented | `thvrpd/pricing.py` | Elementarity, payload, active pad/wait, SR counters, branch state, reduced-cost recursion. |
| Bidirectional pricing and joins | implemented | `thvrpd/pricing.py` | Backward labels, same-node joins, direct decode/reduced-cost verification, threaded expansion/joining. |
| Backward dominance acceleration | implemented | `thvrpd/pricing.py`, `tests/test_pricing_rmp_bpc.py` | Conservative non-identical suffix dominance; retain labels when dominance cannot be certified exactly. |
| Farkas pricing | implemented | `thvrpd/rmp.py`, `thvrpd/pricing.py` | Farkas ray validation, Farkas reduced cost, Farkas-valid backward dominance. |
| SR cuts | implemented | `thvrpd/bpc.py`, `thvrpd/pricing.py` | Residual triplets and pricing counters. |
| Paper-defined acceleration | implemented | `thvrpd/phasei.py`, `thvrpd/columns.py`, `thvrpd/heuristics.py` | Phase-I, productive/certification modes, duplicate/cost filtering, route-pool diving/repair. |
| Diagnostics and outputs | implemented | `thvrpd/bpc.py`, `thvrpd/solve.py`, `thvrpd/experiments.py` | Termination, certification, backward-dominance tests/rejections, and run summaries. |

## Fixes Made In This Revision

- Extended `_BackwardLabel` with exact suffix comparison resources: service-mode sets, SR counts, branch-state metadata, suffix profiles, leading block interface, physical-node sets, and payload.
- Added symbolic suffix time profiles and direct tests against `route_from_path` after joining.
- Replaced duplicate-only backward suffix insertion with `_insert_nondominated_backward_label` and `_backward_dominates`.
- Added standard and Farkas tests for non-identical backward dominance, branch rejection, resource rejection, direct reduced-cost agreement, and bidirectional-vs-forward pricing equivalence.
- Added `backward_dominance_tests` to pricing diagnostics, BPC stats, solve summaries, and experiment summaries.

## Verification Status

Syntax verification passed:

```text
python -m compileall thvrpd tests
```

Full Gurobi-backed test suite passed outside the sandbox license user:

```text
67 passed in 2.45s
```

Tiny compact-vs-BPC validation passed:

```text
validated seed=1 dist=PS obj=0.06004885
```

Fresh small/medium/large audit experiments were written to:

```text
numerical_experiments\run_20260703_backward_dominance_audit
```

The small case solved to optimality. Medium reached the 60-second solver time limit with an incumbent. Large reached the parent external timeout during root/Phase-I startup. Those time-limited rows are performance evidence, not exact convergence proof.
