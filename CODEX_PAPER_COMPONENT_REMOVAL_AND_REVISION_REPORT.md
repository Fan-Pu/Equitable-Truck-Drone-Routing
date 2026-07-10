# Paper Component Implementation and Efficiency Audit

## Purpose

This report is a revision brief for the manuscript contained in `pasted-text.txt`. Its purpose is to identify algorithmic components that should be removed, rewritten, or demoted because they are not realized as described in the current production implementation, are inactive in the completed computational runs, or have not demonstrated an efficiency benefit. It also identifies the components that should remain because they are mathematically central or supported by implementation and diagnostic evidence.

The intended use of this document is as input to a subsequent paper-revision model. The revision model should edit the LaTeX manuscript conservatively: it should remove unsupported claims without weakening the valid transformed-network formulation, exact forward-pricing theory, dominance propositions, route-space equivalence, or the primal validity of verified route-pool solutions.

## Sources Audited

The audit uses the following sources:

1. The attached LaTeX manuscript, especially:
   - the abstract and contribution statements;
   - Section `Branch-Price-and-Cut Algorithm`;
   - `Post-root node management`;
   - `Source-Neighbor Parallel Forward Pricing`;
   - `Forward Candidate Management and Dominance Throughput`;
   - `Primal Heuristics and Column Pooling`;
   - `Computational Audit and Revision Implications`;
   - Proposition `Exactness of forward candidate management` and its proof.
2. The current implementation in `thvrpd/config.py`, `thvrpd/bpc.py`, `thvrpd/pricing.py`, `thvrpd/rmp.py`, and `thvrpd/heuristics.py`.
3. Completed representative run artifacts:
   - medium: `run_medium15_3t_optional4_p015_h040_seed3_bpc_tol001_prefix2_srb32_30min`;
   - large: `run_balanced_dynamic_kcore_large25_tol001_w6_1h_resumed`.
4. The accumulated numerical evidence in `numerical_test_report.md`.
5. The current selected scale defaults, which use six pricing workers, a thread backend, no manually imposed service windows, child prefix depth one, and pricing tolerance `0.05`.

The completed medium and large runs used an earlier promised-window process configuration. They remain useful for determining whether a component ever became active, but they do not describe the current default experimental configuration. This distinction must be made explicit in the revised paper.

## Audit Classification

Each paper component is assigned one of four statuses.

- **Remove**: the described mechanism is not implemented, is materially different from the implementation, or was repeatedly inactive and should not be presented as a contribution.
- **Rewrite**: a related mechanism exists, but the manuscript overstates or mischaracterizes it.
- **Demote**: the component is implemented, but no controlled evidence establishes a net efficiency improvement. It may remain as an implementation detail but should not be listed as a principal contribution.
- **Retain**: the component is mathematically central or has clear implementation and diagnostic support.

## Executive Decision Matrix

| Component claimed in the paper | Implementation/evidence status | Revision decision |
|---|---|---|
| Resumable child-certification tasks | No open-label queue or exhausted-task state is restored; representative runs recorded zero saves and zero resumptions | Remove |
| Build-burden-aware SR cut removal/reactivation | Implemented control path, but zero removals and zero reactivations in representative runs | Remove as an efficiency contribution |
| True wall-clock-bounded compact warm start | Current budget applies only to the Gurobi solve; build, decode, verification, and insertion occur outside it | Rewrite |
| Conditional compact call after weak constructive solution | Current default is forced `full_budget`, not conditional | Rewrite |
| Constructive root incumbent | Constructive procedure was partial and produced no incumbent in representative medium and large runs | Rewrite as initial-column seeding |
| Constructive drone diversification | Not attempted; zero accepted columns and zero incumbent improvements | Remove |
| First-incumbent route-pool trigger | Zero calls in representative runs | Remove |
| LP-support route-pool diving as an effective incumbent mechanism | Multiple calls but zero incumbent updates in medium and large runs | Remove as a contribution; at most mention briefly as an implementation filter |
| Full node-admissible route-pool search | Produced incumbent updates in both representative runs | Retain |
| Runtime dynamic splitting/task stealing | Current code preconstructs prefix tasks before worker execution; it does not split a live label queue | Remove runtime/task-stealing claim; rewrite as static prefix refinement |
| Persistent process-worker implementation | Process path exists, but current selected defaults use threads | Rewrite as an optional backend, not the default production behavior |
| Diversity-aware negative-column insertion | Active in persistent process merging, but the current thread merge passes diversity fraction `0.0` | Remove from default-method claim or qualify as process-only |
| Promised-window acceleration as the default computational regime | Current small, medium, and large defaults use `service_deadline_mode="none"` | Remove from default-results narrative or explicitly restore a promised-window experiment profile |
| Exact computational optimality under pricing tolerance `0.05` | Columns with reduced cost in `[-0.05,0)` are ignored | Replace with tolerance-closure language |
| Incremental RMP and active-SR coefficient cache | Active, with substantial cache use | Retain, but avoid unablated speedup claims |
| Inactive-column deactivation and hydration | Active on the large run | Retain as implementation detail; efficiency benefit remains unablated |
| Branch-indexed inheritance | Active and rejects many incompatible inherited routes | Retain |
| Dominance frontier and mask gates | Avoid millions of impossible comparisons | Retain |
| Physical-location dominance with return-time credit | Deletes labels with moderate measured cost | Retain |
| Route signatures, lazy decoding, and direct verification | Active and necessary for safe candidate handling | Retain |
| Deadline-aware/resource-reward pruning | Active but computationally expensive; net benefit not isolated | Demote pending ablation |

## High-Priority Removals and Rewrites

### 1. Resumable Child Certification

#### Paper claim

The manuscript states that a child-certification state records exhausted source-neighbor or prefix tasks and can resume the remaining exact search under an unchanged epoch. It further claims that continuation reduces repeated child-certification work.

This claim appears in:

- `Post-root node management`;
- `Source-Neighbor Parallel Forward Pricing`;
- Proposition `Exactness of forward candidate management`;
- the proof of that proposition;
- the abstract and introduction's acceleration lists.

#### Implementation finding

The implementation records a certification signature and aggregate counts. If a later call has the same signature, a resumption counter is incremented. However, the subsequent pricing call starts a new pricing search. The implementation does not pass a saved open-label queue, saved nondominated frontier, exact set of exhausted task identifiers, or partial worker state into `price_route`.

Therefore, the production behavior is epoch validation and restart accounting, not resumable label-setting certification.

#### Diagnostic evidence

| Diagnostic | Medium | Large |
|---|---:|---:|
| Certification states saved | 0 | 0 |
| Certification states resumed | 0 | 0 |
| Certification-state discards | 0 | 116 |

There is no observed reuse benefit. On the large run, epoch invalidation occurred repeatedly without a single saved or resumed exact state.

#### Required paper revision

Remove the paragraph that defines `ChildCertState` as a resumable search state. Remove `resumable child-certification tasks` from the abstract, contribution list, candidate-management proposition, diagnostics list, computational implications, and proof.

The paper may retain the following narrower statement:

> Child pricing closure is accepted only when all pricing subproblems are exhausted under one unchanged pricing epoch. Any change in the true dual solution, active subset-row set, residual customer set, branch restrictions, fixed routes, active columns, or RMP structure invalidates previously reported task certificates.

This replacement accurately describes an epoch-validity rule without claiming computational reuse.

### 2. Build-Burden-Aware SR Cut Removal and Reactivation

#### Paper claim

The manuscript assigns age, activity, violation history, density, and build-burden metadata to active SR cuts and claims that selected cuts are removed, followed by RMP reoptimization and exact repricing. Removed cuts may later reactivate.

#### Implementation and diagnostic finding

The code contains a removal path, but neither representative run used it:

| Diagnostic | Medium | Large |
|---|---:|---:|
| SR cuts removed | 0 | 0 |
| SR cuts reactivated | 0 | 0 |

Consequently, this component did not reduce the active RMP or coefficient burden in the experiments used to motivate it. Describing it as a proof-throughput improvement is unsupported.

#### Required paper revision

Remove the cut-removal score, lifecycle rules, removal/reactivation diagnostics, and associated efficiency claims. Retain:

- ordinary three-customer SR separation;
- active-SR coefficient computation;
- exact reoptimization and repricing whenever the active cut set changes;
- row-local coefficient caching, which was active.

The revised paper should not state that density-aware cut removal contributed to the reported results.

### 3. Compact Warm-Start Budget Semantics

#### Paper claim

The manuscript describes a conditional compact warm start under a true wall-clock budget that includes model construction, solver setup, Gurobi optimization, route extraction, decoding, verification, and insertion. It also states that compact is called only if the constructive and drone-diversification solutions remain weak.

#### Current implementation

The current defaults are:

```text
root_compact_after_constructive = full_budget
root_compact_solve_time_limit = 60.0
root_compact_wall_time_limit = 0.0
```

The 60-second budget is passed to the compact Gurobi solve. Build, decode, verification, and insertion are timed separately and occur after the solver budget. Compact is forced after constructive initialization when sufficient global time remains; it is not conditional on constructive quality.

#### Required paper revision

Delete the phrases:

- `true wall-clock-bounded compact warm start`;
- `the budget includes build, solve, decode, verification, and insertion`;
- `only if constructive and diversification remain weak`;
- `conditional compact extraction`, unless the code policy is changed back.

Use the following replacement:

> The compact arc-based formulation is used as a primal-only warm start under a fixed Gurobi solve-time limit. Model construction, route extraction, canonical route decoding, verification, and RMP insertion are recorded separately and are not charged against the solver limit. Every extracted route is reconstructed from its physical truck path and drone-service blocks before it enters the route pool. Compact optimization cannot certify pricing closure, strengthen a BPC lower bound, or fathom a node.

The decomposability proposition may remain because it concerns validity of extracted routes, not timing policy.

### 4. Constructive Incumbent and Drone Diversification

#### Paper claim

The paper presents a constructive root incumbent followed by drone-diversification exchanges as a principal primal mechanism.

#### Diagnostic evidence

In both representative runs:

- constructive status was `partial`;
- constructive incumbent found was `false`;
- drone diversification was not attempted;
- accepted diversification columns were zero;
- diversification incumbent improvement was false;
- the incumbent source was `compact_root`.

The constructive procedure can still generate useful verified starting routes, but it did not construct the incumbent described in the manuscript. The diversification component had no computational role.

#### Required paper revision

Rename `constructive root incumbent` to `constructive initial-column generation`. Remove drone diversification from:

- the abstract;
- the introduction's contribution list;
- the primal-heuristic subsection;
- Proposition `Exactness of forward candidate management`;
- the associated proof and diagnostic list.

Do not claim that constructive diversification explains drone-using incumbents. In the measured runs, compact Gurobi supplied the initial complete incumbent.

### 5. LP-Support Route-Pool Diving and First-Incumbent Trigger

#### Paper claim

The manuscript gives a full mathematical formulation for the LP-support route-pool heuristic, emphasizes a first-incumbent trigger, and presents the sequence `support pool -> full pool -> soft repair` as an effective incumbent mechanism.

#### Diagnostic evidence

| Diagnostic | Medium | Large |
|---|---:|---:|
| Support-pool calls | 6 | 18 |
| Support-pool incumbent updates | 0 | 0 |
| Full-pool calls | 5 | 14 |
| Full-pool incumbent updates | 1 | 4 |
| First-incumbent route-pool calls | 0 | 0 |
| Repair columns generated | 6 | 10 |

The full node-admissible hard-pool solve produced improvements. The support-only solve and first-incumbent trigger did not.

#### Required paper revision

Remove the first-incumbent trigger and its notation. Remove LP-support route-pool diving as a stated contribution. The lengthy RPH formulation can be deleted or reduced to one implementation sentence if exact code correspondence is desired.

Retain and emphasize:

1. the verified full node-admissible route pool;
2. the hard full-pool set-partitioning solve evaluated under true route cost;
3. the soft uncovered-customer model as a repair target generator;
4. branch-feasible repair pricing;
5. the rule that only a hard feasible true-cost solution may update the incumbent.

The primal-heuristic algorithm should be simplified to:

```text
Build the verified node-admissible route pool.
Solve the hard full-pool set-partitioning model.
If no improvement is obtained, solve the soft coverage model.
Run branch-feasible repair pricing for uncovered customers.
Resolve the hard full-pool model and update the incumbent only if its true objective improves.
```

### 6. Parallel Scheduling Claims

#### Paper claim

The paper describes dynamic one-source-neighbor tasks pulled by a persistent pool of processor workers. Some wording implies runtime subdivision, load transfer, or continuation of unfinished worker subspaces.

#### Current implementation

Two distinct implementations exist:

- a persistent process-pool path;
- a thread path that constructs a new thread executor for the pricing call.

The selected scale defaults currently use six thread workers. Moreover, the so-called dynamic refinement routine builds a task plan before workers begin. It enumerates transformed prefixes, estimates workload, and converts selected source-neighbor blocks into prefix tasks. It does not:

- checkpoint a running worker;
- transfer a live open-label queue;
- split an active nondominated frontier;
- assign an idle worker to a running worker's remaining state.

The diagnostic name `labels_transferred_to_idle_workers` counts generated prefix tasks, not transferred live labels.

#### Required paper revision

Retain the exact mathematical partition by the first transformed successor. Remove claims of runtime task stealing, live-state subdivision, or checkpoint transfer. Describe the current refinement as deterministic pre-search prefix partitioning.

Because the current defaults use threads, replace `persistent process workers` with:

> Pricing subproblems may be evaluated using a configurable thread or process backend. The route-space partition and closure conditions are backend independent.

Do not claim processor-level speedup in a table unless the reported experiment explicitly uses the process backend and reports CPU core-equivalent utilization.

The source-neighbor partition exactness proposition should remain; only the implementation narrative needs revision.

### 7. Diversity-Aware Batch Insertion

#### Paper claim

The manuscript states that every productive pricing batch reserves positions for source-neighbor and customer-coverage diversity.

#### Implementation finding

The persistent process merge passes the configured diversity fraction. The current thread merge passes `0.0`, which disables the diversity quota. Since the selected default backend is thread, diversity-aware insertion is not part of current default production behavior.

#### Required paper revision

Either remove diversity-aware insertion from the general method or state explicitly that it is an optional process-backend scheduling rule. It should not appear in the abstract, main contribution list, or exactness proposition as a universal production component.

### 8. Promised-Service-Window Computational Regime

#### Paper claim

The abstract, algorithm discussion, and computational audit are framed around random absolute promised-service windows. Service-envelope pruning and deadline-aware reward bounds are presented as central computational accelerations.

#### Current default configuration

The selected small, medium, and large defaults now use:

```text
service_deadline_mode = none
service_deadline_offset_min = 45
service_deadline_offset_max = 120
service_deadline_witness_slack = 5
```

The offset and slack values are inactive when the mode is `none`. The implementation can still solve promised-window instances when explicitly configured, but promised windows are no longer the default experiment regime.

#### Required paper revision

The paper must choose one of two consistent positions:

1. **No-window default paper:** remove promised-window-specific acceleration claims and regenerate the computational section under the no-window defaults.
2. **Promised-window paper:** explicitly define a separate promised-window experiment profile and rerun all reported instances under that profile.

Under the current selected defaults, the entire existing `Computational Audit and Revision Implications` section is stale and should be removed or replaced. Its run definitions, process backend, window regime, objective values, lower bounds, and bottleneck conclusions do not represent the current default benchmark.

### 9. Exactness Language Under Pricing Tolerance 0.05

#### Mathematical issue

The theoretical BPC algorithm is exact when pricing certifies that every feasible route has reduced cost at least zero. The current default implementation accepts pricing closure when no route has reduced cost below `-0.05`. Thus, a route with reduced cost `-0.049` is treated as nonnegative and is not inserted.

This threshold is too large to be described only as floating-point protection. It changes the operational pricing problem and may change the returned primal solution or lower-bound trajectory.

#### Required paper revision

Retain the exact mathematical theorems for zero-threshold pricing. Revise computational language as follows:

- use `pricing closure under tolerance 0.05` or `0.05-tolerance closure`;
- do not write `proved optimal` or `exact optimal solution` for a run closed only under this threshold;
- report the best reduced cost at termination beside every closure claim;
- distinguish the Gurobi MIP gap from the BPC pricing tolerance;
- do not describe `0.05` as merely a numerical-stability safeguard.

If the manuscript requires computational claims of exact optimality, rerun the reported cases using a defensible numerical threshold such as `1e-7` or tighter and verify closure under the true current dual.

## Components to Demote Pending Ablation

The following components are implemented and active. They should not be removed solely from the present evidence, but they should be moved from the main contribution list to implementation details until matched ablations establish net efficiency gains.

### Deadline-Aware Resource Reward Bound

The large run recorded approximately 616,818 bound-pruned labels, so the bound has an observable pruning effect. It also consumed approximately 125.1 seconds, compared with approximately 28.2 seconds for physical-location dominance. Without a matched run disabling the bound, its net benefit is unknown.

Recommended wording:

> We use a resource-relaxed dual-reward bound as a valid pruning rule. Its net computational effect is evaluated separately because bound construction itself is nontrivial.

Do not write that it improves runtime unless an ablation confirms this.

### Incremental RMP and Active-Coefficient Caching

These mechanisms are active:

| Diagnostic | Medium | Large |
|---|---:|---:|
| Incremental RMP updates | 19 | 145 |
| Full RMP rebuilds | 2 | 84 |
| Active-coefficient cache hits | 83,680 | 37,605,157 |
| Active-coefficient cache misses | 384 | 101,260 |

The high hit counts establish use, not net speedup. Retain the mechanism but state that it reduces coefficient recomputation opportunities rather than claiming a measured runtime reduction.

### Inactive-Column Deactivation and Hydration

The medium root-only run did not activate this mechanism. The large run deactivated 1,536 columns and rehydrated 222. This shows functional activity, but there is no matched run proving that total RMP plus repricing time decreased.

Retain it as a post-root RMP-size control, but do not list it as a demonstrated efficiency contribution without an ablation.

### Productive Slices and Candidate Quotas

These controls govern when columns return to the master and cannot certify closure. Their exactness role is clear, but their efficiency value has not been isolated from batch size, backend choice, and pricing tolerance. Present them as scheduling details.

## Components Supported by Current Evidence

The subsequent paper revision should preserve the following elements.

### Transformed Route Encoding

Retain the transformed network, cost-preserving route encoding, elementary-route resources, service-time recursion, drone-block waiting-time recursion, and route-space equivalence. These define the methodological core and are not implementation-only accelerations.

### Forward-Only Label-Setting Pricing

Retain forward-only pricing, standard pricing/Farkas mode separation, the nonempty-column convention, exhaustive certification requirement, and direct main-process verification of every entering route.

### Canonical Drone-Block Ordering

Retain the canonical ordering and its symmetry argument. This removes duplicate encodings by construction and is theoretically justified.

### Dominance Frontier and Mask Gates

The mechanisms avoid substantial comparison work:

| Diagnostic | Medium | Large |
|---|---:|---:|
| Dominance pairs avoided before materialization | 8,117,451 | 56,113,954 |
| Candidate pairs materialized | 512,024 | 3,799,965 |

Retain the frontier and necessary-condition gates. Clarify that the gates never delete labels; only a full dominance theorem can do so.

### Physical-Location Dominance With Return-Time Credit

The mechanism deleted 5,085 labels in the medium run and 20,845 in the large run. Its measured large-run cost was approximately 28.2 seconds, which was material but much smaller than total pricing time. Retain both the proposition and implementation.

### Route Signatures, Lazy Decoding, and Direct Verification

Retain these components. They support safe duplicate management and ensure that worker candidates cannot enter the RMP without physical decoding, branch verification, service feasibility, and direct reduced-cost recomputation.

### Branch-Indexed Inheritance

The large run performed 235 global-index queries and rejected 47,937 incompatible candidates. This is direct evidence that the index filters inherited routes before expensive refresh. Retain it.

### Row-Local Active-SR Coefficient Cache

Retain coefficient reconstruction from the served-customer mask and row-local cache. Remove only the unsupported cut-removal lifecycle layered on top of it.

### Full Node-Admissible Route-Pool Search and Repair

The full-pool model produced incumbent updates in both representative runs. Retain its primal-validity proposition and true-cost evaluation. Retain soft repair as a target-generation mechanism, while making clear that soft solutions never become incumbents directly.

### Compact Gurobi Warm Start as a Primal Device

Compact Gurobi supplied the accepted root incumbent routes in the representative runs. Retain compact extraction and route verification, but use the corrected fixed-solver-budget description.

## Section-by-Section Revision Instructions

### Abstract

The abstract currently contains an excessively long list of implementation components. Remove:

- resumable child certification;
- build-burden-aware active-cut lifecycle management;
- true wall-clock-bounded compact warm starts;
- constructive drone diversification;
- LP-support route-pool search;
- first-incumbent route-pool triggers;
- unconditional diversity-aware insertion;
- any promised-window default claim if the no-window defaults are retained.

Retain a shorter contribution statement centered on:

- the truck-as-hub problem and transformed route encoding;
- exact forward pricing;
- physical-location dominance;
- dominance-frontier candidate management;
- branch-indexed route inheritance;
- active-SR coefficient caching;
- verified full-pool primal search;
- compact warm starts as primal-only initialization.

### Introduction Contribution List

Replace the long fourth contribution bullet with a concise contribution. Suggested text:

> We develop an exact forward-pricing architecture for the transformed truck-as-hub route space. The implementation combines source-successor partitioning, branch-language-preserving dominance, physical-location dominance with return-time credit, dominance-frontier filtering, route-signature caching, branch-indexed route inheritance, and row-local active-SR coefficient caching. Every entering column is decoded and verified under the current node state. A separate full route-pool heuristic and compact arc-based warm start improve primal incumbents without affecting lower-bound certification.

If computations use pricing tolerance `0.05`, replace `exact` in computational-result sentences with `exact mathematical framework evaluated under the stated pricing tolerance`.

### BPC Node-Processing Algorithm

The pseudocode says to add all violated SR cuts. The implementation adds a deterministic batch capped by `sr_cut_add_batch_size=32`. Revise the line to:

> Separate residual SR inequalities and add up to the configured batch limit of the most violated inactive cuts; reoptimize and reprice whenever the active cut set changes.

Remove any step suggesting that partial productive pricing can lead directly to cut separation or branching. Certification must remain exhaustive under one current epoch.

### Post-Root Node Management

Delete the active-cut removal-score paragraph and the resumable-certification paragraph. Retain and streamline:

- branch-indexed inheritance;
- inactive-column management, qualified as unablated;
- incremental RMP maintenance;
- row-local active-SR coefficient caching.

### Source-Neighbor Parallel Forward Pricing

Retain the route-set partition and exactness proposition. Rewrite the implementation paragraphs so they do not require one-neighbor tasks, persistent processes, or runtime task stealing. State that deterministic source-successor or prefix subproblems are assigned to a configurable thread/process backend and that closure requires every subproblem to exhaust under the same epoch.

### Forward Candidate Management

Retain:

- two-level route signatures;
- dominance frontier keys;
- dominance-compatible buckets;
- layered necessary-condition gates;
- Pareto-frontier storage;
- lazy decoding and direct verification.

Remove from the omnibus exactness list:

- active-SR cut removal/reactivation;
- resumable child certification;
- wall-clock compact budgets;
- constructive drone diversification;
- first-incumbent triggers;
- LP-support incumbent search;
- unconditional diversity insertion.

The proposition should address only devices that are actually described and used. Primal heuristics do not need to appear in a proposition about the residual route set because their lower-bound neutrality can be stated separately.

### Primal Heuristics and Column Pooling

Reorganize the subsection into:

1. constructive initial-column generation;
2. fixed-solver-budget compact warm start;
3. verified full node-admissible hard-pool search;
4. soft residual identification and repair pricing.

Remove the constructive-diversification paragraph, first-incumbent trigger, and extensive LP-support RPH formulation. Retain the full-pool primal-validity proposition.

### Computational Section

Delete the current `Computational Audit and Revision Implications` section before constructing the final IJOC experiment section. It is an internal development audit, explicitly says it is not a full computational study, and no longer matches the selected defaults.

The replacement computational section should include:

- a frozen table of actual small, medium, and large defaults;
- explicit backend and worker count;
- explicit pricing tolerance;
- whether service windows are active;
- at least several seeds per size rather than one selected seed;
- BPC versus compact Gurobi on identical instances and deadlines;
- root closure, full-tree closure, incumbent, lower bound, and best reduced cost;
- ablations only for components claimed to improve efficiency;
- separate build, LP solve, pricing, dominance, and heuristic times.

### Proof of Candidate-Management Exactness

Remove proof sentences for deleted components. The proof should establish only:

- exact partition of the route set;
- safe service-feasibility extension rejection when windows are active;
- safe reduced-cost pruning;
- dominance deletion only under the stated propositions;
- route-signature equivalence;
- full route verification before insertion;
- backend-independent exhaustive certification.

Do not use the proof to imply that an implementation device is computationally beneficial. Exactness and efficiency are distinct claims.

## Proposed Revised Algorithmic Positioning

The revised paper should position the contribution in three layers.

### Layer 1: Problem and Route Encoding

The paper introduces an elementary truck-as-hub VRPD in which passive pads become operational only when visited by trucks carrying their own drone fleets. The transformed network gives a cost-preserving elementary-path representation of truck paths and simultaneous drone-service blocks.

### Layer 2: Exact Route Generation

The exact method consists of a set-partitioning master, SR inequalities, physical-decision branching, Farkas pricing, and forward label-setting pricing. The principal pricing contributions are branch-language preservation, physical-location dominance with return-time credit, safe reduced-cost pruning, and exhaustive source-successor partition certification.

### Layer 3: Verified Implementation Devices

The implementation uses dominance-frontier filtering, route signatures, lazy decoding, direct reduced-cost verification, branch-indexed inheritance, active-SR coefficient caching, full node-admissible route-pool incumbent search, and a primal-only compact warm start. These devices should be called implementation components unless matched ablations demonstrate a statistically reliable efficiency improvement.

This three-layer structure is substantially clearer than presenting every diagnostic counter and inactive policy as a methodological contribution.

## Language Rules for the Revision Model

The model revising the LaTeX should follow these rules:

1. Do not claim a component improves efficiency merely because it is exact or has a diagnostic counter.
2. Use `implemented` only when the production path actually executes the described mechanism.
3. Use `activated in the reported runs` only when the corresponding counter is positive.
4. Use `improves runtime` only when a matched ablation supports the statement.
5. Distinguish mathematical exactness from tolerance-based computational closure.
6. Distinguish a solver-time budget from an end-to-end wall-clock budget.
7. Distinguish static prefix partitioning from runtime task stealing or state transfer.
8. Distinguish constructive initial columns from a complete constructive incumbent.
9. Distinguish a soft repair target from a valid incumbent.
10. Keep all lower-bound and fathoming claims independent of primal heuristics.

## Final Removal Checklist

Before finalizing the revised manuscript, search for and remove or rewrite every occurrence of:

- `resumable child-certification`;
- `ChildCertState` as a restored label-search state;
- `true wall-clock-bounded compact warm start`;
- `conditional compact warm start` under the current forced policy;
- `constructive drone diversification`;
- `first-incumbent route-pool trigger`;
- `LP-support route-pool diving` as a demonstrated contributor;
- `build-burden-aware SR-cut removal`;
- `SR cut reactivation` as a reported mechanism;
- `runtime dynamic splitting`, `task stealing`, or live-label transfer;
- `persistent process workers` as the universal default;
- `diversity-aware insertion` as backend independent;
- promised-window default claims if `service_deadline_mode="none"` remains selected;
- `proved optimal` for runs closed only with pricing tolerance `0.05`;
- the obsolete `Computational Audit and Revision Implications` tables and conclusions.

After these deletions, verify that all removed macros, equation labels, proposition lists, proof references, algorithm lines, diagnostic fields, and cross-references are also removed or updated. The revised manuscript should remain self-contained and should not refer to a deleted acceleration component elsewhere in the abstract, introduction, theorem statements, proofs, computational tables, or conclusion.

