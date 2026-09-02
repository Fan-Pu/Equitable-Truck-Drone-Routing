# Paper Component Retention and Deletion Audit

## Purpose

This report audits the algorithmic claims in:

`C:\Users\77270\Downloads\Transportation-Science-template (1).tex`

against:

1. the current production implementation in `thvrpd`;
2. the completed 24-case PS/PC campaign artifacts;
3. the completed eight-case large-25 campaign artifacts; and
4. the implementation diagnostics recorded for those runs.

The objective is to identify material that can be deleted or moved out of the
main paper because it is inactive, empirically ineffective, redundant for
exactness, or unsupported by the reported experiment design.

The recommendations distinguish three different situations:

- **Inactive**: the component was implemented but was never invoked.
- **Ineffective in the recorded campaign**: the component was invoked but
  produced no recorded deletion, prune, branch, incumbent update, or other
  direct algorithmic action.
- **Conditionally necessary**: the component was inactive in the experiments
  but remains a valid fallback needed for the stated general exact algorithm.

These distinctions matter. An inactive fallback should usually be condensed or
moved to an online supplement, whereas an empirically ineffective acceleration
should not be advertised as a demonstrated source of efficiency.

## Evidence Base

The primary campaign evidence comes from:

```text
numerical_experiments/ps_pc_bpc_compact_24case_1h_campaign
numerical_experiments/ps_pc_large25_bpc_compact_8case_1h_campaign
```

The combined audit contains 24 distinct instances:

- 8 small cases;
- 8 medium cases;
- 8 large-25 cases;
- PS and PC spatial layouts;
- four seeds per scale and layout combination.

Across these cases:

- all 24 roots were closed;
- 20 cases terminated as optimal;
- 4 large cases ended at the time limit after root closure;
- 75 BPC nodes were processed in total;
- 37 branch decisions were made, all by customer-pair branching.

The archived campaigns should be treated as the authoritative evidence for the
paper's current experiment design. Some earlier standalone runs contain strong
diagnostics for older dominance and frontier implementations. Those older
results should not be used to validate the current corrected dominance
mathematics unless the current implementation is rerun under the same
diagnostic protocol.

## Executive Decision Table

| Component | Implemented | Active in 24 cases | Measured useful action | Recommended paper treatment |
|---|---:|---:|---:|---|
| Customer-pair branching | Yes | Yes | 37 branches | Retain |
| Service-mode branching | Yes | No | 0 branches | Delete from main algorithm |
| Launch-pad branching | Yes | No | 0 branches | Delete from main algorithm |
| Transformed-arc branching | Yes | No | 0 branches | Delete if route-variable fallback is retained |
| Route-variable fallback | Yes | No | 0 branches | Retain in one concise completeness paragraph |
| Farkas pricing | Yes | No | 0 calls | Condense in main text; move details and proofs to supplement |
| Phase-I seeding | Yes | No | 0 solves | Delete from paper; consider deleting from code |
| Same-node dominance | Yes | Yes | 668,522 tests, 0 deletions | Do not claim measured efficiency; retain only concisely or remove |
| Physical-location dominance | Yes | Yes | 132,229 full tests, 0 deletions | Strong deletion candidate |
| Return-time-credit screening | Yes | Yes | 34,080,762 checks, 0 dominance deletions | Strong deletion candidate with physical-location dominance |
| Fractional-knapsack bound pruning | Yes | Yes | 0 direct bound prunes | Remove as an efficiency claim; retain only if used by scheduling |
| Best-bound early termination | Yes | Yes | 0 recorded standard-bound prunes | Move proof/details to supplement or remove with bound logic |
| Deadline service-envelope checks | Yes | Yes | 5,738,305 rejected extensions | Retain |
| Dominance frontier/index | Yes | Not evidenced in primary campaign | 0 recorded frontier queries and 0 avoided pairs | Do not claim current empirical benefit without a fresh audit |
| Dynamic process splitting | Yes | Yes in hard cases | 27 splits; 11,687 labels transferred | Retain |
| SR inequalities | Yes | Yes | 2,745 cuts in 12 cases | Retain |
| Row-local SR coefficient cache | Yes | Yes | Hundreds of millions of cache hits | Retain |
| Incremental RMP updates | Yes | Yes | 1,150 updates | Retain |
| RMP basis reuse | Yes | Attempted | 1,691 attempts, 0 successes | Keep out of paper; consider removing from code |
| Branch-indexed inheritance | Yes | Yes | 34,448 child candidates accepted | Retain, preferably in appendix |
| Inactive-column storage/rehydration | Yes | Yes in 5 cases | 7,936 deactivations; 1,938 rehydrations | Retain as optional implementation detail |
| Constructive initial columns | Yes | Yes | 76 verified routes; no direct incumbent | Retain and call it initial-column generation |
| Compact Gurobi warm start | Yes | Yes | 58 accepted columns in all 24 cases | Retain |
| Full node-admissible route-pool IP | Yes | Yes in large cases | 40 calls; 11 incumbent updates | Retain |
| Soft residual model | Yes | No | 0 solves | Delete from paper |
| Repair pricing | Yes | No | 0 generated routes | Delete from paper |

## High-Confidence Deletions

### 1. Service-Mode Branching

**Current paper location**

- `\paragraph{Service-mode branching.}` near line 1590.
- Related branch-selection text near lines 1670-1672.
- Related extension restrictions near line 1876.
- Related branch-language text near lines 1920-1944.

**Evidence**

```text
service-mode branches across 24 cases = 0
```

The current generator makes this branch structurally unavailable in the
reported benchmark family:

- a fixed fraction of customers is designated mandatory-drone;
- truck arcs incident to mandatory-drone customers are removed;
- optional drone arcs are not retained;
- nonmandatory customers therefore have truck service but no drone service.

Consequently, a customer does not have a fractional truck-versus-drone choice
in the generated instances.

**Recommendation**

Delete service-mode branching from the principal BPC algorithm. If the authors
want to retain it for a broader problem class, place one sentence in an online
supplement describing it as an optional branching extension not exercised in
the reported experiments.

**Dependent cleanup**

- Remove service-mode branch definitions and flow indicators.
- Remove truck-service/drone-service branch cases from the branch-selection
  order.
- Remove service-mode branch rejection rules from the pricing-extension prose.
- Remove service-mode terms from branch-index diagnostic descriptions.
- Do not claim that the computational study evaluates service-mode branching.

### 2. Launch-Pad Branching

**Current paper location**

- `\paragraph{Drone launch-pad branching.}` near line 1610.
- Related selection and pricing restrictions near lines 1670 and 1876.

**Evidence**

```text
launch-pad branches across 24 cases = 0
```

The experiment generator imposes:

```text
max_drone_launch_hubs_per_customer = 1
retain_optional_drone_arcs = False
```

Every drone-required customer therefore has one protected launch pad. A
fractional launch-pad assignment cannot occur.

**Recommendation**

Delete launch-pad branching from the main paper. Its inclusion creates the
appearance that the reported algorithm resolves pad-assignment fractionality,
although the test instances eliminate that choice during generation.

**Dependent cleanup**

- Remove launch-pad branching equations and sets.
- Remove launch-pad branch cases from extension feasibility and branch-language
  descriptions.
- Remove launch-pad rejection diagnostics from the paper.
- Explicitly disclose the one-launch-pad-per-drone-customer instance policy in
  the experiment setup.

### 3. Transformed-Arc Branching

**Current paper location**

- `\paragraph{Fallback transformed-arc branching.}` near line 1632.
- Branch-selection and completeness discussion near lines 1670-1672.
- Transformed-arc branch automata and branch-interface conditions in the
  detailed pricing appendix.

**Evidence**

```text
transformed-arc branches across 24 cases = 0
```

Customer-pair branching resolved every observed fractional node. A
route-variable fallback already provides a finite completeness mechanism.
Therefore transformed-arc branching is redundant for the paper's demonstrated
algorithmic behavior.

**Recommendation**

Delete transformed-arc branching and retain a concise route-variable fallback.
This substantially simplifies:

- branch automata;
- branch-language compatibility;
- branch-interface compatibility;
- dominance proofs;
- child route-index fields; and
- the branch-selection section.

If transformed-arc branching remains in the production code, the manuscript
and implementation will no longer describe exactly the same algorithm.
Therefore this deletion should be paired with code removal or with an explicit
statement that transformed-arc branching is an unreported optional extension.

### 4. Soft Residual Identification and Repair Pricing

**Current paper location**

- Summary near line 1705.
- `\paragraph{Soft residual identification and repair pricing.}` near lines
  2702-2715.
- References in the full-pool validity proposition and proof.

**Evidence**

```text
soft route-pool solves = 0
repair routes generated = 0
```

The hard full-pool model was feasible whenever the route-pool heuristic was
called. The soft model and repair-pricing path did not participate in any
incumbent or proof result.

**Recommendation**

Delete the soft set-partitioning formulation, repair reduced cost, and related
prose. Retain only the hard full node-admissible route-pool IP, which produced
11 incumbent updates in six large cases.

**Dependent cleanup**

- Remove `\RepairCustSet`, `\RepairReducedCost`, and repair-reward notation if
  unused elsewhere.
- Rewrite the primal-validity proposition to concern only the hard full-pool
  model.
- Remove soft/repair diagnostics from the experiment table and discussion.

### 5. Unsupported Mixed-Distribution and Sensitivity Claims

**Current paper location**

- Introduction contribution near line 510.

The introduction claims experiments with:

```text
sparse, clustered, and mixed customer-location distributions
```

and a sensitivity analysis. The current experiment setup defines only:

- purely sparse (PS); and
- purely clustered (PC).

No mixed-distribution experiment or sensitivity-analysis section appears in
the attached paper.

**Recommendation**

Delete "mixed customer-location distributions" and the sensitivity-analysis
claim. Replace the contribution with a precise statement that the study uses
24 PS/PC instances over three scales and compares BPC with direct Gurobi on the
same compact MIQP.

### 6. Broad Solver-Superiority Claim in the Abstract

**Current paper location**

- Abstract near line 432.

The phrase:

```text
outperforms state-of-the-art solvers
```

is broader than the experiment design, which compares against one commercial
solver applied to one compact formulation. It also appears before the paper
contains computational result tables.

**Recommendation**

Delete this phrase until the result section supplies a precisely quantified
comparison. A defensible replacement is:

```text
We compare the proposed BPC algorithm with Gurobi applied directly to the
compact MIQP on the same instances and objective scale.
```

### 7. Finite-Convergence Claim Without a Corresponding Theorem

**Current paper location**

- Abstract near line 432.

The abstract says the paper "show[s] the finite convergence and exactness" of
the algorithm, but the attached version does not contain a standalone global
finite-convergence theorem. It contains local pricing and scheduling exactness
results.

**Recommendation**

Either restore a global finite-convergence theorem or delete the
finite-convergence claim. For page reduction, the cleaner choice is to claim
exact route-space pricing and valid branch-and-bound lower bounds, not a theorem
that is absent from the paper.

## Strong Deletion Candidate: Physical-Location Dominance

### Current material

This component occupies a large amount of manuscript space:

- physical-location dominance statement and definitions near lines 2101-2213;
- return-credit construction;
- branch-interface compatibility;
- references in standard-pricing and candidate-management exactness results;
- proof near lines 2847-2871;
- Farkas-dominance dependence near line 2904.

### Recorded behavior

```text
return-time-credit checks                    34,080,762
full physical-location dominance tests          132,229
labels deleted by physical-location dominance          0
```

The component therefore imposed substantial comparison work but produced no
recorded label deletion in the primary campaign.

### Recommendation

Physical-location dominance with return-time credit is the strongest
algorithmic deletion candidate. Removing it does not invalidate route
enumeration or BPC exactness; it only removes a pruning rule. Under the recorded
instance family, the rule contributed no pruning.

If it is removed:

1. retain same-node dominance as the only label dominance rule;
2. delete return-credit notation and equations;
3. delete branch-interface compatibility for distinct endpoints;
4. simplify Farkas dominance to same-endpoint comparisons;
5. simplify the standard-pricing exactness theorem;
6. simplify frontier keys so they no longer search across distinct transformed
   endpoints sharing one physical pad; and
7. remove the physical-location proof.

### Important qualification

An earlier standalone large-25 run reported nonzero physical-location
deletions under an older implementation. The later 24-case campaign reported
zero deletions under the corrected dominance logic. The older result should not
be used to support the current theorem without a fresh reproducible ablation.

## Same-Node Dominance: Retain Concisely or Remove as a Claimed Contribution

### Recorded behavior

```text
full same-node dominance tests       668,522
labels deleted by same-node dominance      0
```

The corrected rule did not delete a label in the primary campaign.

### Recommendation

Do not present same-node dominance as a demonstrated efficiency improvement.
There are two defensible paper designs:

**Conservative design**

- Retain one concise same-node dominance proposition as a standard label-setting
  safeguard.
- Move its full proof to the online supplement.
- Remove claims that it materially reduced the reported pricing workload.

**Maximum page-reduction design**

- Remove both same-node and physical-location dominance from the algorithm.
- Rely on finite elementary enumeration, service-window feasibility checks, and
  reduced-cost bounds.
- Update the pricing exactness theorem accordingly.

The conservative design is preferable unless a no-dominance timing ablation
shows that removing dominance is harmless outside the current sparse instance
family.

## Fractional-Knapsack Reduced-Cost Bound and Early Termination

### Current material

- Dual-reward pruning near lines 2419-2538.
- Best-bound selection and exact early termination near lines 2540-2547.
- Proofs near lines 2957-3037.
- References in the standard-pricing exactness theorem.
- Use in the dynamic-splitting closure-gap definition.

### Recorded behavior

```text
standard labels pruned directly by the bound = 0
Farkas labels pruned by the standard bound   = 0
```

The latter must be zero by design because the standard bound is invalid in
Farkas mode. More importantly, the standard bound produced no direct prune in
the 24-case campaign.

### Recommendation

Do not describe this component as an empirically effective pruning device.
However, it cannot be deleted independently because the current dynamic
scheduler uses the valid completion bound to define a task's closure gap.

Use one of the following coherent revisions:

**Option A: retain only as scheduler support**

- Keep the completion lower-bound equation.
- Explain that it orders open labels and identifies tasks already close to
  closure.
- Delete or move the long pruning and early-termination proofs to the online
  supplement.
- Remove wording that attributes observed speedup to direct bound pruning.

**Option B: delete the entire bound package**

- Remove the fractional-knapsack bound and its proofs.
- Remove best-bound early termination.
- Redefine dynamic split eligibility using only open-label count, elapsed time,
  and remaining-work estimate.
- Update the pricing and scheduler exactness theorems.

Option A requires less code and paper restructuring.

## Dominance Frontier and Candidate Gates

### Current claim

The paper presents dominance-compatible frontier access and necessary-condition
gates as an exact acceleration.

### Primary campaign evidence

```text
recorded frontier queries                       0
recorded pairs avoided before materialization   0
```

The current source contains frontier-index logic, but the archived primary
campaign does not demonstrate that this path reduced comparisons. This may
reflect a difference between the campaign source snapshot and the current
working tree, or a diagnostic path that was inactive in the archived run.

### Recommendation

Do not claim measured frontier efficiency from the primary campaign. Until a
fresh run records nonzero frontier-query and avoided-pair statistics:

- retain the structure only as an implementation detail in the appendix;
- remove it from the abstract and contribution list;
- avoid reporting it as a source of computational improvement; and
- do not cite older frontier statistics as evidence for the current corrected
  dominance implementation.

If physical-location dominance is deleted, simplify the frontier to
same-endpoint storage or remove the frontier section entirely.

## Inactive but Conditionally Necessary Exactness Fallbacks

### Farkas Pricing

**Evidence**

```text
Farkas pricing calls = 0
Farkas columns added = 0
Phase-I solves       = 0
```

Farkas pricing was not needed because initialization produced a feasible RMP in
every recorded case. Nevertheless, Farkas pricing remains the rigorous way to
handle an infeasible restricted master for the general algorithm.

**Recommendation**

- Keep one short main-text paragraph stating the infeasible-RMP logic.
- Move the Farkas reduced-cost recursion, dominance rule, exactness theorem, and
  proofs to an online supplement.
- Delete Phase-I seeding from the paper because it was inactive and is not
  required if exact Farkas pricing is retained.
- Do not describe Farkas pricing as a measured computational acceleration.

Deleting Farkas pricing entirely is appropriate only if the algorithm is
changed to guarantee a feasible initial RMP at every branch-and-bound node.
The current constructive generator alone did not produce a complete incumbent
in any of the 24 cases, so that guarantee should not be assumed.

### Route-Variable Fallback

**Evidence**

```text
route-variable fallback branches = 0
```

Unlike service-mode, launch-pad, and transformed-arc branching, the
route-variable fallback has a simple theoretical role: it guarantees progress
if customer-pair branching does not expose a fractional physical indicator.

**Recommendation**

Retain it in one concise paragraph, without a long implementation discussion.
The simplified branch hierarchy should be:

1. customer-pair branching; then
2. route-variable fallback.

## Components to Retain

### 1. Customer-Pair Branching

This was the only active branching rule:

```text
customer-pair branches = 37
```

It resolved all observed post-root fractionality. It should be the principal
branching mechanism in the paper.

### 2. Service-Envelope Extension Checks

The latest-service constraints rejected:

```text
5,738,305 forward extensions in 16 cases
```

This is the strongest directly measured label-space reduction. Retain:

- customer-specific service upper bounds;
- extension-time checks;
- deadline reachability;
- their role in generating a smaller exact route set.

The paper should clearly distinguish a hard service-window feasibility check
from a heuristic label cap.

### 3. Subset-Row Inequalities

The algorithm added:

```text
2,745 SR cuts in 12 cases
```

SR separation is active and central to the proof bound. Retain the SR
formulation, batched separation, and the requirement to reoptimize and reprice
after adding cuts.

### 4. Row-Local SR Coefficient Caching and Incremental RMP Updates

Recorded activity includes:

```text
active-coefficient cache hits   268,674,306
SR coefficient cache hits       590,412,260
incremental RMP updates               1,150
```

These components are heavily used. They should remain, although detailed cache
key formats belong in the appendix rather than the main algorithm narrative.

### 5. Balanced Process Pricing and Dynamic Splitting

Dynamic refinement was selective rather than universal:

```text
split candidates               59
committed splits               27
labels transferred         11,687
cases with committed splits     3
idle-work requests          40,639
```

The mechanism activates on difficult pricing tails, which is consistent with
its design. Retain:

- exact source-successor partition;
- unique task ownership;
- epoch invalidation;
- checkpoint splitting;
- no cross-task dominance;
- exhaustive same-epoch closure.

The detailed transfer transaction may be shortened or moved to an online
supplement, but the exact partition and closure proposition should remain.

### 6. Branch-Indexed Inheritance

Recorded activity includes:

```text
global branch-index queries          1,789
child inherited route candidates    34,448
cases with child inheritance             8
```

This is active in the large post-root tree. Retain it as post-root route
management, not as a new lower-bounding method.

### 7. Inactive-Column Storage and Rehydration

The optional mechanism was active in five cases:

```text
columns deactivated    7,936
columns rehydrated     1,938
rehydration events       245
```

Retain it in the appendix. The paper should state that a bound is used only
after reoptimization and exact repricing, so temporary deactivation cannot
create an artificial lower bound.

### 8. Constructive and Compact Initial Columns

The constructive procedure generated:

```text
76 verified routes across all 24 cases
constructive complete incumbents = 0
```

The current paper correctly calls this initial-column generation rather than a
guaranteed constructive incumbent. Retain that wording.

The compact warm start:

```text
was attempted in all 24 cases
accepted 58 verified route columns
```

It should remain because it supplies feasible route sets and useful initial
columns. It remains primal-only and must not be described as contributing a BPC
lower bound.

### 9. Full Node-Admissible Route-Pool Search

Recorded behavior:

```text
hard full-pool calls              40
incumbent updates                 11
cases with an incumbent update     6
```

This is the only route-pool heuristic with demonstrated incumbent impact.
Retain the hard exact-cover IP evaluated by true route costs.

### 10. Canonical Drone-Block Ordering

Canonical ordering is not merely a heuristic acceleration. It establishes a
unique transformed encoding for each simultaneous drone block and removes
permutation symmetry by construction. Retain the definition and a concise
validity argument even if no standalone timing counter is available.

### 11. Route Signatures and Direct Verification

Recorded cache activity includes:

```text
core-signature cache hits      105,727
active-signature cache hits    325,608
```

Retain exact route equivalence, lazy decoding, and master-side recomputation of
route feasibility, coefficients, cost, and reduced cost.

## Code-Only Component That Should Not Be Added to the Paper

The current code attempts RMP basis reuse:

```text
basis reuse attempts  = 1,691
basis reuse successes = 0
```

The attached paper does not emphasize basis reuse, which is appropriate. Do not
add it to the manuscript. It is also a reasonable code-deletion candidate
because the recorded implementation never successfully reused a basis.

## Exact LaTeX Revision Map

Line numbers refer to the attached file and will shift after editing.

### Abstract

Near line 432:

- Delete "problem-specific dominance rules" if physical-location and same-node
  dominance are removed as contributions.
- Replace "safe reduced-cost pruning" with a narrower statement if the
  fractional-knapsack bound remains only for queue ordering or split control.
- Delete "show the finite convergence" unless a global theorem is restored.
- Replace "outperforms state-of-the-art solvers" with a precise Gurobi
  comparison statement supported by result tables.

### Introduction

Near line 510:

- Delete "mixed customer-location distributions."
- Delete the sensitivity-analysis claim unless such experiments are added.
- Align the contribution list with PS and PC layouts only.

### Branching Section

Near lines 1542-1672:

- Retain customer-pair branching.
- Delete service-mode branching.
- Delete launch-pad branching.
- Delete transformed-arc branching.
- Retain one concise route-variable fallback.
- Rewrite the selection order as customer pair followed by route-variable
  fallback.

### Main Pricing Summary

Near lines 1693-1705:

- Retain forward-only pricing.
- Do not claim measured dominance efficiency from the current campaign.
- Condense Farkas pricing to one paragraph.
- Remove soft residual repair.
- Retain deadline feasibility, exact parallel partitioning, route verification,
  SR/RMP caching, compact initialization, and full-pool primal search.

### Detailed Label Appendix

Near lines 1876-1944:

- Remove service-mode and launch-pad branch cases.
- Remove transformed-arc-up automata if transformed-arc branching is deleted.
- Retain only the customer-pair branch state needed for together/separate
  decisions.

Near lines 2054-2213:

- Delete physical-location dominance, return credit, and branch-interface
  compatibility if adopting the recommended simplification.
- Retain a concise same-node rule or remove the entire dominance package.

Near lines 2419-2547:

- Move fractional-knapsack pruning and early-termination proofs to an online
  supplement.
- If retained for dynamic split control, keep only the completion-bound
  definition and its validity statement.

Near lines 2552-2624:

- Retain the exact partition and dynamic-splitting proposition.
- Condense operating-system messaging, transfer bytes, and transaction details
  unless systems-level scheduling is a stated contribution.

Near lines 2626-2657:

- Remove or reduce dominance-frontier detail if the current campaign cannot
  demonstrate nonzero use.
- Retain route signatures, branch inheritance, SR coefficient caching, lazy
  decoding, and direct verification.
- Keep inactive-column control as optional appendix text.

Near lines 2659-2720:

- Retain constructive initial columns.
- Retain compact warm start.
- Retain the hard full-pool IP.
- Delete the soft residual model and repair pricing.

### Proof Appendix

Delete or rewrite the following dependencies if the corresponding components
are removed:

- proof of physical-location dominance near lines 2847-2871;
- physical-location references in the pricing exactness proof;
- physical-location dependence in Farkas dominance;
- dual-reward pruning proof near lines 2957-3013;
- early-termination proof near lines 3014-3037;
- soft-repair references in the full-pool validity proof;
- service-mode, launch-pad, and transformed-arc cases in branch-language proofs.

## Recommended Lean Algorithm Narrative

A page-efficient and empirically faithful algorithm section can use the
following structure:

1. **Route-space master and SR inequalities.**
2. **Canonical transformed network and cost-preserving route encoding.**
3. **Forward label-setting pricing with payload, elementarity, drone-block, and
   hard service-envelope resources.**
4. **Customer-pair branching with route-variable fallback.**
5. **Exact source-successor process partition and dynamic task splitting.**
6. **Incremental RMP construction, row-local SR coefficients, route signatures,
   and branch-indexed inheritance.**
7. **Constructive and compact initial columns plus hard full-pool incumbent
   search, all primal-only.**
8. **Conditional Farkas pricing in a short exactness paragraph or online
   supplement.**

This structure retains every component that was either mathematically
structural or measurably active, while removing branch layers and heuristic
paths that did not participate in the reported experiments.

## Recommended Contribution Claims After Deletion

The paper's computational contribution should center on:

- the truck-as-hub transformed route encoding;
- exact forward pricing under elementary routing and hard service envelopes;
- exact process-based pricing partition and dynamic splitting;
- active SR/RMP coefficient management;
- customer-pair branching and post-root route inheritance; and
- verified primal initialization and full-pool incumbent search.

The paper should not claim, without a fresh ablation, that efficiency is driven
by:

- physical-location dominance;
- same-node dominance;
- fractional-knapsack pruning;
- dominance-frontier access;
- service-mode branching;
- launch-pad branching;
- transformed-arc branching;
- Farkas pricing in the reported cases; or
- soft residual repair.

## Final Deletion Priority

### Delete first

1. Mixed-distribution and sensitivity claims.
2. Service-mode branching.
3. Launch-pad branching.
4. Transformed-arc branching.
5. Soft residual identification.
6. Repair pricing.
7. Broad solver-superiority and unsupported finite-convergence claims.

### Delete next if code is simplified accordingly

8. Physical-location dominance and return-time credit.
9. Detailed dominance-frontier machinery.
10. Detailed fractional-knapsack pruning and early-termination proofs.

### Move to online supplement rather than delete entirely

11. Farkas pricing details and proofs.
12. Detailed split-transfer transaction.
13. Optional inactive-column management.
14. Full cache-key and branch-index implementation details.

### Retain

15. Canonical transformed route encoding.
16. Customer-pair branching and concise route-variable fallback.
17. Hard service-envelope extension checks.
18. SR inequalities and exact repricing.
19. Balanced process pricing and exact dynamic splitting.
20. Incremental RMP and row-local SR coefficient caching.
21. Constructive and compact verified initial columns.
22. Hard full-pool true-objective incumbent search.
23. Route signatures, direct verification, and branch-indexed inheritance.

## Methodological Caution

Event counts establish whether a component acted, but they do not by themselves
prove its causal runtime benefit. A component with zero actions is a strong
deletion candidate. A component with many actions should still be called an
efficiency improvement only after an ablation compares otherwise identical
runs with and without that component.

The attached paper currently contains no ablation table for dominance,
reduced-cost pruning, dynamic splitting, inactive-column management, caching,
or primal search. Therefore the safest journal wording is:

- call these mechanisms exact implementation components;
- report their observed action counts and timing shares;
- reserve "improves efficiency" for components supported by paired ablations.
