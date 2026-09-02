# Appendix Shortening Guide for the Latest TH-VRPD Manuscript

## Objective

The current appendix begins with `Detailed Forward Label-Setting Pricing
Algorithm` and contains approximately 1,200 source lines. A defensible target
is a reduction of roughly 35--45 percent while retaining the mathematical
content required to establish exactness.

The recommended cuts prioritize:

- duplicated explanations already present in the main text;
- formal results that merely restate definitions;
- implementation details that are not theoretical contributions;
- repeated warnings about primal-only or Farkas-only behavior; and
- long proofs of immediate facts that can be replaced by one paragraph.

The central pricing recursions, same-node dominance, physical-location
dominance, Farkas pricing, and valid completion lower bound should remain.

## Important Consistency Issue Before Shortening

The latest main paper retains `Arc-flow branching on the transformed network`.
Therefore, the customer-conditioned transformed-arc states in the pricing
appendix are currently necessary. They should not be deleted unless the main
branching subsection and production code are also changed.

The manuscript also contains a serious missing-result problem:

- Farkas dominance refers to `eq:pld-cust` through `eq:pld-time`;
- the appendix contains `Proof of Proposition~\ref{prop:physical-location-dominance}`;
- that proof invokes `eq:pld-cost`; but
- the physical-location dominance proposition and these labeled equations are
  absent from the supplied source.

Since physical-location dominance remains active in the production solver,
the proposition should be restored. Deleting only its proof does not fix the
problem. If page pressure requires removing this component, it must be removed
from the algorithm, Farkas dominance, diagnostics, proofs, and code together.

## Recommended Retained Appendix Structure

Use four appendix sections:

```latex
\section{Forward Label-Setting Pricing}
\section{Farkas Pricing}
\section{Exact Implementation Components}
\section{Proofs}
```

Avoid placing a subsection directly under a section with essentially the same
title.

## Priority 1: Unconditional Deletions

### 1. Delete duplicated section/subsection titles

At the beginning of the forward-pricing appendix, retain:

```latex
\section{Detailed Forward Label-Setting Pricing Algorithm}
\label{app:label-setting-details}
```

Delete:

```latex
\subsection{Detailed label-setting pricing algorithm}
\label{app:sec-label-setting-detail}
```

Similarly, under:

```latex
\section{Detailed Farkas Pricing}\label{app:farkas-details}
```

delete:

```latex
\subsection{Detailed Farkas pricing for infeasible RMP nodes}
\label{app:sec-farkas-pricing-detail}
```

Update any references to the deleted subsection labels to reference the section
labels instead.

### 2. Delete the unused preliminary SR correction set

Under `\paragraph{Same-node dominance.}`, delete the display beginning:

```latex
Let \(\mathcal C(\PartialPath{i})\) denote ...
```

and defining:

```latex
\hat{\SRset}:=
\left\{
\SRsubset\in\SRset:
\nu_{\SRsubset}<0,
\NumCustPiCoverByLabel{i}{\SRsubset}^{1}\in\{1,3\},
\NumCustPiCoverByLabel{i}{\SRsubset}^{2}\in\{0,2\}
\right\}.
```

The proposition immediately afterward defines the correct adjustment using
exactly dominating count 1 and dominated count 2. The preliminary set is
unused and mathematically inconsistent with the proved result.

### 3. Delete the formal branch-language lemma

Delete the formal result:

```latex
\begin{lemma}[Branch-only language inclusion]
...
\end{lemma}
```

Delete its proof subsection beginning:

```latex
\subsection{Proof of Lemma~\ref{lem:branch-language-inclusion}}
```

Replace the lemma in the pricing appendix with this sentence:

```latex
Equality of all conditional branch states is a conservative sufficient
condition for branch-language inclusion and therefore preserves every
branch-feasible suffix accepted from the dominated label.
```

The original lemma follows immediately from set inclusion and does not warrant
a separately numbered result and proof.

### 4. Delete the repeated completion sentence

Immediately after `eq:label-rc-extension`, delete:

```latex
Any complete label reaching \(\DepotSink\) with negative reduced cost defines
a column to be added to the RMP.
```

The subsequent `Forward Route Completion and Candidate Generation` subsection
provides the more accurate rule, including decoding and master verification.

### 5. Delete the optional inactive-column paragraph

Delete the paragraph beginning:

```latex
\paragraph{Optional inactive-column control.}
```

Its own final sentence states that it is not a demonstrated efficiency
contribution. It is an implementation option rather than part of the exact
methodological contribution, and exact rediscovery is already guaranteed by
pricing.

### 6. Delete repeated Farkas warnings

In `Detailed Farkas Pricing`, the text states twice that standard dual-reward
pruning and standard early termination do not apply in Farkas mode.

Retain the first paragraph beginning:

```latex
Farkas pricing uses the same route-generation rules ...
```

After `eq:farkas-label-rc`, delete the repeated explanation beginning:

```latex
A complete label with negative Farkas reduced cost corresponds to a column ...
```

Replace it with:

```latex
A complete label has negative Farkas reduced cost exactly when its route
violates \eqref{eq:farkas-violating-column}. Standard completion-bound pruning
and early termination are not used in Farkas mode.
```

Also delete the following standalone sentence because it repeats the same
equivalence:

```latex
By construction, a complete label has negative Farkas reduced cost if and only
if its route violates the certificate ...
```

## Priority 2: Replace Formal Results by Short Arguments

### 7. Collapse canonical-ordering proposition and proof

The cost-preserving encoding theorem already relies on a unique canonical
ordering. A second proposition and a long permutation proof are unnecessary.

In `Canonical Ordering of Drone-Service Blocks`, delete:

```latex
\begin{proposition}[Symmetry reduction by canonical block ordering]
...
\end{proposition}
```

Delete the proof subsection beginning:

```latex
\subsection{Proof of Proposition~\ref{prop:canonical-block-ordering}}
```

Retain the two sequence-count formulas if the reduction in representation size
is important. Before them, insert:

```latex
For any block $B$, customer service times, truck waiting time, payload, and
sortie count depend only on the set $B$, not on its ordering. Since $\prec_h$
is a strict total order, sorting $B$ by $\prec_h$ retains exactly one encoding
of every feasible physical block and therefore removes only permutation
symmetry.
```

### 8. Collapse together-branch reachability proposition and proof

Delete the formal proposition beginning:

```latex
\begin{proposition}[Reachability-based together-branch pruning]
```

and delete its proof subsection from `Proofs of Formal Results`.

Replace the proposition with:

```latex
If a together state requires customer $q$ but
$q\notin\ReachableCustSet{\Lambda}$, the label has no branch-feasible
completion and is discarded; the symmetric rule applies when $p$ remains
required. Because this is a feasibility argument, it applies in both standard
and Farkas pricing.
```

### 9. Collapse full-pool primal-validity proposition and proof

Delete:

```latex
\begin{proposition}[Primal validity of full-pool search]
...
\end{proposition}
```

and its proof subsection. Replace both with:

```latex
Because every route in $\NodeRoutePool{m}$ is node-admissible, any feasible
integer solution of $(\mathrm{FRPH}_m)$ satisfies exact customer coverage and
the fleet limit and is therefore a valid incumbent. The soft model is used
only to identify repair customers and never changes a lower bound or certifies
pricing closure.
```

### 10. Convert early termination to a corollary

`Safe early termination of pricing` follows directly from the valid
completion lower bound. Replace the full proposition with:

```latex
\begin{corollary}[Safe early termination]
If no generated complete route is negative and every open nondominated label
$\Lambda$ satisfies $\KnapReducedCostLB{\Lambda}\ge0$, then the current
standard-pricing task contains no negative-reduced-cost route.
\end{corollary}
```

Delete its separate proof subsection. Add one sentence after the corollary:

```latex
The result follows by applying Proposition~\ref{prop:dual-reward-pruning} to
every open label and the applicable dominance result to every discarded label.
```

## Priority 3: Condense Repeated Technical Descriptions

### 11. Condense the label-definition and source-initialization text

The opening label description can be shortened without losing a resource.
Replace the prose beginning:

```latex
The label stores the ordered partial transformed path ...
```

through the source initialization with a compact display such as:

```latex
A label stores
\[
(P,V^{\mathrm T},a,w,h,A_h,W_h,z,\boldsymbol\psi,
\mathfrak b,\bar c),
\]
namely the transformed prefix, visited physical truck nodes, endpoint service
time, payload, active pad, pad-arrival time, active-block waiting time and
drone count, active-SR counters, branch state, and reduced cost. At the source,
all resources and counters are zero, the visited set is
$\{\DepotSource\}$, the active pad is null, and
$\bar c=\ObjCoeffCost\TruckCost-\kappa$.
```

The original distinction between regular and duplicated endpoint time should
remain as one following sentence.

### 12. Remove prose that merely paraphrases displayed feasibility checks

Retain the displays for:

- physical-node elementarity;
- active-pad compatibility;
- customer elementarity;
- payload and drone feasibility; and
- service-window feasibility.

Delete explanatory sentences that simply restate each displayed inequality.
For example, the three sentences after `eq:active-pad-extension` can be
replaced by:

```latex
These conditions preserve active-pad feasibility and canonical block order.
```

Similarly, replace the paragraph following customer elementarity with:

```latex
Together, the two elementarity resources prohibit repeated physical visits
and repeated customer representation.
```

### 13. Condense forward route completion

The extension recursion already enforces most conditions repeated under
`Forward Route Completion and Candidate Generation`.

Replace the three-condition display and its explanatory paragraph with:

```latex
A sink label is a candidate only if it serves at least one customer, all
maintained resources remain feasible, and every inherited branch state is
accepting. Its reduced cost is $\LabelReducedCost{\DepotSink}$.
```

Retain the paragraph describing physical decoding and master-side verification.
Use the experiment's pricing threshold consistently when describing an
entering route.

### 14. Condense the candidate-management section to one paragraph

The following paragraphs can be merged:

- `Two-level route signatures`;
- `Same-endpoint buckets and dominance prefilters`;
- `Branch-indexed inheritance and row-local active-SR coefficients`; and
- `Lazy decoding and direct verification`.

Retain the core-signature and SR-coefficient equations. Replace the surrounding
prose with:

```latex
Core signatures identify canonical routes and their current master and
launch-pad attributes; active signatures append the current SR version and
coefficients. Endpoint, active-pad, branch-state, and resource prefilters only
select candidate dominance pairs and never delete labels. Inherited routes
remain subject to full node-admissibility checks, and cached SR coefficients
are reused only under a compatible active-row structure. Every candidate is
decoded and its feasibility, cost, coefficients, and reduced cost are
recomputed by the master before insertion.
```

### 15. Condense constructive and compact warm-start details

The main text already states that these procedures are primal-only. In the
appendix, retain:

- that constructive routes are verified;
- the fixed compact optimization-time limit;
- canonical extraction and verification; and
- the compact-decomposition proposition if it is retained as a formal
  correctness result.

The definitions `A_k^{\mathrm T}(x)` and `B_{hk}(y)` may be deleted if the
compact-decomposition proof is rewritten directly in terms of selected truck
arcs and drone assignments. They are temporary proof notation and are not used
elsewhere.

The compact-decomposition proof can be reduced to one paragraph: flow and
ordering constraints produce elementary source--sink truck paths; launch,
payload, endurance, and fleet constraints produce feasible pad blocks;
canonical recomputation can only reduce slack times; and exact compact
assignment yields exact route-space coverage.

### 16. Condense soft repair pricing

If the numerical experiments do not separately evaluate the soft model, the
full soft set-partitioning display is more detail than the paper needs. Replace
it with:

```latex
If $(\mathrm{FRPH}_m)$ is infeasible over the current pool, a soft version
adds binary uncovered-customer variables with a dominating penalty. The
uncovered set defines repair rewards, after which every generated repair route
is verified and the hard model is solved again. The soft solution itself is
never an incumbent.
```

Retain `eq:repair-reduced-cost` only if repair pricing is reported in the
computational diagnostics.

## Priority 4: Material Better Moved to an Electronic Companion

### 17. Move the route-duration upper-bound proof

The route-duration bound is needed for valid normalization and big-M values,
but its detailed sorting argument is not central to the BPC contribution.
Keep the proposition and a short proof sketch in the manuscript; move the full
proof beginning:

```latex
\subsection{Proof of Proposition~\ref{prop:route-duration-upper-bound}}
```

to an electronic companion.

### 18. Move low-level dynamic-splitting transfer mechanics

Balanced process pricing is a claimed computational component and should not
be deleted. Retain:

- the prefix-free route-space partition;
- balanced score assignment;
- immutable pricing epochs;
- split eligibility;
- one-owner task semantics; and
- the productive/certification distinction.

The long inventory beginning:

```latex
A transferred state contains the complete canonical prefix, endpoint and
physical location, customer and physical-node masks ...
```

can be replaced by:

```latex
Transferred tasks contain complete live label states and remain associated
with the same pricing epoch. A transfer commits only after donor removal,
unique owner registration, and receiver acknowledgment.
```

Queue protocols, serialized fields, checkpoint polling frequency, and detailed
diagnostics belong in the electronic companion or code documentation.

### 19. Move full primal-model displays if space remains tight

The hard route-pool model is straightforward set partitioning over a verified
pool. In the paper appendix, it can be represented compactly as:

```latex
\[
\min\left\{
\sum_{r\in\NodeRoutePool{m}}\RouteCost{r}z_r:
\sum_r\RouteVisitNode{r}{n}z_r=1~(n\in\CustNodes),
\ \sum_r z_r\le|\Trucks|,
\ z_r\in\{0,1\}
\right\}.
\]
```

Move the separately numbered constraints to the electronic companion unless
they are referenced by a retained proof.

## Proofs That Should Remain in the Manuscript

Retain complete proofs of the results that establish the paper's nonstandard
exactness claims:

1. cost-preserving transformed-route encoding;
2. same-node dominance with the exact SR correction;
3. physical-location dominance with return-time credit;
4. Farkas-valid dominance; and
5. the dual-reward completion lower bound.

The physical-location dominance proposition itself must first be restored,
because its proof currently references missing conditions.

## Proofs That Can Be Shortened or Moved

| Result | Recommended treatment |
|---|---|
| Route-duration upper bound | Short proof sketch; full proof in supplement |
| Branch-language inclusion | Delete formal result and proof |
| Canonical block ordering | Replace proposition/proof with one paragraph |
| Together-branch reachability | Replace proposition/proof with one sentence |
| Safe early termination | Corollary with one-sentence proof |
| Compact decomposition | Retain proposition; shorten proof to one paragraph |
| Full-pool primal validity | Replace proposition/proof with one paragraph |

## Material That Should Not Be Deleted

Do not delete the following merely to save pages:

- active-pad extension conditions;
- separate customer and physical-node elementarity;
- payload, drone-count, payload, endurance, and service-envelope checks;
- service-time, waiting-time, payload, SR-state, and reduced-cost recursions;
- the exact SR count pattern used in dominance;
- the standard/Farkas distinction;
- prefix-free exhaustive process partitioning and certification semantics;
- master-side route decoding and verification; or
- proofs of the nonstandard dominance rules.

These elements support correctness rather than implementation narration.

## Expected Reduction

Applying Priority 1 and Priority 2 should remove roughly 150--220 source lines.
Applying the condensation steps should remove another 180--250 lines. Moving
the route-bound proof and low-level process-transfer details to an electronic
companion should reduce the submitted appendix by another 80--130 lines.

The resulting appendix would be approximately 650--800 source lines, depending
on whether the full route-pool models and route-duration proof remain in the
submitted manuscript.

## Recommended Editing Order

1. Restore or remove physical-location dominance consistently.
2. Delete duplicate headings and the incorrect preliminary SR set.
3. Remove the branch-language, canonical-ordering, reachability, and full-pool
   formal results identified above.
4. Condense Farkas repetition and route-completion repetition.
5. Merge candidate-management paragraphs.
6. Condense warm-start and repair-model descriptions.
7. Move secondary proofs and process-protocol details to an electronic
   companion.
8. Compile and check all references after every structural deletion.

This plan shortens the appendix without weakening the proof chain for the
transformed pricing formulation and exact BPC algorithm.
