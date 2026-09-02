# Exact Paper Revision Guide: Customer-Pair, Launch-Pad, and Route-Variable Fallback Branching

## Scope and retained branching hierarchy

This guide applies to:

```text
C:\Users\77270\.codex\attachments\2500717d-22b9-4557-bfb5-c783f3e17804\pasted-text.txt
```

The intended branching hierarchy is:

1. customer-pair branching;
2. launch-pad branching; and
3. route-variable fallback branching.

In particular, this guide removes:

- service-mode branching; and
- transformed-arc branching.

The phrase **fallback branching** below means the final route-variable
disjunction (z_{r^0}=0) versus (z_{r^0}=1), not transformed-arc branching.

This hierarchy remains complete. Launch-pad branching subsumes the service-mode
decision because every drone-served customer is associated with exactly one
launch pad within a route. If all launch-pad flows are integral, the aggregate
drone-service flow is also integral. Any fractionality not visible through
customer-pair or launch-pad flows is resolved by branching directly on a
fractional route variable.

## Summary of exact deletions

| Current material | Action |
|---|---|
| Customer-pair branching | Keep |
| Service-mode branching | Delete completely |
| Launch-pad branching | Keep |
| Transformed-arc branching | Delete completely |
| Route-variable fallback embedded at the end of the selection paragraph | Keep and promote to the explicit final branching layer |
| Transformed-arc-up automaton | Delete completely |
| Truck/drone service restrictions created only for service-mode branching | Delete |
| Launch-pad extension restrictions | Keep |
| Together/separate branch states | Keep |
| Route-down prohibition | Make explicit through a forbidden-route prefix state or an equivalent exact complete-path test |
| Transformed-arc-specific branch-interface compatibility | Delete |

## 1. Preamble macros

### Delete these macros

Delete the following definitions near current lines 267 and 269--279:

```latex
\newcommand{\DroneModeFlow}[1]{\alpha^{\textnormal{D}}_{#1}}
\newcommand{\TransArcFlow}[2]{\varphi_{#1,#2}}
\newcommand{\BinaryTransArcs}{\Arcs^{\top,\textnormal{bin}}}
\newcommand{\ArcCustSet}[1]{\mathcal{C}(#1)}
\newcommand{\BranchTruckServed}[1]{\mathcal{R}^{\textnormal{T}}_{#1}}
\newcommand{\BranchDroneServed}[1]{\mathcal{R}^{\textnormal{D}}_{#1}}
\newcommand{\BranchTransDown}[1]{\mathcal{R}^{\top,0}_{#1}}
\newcommand{\BranchTransUp}[1]{\mathcal{R}^{\top,1}_{#1}}
```

They are used only by the two branching layers being removed.

### Keep these macros

```latex
\newcommand{\RoutePadServe}[3]{\vartheta_{#1,#2,#3}}
\newcommand{\PadServiceFlow}[2]{\alpha^{\textnormal{P}}_{#1,#2}}
\newcommand{\BranchTogether}[1]{\mathcal{R}^{=}_{#1}}
\newcommand{\BranchSeparate}[1]{\mathcal{R}^{\neq}_{#1}}
\newcommand{\BranchPadDown}[1]{\mathcal{R}^{\textnormal{P},0}_{#1}}
\newcommand{\BranchPadUp}[1]{\mathcal{R}^{\textnormal{P},1}_{#1}}
```

Keep `\UsedTransArcs` if it remains part of the canonical route signature. Its
use there records route identity; it does not imply transformed-arc branching.

### Optional additional cleanup

After deleting service-mode branching, `\RouteTruckServe` and
`\RouteDroneServe` are no longer required as independently stored branching
attributes. They can be derived from coverage and pad assignment:

```latex
\RouteDroneServe{r}{n}=\sum_{h\in\HubNodes}\RoutePadServe{r}{h}{n},
\qquad
\RouteTruckServe{r}{n}=\RouteVisitNode{r}{n}
-\sum_{h\in\HubNodes}\RoutePadServe{r}{h}{n}.
```

If these symbols are removed from the route-signature definition as described
below, also delete their macro definitions near current lines 263--264.

## 2. Introduction contribution statement

### Current text near line 510

```latex
We develop an exact branch-price-and-cut algorithm combining forward
label-setting pricing under hard service envelopes, subset-row inequalities,
and customer-pair branching with a route-variable fallback.
```

### Replace with

```latex
We develop an exact branch-price-and-cut algorithm combining forward
label-setting pricing under hard service envelopes, subset-row inequalities,
customer-pair branching, launch-pad branching, and a route-variable fallback.
The launch-pad disjunction resolves fractional drone-origin assignments,
whereas the route-variable fallback guarantees completeness when no fractional
pair or pad flow remains.
```

This makes the contribution list match the retained hierarchy.

## 3. Opening of the branching subsection

### Delete current line 1543

Delete the sentence ending with:

```latex
... resorts to transformed-arc branching only after these coarser decisions
become integral.
```

### Replace with

```latex
Branching must preserve the structure of the pricing problem: each child node
corresponds to a restriction of the feasible route set, so the same forward
label-setting algorithm remains applicable with additional admissibility
tests. We first branch on fractional customer-pair flows, then on fractional
launch-pad flows. If neither class exposes the remaining fractionality, we use
a route-variable disjunction as a complete fallback.
```

## 4. Route attributes preceding the branching rules

### Delete from the current display near lines 1545--1566

Delete:

```latex
\RouteTruckServe{r}{n}
&:={\bf 1}\{n\textnormal{ is served directly by the truck in }r\},
&& n\in\CustNodes,\label{eq:branch-truck-serve-ind}\\
```

Delete:

```latex
\RouteDroneServe{r}{n}
&:=\sum_{h\in\HubNodes}\RoutePadServe{r}{h}{n},
&& n\in\CustNodes .\label{eq:branch-drone-serve-ind}
```

Delete the entire service-partition equation:

```latex
\begin{equation}\label{eq:branch-service-partition}
    \RouteVisitNode{r}{n}
    =
    \RouteTruckServe{r}{n}+\RouteDroneServe{r}{n}
    =
    \RouteTruckServe{r}{n}+
    \sum_{h\in\HubNodes}\RoutePadServe{r}{h}{n},
    \qquad r\in\RouteSet,~n\in\CustNodes .
\end{equation}
```

These definitions were needed to formulate service-mode branching. They are
not needed to define customer-pair or launch-pad flows.

### Keep only the pad-service route attribute

Replace the entire block at current lines 1545--1567 with:

```latex
For a route column (r), define the launch-pad service attribute
\begin{equation}\label{eq:branch-pad-serve-ind}
    \RoutePadServe{r}{h}{n}
    := {\bf 1}\{n\in\DroneBlock{h}{r}\},
    \qquad h\in\HubNodes,~n\in\CustNodes.
\end{equation}
This quantity is a route attribute rather than an additional master variable.
Under the shared-node transformation, it is determined by the active physical
pad associated with the duplicated-node block containing (\DupCust{n}).
Because a customer is served exactly once, its aggregate drone-service
indicator is (\sum_{h\in\HubNodes}\RoutePadServe{r}{h}{n}), and direct truck
service is the complementary covered mode.
```

The final sentence establishes why a separate service-mode branch is
unnecessary.

## 5. Customer-pair branching

Keep the complete paragraph currently located near lines 1569--1588,
including:

- `eq:same-route-flow`;
- `eq:rf-together-route-set`;
- `eq:rf-separate-route-set`; and
- the extension and complete-label enforcement explanation.

No deletion is needed in this paragraph.

## 6. Service-mode branching

### Delete the complete paragraph

Delete everything from:

```latex
\paragraph{Service-mode branching.}
```

through the end of the paragraph immediately before:

```latex
\paragraph{Drone launch-pad branching.}
```

In the attached file, this is approximately current lines 1590--1608. Delete:

- `eq:drone-mode-flow`;
- `eq:truck-mode-branch`;
- `eq:drone-mode-branch`;
- the truck-service child description; and
- the drone-service child description.

### Mathematical reason the deletion is safe

For any customer (n), exact coverage gives

```latex
\sum_{r}\RouteVisitNode{r}{n}z_r=1.
```

The aggregate drone-service flow equals the sum of its pad flows:

```latex
\sum_r\RouteDroneServe{r}{n}z_r
=\sum_{h:(h,n)\in\DroneArcs}\PadServiceFlow{h}{n}.
```

Therefore, if all launch-pad flows for (n) are integral, its service mode is
also integral. If truck-versus-drone service is fractional, at least one
launch-pad flow must be fractional and launch-pad branching is available.
Service-mode branching is consequently redundant in the retained hierarchy.

## 7. Launch-pad branching

Keep the complete paragraph near current lines 1610--1630, including:

- `eq:pad-service-flow`;
- `eq:pad-down-branch`;
- `eq:pad-up-branch`; and
- the active-pad label-state enforcement explanation.

### Recommended clarification after the paragraph

Add:

```latex
This disjunction also resolves fractional service-mode choices. Indeed, every
drone service of customer (n) contributes to exactly one pad-service flow,
whereas truck service contributes to none. Hence a fractional aggregate drone
flow implies that at least one pad-service flow is fractional.
```

This explicitly explains why the removed service-mode layer is not needed.

## 8. Transformed-arc branching

### Delete the complete paragraph

Delete everything from:

```latex
\paragraph{Fallback transformed-arc branching.}
```

through the sentence immediately before:

```latex
\paragraph{Branch selection and pricing enforcement.}
```

In the attached file, this is approximately current lines 1632--1668. Delete:

- `eq:arc-customer-set`;
- `eq:binary-trans-arcs`;
- `eq:transformed-arc-flow`;
- `eq:trans-down-branch`;
- `eq:trans-up-branch`; and
- all explanation of transformed-arc down/up enforcement.

The route-variable fallback already gives a complete finite disjunction and
does not require transformed-arc branch states.

## 9. Branch selection and fallback

### Find the exact paragraph to replace

Search for this heading:

```latex
\paragraph{Branch selection and pricing enforcement.}
```

Starting with that heading, delete the entire remainder of the branching
subsection. The deleted block ends with the sentence:

```latex
This final branch is rarely invoked but makes the branch-and-bound tree
complete and column-generation compatible.
```

Stop deleting immediately before the next subsection heading:

```latex
\subsection{Formulation of the pricing problem}
```

This context-based replacement removes the old selection order, the obsolete
service-mode and transformed-arc fallback discussion, and the old duplicate-
merging list in one operation.

### Paste this complete replacement block

```latex
\paragraph{Branch selection and pricing enforcement.}
At a fractional node, we first evaluate customer-pair flows. If at least one is
fractional, we branch on the flow closest to (1/2). If every customer-pair
flow is integral, we evaluate the launch-pad flows
\(\PadServiceFlow{h}{n}\) and branch on a fractional flow closest to (1/2).
The down child forbids service of customer (n) from pad (h); the up child
requires every route covering (n) to serve it by a drone launched from
(h). These restrictions are enforced through the represented-customer set
and the active-pad label state.

If all customer-pair and launch-pad flows are integral but the RMP solution
remains fractional, columns are first merged only when their current master
coefficients, active-SR coefficients, launch-pad attributes, canonical route
identities, and route costs are identical. Columns that differ in any of these
quantities remain distinct. We then select a remaining fractional route
variable (z_{r^0}), preferably one closest to (1/2), and create the
disjunction
\[
    z_{r^0}=0
    \qquad\vee\qquad
    z_{r^0}=1.
\]
In the down child, the canonical route (r^0) is removed from the admissible
route set. In the up child, (r^0) is fixed and the residual-node data become
\[
    \FixedRouteSet{m^+}=\FixedRouteSet{m}\cup\{r^0\},\qquad
    \CoveredCustSet{m^+}=\CoveredCustSet{m}\cup\RouteServedSet{r^0},
\]
\[
    \ResidualCustNodes{m^+}=\ResidualCustNodes{m}\setminus\RouteServedSet{r^0},\qquad
    \NodeFleetLimit{m^+}=\NodeFleetLimit{m}-1,
\]
and
\[
    \NodeFixedCost{m^+}
    =\NodeFixedCost{m}+\RouteCost{r^0}.
\]
Pricing and all route-pool procedures at the up child are restricted to
nonempty residual routes (r) satisfying
\[
    \emptyset\neq\RouteServedSet{r}
    \subseteq\ResidualCustNodes{m^+}.
\]
Active SR inequalities involving customers in
\(\RouteServedSet{r^0}\) are not inherited by the up child; the child
separates SR inequalities over its residual candidate pool.

The route-variable children are disjoint and exhaustive. Since the elementary
route set is finite, repeated route-variable fallback branching fixes or
excludes at least one previously fractional route at every fallback node and
therefore guarantees a finite complete branch-and-bound tree. Customer-pair
and launch-pad branching strengthen this generic fallback by imposing
structure that applies simultaneously to many route columns.
```

After this replacement, the next source line should be the unchanged heading

```latex
\subsection{Formulation of the pricing problem}
```

## 10. Detailed feasible-extension paragraph

### Current paragraph near line 1876

The current text contains five branching mechanisms.

### Replace the entire paragraph with

```latex
Sixth, the inherited branching restrictions are enforced. A separate-customer
branch forbids an extension that would represent the second member of a pair
after the first has already been represented. A together branch is tracked by
the branch-state automaton and is accepted at completion only if the route
contains both customers or neither. For a launch-pad branch on ((h,n)), the
down child rejects an extension to (\DupCust{n}) when the active pad is (h).
The up child forbids the original customer node (n) and permits an extension
to (\DupCust{n}) only when (h) is the active pad. A route-variable-down
branch rejects the single forbidden canonical route at completion, whereas a
route-variable-up branch is represented through the residual customer, fleet,
and fixed-cost data of the child node.
```

Delete all references here to:

- truck-service branches;
- drone-service branches;
- forbidden transformed arcs; and
- transformed-arc-up obligations.

## 11. Branch automata

### Keep

Keep the together-branch automaton near lines 1885--1899 and the local
separate-branch explanation near line 1900.

### Replace current line 1902

Delete the current service-mode and transformed-arc wording. Replace it with:

```latex
Launch-pad branches are enforced through local extension restrictions. For a
branch on ((h,n)), the down child rejects
(i\rightarrow\DupCust{n}) whenever the active pad is (h). The up child
forbids the original node (n) and rejects
(i\rightarrow\DupCust{n}) whenever the active pad differs from (h).
```

### Delete the transformed-arc-up automaton

Delete the complete block near lines 1904--1918 beginning with:

```latex
For an up branch on a transformed arc ...
```

and including:

- `\Omega_b^{\uparrow}`;
- `eq:trans-up-automaton-state`; and
- its complete-label acceptance discussion.

### Add exact handling of a route-variable-down branch

A route-variable-down branch must prevent pricing from regenerating the
forbidden canonical path. Add the following replacement after the launch-pad
paragraph:

```latex
For a route-variable-down branch forbidding the canonical transformed path
\(P(r^0)=(v_0,\ldots,v_L)\), a finite prefix state records whether the current
label path still coincides with a prefix of (P(r^0)). The state is (k) when
the current path equals ((v_0,\ldots,v_k)), (D) after the first divergence,
and (V) when a complete label equals (P(r^0)). From state (k<L), extension
to (v_{k+1}) gives state (k+1), whereas every other feasible extension gives
state (D); state (D) remains (D). A complete label in state (L) is sent
to (V) and rejected. Thus the down child removes exactly (r^0) and no other
route. The route-variable-up child requires no such state because (r^0) is
fixed and its customers and one unit of fleet capacity are removed before
residual pricing.
```

This state is necessary if the paper claims that the fallback branch is
enforced inside pricing rather than merely filtered after generation.

## 12. Branch-language definition and dominance compatibility

### Revise the list near line 1920

Replace:

```latex
together-branch, separate-branch, service-mode, launch-pad, and transformed-arc
branch requirements
```

with:

```latex
together-branch, separate-branch, launch-pad, and forbidden-route requirements
```

### Revise the implementation statement near line 1938

Replace:

```latex
... identical automaton states for every conditional branch, namely every
together branch and every transformed-arc up branch ...
```

with:

```latex
... identical automaton states for every together branch and every active
forbidden-route prefix state, while separate and launch-pad restrictions are
enforced directly through represented-customer and active-pad extension tests.
```

### Revise the state-count paragraph near line 1939

Delete the statement that transformed-arc-up automata have four states. Replace
the paragraph with:

```latex
For each label, the stored conditional branch state consists of one
constant-size state for each inherited together branch and one finite prefix
state for each route excluded by a route-variable-down decision. Separate
branches are enforced through customer-set tests, and launch-pad branches are
enforced through the active-pad state. Only requirements inherited along the
current branch-and-bound path are stored.
```

The route-prefix state has at most (L+2) values for a forbidden route of
length (L). It is not constant-size with respect to route length, so the old
constant-size claim should not be retained without qualification.

## 13. Branch-interface compatibility in physical-location dominance

The current branch-interface condition near lines 2155--2167 exists primarily
because transformed-arc up/down branches can distinguish corresponding outgoing
arcs from different duplicated endpoints.

### Delete

Delete:

- the transformed-arc-specific explanation near lines 2155--2167;
- `eq:branch-interface-compatibility`;
- `\NextSuffixSet` if unused elsewhere; and
- `\BranchInterfaceOK` if unused elsewhere.

Delete the condition

```latex
\BranchInterfaceOK{\ForwardLabel{u}^{1}}{\ForwardLabel{v}^{2}}=1
```

from the physical-location dominance proposition near current line 2192.

### Replace with

```latex
Launch-pad restrictions depend only on the active physical pad and the customer
being represented. They therefore apply identically to block-comparable labels
that share the same active pad. Route-variable-down restrictions are included
in the branch-language condition through their forbidden-route prefix states.
Consequently, no separate transformed-arc interface condition is required; the
condition
\(
\BranchLang{\Lambda^2}\subseteq\BranchLang{\Lambda^1}
\)
remains necessary.
```

Do not delete branch-language inclusion itself. Customer-pair together states
and forbidden-route states can still make two otherwise resource-comparable
labels accept different suffixes.

## 14. Farkas dominance

### Current line near 2321

Delete the words:

```latex
including branch-interface compatibility
```

Retain the branch-language and resource conditions inherited from the standard
dominance rule. If physical-location dominance is retained, its Farkas version
must still require that every branch-feasible completion of the dominated label
is available to the dominating label.

## 15. Route signatures

### Current definition near lines 2631--2635

The core signature currently stores service-mode indicators and transformed
arcs as branch-observable attributes.

### Replace with

```latex
\begin{equation}\label{eq:core-route-signature-forward}
    \CoreRouteSignature{m}{r}:=
    \left(
    \CustMask{r},
    \TruckNodeMask{r},
    \{\RoutePadServe{r}{h}{n}:(h,n)\in\DroneArcs\},
    \UsedTransArcs{r},
    \RouteCost{r}
    \right).
\end{equation}
```

Coverage and pad assignments already determine service mode. Retaining the
canonical transformed-arc set or full canonical path distinguishes route
identities for the route-variable fallback; it is no longer described as a
transformed-arc branching attribute.

### Correct the broken cross-reference near line 2646

The paper refers to:

```latex
\eqref{eq:branch-observable-route-signature}
```

but no equation with this label exists in the attached file. Replace that
reference with:

```latex
\eqref{eq:core-route-signature-forward}
```

or add an explicit branch-observable signature equation. The former is shorter.

## 16. Proof of branch-language inclusion

### Current proof near line 2811

Delete the sentences describing transformed-arc-up states (N,R,S,V).

### Replace the proof with

```latex
\proof{Proof.}
For a together requirement, the local state records whether neither customer,
only one customer, or both customers have been represented. Equal states imply
the same remaining obligation. A separate branch is enforced directly by the
represented-customer set, and a launch-pad branch is enforced by the active-pad
extension test. For each route-variable-down decision, the prefix state records
whether the current transformed path still coincides with the forbidden route
and, if so, its current prefix length. Equal forbidden-route states imply that
the same suffixes are excluded. Therefore, equality of all conditional states,
together with the local represented-customer and active-pad restrictions,
implies equality and hence inclusion of the branch-only suffix languages.
\Halmos
\endproof
```

## 17. Proof of physical-location dominance

### Current proof near line 2850

Delete:

```latex
Branch-interface compatibility guarantees that the corresponding first
extension is allowed ...
```

### Replace with

```latex
Block comparability and the common active pad make the corresponding first
extension graph-feasible, while branch-language inclusion preserves all
together and forbidden-route obligations. Launch-pad restrictions apply
identically because both labels carry the same active pad. After the first
extension, both paths end at the same transformed node, so the remaining suffix
is common.
```

### Pricing exactness proof near line 2876

Delete:

```latex
branch-interface compatibility is part of the latter rule
```

Replace it with:

```latex
branch-language inclusion includes the together and forbidden-route states,
while launch-pad restrictions are enforced through the active-pad extension
tests
```

## 18. Proof of Farkas dominance

Near current line 2904, replace:

```latex
Conditions ... together with branch-interface compatibility ...
```

with:

```latex
The resource conditions and branch-language inclusion give the same completion-
set inclusion as in the standard dominance result.
```

## 19. Candidate-management and branch inheritance descriptions

Search the final paper for these phrases and delete or replace them everywhere:

```text
service-mode indicators
service-mode branch
truck-service branch
drone-service branch
transformed-arc branch
transformed-arc up
transformed-arc down
branchable transformed arcs
forbidden transformed arcs
```

The child route index should retain only attributes needed by:

- customer-pair together/separate decisions;
- launch-pad up/down decisions;
- fixed and forbidden route identities;
- residual customer coverage;
- active SR coefficients; and
- route cost.

## 20. What must remain unchanged

Do not delete the following material merely because transformed-arc branching
is removed:

- transformed-network arcs used to encode physical routes;
- transformed-path incidence used to identify a canonical route;
- canonical drone-block ordering;
- customer and truck-node elementarity;
- active-pad label state;
- customer-pair together/separate states;
- route cost and reduced-cost recursion;
- SR coefficients;
- Farkas pricing; or
- route decoding and master verification.

Transformed arcs remain part of the pricing graph. Only **branching on their
aggregate LP flows** is deleted.

## 21. Final retained branching subsection structure

After revision, the subsection should contain exactly these headings:

```latex
\subsection{Branching strategies}\label{sec:branching}

\paragraph{Customer-pair branching.}

\paragraph{Drone launch-pad branching.}

\paragraph{Branch selection and route-variable fallback.}
```

No separate service-mode or transformed-arc paragraph should remain.

## 22. Consistency check after deletion

After editing, search the LaTeX source for the following tokens. Every search
should return zero results unless the term appears only in historical narrative:

```text
DroneModeFlow
BranchTruckServed
BranchDroneServed
TransArcFlow
BinaryTransArcs
ArcCustSet
BranchTransDown
BranchTransUp
eq:drone-mode-flow
eq:truck-mode-branch
eq:drone-mode-branch
eq:transformed-arc-flow
eq:trans-down-branch
eq:trans-up-branch
eq:trans-up-automaton-state
```

Searches for the following should remain nonzero:

```text
SameRouteFlow
BranchTogether
BranchSeparate
PadServiceFlow
BranchPadDown
BranchPadUp
FixedRouteSet
ResidualCustNodes
NodeFleetLimit
NodeFixedCost
```

## Final recommendation

The cleanest paper-faithful hierarchy is customer-pair branching followed by
launch-pad branching and a direct route-variable fallback. Service-mode
branching is mathematically redundant once launch-pad flows are available, and
transformed-arc branching is unnecessary for completeness once the finite
route-variable fallback is retained. Removing both layers also eliminates the
transformed-arc automaton and most branch-interface complexity while preserving
the ability to handle customers with multiple feasible launch pads.
