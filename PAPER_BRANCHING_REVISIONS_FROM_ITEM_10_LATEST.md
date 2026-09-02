# Latest-Paper Revision Guide from Item 10 Onward

## Scope

This guide applies to the latest LaTeX source supplied in
`pasted-text.txt`. The main branching subsection already contains the intended
hierarchy:

1. customer-pair branching;
2. drone launch-pad branching; and
3. route-variable fallback.

Therefore, do not repeat the earlier items that replaced the main branching
strategy. The remaining work starts with the detailed pricing description and
continues through the dominance propositions, candidate-management text, and
appendix proofs.

The latest source still contains descriptions of service-mode and
transformed-arc branching that no longer exist in the main branching
subsection. Some of those passages also call undefined macros such as
`\BinaryTransArcs` and `\ArcCustSet`. They must be removed for both mathematical
and LaTeX consistency.

## Preliminary Corrections in the Existing Main Branching Subsection

These are syntax corrections, not a request to rewrite the already-correct
branching hierarchy.

### Correct the launch-pad indicator

Find the display immediately after:

```latex
For a route column $r$, define the launch-pad service attribute
```

Replace the complete display with:

```latex
\begin{equation}\label{eq:branch-pad-serve-ind}
\RoutePadServe{r}{h}{n}
:= {\bf 1}\{n\in\DroneBlock{h}{r}\},
\qquad h\in\HubNodes,~n\in\CustNodes.
\end{equation}
```

The braces after the indicator must be `\{...\}`. The current expression
`{\bf 1}{...}` does not typeset the intended indicator correctly.

### Correct the route-up residual update

Find the sentence:

```latex
In the down child, the canonical route $r^0$ is removed from the admissible
route set. In the up child, $r^0$ is fixed, and the residual-node data become
```

Replace the complete display that follows it with:

```latex
\begin{equation}
\begin{aligned}
\FixedRouteSet{m^+}
&=
\FixedRouteSet{m}\cup\{r^0\},
&\qquad
\CoveredCustSet{m^+}
&=
\CoveredCustSet{m}\cup\RouteServedSet{r^0},\\
\ResidualCustNodes{m^+}
&=
\ResidualCustNodes{m}\setminus\RouteServedSet{r^0},
&\qquad
\NodeFleetLimit{m^+}
&=
\NodeFleetLimit{m}-1.
\end{aligned}
\end{equation}
```

This fixes the missing set braces around `r^0` and replaces the current lone
backslash after `\RouteServedSet{r^0}` with the required aligned-equation line
break `\\`.

## 10. Replace the Detailed Feasible-Extension Paragraph

### Where to revise

In the pricing section, locate the service-envelope test ending with:

```latex
This check is an exact feasibility test for the service-envelope-restricted
route set and can discard many late-service prefixes before they enter the
dominance pool.
```

The next paragraph begins:

```latex
Sixth, the branching constraints inherited from Section~\ref{sec:branching}
are enforced.
```

Replace that entire paragraph, ending immediately before:

```latex
For branch-compatible dominance, a label records the state of a finite
automaton ...
```

### Replacement text

```latex
Sixth, the branching restrictions inherited from
Section~\ref{sec:branching} are enforced. A separate-customer branch forbids
an extension that would represent the second member of a separated pair after
the first has already been represented. A together branch is tracked by the
branch-state automaton and is accepted at completion only if the route
contains both customers or neither. For a launch-pad branch on $(h,n)$, the
down child rejects an extension to $\DupCust{n}$ when the active pad is $h$.
The up child forbids the original customer node $n$ and permits an extension
to $\DupCust{n}$ only when $h$ is the active pad. A route-variable-down branch
rejects the single forbidden canonical route, whereas a route-variable-up
branch is represented through the fixed-route, residual-customer, residual-
fleet, and fixed-cost data of the child node.
```

This replacement removes the obsolete references to truck-service branches,
drone-service branches, forbidden transformed arcs, and transformed-arc-up
obligations.

## 11. Replace the Obsolete Branch-Automaton Material

### Material to retain

Retain all text from:

```latex
For branch-compatible dominance, a label records the state of a finite
automaton ...
```

through the together-branch state display
`\eqref{eq:together-automaton-state}` and the following paragraph beginning:

```latex
For a separate branch $b=(p,q,\neq)$ ...
```

The together and separate mechanisms are still part of the selected branching
strategy.

### Material to replace

Immediately after the separate-branch paragraph, find the paragraph beginning:

```latex
Service-mode and launch-pad branches are enforced through local extension
restrictions.
```

Delete that paragraph and the complete transformed-arc-up automaton block that
follows it. The deletion ends after the sentence:

```latex
... it never discards a partial label because of an unproven nonreachability
claim.
```

Insert the following text in their place:

```latex
Launch-pad branches are enforced through local extension restrictions. For a
branch on $(h,n)$, the down child rejects
$i\rightarrow\DupCust{n}$ whenever the active pad is $h$. The up child
forbids the original node $n$ and rejects
$i\rightarrow\DupCust{n}$ whenever the active pad differs from $h$.

For a route-variable-down branch forbidding the canonical transformed path
$P(r^0)=(v_0,\ldots,v_L)$, a finite prefix state records whether the current
label path still coincides with a prefix of $P(r^0)$. The state is $k$ when
the current path equals $(v_0,\ldots,v_k)$, is $D$ after the first divergence,
and is $V$ when a complete label equals $P(r^0)$. From a state $k<L$, an
extension to $v_{k+1}$ gives state $k+1$, whereas every other feasible
extension gives state $D$; state $D$ remains unchanged under all subsequent
extensions. A complete label in state $L$ is sent to $V$ and rejected. Thus,
the down child removes exactly route $r^0$ and no other canonical route. The
route-variable-up child requires no corresponding prefix state because
$r^0$ is fixed and its customers and one unit of fleet capacity are removed
before residual pricing.
```

The forbidden-route prefix state is important. Merely deleting `r^0` from the
current RMP is insufficient because pricing could regenerate it. Completion-
only filtering also needs to be represented in the branch language used by
dominance; otherwise, two labels with different ability to reproduce the
forbidden route could be compared incorrectly.

## 12. Replace the Branch-Language and State-Overhead Discussion

### Where to revise

Find the paragraph beginning:

```latex
Let \(\BranchLang{\Lambda}\) denote the set of suffixes accepted by the branch
automata inherited at the current node.
```

Replace everything from that sentence through the state-overhead paragraph
ending with:

```latex
... storing only the active requirements inherited along the current
branch-and-bound path.
```

Do not delete the lemma `Branch-only language inclusion` that follows this
material.

### Complete replacement block

```latex
Let $\BranchLang{\Lambda}$ denote the set of suffixes satisfying all inherited
customer-pair, launch-pad, and forbidden-route restrictions from label
$\Lambda$. This is a \emph{branch-only} suffix language. It does not by itself
encode customer elementarity, physical-node elementarity, residual payload
feasibility, drone-block capacity, canonical block ordering, service-time
envelopes, or the nonempty-column rule. The full feasible completion set is
therefore
\begin{equation}\label{eq:full-completion-set-def}
    \CompletionSet{\Lambda}:=
    \left\{
    \Completion~\middle|~
    \begin{array}{l}
    \Completion\textnormal{ is accepted by }\BranchLang{\Lambda},
    \textnormal{ and}\\
    \Lambda\oplus\Completion\textnormal{ is a resource-feasible, elementary,}\\
    \textnormal{canonically ordered complete path satisfying}\\
    \emptyset\neq\mathcal C(\Lambda\oplus\Completion)
    \subseteq\ResidualCustNodes{m}
    \end{array}
    \right\}.
\end{equation}
In particular, a suffix consisting only of a direct extension to
$\DepotSink$ is not in $\CompletionSet{\Lambda}$ if the resulting complete
path serves no residual customer. The dominance tests require
\begin{equation}\label{eq:branch-language-condition}
    \BranchLang{\Lambda^2}\subseteq\BranchLang{\Lambda^1}.
\end{equation}
This condition preserves only branch feasibility. Full completion-set
inclusion also requires the represented-customer, visited-physical-node,
resource, block-comparability, canonical-order, service-envelope, and
nonempty-column conditions of the applicable dominance proposition.

In implementation, condition~\eqref{eq:branch-language-condition} is verified
conservatively by requiring equal states for every inherited together branch
and every active forbidden-route prefix state. Separate branches are enforced
through represented-customer tests, and launch-pad branches are enforced
through the active-pad extension tests. Equality is sufficient but not
necessary, so this implementation may miss valid dominance relations but
cannot accept one that violates an inherited branching restriction.

For each label, the stored conditional branch state consists of one
constant-size state for each inherited together branch and one finite prefix
state for each route excluded by a route-variable-down decision. For a
forbidden route with $L$ transformed arcs, its prefix state has at most
$L+2$ values, including divergence and violation. Separate and launch-pad
requirements do not require additional automata beyond the represented-
customer and active-pad states already carried by the label. Only restrictions
inherited along the current branch-and-bound path are stored.
```

The qualification about `$L+2$` states is necessary. The forbidden-route
automaton is finite, but it is not constant-size with respect to the length of
the excluded route.

## 13. Remove Transformed-Arc Branch-Interface Compatibility

### Why this material is obsolete

The current branch-interface equation exists to handle transformed-arc up and
down branches that distinguish outgoing encoding arcs from two labels located
at the same physical pad. Those branching classes have been removed. The
remaining launch-pad restrictions depend on the active physical pad, and the
route-variable-down restriction is handled by the forbidden-route prefix
state. Therefore, the separate transformed-arc interface is no longer part of
the selected algorithm.

### Where to revise

Immediately before Proposition `Physical-location dominance with return-time
credit`, find the paragraph beginning:

```latex
The physical-location dominance rule uses the same branch-language condition
as \eqref{eq:branch-language-condition}.
```

Delete that paragraph and the complete equation labeled
`eq:branch-interface-compatibility`, including the explanatory paragraph that
ends with:

```latex
... This may miss a valid dominance relation but cannot create an invalid one.
```

Insert this paragraph in their place:

```latex
The physical-location dominance rule uses the branch-language condition
\eqref{eq:branch-language-condition}. Launch-pad restrictions apply
identically to block-comparable labels carrying the same active physical pad,
whereas route-variable-down restrictions are represented by their forbidden-
route prefix states. Consequently, no separate transformed-arc branch-
interface condition is required. The dominating label must nevertheless
accept every branch-feasible suffix accepted by the dominated label.
```

### Revise the proposition

Within Proposition `Physical-location dominance with return-time credit`, find:

```latex
&\BranchLang{\ForwardLabel{v}^{2}}\subseteq\BranchLang{\ForwardLabel{u}^{1}},\qquad
\BranchInterfaceOK{\ForwardLabel{u}^{1}}{\ForwardLabel{v}^{2}}=1,
\label{eq:pld-branch}\\
```

Replace it with:

```latex
&\BranchLang{\ForwardLabel{v}^{2}}
\subseteq
\BranchLang{\ForwardLabel{u}^{1}},
\label{eq:pld-branch}\\
```

### Delete obsolete macros

After completing these edits, delete the following macro definitions from the
preamble:

```latex
\newcommand{\NextSuffixSet}[1]{\mathcal{X}(#1)}
\newcommand{\BranchInterfaceOK}[2]{\mathsf{I}_{B}(#1,#2)}
```

Do not remove `\BranchLang`, `\BranchState`, or `\UsedTransArcs`.

## 14. Revise Farkas-Pricing Dominance

### Where to revise

Find Proposition `Farkas-pricing dominance`. Its opening sentence currently
contains:

```latex
... conditions \eqref{eq:pld-cust}--\eqref{eq:pld-time}, including
branch-interface compatibility, hold ...
```

Replace the opening with:

```latex
\begin{proposition}[Farkas-pricing dominance]
\label{prop:farkas-pricing-dominance}
Consider two block-comparable labels $\ForwardLabel{u}^{1}$ and
$\ForwardLabel{v}^{2}$ in Farkas pricing. Label
$\ForwardLabel{u}^{1}$ dominates $\ForwardLabel{v}^{2}$ if conditions
\eqref{eq:pld-cust}--\eqref{eq:pld-time} hold; their active SR count vectors
are identical,
```

Retain the existing SR-state equality and Farkas reduced-cost inequality that
follow. Farkas pricing must remain exhaustive and must not use the standard
dual-reward completion bound or standard early-termination rule.

## 15. Simplify the Route Signature and Correct Its Cross-Reference

### Replace the core signature

In `Forward Candidate Management and Dominance Throughput`, find the paragraph
`Two-level route signatures.` Replace the equation labeled
`eq:core-route-signature-forward` with:

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

The customer-coverage mask together with the launch-pad attributes identifies
whether a covered customer is truck-served or drone-served. Separate
`\RouteTruckServe` and `\RouteDroneServe` entries are therefore redundant.
`\UsedTransArcs{r}` remains useful as part of the canonical route identity for
route-variable fallback; it is no longer described as a transformed-arc
branching attribute.

After replacing the signature, delete these unused macros from the preamble:

```latex
\newcommand{\RouteTruckServe}[2]{\delta^{\textnormal{T}}_{#1,#2}}
\newcommand{\RouteDroneServe}[2]{\delta^{\textnormal{D}}_{#1,#2}}
```

### Correct the undefined reference

In the paragraph `Branch-indexed inheritance and row-local active-SR
coefficients`, replace:

```latex
The signature in \eqref{eq:branch-observable-route-signature} ...
```

with:

```latex
The core signature in \eqref{eq:core-route-signature-forward} is used as a
necessary-condition index for inherited or globally stored routes.
```

The label `eq:branch-observable-route-signature` is not defined in the latest
source.

## 16. Replace the Proof of Branch-Language Inclusion

### Where to revise

In the appendix, locate:

```latex
\subsection{Proof of Lemma~\ref{lem:branch-language-inclusion}}
```

Replace its complete proof with:

```latex
\proof{Proof.}
For a together requirement, the local state records whether neither customer,
only one customer, or both customers have been represented. Equal states imply
the same remaining obligation. A separate branch is enforced directly by the
represented-customer set, and a launch-pad branch is enforced by the active-
pad extension test. For each route-variable-down decision, the prefix state
records whether the current transformed path still coincides with the
forbidden route and, if so, its current prefix length. Equal forbidden-route
states imply that the same route-completion suffixes are excluded. Therefore,
equality of all conditional states, together with the local represented-
customer and active-pad restrictions, implies equality and hence inclusion of
the branch-only suffix languages. \Halmos
\endproof
```

This proof replaces the obsolete discussion of the four-state transformed-arc
up automaton.

## 17. Revise the Physical-Dominance and Pricing-Exactness Proofs

### Physical-location dominance proof

In `Proof of Proposition~\ref{prop:physical-location-dominance}`, replace the
first paragraph, beginning:

```latex
Let \(\Completion\in\CompletionSet{\ForwardLabel{v}^{2}}\), and let \(x\) be
its first transformed node.
```

and ending:

```latex
... so the remainder of the suffix is common.
```

with:

```latex
Let $\Completion\in\CompletionSet{\ForwardLabel{v}^{2}}$, and let $x$ be its
first transformed node. If $u=v$, the same first arc is used. Otherwise, both
labels have the same physical location $h$. If $x=\DupCust{j}$, block
comparability implies that its canonical rank is available after label~1
whenever it is available after label~2. If $x$ is regular, both labels make
the same physical truck departure from $h$, although the corresponding
transformed arcs have different tails. Block comparability and the common
active pad make this first extension graph-feasible, while branch-language
inclusion preserves all together and forbidden-route obligations. Launch-pad
restrictions apply identically because both labels carry the same active pad.
After the first extension, both paths end at the same transformed node, so the
remaining suffix is common.
```

Keep the remaining time-resource, return-credit, SR-adjustment, and reduced-
cost argument unchanged.

### Pricing-exactness proof

In `Proof of Theorem~\ref{thm:pricing-exactness}`, find the sentence:

```latex
Same-node and physical-location dominance remove a label only after proving
full completion-set inclusion and a no-larger reduced cost for every
corresponding completion; branch-interface compatibility is part of the latter
rule.
```

Replace it with:

```latex
Same-node and physical-location dominance remove a label only after proving
full completion-set inclusion and a no-larger reduced cost for every
corresponding completion. Branch-language inclusion preserves together and
forbidden-route states, while separate and launch-pad restrictions are
enforced through represented-customer and active-pad extension tests.
```

## 18. Revise the Proof of Farkas Dominance

### Where to revise

In `Proof of Proposition~\ref{prop:farkas-pricing-dominance}`, replace its
opening sentence:

```latex
Conditions \eqref{eq:pld-cust}--\eqref{eq:pld-time}, together with
branch-interface compatibility, give the same full completion-set inclusion
as in Proposition~\ref{prop:physical-location-dominance}.
```

with:

```latex
The resource conditions and branch-language inclusion in
\eqref{eq:pld-cust}--\eqref{eq:pld-time} give the same full completion-set
inclusion as in Proposition~\ref{prop:physical-location-dominance}.
```

The rest of the Farkas proof remains valid: equal active-SR count vectors make
future SR-ray increments identical, and physical time affects feasibility but
not the Farkas objective.

## 19. Clean the Candidate-Management and Inheritance Terminology

After making the specific replacements above, search the paper for branch-
specific uses of the following phrases:

```text
truck-service branch
drone-service branch
service-mode branch
transformed-arc branch
transformed-arc up
transformed-arc down
forbidden transformed arcs
branchable transformed arcs
branch-interface compatibility
```

Delete or revise every occurrence that describes an inherited branching rule.
Do not delete ordinary modeling statements that distinguish truck service from
drone service.

The retained route-index information should support only:

- customer-pair together/separate decisions;
- launch-pad up/down decisions;
- fixed and forbidden canonical route identities;
- residual customer coverage and residual fleet accounting;
- active SR coefficients; and
- route cost.

For inherited columns, the core signature is only a necessary-condition
index. Every inherited route must still pass the complete node-admissibility
test before insertion into the child RMP.

## 20. Material That Must Remain

Removing transformed-arc branching does not remove transformed arcs from the
pricing formulation. Retain all material needed for the transformed-network
encoding, including:

- transformed-network arc construction and its size bound;
- `\UsedTransArcs{r}` or the equivalent canonical path identity;
- canonical ordering of drone-service blocks;
- customer and physical-node elementarity;
- the active-pad label state;
- customer-pair together/separate states;
- physical-location dominance with return-time credit;
- the exact three-customer SR adjustment with dominating count 1, dominated
  count 2, and negative SR dual;
- route-cost and reduced-cost recursions;
- standard and Farkas pricing;
- route decoding and master-side verification; and
- the route-variable fallback.

The term `transformed arc` should therefore remain in the network-construction,
path-encoding, canonical-route, and complexity discussions. Only branching on
aggregate transformed-arc flows is removed.

## 21. Resulting Branching and Pricing Structure

The main branching subsection should retain these paragraphs:

```latex
\subsection{Branching strategies}\label{sec:branching}

\paragraph{Customer-pair branching.}

\paragraph{Drone launch-pad branching.}

\paragraph{Branch selection and pricing enforcement.}
```

The detailed pricing material should then describe only:

1. local separate-pair rejection;
2. together-pair completion states;
3. active-pad launch restrictions;
4. forbidden-route prefix states for route-variable-down children; and
5. residualization for route-variable-up children.

No service-mode branch or transformed-arc branch should remain.

## 22. Final Consistency and Compilation Check

### Tokens that should disappear

After revision, searches for the following definitions, labels, or branch
descriptions should return no results:

```text
NextSuffixSet
BranchInterfaceOK
RouteTruckServe
RouteDroneServe
BinaryTransArcs
ArcCustSet
BranchTransDown
BranchTransUp
DroneModeFlow
BranchTruckServed
BranchDroneServed
eq:trans-up-automaton-state
eq:branch-interface-compatibility
eq:branch-observable-route-signature
truck-service branch
drone-service branch
service-mode branch
transformed-arc-up
transformed-arc up branch
transformed-arc down branch
```

The latest source already invokes `\BinaryTransArcs` and `\ArcCustSet` without
defining them. Removing the obsolete transformed-arc automaton resolves those
undefined-control-sequence risks rather than reintroducing retired macros.

### Tokens that should remain

Searches for these terms should remain nonzero:

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
BranchLang
BranchState
UsedTransArcs
```

### Cross-reference checks

Verify that:

- `eq:full-completion-set-def` is defined exactly once;
- `eq:branch-language-condition` is defined exactly once;
- `eq:pld-branch` contains only branch-language inclusion;
- no reference to `eq:branch-observable-route-signature` remains;
- no reference to the deleted transformed-arc automaton remains; and
- every macro used in the revised branch-language proof is defined.

### Recommended edit order

Apply the revisions in this order to avoid temporary inconsistencies:

1. correct the launch-pad indicator and route-up residual display;
2. replace the feasible-extension paragraph;
3. replace the obsolete automaton material;
4. replace the branch-language discussion;
5. remove branch-interface compatibility from the proposition;
6. simplify the route signature and inheritance reference;
7. replace the three appendix proof passages;
8. delete now-unused macros; and
9. run a full LaTeX compilation and unresolved-reference check.

This produces a paper that is internally consistent with customer-pair plus
launch-pad branching and exact route-variable fallback, while preserving the
transformed pricing network and the forward-label dominance theory.
