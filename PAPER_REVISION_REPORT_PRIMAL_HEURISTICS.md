# Paper Revision Report: Non-Trivial Primal Components

## 1. Scope

Only the implementation-specific or exactness-relevant components should be presented in the paper. The following standard material should not receive full mathematical treatment:

- the greedy constructive procedure;
- the complete binary formulation of a standard route-pool set-partitioning model;
- the complete soft set-partitioning formulation with uncovered-customer variables;
- the elementary large-penalty argument; and
- implementation-level timing or solver-control details.

The paper should retain only four points:

1. canonical decomposition of a compact truck-drone solution into verified route columns;
2. branch-compatible residual route-pool construction;
3. exact hard-soft-repair sequencing; and
4. separation of primal discovery from lower-bound certification.

## 2. Canonical Compact-to-Route Decomposition

This is the main non-trivial primal component because the compact model and the route-space master use different representations. A compact incumbent identifies physical truck arcs and drone assignments, whereas the BPC requires canonical transformed source-sink paths with recomputed synchronization and service-time resources.

For every used compact-model truck (k), define

\[
    A_k^{\mathrm T}
    :=\{(i,j)\in\TruckArcs:x_{ijk}=1\}
\]

and the drone-service block activated at pad (h) by

\[
    B_{hk}
    :=
    \left\{
        n\in\CustNodes:
        \sum_{d\in\DronesOfTruck{k}}y_{hnd}=1
    \right\}.
\]

The selected truck arcs induce a source-sink physical path. Every nonempty block (B_{hk}) is inserted immediately after pad (h) according to the canonical customer order `\prec_h`. This eliminates permutation-equivalent encodings of the same simultaneous launch event.

The extracted route must not copy potentially slack compact timing variables. Its physical resources are recomputed as

\[
    a_n(r)
    =a_h^{\mathrm{truck}}(r)+\DroneTravelTime{h}{n},
    \qquad n\in B_{hk},
\]

and

\[
    \TruckWaitAtHub{h}{r}
    =\max_{n\in B_{hk}}\DroneTripTime{h}{n}.
\]

The resulting wait is propagated through every downstream truck arc to recover all subsequent service times and the return time. The canonical route cost is

\begin{equation}
\label{eq:canonical-compact-route-cost}
    \RouteCost{r}
    =\ObjCoeffDelay
      \sum_{n\in\RouteServedSet{r}}
      \left(a_n(r)-\arriveLB{n}\right)^2
    +\ObjCoeffReturn T_r^{\mathrm{return}}
    +\ObjCoeffCost
      \left(
        \TruckCost
        +\DroneCost\sum_{h\in\HubNodes}|\DroneBlock{h}{r}|
      \right).
\end{equation}

The decoder accepts the route only after checking transformed-arc validity, physical-node elementarity, unique customer representation, nonemptiness, truck payload, drone-block size, drone payload, endurance, and customer service envelopes.

Use the following proposition in the main paper and place its proof in the appendix.

```latex
\begin{proposition}[Canonical decomposition of a compact solution]
\label{prop:arc-solution-decomposition}
Let $(x,y,a)$ be a feasible compact solution. Delete every used truck path
that serves no customer, order every nonempty pad block canonically, and
recompute pad waits, downstream service times, return times, and route costs
from the resulting transformed paths. Every accepted path defines a nonempty
feasible route column. If the compact solution covers every customer exactly
once, the extracted columns form a feasible set-partitioning solution. On the
common normalized objective scale,
\[
    \RouteObjShift
    +\sum_{r\in\mathcal R(x,y)}\RouteCost{r}
    \le Z^{\mathrm{compact}}(x,y,a).
\]
\end{proposition}
```

The inequality is relevant because canonical recomputation can remove slack from the compact waiting or timing variables. It preserves the truck arcs and drone assignments and therefore cannot increase the associated nonnegative delay, return-time, or operating-cost terms.

## 3. Branch-Compatible Residual Route Pool

The route-pool MIP itself is standard. The non-trivial part is determining which globally stored routes remain admissible at a residual branch-and-bound node.

Let `\FixedRouteSet{m}` be the routes fixed to one at node (m), and define

\[
    \CoveredCustSet{m}
    :=\bigcup_{r\in\FixedRouteSet{m}}\RouteServedSet{r},
    \qquad
    \ResidualCustNodes{m}
    :=\CustNodes\setminus\CoveredCustSet{m}.
\]

Let `\NodeFleetLimit{m}` be the remaining number of trucks and `\NodeFixedCost{m}` the additive cost of fixed routes. The verified node-admissible pool is

\begin{equation}
\label{eq:node-admissible-pool-revised}
    \NodeRoutePool{m}
    :=
    \left\{
        r\in\RouteSubset{m}\cup\RoutePool:
        \emptyset\neq\RouteServedSet{r}
        \subseteq\ResidualCustNodes{m},\;
        r\in\BranchFeasibleRoutes{m}
    \right\}.
\end{equation}

This definition imposes three necessary conditions:

1. the route is nonempty;
2. it does not cover a customer already covered by a fixed route; and
3. it satisfies all customer-pair, launch-pad, and route-variable restrictions inherited by node (m).

The hard pool search minimizes the true residual objective

\[
    \NodeFixedCost{m}
    +\sum_{r\in\NodeRoutePool{m}}\RouteCost{r}z_r
\]

subject to exact coverage of `\ResidualCustNodes{m}` and the remaining fleet limit `\NodeFleetLimit{m}`. The paper does not need to display the complete binary program because it is a standard set-partitioning model. It is sufficient to state that any accepted incumbent must satisfy

\[
    \sum_{r\in\NodeRoutePool{m}}
    \RouteVisitNode{r}{n}z_r=1,
    \qquad \forall n\in\ResidualCustNodes{m},
\]

and

\[
    \sum_{r\in\NodeRoutePool{m}}z_r
    \le\NodeFleetLimit{m}.
\]

The residual formulation is important. Using all original customers and the original fleet in a child-node pool model would not describe the implementation.

## 4. Exact Hard-Soft-Repair Sequence

The soft set-partitioning model is standard. The paper-worthy part is the control rule that prevents a soft solution or heuristic repair score from being mistaken for a feasible incumbent or an exact pricing certificate.

Let the hard route-pool outcome be

\[
    \mathsf{status}(\mathrm{FRPH}_m)
    \in
    \{\mathsf{feasible},\mathsf{infeasible},\mathsf{unresolved}\}.
\]

The implemented sequence is

\[
\begin{array}{rcl}
\mathsf{feasible}
&\Longrightarrow&
\text{evaluate the hard solution using the true route objective},\\[0.3em]
\mathsf{infeasible}
&\Longrightarrow&
\text{identify uncovered customers and invoke repair pricing},\\[0.3em]
\mathsf{unresolved}
&\Longrightarrow&
\text{no soft repair and no incumbent update}.
\end{array}
\]

Thus, a hard model that reaches a time limit without a feasible solution and without proving infeasibility does not trigger repair.

For a soft-pool solution, define the uncovered set

\[
    \RepairCustSet{m}
    :=\{n\in\ResidualCustNodes{m}:s_n=1\}
\]

and the customers already covered by its selected routes as

\[
    \HeurCoveredCustSet{m}
    :=\bigcup_{r:z_r=1}\RouteServedSet{r}.
\]

Repair pricing excludes `\HeurCoveredCustSet{m}` and searches node-admissible routes using

\begin{equation}
\label{eq:repair-pricing-score-revised}
    \RepairReducedCost{r}
    :=\RouteCost{r}
      -\RepairReward
       \sum_{n\in\RepairCustSet{m}}
       \RouteVisitNode{r}{n}.
\end{equation}

Equation `\eqref{eq:repair-pricing-score-revised}` is a primal search score, not the standard reduced cost associated with an RMP dual solution. Consequently, it cannot establish pricing closure or improve a lower bound.

Every repair path must pass the same route decoder and node-admissibility checks as any other pooled route. After verified repair routes are added, the algorithm rebuilds `\NodeRoutePool{m}` and solves the hard pool model again. Only a feasible solution of this final hard model may update the incumbent:

\[
    \text{hard infeasibility}
    \Longrightarrow
    \text{soft residual identification}
    \Longrightarrow
    \text{verified repair pricing}
    \Longrightarrow
    \text{hard re-solve}
    \Longrightarrow
    \text{possible incumbent update}.
\]

## 5. Primal-Proof Separation

The paper should explicitly preserve the exactness boundary:

```latex
All constructive, compact, pool, and repair procedures are primal-only. They
may add decoded and verified route columns or improve the incumbent upper
bound, but they cannot certify standard-pricing closure, validate a Farkas
certificate, or strengthen a node lower bound. They cannot fathom a node by
themselves. A better incumbent may subsequently permit ordinary bound
fathoming only when compared with a lower bound certified independently by
exact cut-and-price.
```

Formally, let (L_m) be a lower bound obtained after exact pricing and cut separation, and let (U) be the incumbent value returned by a primal procedure. Bound fathoming uses

\[
    L_m\ge U-\varepsilon^{\mathrm{int}}.
\]

The primal procedure supplies only (U); it does not supply or modify (L_m).

## 6. Concise Replacement Subsection

The following is the recommended paper text. It retains only the non-trivial components.

```latex
\subsection{Verified Primal Column Management}
\label{sec:prim_heur}

All constructive, compact, pool, and repair procedures are primal-only. They
may add decoded and verified route columns or improve the incumbent upper
bound, but they cannot certify standard-pricing closure, validate a Farkas
certificate, or strengthen a node lower bound. They cannot fathom a node by
themselves. A better incumbent may subsequently permit ordinary bound
fathoming only when compared with a lower bound certified independently by
exact cut-and-price.

\paragraph{Canonical compact-solution decomposition.}
The constructive procedure supplies verified initial columns and, when it
covers every customer exactly once within the fleet limit, a feasible root
incumbent. We additionally solve the compact formulation in
Section~\ref{arc-form} under a fixed Gurobi optimization-time limit. For each
used compact-model truck $k$, define
\[
    A_k^{\mathrm T}:=\{(i,j)\in\TruckArcs:x_{ijk}=1\},
    \qquad
    B_{hk}:=
    \left\{n\in\CustNodes:
    \sum_{d\in\DronesOfTruck{k}}y_{hnd}=1\right\}.
\]
Empty truck paths are discarded, and every nonempty block $B_{hk}$ is placed
in the canonical transformed order after pad $h$. Rather than copying
potentially slack compact timing variables, the route decoder recomputes
\[
    a_n(r)=a_h^{\mathrm{truck}}(r)+\DroneTravelTime{h}{n},
    \qquad n\in B_{hk},
\]
and
\[
    \TruckWaitAtHub{h}{r}
    =\max_{n\in B_{hk}}\DroneTripTime{h}{n},
\]
and propagates these values through all downstream truck movements. The
resulting path is admitted only after elementarity, customer representation,
payload, endurance, fleet, and service-envelope feasibility are verified.

\begin{proposition}[Canonical decomposition of a compact solution]
\label{prop:arc-solution-decomposition}
After empty truck paths are deleted and all pad blocks and timing resources
are canonicalized, every accepted path extracted from a feasible compact
solution defines a nonempty feasible route column. If the compact solution
covers every customer exactly once, the extracted columns form a feasible
set-partitioning solution. On the common normalized objective scale,
\[
    \RouteObjShift
    +\sum_{r\in\mathcal R(x,y)}\RouteCost{r}
    \le Z^{\mathrm{compact}}(x,y,a).
\]
\end{proposition}
\noindent\emph{Proof.}
See Appendix~\ref{app:proof:prop-arc-solution-decomposition}.\par

\paragraph{Branch-compatible residual route pool.}
At node $m$, define
\[
    \NodeRoutePool{m}
    :=
    \left\{
        r\in\RouteSubset{m}\cup\RoutePool:
        \emptyset\neq\RouteServedSet{r}
        \subseteq\ResidualCustNodes{m},\;
        r\in\BranchFeasibleRoutes{m}
    \right\}.
\]
The hard pool search selects routes from $\NodeRoutePool{m}$ to cover every
customer in $\ResidualCustNodes{m}$ exactly once within
$\NodeFleetLimit{m}$ and evaluates each candidate using
\[
    \NodeFixedCost{m}
    +\sum_{r\in\NodeRoutePool{m}}\RouteCost{r}z_r.
\]
Thus every accepted pool incumbent satisfies the residual branch restrictions
and is evaluated by the true route objective.

\paragraph{Conditional repair pricing.}
Soft residual identification is invoked only when the hard pool model is
proved infeasible. A time-limited or otherwise unresolved hard solve does not
trigger repair. If $\RepairCustSet{m}$ is the uncovered set identified by the
soft model, repair pricing searches node-admissible routes using
\[
    \RepairReducedCost{r}
    :=\RouteCost{r}
      -\RepairReward
       \sum_{n\in\RepairCustSet{m}}
       \RouteVisitNode{r}{n}.
\]
This is a primal search score rather than a reduced cost used for exact
pricing certification. Every repair route is decoded and verified before
entering $\RoutePool$. The hard pool model is then solved again; the soft
solution itself is never accepted as an incumbent.
```

## 7. Material intentionally omitted

The shortened subsection intentionally omits:

- a detailed constructive algorithm;
- constructive feasibility displays;
- the complete hard set-partitioning formulation;
- the complete soft set-partitioning formulation;
- a proof that a hard exact-cover solution is feasible;
- the elementary dominating-penalty derivation;
- Gurobi parameter details; and
- timing-counter definitions.

These are standard or implementation-level details and do not constitute the methodological contribution.
