# Added Component for Manuscript Revision: Full Node-Admissible Route-Pool Incumbent Search

## Purpose

This note records the only substantive algorithmic component added beyond the current V4 manuscript text in `TH_VRPD_IJOC_postroot_v4_revised.tex`. The component should be incorporated into the paper so that the manuscript matches the restored production solver.

The addition is a primal-only incumbent-search intensification over the full node-admissible verified route pool. It strengthens the route-pool heuristic described in the manuscript, but it does not change the exact branch-price-and-cut lower-bound proof, pricing closure certificate, Farkas pricing certificate, subset-row cut validity, or branching logic.

## Component Summary

The current manuscript defines the node-admissible route pool
\[
    \NodeRoutePool{m}:=
    \left(\RouteSubset{m}\cup\RoutePool\cup\CandidateSidePool{m}\right)
    \cap \left\{r\in\BranchFeasibleRoutes{m}:
    \emptyset\neq\RouteServedSet{r}\subseteq\ResidualCustNodes{m}\right\},
\]
and then constructs a smaller support set \(\SupportRouteSet{m}(\varepsilon)\) for the route-pool diving heuristic. The manuscript's hard route-pool model \((\mathrm{RPH}_m)\) is solved over \(\SupportRouteSet{m}(\varepsilon)\).

The restored code adds a second hard set-partitioning solve over the entire \(\NodeRoutePool{m}\), whenever \(\NodeRoutePool{m}\) is strictly larger than \(\SupportRouteSet{m}(\varepsilon)\). This solve is applied after the support-pool hard solve and before the soft repair model. It uses only route columns that have already been decoded and verified as node-admissible. Any feasible solution is evaluated by the true route objective before incumbent update.

This component recovered the strong medium-instance incumbent in the restored V4 audit:

- restored V4 incumbent: `0.0811406255`;
- recorded V4 incumbent: `0.0846704491`;
- V5/V6 incumbent: about `0.2257310146`;
- final restored solution: three truck routes, zero drone sorties.

Thus, the improved incumbent came from selecting a better feasible combination of verified route columns, not from compact extraction or drone-diversification routes.

## Suggested Manuscript Insertion

Insert the following material in Section `Primal Heuristics and Column Pooling`, after the definition and explanation of \((\mathrm{RPH}_m)\) over \(\SupportRouteSet{m}(\varepsilon)\), and before the soft model \((\mathrm{SRPH}_m)\).

### Full Node-Admissible Route-Pool Intensification

Although the support set \(\SupportRouteSet{m}(\varepsilon)\) keeps the route-pool model small, it may exclude useful verified columns whose LP value is currently zero or whose primal score is outside the per-customer shortlist. To improve incumbent discovery without affecting lower-bound certification, we also solve a hard set-partitioning model over the full node-admissible pool \(\NodeRoutePool{m}\) whenever this pool is larger than the support set. Let
\[
    \widehat z_{r,m}^{\mathrm{LP}} :=
    \begin{cases}
        z_{r,m}^{\mathrm{LP}}, & r\in\RouteSubset{m},\\
        0, & r\in\NodeRoutePool{m}\setminus\RouteSubset{m}.
    \end{cases}
\]
The full-pool incumbent-search model is
\[
\begin{aligned}
(\mathrm{FRPH}_m)\qquad
\widehat Z_m:=
\min\quad
& \sum_{r\in\NodeRoutePool{m}}\RouteCost{r}z_r
  -\DiveReward\sum_{r\in\NodeRoutePool{m}}\widehat z_{r,m}^{\mathrm{LP}}z_r \\
\mathrm{s.t.}\quad
& \sum_{r\in\NodeRoutePool{m}}\RouteVisitNode{r}{n}z_r=1,
&& \forall n\in\ResidualCustNodes{m},\\
& \sum_{r\in\NodeRoutePool{m}}z_r\le \NodeFleetLimit{m},\\
& z_r\in\{0,1\},
&& \forall r\in\NodeRoutePool{m}.
\end{aligned}
\]
The LP-support term remains only a search bias. If \((\mathrm{FRPH}_m)\) returns a feasible solution, the incumbent test uses the true residual-node objective
\[
    \NodeFixedCost{m}+\sum_{r\in\NodeRoutePool{m}}\RouteCost{r}z_r,
\]
not the biased heuristic objective. If this value improves the incumbent, the incumbent is updated. Otherwise, the algorithm proceeds to the soft route-pool model and residual repair pricing described below.

This full-pool solve is a primal intensification over already verified route columns. It may improve the incumbent and enrich the column pool, but it is never used to certify pricing closure, fathom a node, strengthen a lower bound, or replace exact pricing.

## Suggested Algorithm Update

Revise Algorithm `Route-Pool Diving and Repair Heuristic at Node \(m\)` as follows.

```text
Build the node-admissible pool P_m by (node-admissible pool definition).
Construct the support set S_m(epsilon).
Solve RPH_m over S_m(epsilon).
If a feasible hard solution is found, evaluate it by the true route cost and update the incumbent if improved.
If P_m is larger than S_m(epsilon), solve FRPH_m over P_m.
If a feasible full-pool hard solution is found, evaluate it by the true route cost and update the incumbent if improved.
If no hard route-pool solution improves the incumbent, solve SRPH_m and run residual repair pricing as described.
Return the updated incumbent and pool.
```

The algorithm should explicitly state that both hard solves use only verified routes in \(\NodeRoutePool{m}\), and that all incumbent updates are based on the true objective rather than the LP-support-biased heuristic objective.

## Suggested Proposition

The following result can be added after the primal heuristic description, or folded into the existing validity paragraph.

**Proposition (Primal validity of full-pool incumbent search).**  
At any branch-and-bound node \(m\), every feasible solution of \((\mathrm{FRPH}_m)\), together with the fixed routes in \(\FixedRouteSet{m}\), defines a feasible integer solution for the residual node problem. Moreover, using \((\mathrm{FRPH}_m)\) only for incumbent updates does not affect the validity of RMP lower bounds, pricing closure, Farkas infeasibility certificates, or branch-and-bound fathoming.

**Proof sketch.**  
By construction, \(\NodeRoutePool{m}\subseteq\BranchFeasibleRoutes{m}\) and every route in \(\NodeRoutePool{m}\) is nonempty, branch-feasible, service-window-feasible, and satisfies residual customer compatibility. The covering constraints of \((\mathrm{FRPH}_m)\) assign every residual customer exactly once, and the fleet constraint respects \(\NodeFleetLimit{m}\). Hence any feasible solution of \((\mathrm{FRPH}_m)\), combined with \(\FixedRouteSet{m}\), is a feasible integer solution for node \(m\). Since the model is used only to update the incumbent after evaluation by the true route objective, it cannot alter dual bounds, pricing certificates, cut validity, or fathoming decisions. Therefore it preserves the exactness of the branch-price-and-cut algorithm.

## How to Position This in the Paper

This component should be presented as a primal incumbent-search intensification, not as a new exact-pricing device. The exact contribution remains the transformed route encoding, forward pricing, dominance design, branch-feasible route generation, and post-root certification mechanics. The full-pool route-pool solve is best described as an implementation-level primal heuristic that exploits the verified column pool more aggressively.

Recommended wording for the computational section:

> The restored V4 implementation also includes a full node-admissible route-pool incumbent search. After solving the smaller LP-support route-pool model, we solve the same hard set-partitioning model over all verified node-admissible routes available at the current node. This procedure is primal-only: it may improve the incumbent, but it is not used for lower-bound certification or node fathoming. In the medium promised-window audit, this full-pool incumbent search recovered a three-route truck-only incumbent with objective `0.0811406255`, improving on the recorded V4 incumbent `0.0846704491` and substantially improving over the V5/V6 incumbent around `0.2257310146`.

## Implementation Location

The corresponding implementation is in:

- `thvrpd/heuristics.py`, inside `run_route_pool_heuristic`;
- the support-pool solve is performed first;
- the added full-pool hard solve is triggered when `pool_paths != support_paths`;
- incumbent updates still occur only through `thvrpd/bpc.py` after the heuristic result is returned and evaluated as a true feasible route set.

## Important Exactness Boundary

Do not describe this component as a closure mechanism, dual stabilization device, or lower-bound improvement. It is a general primal search over verified columns. The manuscript should preserve the following boundary:

- productive pricing can generate columns but cannot certify closure;
- certification requires exhaustive source-neighbor pricing under the current dual solution;
- route-pool hard solves can improve incumbents but cannot change lower bounds;
- \((\mathrm{FRPH}_m)\) feasible solutions are accepted only as primal incumbents after true-cost evaluation.
