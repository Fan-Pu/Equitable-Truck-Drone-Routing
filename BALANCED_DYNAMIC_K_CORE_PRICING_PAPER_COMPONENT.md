# Balanced and Dynamically Refined K-Core Pricing for Exact Forward BPC

## Purpose of This Note

This note specifies a new computational component for the truck-as-hub VRPD branch-price-and-cut framework: a certification-preserving parallel pricing scheduler that improves load balance across \(K\) workers. The component is intended to be added to the paper as an exact candidate-management and scheduling device. It does not change the route set, the reduced-cost definition, the dominance theory, the RMP, the subset-row cut logic, or the lower-bound proof. It changes only how the transformed forward pricing search is partitioned and assigned to parallel workers.

The motivation is empirical and structural. In the current fixed \(K\)-core source-neighbor partition, some workers finish early while one or two workers continue exploring much harder pricing subspaces. The idle workers remain alive but do not contribute to the unresolved subspaces. This limits CPU utilization and increases wall-clock proof time. The proposed component addresses this by:

1. constructing a more balanced initial partition of the pricing route space over \(K\) workers; and
2. allowing idle workers to assist active workers through exact dynamic subspace refinement, but only when the active worker is far from closure.

The second condition is important. If an active worker is close to certification, subdividing its remaining work can introduce synchronization and transfer overhead that delays closure. Dynamic refinement should therefore be triggered only for subspaces with clear evidence of substantial remaining search.

## Suggested Paper Placement

This component should be added to the algorithm section, preferably after the current source-neighbor parallel forward pricing subsection and before candidate-management diagnostics. A suitable subsection title is:

> Balanced and Dynamically Refined K-Core Forward Pricing

If the paper already has a subsection named source-neighbor parallel pricing, this component can be written as an extension:

> We refine the source-neighbor parallel pricing schedule by replacing static equal-cardinality partitions with balanced workload partitions and by allowing certification-preserving dynamic refinement of unresolved subspaces.

The contribution should be described as an exact scheduling layer, not as a heuristic. It may reduce wall-clock time and improve CPU utilization, but it cannot certify closure unless all leaf subspaces in the current pricing epoch are exhausted.

## Notation

Let \(m\) denote a branch-and-bound node. Let \(\mathcal{R}_m^B\) be the set of nonempty residual route columns that satisfy all branch restrictions and residual-customer restrictions at node \(m\). Let \(\bar c_r\) be the reduced cost of route \(r\) under the current RMP dual solution and active subset-row cut set.

Let \(\mathcal{S}_m\) be the set of admissible first transformed successors after the depot source:
\[
\mathcal{S}_m :=
\{v : (\DepotSource,v)\in \Arcs^\top,\; \DepotSource\to v
\text{ is feasible under node }m\}.
\]
Every nonempty route \(r\in\mathcal{R}_m^B\) has a unique first successor \(s(r)\in\mathcal{S}_m\).

For a transformed prefix \(p=(\DepotSource,v_1,\ldots,v_q)\), define the prefix subspace
\[
\mathcal{R}_m(p)
:=
\{r\in\mathcal{R}_m^B:
\text{the canonical transformed encoding of }r\text{ begins with }p\}.
\]
The empty prefix is not used as a route column. Prefixes are used only to partition pricing work.

Let a pricing epoch be denoted by
\[
E_m =
(\lambda_m,\Pi_m^{\mathrm{act}},v_m^{\mathrm{SR}},
M_m^{\mathrm{res}},B_m,F_m,V_m^{\mathrm{col}},V_m^{\mathrm{RMP}},V_m^{\mathrm{obj}}),
\]
where \(\lambda_m\) is the true dual signature, \(\Pi_m^{\mathrm{act}}\) is the active SR set, \(v_m^{\mathrm{SR}}\) is the active SR version, \(M_m^{\mathrm{res}}\) is the residual-customer mask, \(B_m\) is the branch-state signature, \(F_m\) is the fixed-route signature, \(V_m^{\mathrm{col}}\) is the active-column version, \(V_m^{\mathrm{RMP}}\) is the RMP structure version, and \(V_m^{\mathrm{obj}}\) is the service-window/objective version. Saved labels or certificates are reusable only under an identical epoch.

## Balanced Initial K-Core Partition

The original \(K\)-core partition can assign source neighbors to workers by cardinality or by deterministic contiguous blocks. This is exact but may be poorly balanced because different first successors can induce very different label spaces. We propose a balanced workload partition based on deterministic workload scores.

For each source neighbor \(s\in\mathcal{S}_m\), define a nonnegative workload score
\[
\widehat W_m(s)
=
1
+\alpha_1 |\mathcal{U}(s)|
+\alpha_2 |\mathcal{A}^{\top,+}(s)|
+\alpha_3 |\mathcal{H}^{D}(s)|
+\alpha_4 |\mathcal{C}^{\mathrm{deadline}}(s)|,
\]
where \(\mathcal{U}(s)\) is the relaxed reachable residual customer set after the first extension, \(\mathcal{A}^{\top,+}(s)\) is the transformed outgoing arc set, \(\mathcal{H}^{D}(s)\) is the set of feasible launch pads still reachable from the prefix, and \(\mathcal{C}^{\mathrm{deadline}}(s)\) is the service-envelope-compatible customer set. The coefficients \(\alpha_i\ge0\) are fixed deterministic scaling parameters used only for scheduling. They do not alter reduced costs, dominance tests, or route feasibility.

The \(K\) worker blocks \(\mathcal{S}_{m,1},\ldots,\mathcal{S}_{m,K}\) are constructed as a deterministic balanced partition:
\[
\mathcal{S}_m = \biguplus_{k=1}^{K} \mathcal{S}_{m,k}.
\]
The partition objective is to reduce the maximum workload imbalance:
\[
\max_{k=1,\ldots,K}
\sum_{s\in\mathcal{S}_{m,k}} \widehat W_m(s).
\]
A deterministic list-scheduling rule can be used in implementation: sort source neighbors by decreasing \(\widehat W_m(s)\), break ties by transformed-node index, and assign each source neighbor to the currently least-loaded worker block, breaking worker ties by worker index. This rule is a scheduling device. Its exactness follows from disjoint exhaustive assignment, not from optimality of the load-balancing rule.

For worker \(k\), the assigned pricing route set is
\[
\mathcal{R}_{m,k}
=
\bigcup_{s\in\mathcal{S}_{m,k}} \mathcal{R}_m((\DepotSource,s)).
\]
Then
\[
\mathcal{R}_m^B
=
\biguplus_{k=1}^{K}\mathcal{R}_{m,k}.
\]

## Dynamic Subspace Refinement

Balanced initial partitioning cannot eliminate all imbalance because actual label growth depends on dual values, dominance behavior, branch restrictions, service envelopes, and SR state. Therefore, when some workers become idle, they should be allowed to assist workers whose assigned subspaces remain far from closure.

### Active Subspace State

Each worker maintains a set of active pricing tasks. A task \(\tau\) is associated with a prefix set \(\mathcal{P}_{\tau}\) and route subspace
\[
\mathcal{R}_m(\tau)
=
\bigcup_{p\in\mathcal{P}_{\tau}}\mathcal{R}_m(p).
\]
The worker stores an open-label set \(\mathcal{L}_{\tau}^{\mathrm{open}}\), a local dominance frontier, completed negative candidates, and a local lower-bound key for open labels. For standard pricing, define
\[
\underline c(\Lambda)
\]
as the reduced-cost lower bound for all completions of label \(\Lambda\), such as the dual-reward-bounded lower bound already used in the paper.

The task-level closure gap is
\[
\Delta(\tau)
:=
\max\left\{0,\,
-\min_{\Lambda\in\mathcal{L}_{\tau}^{\mathrm{open}}}
\underline c(\Lambda)
\right\}.
\]
If \(\Delta(\tau)\) is small, the task is close to proving nonnegativity under the current pricing tolerance. If \(\Delta(\tau)\) is large and many labels remain open, the task is far from closure.

### Split Eligibility

An active task \(\tau\) is eligible for dynamic refinement only if all of the following conditions hold:
\[
|\mathcal{L}_{\tau}^{\mathrm{open}}|\ge L^{\mathrm{split}},
\]
\[
\Delta(\tau)\ge \varepsilon^{\mathrm{split}},
\]
\[
T_{\tau}^{\mathrm{elapsed}}\ge T^{\mathrm{split}},
\]
and
\[
\widehat W(\tau)\ge W^{\mathrm{split}},
\]
where \(L^{\mathrm{split}}\), \(\varepsilon^{\mathrm{split}}\), \(T^{\mathrm{split}}\), and \(W^{\mathrm{split}}\) are deterministic scheduler parameters. The purpose of these tests is to prevent splitting tasks that are nearly closed. A task with a small reduced-cost closure gap or a small open-label set should be allowed to finish locally.

This rule implements the user's requirement: idle workers help only active workers that are far from closure. It avoids distracting workers that are close to exhausting their subspaces.

### Exact Split Operation

Dynamic refinement is performed only at a synchronization checkpoint. A checkpoint occurs after the active worker has completed a deterministic batch of label extensions and dominance updates. At a checkpoint, the worker can serialize a subset of open labels and transfer them to the master process.

Let \(d\) be a refinement depth. For each open label \(\Lambda\), define its prefix signature
\[
\pi_d(\Lambda)
\]
as the first \(d\) transformed nodes of the canonical transformed path represented by \(\Lambda\). Labels with the same prefix signature belong to the same refined subspace.

Given an eligible task \(\tau\), partition its open labels by prefix signatures:
\[
\mathcal{L}_{\tau}^{\mathrm{open}}
=
\biguplus_{q\in Q_\tau}
\mathcal{L}_{\tau,q}^{\mathrm{open}},
\]
where
\[
\mathcal{L}_{\tau,q}^{\mathrm{open}}
=
\{\Lambda\in\mathcal{L}_{\tau}^{\mathrm{open}}:\pi_d(\Lambda)=q\}.
\]
The corresponding route subspaces are disjoint because canonical transformed prefixes are unique:
\[
\mathcal{R}_m(\tau)
=
\biguplus_{q\in Q_\tau}\mathcal{R}_m(q).
\]

The master assigns selected prefix classes \(q\) to idle workers as new child tasks. The original worker retains the remaining prefix classes. The split is exact because every open label is assigned to exactly one child task or retained by the parent, and no route completion can cross between prefix-defined subspaces.

### Dominance After Splitting

Dominance remains local to each task unless a cross-task dominance rule is explicitly proven. The simplest exact policy is:

1. transfer labels together with their full resource state, SR state, branch-language state, and reduced-cost state;
2. initialize a child-task dominance frontier from the transferred labels;
3. do not delete labels using information from another task unless the existing same-node or physical-location dominance theorem is explicitly satisfied under the same epoch.

Omitting cross-task dominance may leave more labels alive, but it cannot remove a negative reduced-cost route. Therefore it preserves exactness and only affects computational efficiency.

## Productive Mode and Certification Mode

The scheduler should preserve the existing distinction between productive pricing and certification pricing.

In productive mode, workers may return verified negative columns early. Once the master verifies a negative column and inserts it into the RMP, the current pricing epoch changes. All open worker labels and saved subspace certificates from the old epoch become stale and must be discarded. Productive mode can generate columns but cannot certify closure.

In certification mode, closure is valid only if every leaf task in the current dynamically refined task tree is exhausted or safely terminated by an exact lower-bound rule under the same epoch \(E_m\). If any route is inserted, if the RMP dual changes, if active SR cuts change, if branch restrictions change, if residual customers change, or if the active column version changes, all certificates and saved open-label states are invalidated.

Let \(\mathcal{T}_m(E_m)\) be the set of current leaf tasks in epoch \(E_m\). Certification closure requires
\[
\forall \tau\in \mathcal{T}_m(E_m),\quad
\min_{r\in\mathcal{R}_m(\tau)}\bar c_r \ge -\varepsilon^{\mathrm{price}},
\]
where \(\varepsilon^{\mathrm{price}}\) is the configured pricing tolerance. Since
\[
\mathcal{R}_m^B
=
\biguplus_{\tau\in\mathcal{T}_m(E_m)}\mathcal{R}_m(\tau),
\]
this implies
\[
\min_{r\in\mathcal{R}_m^B}\bar c_r \ge -\varepsilon^{\mathrm{price}}.
\]

## Suggested Algorithm Text

The following pseudocode can be adapted into the paper.

```text
Algorithm: Balanced and Dynamically Refined K-Core Pricing

Input:
    Node m, transformed graph, current RMP dual solution,
    active SR set, branch restrictions, pricing tolerance epsilon_price,
    worker count K.

Output:
    A verified negative reduced-cost route, or a certificate that no route
    has reduced cost below -epsilon_price under the current pricing epoch.

1. Build the pricing epoch signature E_m.
2. Construct the admissible first-successor set S_m.
3. For each s in S_m, compute deterministic workload score W_hat_m(s).
4. Partition S_m into K disjoint worker blocks using deterministic balanced assignment.
5. Initialize one pricing task per nonempty worker block; initialize empty-core certificates for empty blocks.
6. Run forward label-setting independently on all active tasks.
7. If a worker finds a candidate negative route:
       send the candidate to the master;
       master decodes and verifies the route;
       if verified reduced cost < -epsilon_price:
           insert the route into the RMP;
           discard all labels and certificates from epoch E_m;
           return the verified route.
8. If one or more workers become idle:
       identify active tasks satisfying split eligibility;
       for each eligible task, request a synchronization checkpoint;
       partition its open labels by refined prefix signature;
       assign selected child prefix tasks to idle workers.
9. If every leaf task is exhausted or safely terminated under unchanged E_m:
       certify pricing closure under epsilon_price.
10. Otherwise continue from Step 6.
```

## Exactness Statements for the Paper

### Proposition 1: Balanced Partition Exactness

For any deterministic balanced partition \(\mathcal{S}_m=\biguplus_{k=1}^K\mathcal{S}_{m,k}\), the induced worker route sets
\[
\mathcal{R}_{m,k}
=
\bigcup_{s\in\mathcal{S}_{m,k}}\mathcal{R}_m((\DepotSource,s))
\]
form a disjoint exhaustive partition of \(\mathcal{R}_m^B\). Therefore, balanced source-neighbor assignment changes only the order and processor assignment of pricing search; it does not change the pricing problem.

Proof idea: every nonempty canonical transformed route has exactly one first successor after \(\DepotSource\). Since the source-neighbor blocks are disjoint and exhaustive, each route belongs to exactly one worker route set.

### Proposition 2: Dynamic Refinement Exactness

Suppose a task \(\tau\) is split at a synchronization checkpoint by prefix signatures \(Q_\tau\). If every open label of \(\tau\) is assigned to exactly one child task or retained by the parent, and if each child task continues the same feasible-extension, dominance, pruning, and completion rules under the same epoch, then the split preserves the pricing route set and cannot remove any negative reduced-cost route.

Proof idea: canonical transformed prefixes define disjoint route subspaces. The split only partitions open labels by prefix signature. No label is discarded during the split. Local dominance remains theorem-based. Therefore the union of child completions equals the parent completion set.

### Proposition 3: Certification Under Dynamic Refinement

At node \(m\), suppose the current pricing epoch \(E_m\) remains unchanged. If every leaf task generated by balanced partitioning and dynamic refinement is exhausted or safely terminated by a valid reduced-cost lower bound, and no leaf task returns a verified route with reduced cost below \(-\varepsilon^{\mathrm{price}}\), then no route in \(\mathcal{R}_m^B\) has reduced cost below \(-\varepsilon^{\mathrm{price}}\).

Proof idea: by Propositions 1 and 2, the leaf tasks form a disjoint exhaustive partition of the residual branch-feasible route set. Exhaustion or safe lower-bound termination of each leaf task proves nonnegativity over each subspace. Taking the union over all leaf tasks proves global pricing closure under the configured tolerance.

## Diagnostics to Report

The computational section should report diagnostics that reveal whether the new component actually improves load balance and CPU utilization. Recommended fields include:

| Diagnostic | Meaning |
|---|---|
| `pricing_worker_count` | Number of process workers \(K\). |
| `pricing_initial_source_neighbors` | Number of admissible first successors. |
| `pricing_initial_task_count` | Number of initial worker tasks. |
| `pricing_initial_block_loads` | Workload scores assigned to each worker. |
| `pricing_initial_load_imbalance_max_mean` | \(\max_k L_k / \operatorname{mean}_k L_k\). |
| `pricing_idle_worker_seconds` | Total time workers are alive but idle during pricing. |
| `pricing_dynamic_split_candidates` | Active tasks satisfying the far-from-closure tests. |
| `pricing_dynamic_splits_performed` | Number of accepted subspace refinements. |
| `pricing_dynamic_split_rejected_close_to_closure` | Tasks not split because \(\Delta(\tau)\) was too small. |
| `pricing_dynamic_split_rejected_small_queue` | Tasks not split because too few open labels remained. |
| `pricing_dynamic_child_tasks_created` | Number of child prefix tasks created. |
| `pricing_labels_transferred_to_idle_workers` | Number of open labels moved during splits. |
| `pricing_split_overhead_time` | Serialization, transfer, and child-frontier initialization time. |
| `pricing_leaf_tasks_closed` | Leaf tasks exhausted or safely lower-bound-closed. |
| `pricing_leaf_tasks_stale_discarded` | Tasks discarded after epoch changes. |
| `pricing_cpu_core_equivalent_weighted` | Effective average core usage during pricing. |
| `pricing_cpu_core_equivalent_max` | Maximum observed pricing parallelism. |
| `pricing_best_active_task_gap` | Largest \(\Delta(\tau)\) among active tasks. |
| `pricing_open_labels_by_task_max` | Largest open-label queue among leaf tasks. |

The key load-balance metrics are idle worker time, initial load imbalance, number of dynamic splits, split overhead, and weighted CPU-core equivalent. A good result would show lower idle time and higher weighted core-equivalent usage without increasing route-verification errors, stale certificate use, or unresolved task counts.

## Suggested Academic Wording

The following paragraph can be adapted directly into the paper:

> The fixed source-neighbor partition is exact, but it can be poorly balanced because different first successors induce substantially different label spaces. We therefore introduce a balanced and dynamically refined \(K\)-core pricing schedule. The schedule first assigns source-neighbor subspaces to workers using deterministic workload scores computed from relaxed reachability, transformed out-degree, drone-launch availability, and deadline-compatible customer counts. During certification, if a worker becomes idle, the master may request a synchronization checkpoint from an active worker whose task remains far from closure. The active task is then split into disjoint child tasks by canonical transformed-prefix signatures, and idle workers resume search on selected child subspaces. This operation does not change the pricing problem: it partitions open labels and their completion sets without deleting labels, modifying reduced costs, or changing dominance rules. Pricing closure is claimed only when all leaf tasks generated by the initial partition and subsequent refinements are exhausted under the same dual, active-SR set, residual-customer set, and branch-state epoch.

Another concise contribution statement:

> We strengthen the parallel forward-pricing layer with certification-preserving dynamic subspace refinement. Unlike static \(K\)-core assignment, the refined scheduler can reallocate unresolved prefix-defined pricing subspaces to idle workers, while preserving exact closure because every route remains assigned to exactly one active leaf task and every saved task state is invalidated after any RMP or branching epoch change.

## Implementation Requirements

The implementation should satisfy the following requirements:

1. The initial \(K\)-worker assignment must be a disjoint exhaustive partition of the admissible first-successor route set.
2. Dynamic splitting must occur only at synchronization checkpoints, not by unsafe asynchronous mutation of a worker's label queue.
3. Splitting must transfer complete label state: transformed endpoint, physical location, resource state, represented-customer mask, physical-node mask, branch-language state, active SR state, reduced cost, and lower-bound key.
4. A label may not be deleted merely because a split occurs.
5. After splitting, local dominance may delete labels only through the already-proven same-node or physical-location dominance tests.
6. Cross-task dominance should be disabled unless a separate theorem proves its validity under prefix partitioning.
7. Productive-mode splits may help find columns, but productive mode cannot certify closure.
8. Certification closure requires every leaf task to close under the same unchanged pricing epoch.
9. Any inserted column, RMP reoptimization, active SR change, residual customer change, branch change, fixed-route change, service-window/objective change, or active-column version change invalidates all open labels and certificates from the previous epoch.
10. The split eligibility rule must prevent refinement of tasks that are near closure, using a reduced-cost closure gap, minimum open-label count, elapsed task time, and workload estimate.

## What This Component Should Not Do

The paper should explicitly avoid presenting this as an approximate acceleration. The component should not introduce:

- label caps;
- heuristic closure;
- skipped route verification;
- fallback pricing solvers;
- approximate certificates;
- cross-task dominance without proof;
- lower bounds from partially explored tasks;
- reuse of task certificates after dual, cut, branch, residual, or column changes.

The correct interpretation is:

> Dynamic refinement is an exact parallel scheduling mechanism. It may improve CPU utilization and reduce wall-clock pricing time, but it does not alter the mathematical route set or the conditions for pricing closure.

## Recommended Numerical Experiment for This Component

To validate the component, compare three configurations on the same promised-window large-lite instance:

1. static equal-cardinality source-neighbor partition;
2. balanced initial \(K\)-core partition;
3. balanced initial partition plus dynamic subspace refinement.

Report:

- root closure status;
- root standard pricing time;
- total pricing time;
- worker idle time;
- weighted CPU-core equivalent;
- labels generated;
- labels dominated;
- completed routes generated;
- dominance pairs materialized;
- dynamic split count;
- split overhead;
- stale task discards;
- incumbent value;
- lower bound;
- full gap.

The expected evidence should not be framed as a guarantee of faster solution on every instance. The claim should be narrower and defensible: the component reduces idle worker time and improves effective core utilization on instances where fixed source-neighbor partitions are imbalanced, while preserving exact pricing closure.
