# Paper-Revision Guidance: Balanced Process-Based Forward Pricing with Work Stealing and Exact Dynamic Subspace Splitting

## 1. Purpose and Scope

This document is a focused handoff for revising the paper's current subsection

> `Deterministic Source-Successor Parallel Forward Pricing`

into a mathematically complete subsection on **balanced process-based forward pricing with exact work redistribution and dynamic subspace splitting**.

The revision should address one specific computational weakness of the current static partition: source-successor subspaces can differ greatly in difficulty, so some workers finish early while one or two workers retain large unresolved label frontiers. Merely creating (K) workers does not prevent this tail imbalance. The replacement component should therefore have two layers:

1. a deterministic, workload-balanced initial assignment of source-successor subspaces to (K) long-lived worker processes; and
2. checkpoint-based splitting of unresolved search obligations, allowing idle workers to assist workers that remain far from pricing closure.

This component must be presented as an **exact scheduling mechanism**. It must not modify route feasibility, reduced costs, label resources, dominance conditions, reduced-cost lower bounds, Farkas logic, or the definition of pricing closure. It also must not change constructive or compact warm-start logic, the RMP, branching, or subset-row separation.

The replacement subsection should be self-contained. It should not preserve statements suggesting that a static deterministic assignment is sufficient to balance the workers. It may retain the source-successor partition as the mathematical starting point, but it must extend that partition with explicit workload assignment, task ownership, checkpoint splitting, epoch invalidation, and global closure conditions.

## 2. Claims the Revised Paper May and May Not Make

The paper may claim that the mechanism:

- preserves the exact route space and pricing threshold;
- reduces initial workload imbalance using a deterministic scheduling score;
- allows idle worker processes to take ownership of unresolved label subspaces;
- avoids splitting tasks that are already close to certification;
- preserves closure because every unresolved search obligation has exactly one owner;
- avoids Python thread serialization in the pricing kernels by using separate processes; and
- changes search order and processor assignment only.

The paper should not claim that:

- all physical CPU cores are continuously saturated;
- workload scores predict exact label counts;
- splitting always reduces wall-clock time;
- process workers are guaranteed to run on distinct physical processors unless CPU affinity is explicitly implemented and reported;
- partial task exhaustion proves pricing closure;
- a worker-reported route can enter the RMP without master-side decoding and reduced-cost verification; or
- an implementation tolerance such as (0.05) is machine precision. In ideal arithmetic, exact pricing corresponds to (arepsilon^{\mathrm{price}}=0); with a positive implementation threshold, the algorithm certifies closure relative to that stated threshold.

## 3. Mathematical Objects Required by the Replacement

### 3.1 Canonical route encodings and prefix subspaces

Let (mathcal R_m^B) denote the set of nonempty residual routes satisfying all route-feasibility and branching restrictions at branch-and-bound node (m). Let

\[
    \mathbf v(r)=(\underline n,v_1,\ldots,v_{q(r)},\overline n)
\]

be the unique canonical transformed-path encoding of route (r\in\mathcal R_m^B). Canonical ordering of each simultaneous drone block is essential here: without a unique transformed encoding, a route could be assigned to more than one prefix task.

For a feasible transformed prefix

\[
    p=(\underline n,v_1,\ldots,v_d),
\]

write (p\preceq\mathbf v(r)) when (p) is a prefix of the canonical encoding of (r), and define

\[
    \mathcal R_m(p):=
    \{r\in\mathcal R_m^B:p\preceq\mathbf v(r)\}.
\]

A family of prefixes (mathcal P) is prefix-free if no distinct (p,q\in\mathcal P) satisfy (p\prec q). It is exhaustive for (mathcal R_m^B) if every route in (mathcal R_m^B) begins with exactly one prefix in (mathcal P). Consequently,

\[
    \mathcal R_m^B
    =\biguplus_{p\in\mathcal P}\mathcal R_m(p).
\]

This is the fundamental partition result on which both the initial assignment and later splitting must rest.

### 3.2 Pricing epoch

All parallel work must be attached to one immutable pricing epoch. Define

\[
E_m:=
\bigl(
\operatorname{hash}(\boldsymbol\lambda_m),
\Pi_m^{\mathrm{act}},v_m^{\mathrm{SR}},
M_m^{\mathrm{res}},B_m,F_m,
v_m^{\mathrm{col}},v_m^{\mathrm{RMP}},
v_m^{\mathrm{obj}},v_m^{\mathrm{win}}
\bigr),
\]

where:

- (oldsymbol\lambda_m) is the complete current RMP dual vector, including active SR duals;
- (Pi_m^{\mathrm{act}}) and (v_m^{\mathrm{SR}}) identify the active SR set and version;
- (M_m^{\mathrm{res}}) is the residual-customer mask;
- (B_m) is the complete branch-language signature;
- (F_m) is the fixed-route signature;
- (v_m^{\mathrm{col}}) is the active-column version;
- (v_m^{\mathrm{RMP}}) is the RMP structure version; and
- (v_m^{\mathrm{obj}}) and (v_m^{\mathrm{win}}) identify the objective scaling and service-window data.

Every task, open label, candidate route, lower-bound value, and local closure certificate must carry this epoch identity. No object generated under (E_m) may be used under (E'_m\neq E_m).

### 3.3 Search obligations rather than naive route enumeration

The paper should describe a dynamic task in terms of unresolved **label-search obligations**, not claim that all physical routes remain explicitly represented after dominance. Valid dominance can eliminate a label because another label represents completions with no larger reduced cost. Therefore, route-set equality after every dominance operation is unnecessarily strong and generally not the right invariant.

For a label (Lambda), let (mathcal C(\Lambda)) denote its feasible completion set. An open label creates the obligation to either:

1. extend it exhaustively;
2. prune it with a valid completion lower bound;
3. discharge it using a proved same-node or physical-location dominance rule; or
4. transfer the complete label state to exactly one child task.

The scheduling invariant should be stated as follows:

> At every instant of an unchanged epoch, each unresolved label-search obligation is owned by exactly one live leaf task. A split changes ownership but neither creates nor discharges an obligation.

This ownership invariant is more precise than saying that labels are merely copied into new queues.

## 4. Balanced Initial Source-Successor Assignment

Let

\[
\mathcal N_m^0
:=
\{v\in\mathcal N\cup\mathcal N^{C^\top}:
(\underline n,v)\in\mathcal A^\top,
\ \underline n\to v\text{ is feasible at node }m\}
\]

be the admissible first successors. Every nonempty canonical route has exactly one member of (mathcal N_m^0) as its first transformed node.

### 4.1 Deterministic workload score

Cardinality-balanced blocks can be highly unbalanced computationally. Define a nonnegative scheduling score for each (v\in\mathcal N_m^0):

\[
\widehat W_m(v)
=
1
\alpha_C\,|\mathcal U_m(v)|
\alpha_A\,|\delta^{\top,+}(v)|
\alpha_H\,|\mathcal H_m^D(v)|
\alpha_T\,|\mathcal U_m^{\mathrm{time}}(v)|.
\]

Here:

- (mathcal U_m(v)) is a relaxed set of residual customers reachable after the first extension;
- (delta^{\top,+}(v)) is the transformed out-neighborhood;
- (mathcal H_m^D(v)) is the set of reachable pads from which a residual customer remains drone-serviceable; and
- (mathcal U_m^{\mathrm{time}}(v)) is the relaxed set of customers still compatible with their service upper bounds.

The coefficients (alpha_C,alpha_A,alpha_H,alpha_T\ge0) are deterministic scheduler parameters. They must not enter reduced costs or any feasibility test. A paper-compatible implementation choice is

\[
(\alpha_C,\alpha_A,\alpha_H,\alpha_T)=(1,0.25,0.5,0.5),
\]

but the paper should identify these as implementation parameters, not mathematical requirements for exactness.

### 4.2 Least-loaded deterministic assignment

Sort first successors by nonincreasing (widehat W_m(v)), breaking ties by transformed-node identifier. Starting with (K) empty worker blocks, assign each successor to the currently least-loaded block, breaking worker-load ties by worker identifier. If (mathcal N_{m,k}^0) is the resulting block for worker (k), then

\[
\mathcal N_m^0=\biguplus_{k=1}^K\mathcal N_{m,k}^0,
\qquad
L_{m,k}:=\sum_{v\in\mathcal N_{m,k}^0}\widehat W_m(v).
\]

The initial worker route spaces are

\[
\mathcal R_{m,k}^B
:=
\{r\in\mathcal R_m^B:\operatorname{first}(r)\in\mathcal N_{m,k}^0\},
\]

and satisfy

\[
\mathcal R_m^B=\biguplus_{k=1}^K\mathcal R_{m,k}^B.
\]

The score and assignment rule seek lower imbalance, but exactness follows only from disjointness and exhaustiveness.

## 5. Process-Based Execution Model

The revised paper should say **worker processes**, not physical processors. The operating system may schedule multiple processes on the same physical core unless affinity is explicitly controlled. The mathematical component requires only independent process address spaces and a master-coordinated task registry.

Use (K) long-lived pricing worker processes. Each process owns:

- one or more task-local open-label priority queues;
- task-local same-node and physical-location dominance frontiers;
- task-local reduced-cost lower-bound data;
- local completed-route candidates; and
- lightweight workload and closure summaries.

The master owns:

- the immutable epoch description;
- the authoritative task registry and task generations;
- task-to-worker ownership;
- candidate-route decoding and verification;
- RMP insertion and reoptimization; and
- the global pricing-closure decision.

The master should not merge worker dominance frontiers and should not perform label-by-label dominance. Workers communicate only at bounded checkpoints or when reporting a candidate. This design limits serial merge/control burden.

## 6. Task State and Closure Distance

For a live task (	au), let:

- (mathcal L_	au^{\mathrm{open}}) be its open labels;
- (b(\Lambda)) be a valid lower bound on every completion of label (Lambda) in standard pricing;
- (b_	au:=\min_{\Lambda\in\mathcal L_	au^{\mathrm{open}}}b(\Lambda));
- (N_	au:=|\mathcal L_	au^{\mathrm{open}}|);
- (t_	au) be elapsed processing time; and
- (widehat R_	au) be a deterministic remaining-work estimate.

Pricing accepts a column only when (ar c_r<-\varepsilon^{\mathrm{price}}). Define the task's distance from lower-bound closure as

\[
G_	au
:=
\bigl[-\varepsilon^{\mathrm{price}}-b_	au\bigr]_+.
\]

Thus, (G_	au=0) when the task can already be closed by its valid lower bound. A small positive (G_	au) means the best open lower bound lies only slightly below the certification threshold; a large value indicates that the task is not close to lower-bound closure.

A simple work estimate is

\[
\widehat R_	au
=
N_	au
+\beta_E N_	au^{\mathrm{ext}}
+\beta_D N_	au^{\mathrm{dom}},
\]

where (N_	au^{\mathrm{ext}}) and (N_	au^{\mathrm{dom}}) are recent-window extension and dominance workloads. The estimate is used only to select a donor.

## 7. When an Idle Worker May Split a Running Task

Dynamic refinement should be requested only if at least one worker is idle and an active task is demonstrably far from closure. Let (N^{\mathrm{split}}), (G^{\mathrm{split}}), (T^{\mathrm{split}}), and (R^{\mathrm{split}}) be deterministic thresholds. A task is split-eligible only if

\[
N_	au\ge N^{\mathrm{split}},
\qquad
G_	au\ge G^{\mathrm{split}},
\qquad
t_	au\ge T^{\mathrm{split}},
\qquad
\widehat R_	au\ge R^{\mathrm{split}}.
\]

An implementation matching the previously discussed design may use

\[
N^{\mathrm{split}}=2000,
\quad
G^{\mathrm{split}}=10\varepsilon^{\mathrm{price}},
\quad
T^{\mathrm{split}}=5\text{ seconds},
\quad
R^{\mathrm{split}}=2000,
\]

with a checkpoint every 5,000 label extensions. These numbers are calibration choices and should be reported in the computational setup rather than built into the exactness theorem.

If several donors are eligible, choose

\[
\tau^*\in
\arg\max_{	au}
\bigl(\widehat R_	au,G_	au,N_	au,-\operatorname{id}(\tau)\bigr)
\]

lexicographically. This selects the task with the largest estimated remaining work and resolves ties deterministically. A near-closed task is excluded before donor ranking; an idle worker therefore cannot distract a task merely because its queue is nonempty.

## 8. Exact Checkpoint-Based Dynamic Splitting

### 8.1 Why asynchronous queue copying is invalid

The paper must not describe splitting as copying a running worker's queue while that worker continues to mutate it. Such a procedure can duplicate labels, lose labels, or allow two tasks to claim the same closure obligation. Splitting must be an atomic ownership transfer at a synchronization checkpoint.

### 8.2 Prefix-free frontier property

At a checkpoint, the donor pauses task (	au) after completing its current extension and all resulting dominance updates. The remaining open labels form a frontier of the task's forward search tree. Let (p(\Lambda)) be the complete canonical transformed prefix represented by label (Lambda). Because an expanded ancestor is removed before its children enter the open queue, the open prefixes form an antichain:

\[
\Lambda\neq\Lambda'
\quad\Longrightarrow\quad
p(\Lambda)\nprec p(\Lambda')
\ \text{and}\
p(\Lambda')\nprec p(\Lambda).
\]

Hence their completion languages are disjoint.

### 8.3 Refinement signatures

Let (a_	au) be the root prefix of task (	au), and let (d^{\mathrm{ref}}) be a refinement depth. For an open label whose prefix extends at least (d^{\mathrm{ref}}) arcs beyond (a_	au), define

\[
\sigma_{\tau,d^{\mathrm{ref}}}(\Lambda)
:=
\text{the first }d^{\mathrm{ref}}	ext{ transformed arcs after }a_	au.
\]

Labels that have not yet reached this depth remain with the donor. The deeper open labels are grouped by identical refinement signatures:

\[
\mathcal L_{	au,q}^{\mathrm{open}}
:=
\{\Lambda\in\mathcal L_	au^{\mathrm{open}}:
\sigma_{\tau,d^{\mathrm{ref}}}(\Lambda)=q\}.
\]

The collection of nonempty groups and the donor-retained shallow group form a disjoint partition of the open-label set. A practical initial value is (d^{\mathrm{ref}}=2).

### 8.4 Atomic ownership transfer

Suppose (J) idle workers are available. Select at most (J) nonempty groups for transfer, preferably the groups with the largest estimated workloads. Replace the parent open queue by

\[
\mathcal L_	au^{\mathrm{open}}
=
\mathcal L_{	au,0}^{\mathrm{open}}
\uplus
\mathcal L_{	au,1}^{\mathrm{open}}
\uplus\cdots\uplus
\mathcal L_{	au,J'}^{\mathrm{open}},
\qquad J'\le J,
\]

where group 0 remains with the donor and each group (j\ge1) becomes a child task. The transfer commits only when:

1. the donor is paused at the checkpoint;
2. every transferred label has been removed from the donor queue;
3. the master records exactly one owner and a new task generation;
4. the receiving process acknowledges the complete state; and
5. donor and child tasks carry the identical epoch (E_m).

If transfer has not committed, ownership remains with the donor. This transaction is an implementation realization of the one-owner invariant.

### 8.5 State that must accompany a transferred label

Every transferred label must include all information needed to continue the exact recursion:

- canonical transformed prefix or predecessor chain;
- transformed endpoint and physical truck location;
- represented-customer mask;
- visited physical truck-node mask;
- payload and drone-block resources;
- truck arrival, active-pad arrival, waiting, and service-time resources;
- active-block position and canonical-order state;
- branch automaton/language state;
- active SR parity/count state;
- accumulated reduced cost;
- valid completion lower-bound key; and
- pricing epoch and task generation.

Reconstructing a child from only an endpoint, customer mask, or reduced cost is not exact.

### 8.6 Dominance after a split

Dominance remains task-local. A child builds a local frontier from its transferred labels and subsequently applies only the paper's proved same-node and physical-location dominance rules. A donor removes transferred labels from its own frontier or rebuilds the affected local frontier.

No cross-task dominance is required. Omitting it may increase label counts but cannot remove a negative column. The paper should not claim cross-task dominance unless it provides a separate proof that includes branch-interface compatibility, active SR state, physical-location return credit, and completion-language inclusion.

## 9. Work Stealing Protocol

The term **work stealing** should be defined precisely. In this design, an idle worker does not read another process's mutable queue. Instead:

1. the idle worker reports availability to the master;
2. the master selects a split-eligible donor using the donor rule;
3. the donor reaches its next deterministic checkpoint and exports one or more disjoint frontier groups;
4. the master atomically registers child ownership; and
5. the idle worker resumes forward labeling from the transferred complete states.

This is coordinator-mediated work stealing. It retains an authoritative task registry and avoids unsafe concurrent queue access. The serial master receives only task summaries, split descriptors, and complete candidate routes; it does not process ordinary label extensions.

The worker/task state machine should be described as

\[
\mathsf{ready}\to\mathsf{running}
\to\{\mathsf{checkpoint},\mathsf{closed},\mathsf{stale}\},
\]

with

\[
\mathsf{checkpoint}\to
\{\mathsf{running},\mathsf{split}\},
\qquad
\mathsf{split}\to
\mathsf{running}+\mathsf{ready\ children}.
\]

A task marked stale can never contribute a route or certificate to the current epoch.

## 10. Productive Pricing, Certification Pricing, and Epoch Changes

### 10.1 Productive mode

In productive mode, any process may return a complete candidate before exhausting its task. The master must:

1. decode the physical truck path and drone blocks;
2. verify payload, endurance, service windows, elementarity, and branch feasibility;
3. recompute route cost and all master coefficients;
4. recompute reduced cost under the current epoch; and
5. accept the route only if (ar c_r<-\varepsilon^{\mathrm{price}}).

Once an accepted column changes the RMP, the epoch ends. All unfinished parent and child tasks, open labels, local frontiers, and partial certificates from the old epoch are marked stale and discarded. Productive mode never certifies pricing closure.

### 10.2 Certification mode

Let (mathcal T_m^{\mathrm{leaf}}(E_m)) be the authoritative set of leaf tasks created by the initial partition and all committed splits in epoch (E_m). A leaf task is closed only if its queue is exhausted or every remaining label satisfies a valid completion lower bound at least (-\varepsilon^{\mathrm{price}}). Global pricing closure requires

\[
\operatorname{epoch}(\tau)=E_m
\quad\text{and}\quad
\operatorname{status}(\tau)=\mathsf{closed}
\qquad
\forall\tau\in\mathcal T_m^{\mathrm{leaf}}(E_m),
\]

and no accepted negative candidate under that epoch.

Closure is forbidden while any leaf is ready, running, splitting, unresolved, unacknowledged, or stale.

### 10.3 Events that invalidate the epoch

The paper should explicitly list the invalidating events:

- insertion, deletion, or reactivation of an RMP column;
- RMP reoptimization and a changed true dual vector;
- addition or structural change of active SR rows;
- change in residual customers, fixed routes, or fleet limit;
- addition of a branch decision or change in the branch automaton;
- change in objective normalization or service windows; or
- any change in active-column or RMP-structure version.

Old task states may be retained for diagnostics but not resumed for proof.

## 11. Farkas Pricing

The same partition and work-transfer mechanism may schedule Farkas pricing, but the paper must preserve the distinction between standard and Farkas bounds.

In Farkas mode:

- labels use the Farkas reduced-cost recursion;
- only Farkas-valid dominance may delete labels;
- the standard dual-reward fractional-knapsack bound is not used;
- the standard zero-threshold early-termination rule is not used; and
- certification requires exhaustive closure of every leaf task unless a separately proved Farkas lower bound is available.

If no valid Farkas task lower bound exists, define split eligibility using queue size, elapsed time, and remaining-work estimates only. Do not compute (G_	au) from the standard-pricing bound.

## 12. Formal Results to Add to the Paper

### Proposition 1: Balanced initial partition

**Statement.** Let (mathcal N_m^0=\biguplus_{k=1}^K\mathcal N_{m,k}^0) be any disjoint exhaustive assignment of admissible first successors. Then

\[
\mathcal R_m^B=\biguplus_{k=1}^K\mathcal R_{m,k}^B.
\]

Therefore the workload scores and least-loaded assignment change only worker ownership and search order.

**Proof.** Every nonempty canonical transformed route has exactly one first successor in (mathcal N_m^0). Disjointness assigns that successor to exactly one block, while exhaustiveness assigns every successor to some block. Hence every route belongs to exactly one worker subspace. (square)

### Proposition 2: Conservation under a committed split

**Statement.** Consider a task (	au) at a checkpoint in an unchanged epoch. Suppose its open labels are partitioned as

\[
\mathcal L_	au^{\mathrm{open}}
=\biguplus_{j=0}^{J'}\mathcal L_{	au,j}^{\mathrm{open}},
\]

group 0 remains with the donor, each group (j\ge1) is transferred with complete label state to exactly one child, and no label is deleted by the transfer. Then the split preserves all unresolved pricing obligations.

**Proof.** Before the split, every unresolved obligation is represented by one label in the parent queue. The disjoint partition assigns each such label to exactly one post-split queue. Complete-state transfer leaves its extension, feasibility, reduced-cost, and dominance semantics unchanged. Therefore the split changes ownership only and neither loses nor duplicates an unresolved obligation. (square)

### Lemma 1: Preservation of the one-owner invariant

**Statement.** If the initial tasks satisfy the one-owner invariant and every dynamic split uses the atomic commit protocol, the invariant holds after any finite sequence of splits in an unchanged epoch.

**Proof.** Use induction on the number of committed splits. Proposition 2 supplies the induction step because one parent ownership assignment is replaced by a disjoint exhaustive set of donor/child ownership assignments. (square)

### Theorem 1: Exact pricing detection under balanced dynamic scheduling

**Statement.** Fix a pricing epoch (E_m). Assume that each worker applies the exact forward extension rules, only proved dominance rules, and only mode-valid lower-bound pruning. The balanced process scheduler either returns a master-verified route (r\in\mathcal R_m^B) with

\[
\bar c_r<-\varepsilon^{\mathrm{price}},
\]

or, after every current-epoch leaf task closes, certifies

\[
\min_{r\in\mathcal R_m^B}\bar c_r
\ge-\varepsilon^{\mathrm{price}}.
\]

**Proof outline.** Proposition 1 gives an exhaustive initial partition. Lemma 1 shows that dynamic splits preserve every unresolved obligation with exactly one owner. Valid dominance and pruning discharge an obligation only when its completions cannot provide a uniquely better negative route. Thus every potentially negative route is either represented by a live obligation, replaced by a no-worse dominating obligation, or proved nonnegative by a valid lower bound. If the master verifies a negative route, the first outcome holds. Otherwise, closure of every leaf exhausts or validly discharges every obligation in the exhaustive partition, yielding the stated global bound. (square)

### Corollary 1: Scheduling independence

For an unchanged epoch, initial workload scores, donor selection, number of processes, split timing, and assignment of child tasks affect only execution order and wall-clock performance. They do not change the accepted route set or the pricing-closure condition.

The corollary assumes deterministic route verification and the same configured threshold. Different search orders may return different negative columns when several exist, but every returned column must satisfy the same exact acceptance test.

## 13. Paper-Ready Algorithm Outline

The revised subsection should include pseudocode similar to the following.

```text
Algorithm: Balanced Process-Based Forward Pricing with Dynamic Work Stealing

Input:
    Node m; transformed pricing graph; current epoch E_m;
    worker count K; pricing threshold epsilon_price;
    split and checkpoint parameters.

Output:
    A master-verified route with reduced cost below -epsilon_price,
    or current-epoch pricing closure.

1. Construct the admissible first-successor set N_m^0.
2. Compute workload score W_hat_m(v) for every v in N_m^0.
3. Build K disjoint blocks by decreasing-score least-loaded assignment.
4. Register one initial task per nonempty block and closed-empty tasks for empty blocks.
5. Dispatch tasks to K long-lived worker processes.
6. In each worker, apply the exact forward extension, local dominance,
   mode-valid pruning, and completion rules.
7. If a worker reports a complete candidate, master-decode and verify it.
8. If its recomputed reduced cost is below -epsilon_price:
       insert it into the RMP;
       invalidate E_m and all unfinished tasks;
       return the verified route.
9. When a worker becomes idle:
       identify tasks satisfying all far-from-closure split conditions;
       select the largest eligible donor;
       pause the donor at its next checkpoint;
       partition its open frontier into disjoint prefix groups;
       atomically transfer selected groups to idle workers.
10. If every leaf task is closed under unchanged E_m and no transfer is pending:
       certify pricing closure under epsilon_price.
11. Otherwise continue from Step 6.
```

For `algorithm2e`, the final paper should explicitly declare inputs, outputs, epoch invalidation, and the all-leaf closure condition. The pseudocode should not include Python queue or serialization syntax.

## 14. Complexity and Overhead Statement

Let (n_0=|\mathcal N_m^0|). Sorting first successors costs (O(n_0\log n_0)), and least-loaded assignment with a heap costs (O(n_0\log K)). At a checkpoint, grouping (Q_	au) open labels costs (O(|Q_	au|)), excluding serialization proportional to the transferred state size.

The underlying elementary pricing problem remains exponential in the worst case. Dynamic scheduling does not improve its worst-case complexity class. Its purpose is to reduce parallel idle time and the wall-clock impact of heterogeneous subspaces. The paper should state this directly.

If (S_	au) bytes of label state are transferred, split communication is (O(S_	au)). Therefore splitting small or near-closed tasks can be counterproductive, which motivates the eligibility thresholds.

## 15. Diagnostics Required to Support the Computational Claim

The implementation and numerical section should report at least:

| Diagnostic | Mathematical/computational interpretation |
|---|---|
| `pricing_process_workers` | Number (K) of long-lived worker processes. |
| `initial_source_successors` | (|\mathcal N_m^0|). |
| `initial_block_scores` | (L_{m,k}) for every worker. |
| `initial_load_max_mean` | (max_k L_{m,k}/(K^{-1}\sum_kL_{m,k})). |
| `worker_busy_seconds[k]` | Time worker (k) executes pricing kernels. |
| `worker_idle_seconds[k]` | Time worker (k) has no owned ready/running task. |
| `pricing_core_equivalent` | (sum_k T_k^{\mathrm{CPU}}/T^{\mathrm{wall}}). |
| `idle_worker_requests` | Number of requests for additional work. |
| `split_candidates` | Tasks satisfying queue/work eligibility before closure screening. |
| `splits_performed` | Number of committed transfers. |
| `split_rejected_near_closure` | Donors excluded by (G_	au<G^{\mathrm{split}}). |
| `split_rejected_small_frontier` | Donors excluded by (N_	au<N^{\mathrm{split}}). |
| `split_rejected_low_work` | Donors excluded by (widehat R_	au<R^{\mathrm{split}}). |
| `labels_transferred` | Number and serialized bytes of transferred labels. |
| `split_control_seconds` | Checkpoint, grouping, registration, and acknowledgment time. |
| `leaf_tasks_created` | Initial plus dynamically created leaf tasks. |
| `leaf_tasks_closed` | Current-epoch leaf tasks validly closed. |
| `stale_tasks_discarded` | Tasks invalidated after an epoch change. |
| `pending_transfer_peak` | Maximum uncommitted transfers; closure requires zero. |
| `productive_first_hit_worker` | Worker returning the accepted candidate. |
| `candidate_verification_seconds` | Master-side route decoding and verification time. |
| `master_control_seconds` | Serial scheduling/control time excluding RMP and SR work. |

CPU utilization should be interpreted against the machine's logical-core count. Six busy workers on a many-core machine will not appear as 100% total system utilization. The paper should compare static and dynamic schedules using process busy/idle time and core-equivalent utilization rather than relying only on a Task Manager screenshot.

## 16. Required Ablation for the Numerical Section

To justify the new component, use the same instances and compare:

1. static equal-cardinality source-successor assignment;
2. balanced initial assignment without splitting; and
3. balanced initial assignment with dynamic splitting/work stealing.

Use identical route generation, duals, tolerances, warm starts, RMP logic, SR cuts, process count, and time limits. Report:

- root closure and total node closure;
- pricing wall time;
- process CPU time and core equivalent;
- total and maximum per-worker idle time;
- initial and terminal load imbalance;
- labels generated, dominated, and lower-bound pruned;
- split count, transferred labels, and split overhead;
- accepted negative columns and best reduced cost at termination;
- stale work discarded after RMP changes; and
- incumbent, lower bound, and final gap.

The defensible efficiency claim is conditional:

> Balanced dynamic scheduling reduces worker idle time and tail latency when static source-successor subspaces are heterogeneous, while preserving the same pricing problem and closure criterion.

Do not claim universal speedup. If split overhead offsets the saved idle time on small instances, report that boundary explicitly.

## 17. Text That Should Replace the Current Subsection

The paper-revision model should replace the entire current static subsection, not append one vague sentence about work stealing. The replacement should follow this order:

1. define canonical source-successor subspaces;
2. state the disjoint exhaustive partition;
3. define workload scores and deterministic least-loaded assignment;
4. define immutable pricing epochs;
5. define process-local task state and the one-owner invariant;
6. define closure distance and far-from-closure split eligibility;
7. define checkpoint frontier splitting and atomic transfer;
8. distinguish productive from certification mode;
9. state epoch invalidation and master verification;
10. state Farkas restrictions;
11. give the partition, split-conservation, and global-exactness results; and
12. end with diagnostics and a restrained computational claim.

The existing equations defining (mathcal N_m^0), its disjoint source-successor cover, and the induced route-space partition may be retained as the opening equations. The remainder of the current text should be replaced because a configurable backend and fixed prefix family alone do not realize balanced work stealing or dynamic subspace splitting.

## 18. Nonnegotiable Exactness Checklist for the Paper Revision

Before accepting the revised LaTeX, verify that it states all of the following:

- Every nonempty route has one canonical transformed encoding.
- Initial tasks are disjoint and exhaustive.
- Workload scores affect scheduling only.
- Workers are processes; physical-core affinity is not claimed without evidence.
- Every unresolved label obligation has exactly one owner.
- A donor pauses before its queue is partitioned.
- A split transfers complete label states and never deletes a label.
- Near-closed tasks are excluded from splitting by a valid closure-distance test.
- Dominance remains theorem-based and task-local.
- Productive pricing cannot certify closure.
- Candidate routes are independently verified by the master.
- Every leaf must close under one unchanged epoch.
- No closure is possible during a pending transfer.
- Any epoch change invalidates all unfinished work.
- Farkas mode does not use standard reduced-cost pruning.
- Ideal exactness and positive-tolerance closure are described separately.
- Warm-start, RMP, branching, and SR logic are explicitly outside this component.

## 19. Concise Contribution Wording

A contribution statement suitable for the introduction is:

> We develop a balanced process-based forward-pricing scheduler with certification-preserving dynamic subspace splitting. A deterministic workload assignment reduces initial source-successor imbalance, while checkpoint transfers allow idle processes to assume ownership of unresolved prefix-defined label frontiers from tasks that remain far from closure. The mechanism preserves exact pricing because splitting neither deletes nor alters labels, every unresolved search obligation has exactly one current-epoch owner, and pricing closure requires exhaustion of all dynamically generated leaf tasks under one unchanged RMP dual state.

## 20. Final Instruction to the Paper-Revision Model

Revise only the parallel-pricing subsection and any immediately dependent notation, proposition, algorithm, contribution, and diagnostic paragraphs. Do not reintroduce backward labeling, bidirectional joins, DSS/ng relaxations, label caps, heuristic closure, skipped verification, alternative pricing solvers, or changes to warm-start logic. Ensure that all symbols are reconciled with the paper's existing macro definitions and that equation/proposition labels do not conflict with the rest of the manuscript.

