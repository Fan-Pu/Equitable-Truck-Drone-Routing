# Parallel Pricing Implementation Misalignment Report

## Scope

This report compares the proposed paper subsection titled
`Parallel Pricing with Dynamic Route-Space Partitioning` with the current
production implementation in `thvrpd/pricing.py`, `thvrpd/bpc.py`, and
`thvrpd/config.py`.

The subsection is directionally consistent with the implementation: both use
a disjoint first-successor partition, long-lived pricing processes, task-local
forward labeling, checkpoint-based redistribution of live labels, local
dominance, master-side column verification, and invalidation after RMP
reoptimization. The following details, however, do not align.

## Indexed Misalignments

1. **The initial partition should be defined by transformed first successors, not only first physical truck nodes.**

   - **Paper statement.** The proposed text defines
     `\SourceNeighborSet{m}:=\{v_1^r:r\in\BranchFeasibleRoutes{m}\}` using the
     first physical node of the truck route.

   - **Implementation.** The scheduler enumerates admissible outgoing
     neighbors of `\DepotSource` in the transformed graph, excludes
     `\DepotSink`, and then applies the actual extension and service-deadline
     tests. The partition is therefore based on the first node of the
     canonical transformed path, not on a separately reconstructed physical
     truck route.

   - **Why the distinction matters.** The exact partition argument relies on
     every nonempty canonical transformed route having exactly one first
     transformed successor. Describing the partition through a physical route
     is equivalent only when every admissible first transformed successor is a
     physical node. The implementation and theorem are clearer if they use the
     same transformed-path object directly.

   - **Required paper revision.** Define

     ```latex
     \SourceNeighborSet{m}
     :=
     \left\{
     v:(\DepotSource,v)\in\Duplicate{\Arcs},\;
     v\neq\DepotSink,\;
     \DepotSource\rightarrow v
     \textnormal{ is feasible at node }m
     \right\}.
     ```

     Then define each worker route set using the unique first successor of the
     canonical transformed encoding.

2. **The initial workload score is materially more detailed than the single reachable-customer count stated in the paper.**

   - **Paper statement.** The proposed text uses only
     `\lvert\ReachableCustSet{\ForwardLabel{v}}\rvert` as the score of first
     successor `v`.

   - **Implementation.** The production score is

     ```latex
     S_m(v)
     =
     1
     +|\mathcal U_m(v)|
     +0.25|\delta^{\top,+}(v)|
     +0.5|\mathcal H_m^{\mathrm D}(v)|
     +0.5|\mathcal U_m^{\mathrm{time}}(v)|.
     ```

     It combines relaxed reachable customers, transformed out-degree,
     reachable pads retaining residual drone opportunities, and customers
     that remain compatible with service deadlines.

   - **Why the distinction matters.** The score does not change feasibility or
     exactness, but it changes the initial task assignment and is the principal
     implemented mechanism for reducing initial process imbalance. Reporting
     only customer reachability understates and mischaracterizes the balancing
     mechanism used in the numerical experiments.

   - **Required paper revision.** State the complete weighted score and clarify
     that its coefficients affect scheduling only. Retain the decreasing-score,
     least-loaded assignment with deterministic node and process tie breaking.

3. **The proposed text omits the implemented task-splitting eligibility conditions.**

   - **Paper statement.** Repartitioning is attempted whenever at least one
     process is idle. The paper does not impose minimum frontier size, elapsed
     time, estimated work, or distance from closure.

   - **Implementation.** An active task is eligible only when all applicable
     conditions hold:

     ```text
     open labels          >= 500
     elapsed task time    >= 5 seconds
     remaining work       >= 2,000
     standard closure gap >= 10 times the pricing tolerance
     ```

     The closure-gap condition is omitted in Farkas mode because the standard
     completion lower bound is not valid for the Farkas objective.

   - **Why the distinction matters.** The eligibility screen is not merely an
     implementation detail. It explains why an idle process does not always
     receive work and prevents process-transfer overhead from interrupting a
     task that is small or close to certification.

   - **Required paper revision.** Introduce the open-frontier, elapsed-time,
     remaining-work, and standard closure-gap thresholds before defining donor
     selection. State separately that Farkas splitting does not use the
     standard closure gap.

4. **The implemented donor is not selected solely by the number of open labels.**

   - **Paper statement.** The active process with the largest number of open
     nondominated labels is selected as the donor.

   - **Implementation.** Among eligible tasks, the master chooses the donor
     lexicographically by:

     1. largest remaining-work estimate;
     2. largest closure gap when available;
     3. largest open-label count; and
     4. deterministic task-ID tie breaking.

     The remaining-work estimate is the sum of the current open-label count
     and recent extension and full-dominance workloads.

   - **Why the distinction matters.** Frontier size alone can be a poor proxy
     for remaining cost. A smaller frontier can be more expensive if its labels
     have recently caused many extensions or dominance comparisons. The
     implementation deliberately prioritizes estimated work and distance from
     closure before raw queue size.

   - **Required paper revision.** Define a remaining-work estimate and state
     the lexicographic donor rule. Do not claim that the largest queue is always
     selected.

5. **The implementation does not sort all open labels by completion bound and transfer one half.**

   - **Paper statement.** The donor orders open labels by nondecreasing
     `\KnapReducedCostLB{\Lambda}`, retains the first half, and transfers the
     second half to one idle process.

   - **Implementation.** The donor performs deterministic structural splitting:

     - labels shallower than the refinement depth remain with the donor;
     - sufficiently deep labels are grouped by the first two transformed arcs
       following the task-root prefix;
     - groups are ordered by decreasing cardinality with deterministic prefix
       tie breaking;
     - the largest groups are assigned to available idle processes;
     - at least one complete group remains with the donor; and
     - several idle processes may receive child tasks in the same committed
       split.

   - **Why the distinction matters.** Dividing a priority queue in half by
     lower-bound order does not define stable prefix-based route subspaces and
     may intermingle labels from many unrelated portions of the transformed
     route tree. The implemented prefix grouping provides identifiable,
     disjoint child tasks and supports explicit ownership and certification.

   - **Required paper revision.** Replace the half-frontier rule with the
     depth-two prefix-signature partition. State that lower bounds influence
     donor eligibility and queue processing but do not define the transferred
     route subspaces.

6. **The paper does not describe the implemented pending-transfer and unique-ownership protocol.**

   - **Paper statement.** A transferred label is removed from the donor and
     assigned to a receiver. No intermediate ownership state is described.

   - **Implementation.** A split uses an explicit transaction:

     - the donor pauses at a checkpoint and returns proposed child groups;
     - transferred labels are removed from the donor state;
     - the master creates new task IDs and generations;
     - child tasks are marked pending and assigned to unique workers;
     - a receiver emits a started acknowledgment; and
     - pending children prevent certification until acknowledgment and eventual
       closure.

   - **Why the distinction matters.** The claim that repartitioning preserves
     exhaustive coverage requires more than set conservation. At every stage,
     each unresolved label must have exactly one authoritative owner, and
     pricing cannot close while a transfer is pending.

   - **Required paper revision.** Add a concise one-owner invariant and state
     that a pending or failed transfer blocks certification. Low-level queue or
     serialization details need not be included.

7. **The local-frontier reconstruction after transfer is omitted.**

   - **Paper statement.** The receiver obtains complete label states, and
     dominance is local to each process.

   - **Implementation.** A child process inserts its received open labels into
     a newly constructed task-local dominance frontier before continuing the
     search. Exact dominance may remove a received label at this stage, but the
     transfer operation itself does not delete any label.

   - **Why the distinction matters.** A dominance frontier contains historical
     comparison structure that is not transferred as an authoritative global
     object. Rebuilding it from live labels preserves exactness while possibly
     losing some cross-task pruning opportunities.

   - **Required paper revision.** State that donor and child dominance
     frontiers are task-local and reconstructed from their retained or
     transferred live labels. Clarify that cross-task dominance is disabled.

8. **The proposed global batching policy does not match the process scheduler.**

   - **Paper statement.** Negative routes generated by all processes are
     accumulated until either `\BatchLimit` distinct routes have been collected
     globally or all searches have terminated. The global collection is then
     inserted simultaneously.

   - **Implementation.** Each worker has a local batch target. The first worker
     that returns a nonempty candidate batch sends it to the master. The master
     reconstructs and verifies that worker's routes under the current epoch. If
     the batch contains an accepted entering route, the epoch ends and all
     other active workers are cancelled. The implementation does not wait to
     aggregate candidates across all workers.

   - **Why the distinction matters.** Waiting for a global batch would delay RMP
     reoptimization and alter the scheduling behavior measured in the
     experiments. The implemented first-return policy favors rapid dual refresh
     and deliberately discards unfinished old-dual work after acceptance.

   - **Required paper revision.** Describe `\BatchLimit` as a worker-local
     candidate target. State that the first master-verified nonempty batch ends
     the current pricing epoch and triggers RMP reoptimization.

9. **Master-side candidate verification is understated.**

   - **Paper statement.** A process-generated negative route is retained for
     insertion, and collected routes are inserted into the RMP.

   - **Implementation.** Worker output is only a candidate. Before acceptance,
     the master:

     - reconstructs the canonical route from its path;
     - verifies route and branch feasibility;
     - recomputes customer service times and waiting times;
     - reconstructs master and active-SR coefficients;
     - checks duplicate-equivalent existing columns; and
     - recomputes reduced cost under the current dual.

   - **Why the distinction matters.** Worker calculations and serialized state
     are not authoritative for RMP insertion. Master recomputation is part of
     the implementation's exactness contract and should appear explicitly in
     the paper.

   - **Required paper revision.** Replace “the route is retained for insertion”
     with a master-verification step followed by conditional acceptance.

10. **The pricing epoch is incompletely specified.**

    - **Paper statement.** All processes use the same RMP dual solution,
      active-SR set, and inherited branching decisions.

    - **Implementation.** The immutable epoch identity includes:

      1. the complete dual signature;
      2. active-SR identifiers and SR version;
      3. residual-customer mask;
      4. branch-restriction signature;
      5. fixed-route signature;
      6. active-column version;
      7. RMP-structure version;
      8. objective-scaling version; and
      9. service-window version.

    - **Why the distinction matters.** Identical dual values alone do not make
      unfinished tasks reusable if the residual route set, active columns,
      fixed routes, objective normalization, or service data have changed.
      Each of these changes can alter route feasibility, coefficients, or
      certification meaning.

    - **Required paper revision.** Define the complete immutable pricing epoch
      and state that every task, checkpoint, split offer, candidate, and closure
      report is associated with that epoch.

11. **The subsection does not state the implemented stale-result rule.**

    - **Paper statement.** Unfinished searches are discarded after RMP
      reoptimization.

    - **Implementation.** Every worker event is checked against the current
      epoch. Results tagged with another epoch are rejected. Accepting an
      entering route broadcasts cancellation to active workers, which stop at a
      checkpoint and discard their old state.

    - **Why the distinction matters.** Stale candidate or closure reports cannot
      be used merely because they arrive after the RMP has changed. Explicit
      rejection is necessary for the certification claim.

    - **Required paper revision.** State that stale candidates, split offers,
      acknowledgments, and closure reports are ignored and cannot contribute to
      the next pricing epoch.

12. **The closure condition is missing from the proposed subsection.**

    - **Paper statement.** The text explains that unfinished searches are
      discarded after a column is inserted but does not define when pricing can
      certify that no entering route remains.

    - **Implementation.** Pricing closure requires all of the following under
      one unchanged epoch:

      - every authoritative leaf task is exhausted or validly lower-bound
        closed;
      - no child transfer remains pending;
      - no donor split remains unresolved; and
      - no master-verified entering route has been returned.

      Empty initial worker blocks are treated as immediately closed leaves.

    - **Why the distinction matters.** Parallel route generation and exact
      pricing certification are different operations. A worker finishing its
      own task, or a productive call returning no route, cannot certify global
      closure.

    - **Required paper revision.** Add a separate certification paragraph that
      defines closure over the complete authoritative leaf-task registry.

13. **Productive and certification behavior are not distinguished.**

    - **Paper statement.** The subsection presents one generic pricing process
      in which negative routes are collected and all searches may terminate.

    - **Implementation.** Productive calls may stop after returning entering
      columns and never certify closure. Closure-mode calls can also return
      columns, in which case they are not certificates; they certify only when
      all authoritative tasks close without an entering route.

    - **Why the distinction matters.** Column discovery is not evidence that
      the current dual is optimal for the full route set. Conversely, a
      time-limited or interrupted call returning no column does not prove
      nonexistence.

    - **Required paper revision.** Explicitly separate productive return from
      exhaustive certification and state that only the latter can establish
      pricing closure.

14. **Farkas-mode scheduling requires a separate qualification.**

    - **Paper statement.** The proposed subsection is written only in terms of
      the standard completion bound and negative reduced-cost routes.

    - **Implementation.** The same process scheduler is used for Farkas pricing,
      but standard dual-reward pruning, the standard completion lower bound,
      and the standard closure-gap eligibility test are disabled. A Farkas task
      must be exhausted subject only to Farkas-valid dominance unless a separate
      Farkas lower bound is proved.

    - **Why the distinction matters.** Applying the standard bound to a Farkas
      ray objective would make the infeasibility certificate invalid.

    - **Required paper revision.** Add one final sentence distinguishing
      Farkas splitting eligibility and exhaustive closure from standard
      pricing.

## Concise Alignment Summary

The current subsection correctly describes the high-level idea of disjoint
parallel route-space search and dynamic redistribution. It does not yet match
the implementation in its workload score, split eligibility, donor selection,
prefix-based subdivision, transfer transaction, batching policy, epoch
identity, stale-result rejection, certification condition, or Farkas-mode
qualification.

The most consequential corrections are Items 5, 8, 10, and 12. Those items
affect the mathematical description of disjoint subspaces, the actual timing
of RMP reoptimization, and the validity of parallel pricing closure. The other
items primarily align the scheduling and implementation details with the
reported computational procedure.
