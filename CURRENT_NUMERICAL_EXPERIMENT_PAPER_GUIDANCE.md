# Guidance for Rebuilding the Numerical Experiment Section

## 1. Purpose and scope

This document explains how to populate the empty computational-experiment portion of the current manuscript using the experiment design and artifacts in the current TH-VRPD project. The original manuscript provides a useful organizational template, but its numerical values, benchmark dimensions, algorithm settings, and performance claims are no longer authoritative.

The recommended approach is:

1. retain the original paper's broad narrative sequence;
2. replace the original instance design with the current generator and campaign design;
3. compare the current BPC implementation with the direct compact arc-based MIQP on exactly matched instances;
4. distinguish threshold-relative BPC closure from zero-tolerance mathematical exactness;
5. separate small/medium validation evidence from large-scale proof-throughput evidence; and
6. omit unresolved or historically obsolete claims.

The current paper should not copy the original numerical tables verbatim. The two studies solve materially different generated instances.

## 2. Recommended placement in the current manuscript

Insert the main computational section and conclusion immediately before:

```latex
\begin{APPENDICES}
```

The main-paper sequence should become:

```latex
\section{Branch-Price-and-Cut Algorithm}
...
\section{Computational Experiments}\label{sec:exp}
...
\section{Conclusions}\label{sec:conc}

\begin{APPENDICES}
...
```

This placement keeps implementation proofs and detailed recursions in the appendices while allowing the main text to evaluate the method described in Sections 3--5.

Use **Computational Experiments**, rather than **Numeric Experiments**, as the section title.

## 3. What can be retained from the original paper

The following structural elements can be adapted:

- a setup subsection describing physical parameters, generated instances, hardware, software, and time limits;
- separate discussion of small, medium, and large cases;
- paired comparison between BPC and direct Gurobi solution of the compact arc-based formulation;
- reporting of incumbent, lower bound, gap, runtime, root bound, generated columns, and branch-and-bound nodes;
- route/service visualizations for a small number of representative instances; and
- an operational discussion of customer delays, return times, truck use, and drone sorties.

The following original content must be replaced:

| Original item | Current project item |
|---|---|
| 5, 15, and 25 customers | 5, 10, and 25 customers |
| 2, 5, and 8 trucks | 2, 3, and 5 trucks |
| PS, PC, and mixed layouts | PS and PC layouts in the authoritative campaign |
| Three replications per profile | Four PS seeds and four PC seeds per scale |
| 27 instances | 24 instances: 3 scales x 2 layouts x 4 seeds |
| Truck/drone speeds 40/40 km/h | Truck/drone speeds 40/100 km/h |
| Drone payload 2.3 kg | Drone payload 6 kg |
| Drone endurance 30 minutes | Drone endurance 75 minutes |
| Drone launch cost 6 | Drone launch cost 1 |
| Objective weights 1/3, 1/3, 1/3 | Delay/return/cost weights 0.4/0.3/0.3 |
| Truck/hub arc probabilities 0.20/0.50 | Truck/hub arc probabilities 0.05/0.18 |
| No promised-service-window design | Random-absolute promised-service windows |
| 1,800-second limit | 3,600-second limit in the paired campaigns |
| Earlier parallel pricing implementation | 12-process balanced forward pricing with exact dynamic splitting |

Do not retain the original claims that all medium and large BPC cases were proven optimal, that the average large-case BPC runtime was approximately 730 seconds, or that direct Gurobi had a 99.31% mean gap. Those claims describe the old experiment, not the current project.

## 4. Authoritative current experiment design

### 4.1 Scale definitions

Use the following scale table.

| Scale | Customers | Available trucks | Hubs | Drones per truck | Structurally drone-required customers |
|---|---:|---:|---:|---:|---:|
| Small | 5 | 2 | 2 | 4 | 1 |
| Medium | 10 | 3 | 2 | 4 | 2 |
| Large | 25 | 5 | 2 | 4 | 4 |

The last column follows `ceil(0.16 * number of customers)`.

### 4.2 Spatial distributions and seeds

Use two spatial patterns:

- **PS (purely sparse):** customer coordinates are sampled independently and uniformly from the square `[0,14] x [0,14]`.
- **PC (purely clustered):** the generator creates `max(1, floor(C/5))` cluster centers uniformly in the square. Customer coordinates are sampled around these centers from normal distributions with coordinate standard deviation `0.05 x 14`, then clipped to the square.

Use the following deterministic seeds:

| Pattern | Seeds | Replications per scale |
|---|---|---:|
| PS | 1, 2, 3, 4 | 4 |
| PC | 5, 6, 7, 8 | 4 |

Thus, each scale contains eight cases and the full campaign contains 24 cases. Each case is solved by BPC and direct compact Gurobi, giving 48 solver runs.

The generator supports a mixed distribution, but the current authoritative campaign does not use it. Do not describe mixed cases unless they are rerun as an explicitly defined supplementary experiment.

### 4.3 Depot, hubs, distances, and travel times

The current generator uses:

- region: `[0,14] x [0,14]`, with area 196 square kilometers if coordinates are interpreted as kilometers;
- depot: `(0,0)`, represented by source and sink copies;
- hubs: two K-means centroids computed from customer coordinates;
- truck distance: Manhattan distance;
- drone distance: Euclidean distance;
- truck speed: 40 km/h;
- drone speed: 100 km/h.

This differs from the original manuscript, which placed the depot at the customer centroid. The paper must use `(0,0)` unless the generator is changed and every experiment is rerun.

### 4.4 Demand, capacity, endurance, and cost

Use the following physical parameters.

| Parameter | Value |
|---|---:|
| Truck payload | 50 kg |
| Drone payload | 6 kg |
| Drone endurance | 75 minutes round trip |
| Truck fixed deployment cost | 20 |
| Drone sortie cost | 1 |
| Drones carried per truck | 4 |

Half of the customers, rounded down when the customer count is odd, are selected as low-demand customers. Their demands are sampled from a normal distribution with mean 1 and standard deviation 5, truncated below at 0.5. Remaining demands are sampled from a normal distribution with mean 10 and standard deviation 5, truncated below at 6. Demands are rounded to two decimal places.

### 4.5 Sparse truck graph

The graph generator first adds source-to-physical-node and physical-node-to-sink truck arcs. It then samples directed internal arcs:

- customer-to-customer arc probability: 0.05;
- customer-hub or hub-customer arc probability: 0.18.

The deterministic feasibility witness may add truck arcs needed by its route backbone. Consequently, the realized number of truck arcs is an output of instance generation and should be reported per case; it should not be inferred solely from the two probabilities.

### 4.6 Drone-required graph policy

The current benchmark deliberately prevents truck-only solutions.

1. Feasible hub-customer drone arcs are generated after drone payload and endurance checks.
2. Candidate hub-customer pairs are ranked by drone service-time advantage, then by shorter round-trip time and deterministic identifiers.
3. `ceil(0.16 C)` customers are selected as mandatory drone customers.
4. All truck arcs incident to a mandatory drone customer are removed.
5. One protected drone arc is retained for each mandatory drone customer.
6. At most two drone-accessible customers are retained per hub and at most one launch hub is retained per customer.
7. Optional drone arcs are disabled in the baseline campaign.
8. A deterministic witness route set is built, and required witness-backbone truck arcs are inserted. Generation fails if the witness violates fleet or payload limits.

Because optional drone arcs are disabled, the retained drone arcs are the protected arcs. Every feasible solution must therefore use at least 1, 2, and 4 drone sorties in the small, medium, and large cases, respectively.

This design has an important interpretation consequence: the baseline experiments demonstrate performance on a **drone-required TH-VRPD benchmark**. They do not, by themselves, demonstrate that drones are selected endogenously over truck service because of their speed or cost. Any managerial claim about voluntary drone adoption requires a separate experiment with `mandatory_drone_customer_fraction=0` and optional drone arcs enabled.

### 4.7 Promised-service windows

The baseline uses `random_absolute` promised-service windows for every customer.

For customer `n`, the generator first computes the relaxed earliest-service benchmark `arrival_lb[n]`. A deterministic random offset is sampled from `[30,90]`, producing the candidate bound

```text
arrival_lb[n] + random_offset[n].
```

A deadline-free feasible witness is then constructed. The final deadline is

```text
max(arrival_lb[n] + random_offset[n], witness_service_time[n] + 3).
```

The 3-minute witness slack guarantees that the generated deadline does not exclude the witness. The effective service upper bound is the minimum of this promised deadline and the graph-derived service upper bound.

The setup should explicitly report:

- offset interval `[30,90]` minutes;
- witness slack 3 minutes;
- witness method `constructive_then_compact`;
- witness construction limit 30 seconds; and
- the number of witness-lifted deadlines in each case.

The configuration field `service_deadline_fraction=0.60` is not used to select only 60% of customers in `random_absolute` mode. The implementation constructs a deadline for every customer. Therefore, do not write that only 60% of customers have promised-service windows.

### 4.8 Objective weights and reported objective

Use:

| Objective component | Weight |
|---|---:|
| Quadratic customer-delay term | 0.4 |
| Truck return-time term | 0.3 |
| Fixed operating-cost term | 0.3 |

All tables must report the **full normalized objective**, including the constant normalization shift. Do not mix the shifted route-space/RMP objective with the full compact objective. The same full-objective convention must be used for incumbents, lower bounds, root bounds, gaps, and cross-method comparisons.

## 5. Solver and implementation settings

### 5.1 BPC settings

The paired campaigns use:

| Setting | Value |
|---|---:|
| Wall-clock limit | 3,600 s |
| Gurobi threads for each RMP | 1 |
| Pricing tolerance | 0.01 |
| Cut tolerance | `1e-7` |
| Integrality tolerance | `1e-6` |
| Pricing workers | 12 processes |
| Pricing batch size | 64 |
| Farkas batch size | 16 |
| Root constructive limit | 5 s |
| Compact root warm-start solve limit | 60 s |
| Full route-pool solve limit | 2 s per call |
| SR cuts added per batch | 32 |
| Dynamic split open-label threshold | 500 |
| Dynamic split work threshold | 2,000 |
| Dynamic split elapsed threshold | 5 s |
| Dynamic refinement depth | 2 |
| Worker checkpoint period | 1,000 extensions |

The paper should state that the production pricing engine is forward-only. The 12 workers are operating-system processes, not 12 Gurobi threads. Productive pricing may return verified negative columns but cannot certify closure. Certification requires all current-epoch pricing tasks to close.

Theoretical exactness corresponds to zero reduced-cost tolerance. In the implementation, a route enters only when its recomputed reduced cost is below `-0.01`; an `optimal` BPC status therefore means closure relative to the configured `0.01` threshold. Recommended wording is:

> The mathematical algorithm is exact at zero pricing tolerance. The implementation uses a reduced-cost tolerance of 0.01; accordingly, reported BPC closure and optimality are threshold-relative to this value.

Do not write unqualified “global optimum” for a BPC run unless the experiment is rerun at a tolerance appropriate for that claim and the numerical implications are justified.

### 5.2 Compact Gurobi benchmark

For every instance, solve the compact arc-based MIQP directly with Gurobi for 3,600 seconds using one thread. The direct compact run must use the same generated instance, objective coefficients, normalization bounds, and service windows as BPC.

In tables, use the underlying Gurobi termination status:

- `OPT` for proven optimality;
- `TL` for time limit with an incumbent;
- `INF` for proven infeasibility; and
- `--` where no incumbent or bound exists.

Do not use the wrapper status `success` as though it meant Gurobi proved optimality. In stored compact results, `success` means that the benchmark command completed and wrote a result; it can still represent a time-limited Gurobi solve.

### 5.3 Warm-start interpretation

The BPC root initialization consists of:

1. verified constructive route generation;
2. one direct compact warm-start call with a fixed 60-second Gurobi solve budget;
3. route extraction and canonical reconstruction;
4. `route_from_path` feasibility verification; and
5. insertion of verified columns and an incumbent update only if the full route set is feasible and better.

This compact call is primal-only. Its bound cannot be used as the BPC node lower bound, and it cannot certify root closure.

### 5.4 Hardware and software

The current machine is identified as:

- AMD Ryzen 5 5600 6-Core Processor;
- Windows, build reported by the environment as `10.0.26200`;
- Python 3.11.7;
- Gurobi 12.0.1.

The previous paper states 32 GB RAM, but the current sandbox could not independently query installed memory. Confirm the physical memory on the licensed run machine before retaining “32 GB” in the final manuscript.

The campaign monitor records the complete BPC process tree once per second. This permits reporting mean/peak core equivalent and peak resident memory. CPU percentages should be computed from the process tree, not from the parent process alone.

## 6. Recommended section structure

### 6.1 Computational setup and benchmark design

Combine the material in Sections 4 and 5 above into two compact tables:

1. scale and replication design; and
2. physical, graph, deadline, objective, and solver parameters.

End the subsection by defining threshold-relative closure and the paired-instance comparison protocol.

### 6.2 Performance measures

Define the following measures before presenting results:

- **UB:** objective of the best verified feasible solution;
- **LB:** valid full-objective lower bound at termination;
- **Gap:** `(UB-LB)/abs(UB)` when a finite incumbent and bound exist;
- **Root LB:** valid lower bound after root cut-and-price closure;
- **Root closed:** all standard/Farkas pricing tasks certified under one unchanged pricing epoch;
- **Time:** wall-clock seconds including initialization;
- **Nodes:** BPC nodes processed or Gurobi MIP nodes, clearly distinguished;
- **Columns:** verified route columns inserted into the RMP;
- **Labels:** forward labels generated by pricing;
- **Core equivalent:** process-tree CPU seconds divided by wall-clock seconds;
- **Drone sorties:** number of drone-served customers in the selected solution; and
- **Service feasibility:** maximum violation of promised-service bounds, payload, coverage, and endurance.

Avoid the column heading “Best IP.” Use “UB” or “Incumbent,” because time-limited values are not necessarily optimal integer objectives.

### 6.3 Small and medium validation

Use the 16 small/medium paired cases to validate implementation consistency and formulation agreement.

The stored evidence currently supports the following descriptive statements:

- all eight small BPC runs and all eight medium BPC runs closed at the root under pricing tolerance 0.01;
- direct compact Gurobi reached zero reported gap on all 16 cases;
- paired full objectives agree within numerical precision on all 16 cases;
- mean BPC runtimes were approximately 2.879 seconds for small cases and 4.699 seconds for medium cases;
- mean compact runtimes were approximately 0.045 seconds and 0.511 seconds, respectively; and
- every selected solution used exactly the structurally required number of drone sorties: one for small and two for medium.

The correct interpretation is that direct Gurobi is faster on these small validation cases, whereas objective agreement validates the compact/route-space implementation on this subset. Do not claim BPC computational superiority on small or medium cases.

### 6.4 Large-scale comparison

Use the eight 25-customer paired cases for the main scalability comparison. Present one row per case and method.

Recommended columns:

```text
Case | Pattern | |A^truck| | |A^drone| | |A^top| | Method |
Status | UB | LB | Gap (%) | Time (s) | Root time (s) |
Columns | Labels | Nodes | Core equiv. | Drone sorties
```

Split this into two tables if necessary for readability.

The stored large-case campaign currently supports these statements:

- all eight BPC roots closed within one hour under pricing tolerance 0.01;
- BPC closed the full tree in four cases and reached the time limit in four cases;
- BPC found a lower incumbent than direct compact Gurobi in six of eight cases;
- all BPC and compact incumbents pass the stored service-window and payload checks;
- every reported large solution uses four drone sorties, exactly the structurally required number; and
- the BPC process tree used a mean of about 1.08--6.04 CPU-core equivalents across individual large cases, showing substantial variation in parallel workload availability.

Do not headline the arithmetic mean of the large relative gaps. Several lower bounds are negative while incumbents are small and positive, making the conventional relative gap exceed 100% and rendering its mean difficult to interpret. Report per-case gaps, the number of closed cases, and optionally the median gap among time-limited cases.

### 6.5 Root versus post-root proof throughput

The current algorithm paper places significant emphasis on pricing and post-root proof throughput. Add a BPC-only diagnostic table for the large cases with:

- root closure time;
- root standard/Farkas pricing time;
- post-root standard/Farkas pricing time;
- RMP build/update/solve time;
- SR separation and coefficient-build time;
- forward labels generated;
- labels pruned and dominated;
- dynamic split candidates and committed splits;
- worker busy/idle seconds;
- effective core equivalent;
- active SR cuts;
- inserted columns; and
- open nodes at termination.

Use these diagnostics to identify whether each time-limited case is limited by root pricing, child certification, RMP/SR maintenance, or tree size. Do not infer a bottleneck from wall-clock time alone.

### 6.6 Operational outcomes

Report the following components separately from the normalized objective:

- mean and maximum customer delay;
- sum of squared delays;
- number of trucks used;
- truck return times;
- operating cost;
- truck-served customers;
- drone-served customers; and
- promised-window slack.

Because drone use is structurally required, phrase the findings as route-structure descriptions rather than evidence that the objective naturally prefers drones. For example:

> Across the baseline campaign, the mandatory-drone construction requires one, two, and four drone-served customers in the small, medium, and large cases. The computational results therefore evaluate how the exact algorithm integrates these required drone-service blocks into truck routes, rather than whether a truck-only solution should be replaced by drone service.

## 7. Paper-ready setup text

The following text can be adapted directly into the manuscript.

```latex
\section{Computational Experiments}\label{sec:exp}

\subsection{Experimental Design and Implementation}
We evaluate the proposed BPC algorithm on generated truck-as-hub instances with purely sparse (PS) and purely clustered (PC) customer layouts. Customer coordinates lie in the square $[0,14]\times[0,14]$. In PS instances, coordinates are sampled independently from the uniform distribution over this square. In PC instances, we generate $\max\{1,\lfloor |\CustNodes|/5\rfloor\}$ cluster centers and sample customers around these centers using truncated Gaussian perturbations. The source and sink depot copies are located at $(0,0)$, and two synchronization pads are placed at K-means centroids of the customer coordinates. Truck travel times use Manhattan distances at 40 km/h, whereas drone flight times use Euclidean distances at 100 km/h.

We consider small, medium, and large instances with $(|\CustNodes|,|\Trucks|,|\HubNodes|,\DroneFleetSize)$ equal to $(5,2,2,4)$, $(10,3,2,4)$, and $(25,5,2,4)$, respectively. For each scale, PS instances use seeds 1--4 and PC instances use seeds 5--8, yielding 24 instances. Each instance is solved by the proposed BPC algorithm and by Gurobi applied directly to the compact arc-based MIQP.

The truck and drone payload capacities are 50 kg and 6 kg, respectively, and the drone round-trip endurance is 75 minutes. The fixed truck-deployment and drone-sortie costs are 20 and 1. We use objective weights $(\WeightDelay,\WeightReturn,\WeightCost)=(0.4,0.3,0.3)$. Internal customer-to-customer truck arcs are sampled with probability 0.05, while truck arcs incident to a synchronization pad are sampled with probability 0.18. To obtain feasible drone-required benchmarks, we designate $\lceil0.16|\CustNodes|\rceil$ customers as mandatory drone customers, retain one protected launch-pad arc for each such customer, and construct a deterministic feasible witness route set. Optional drone arcs are disabled in the baseline experiment.

Every customer receives a promised-service upper bound. Starting from the relaxed earliest-service benchmark $\arriveLB{n}$, we draw a deterministic offset from $[30,90]$ minutes and set the requested bound to the larger of $\arriveLB{n}$ plus this offset and the service time in a deadline-free witness plus a 3-minute slack. This construction preserves at least one feasible witness while producing customer-specific hard service envelopes.

All runs use a 3,600-second wall-clock limit. Gurobi uses one thread in both the RMP and direct compact benchmark. BPC uses 12 persistent pricing processes, a root constructive limit of 5 seconds, a 60-second compact-solve warm-start limit, and a 2-second full route-pool integer-search limit per call. The implementation uses pricing tolerance $\PricingTol=0.01$, cut tolerance $10^{-7}$, and integrality tolerance $10^{-6}$. Consequently, an ``optimal'' BPC status denotes closure relative to the configured reduced-cost tolerance; mathematical exactness corresponds to zero pricing tolerance.
```

Replace or supplement the final paragraph with verified hardware memory and the precise operating-system description before submission.

## 8. Recommended tables

### Table 1: Experiment profiles

```latex
\begin{table}[htbp]
\centering
\caption{Computational experiment profiles}
\label{tab:experiment-profiles}
\begin{tabular}{lrrrrrr}
\toprule
Scale & Customers & Trucks & Hubs & Drones/truck & Mandatory drone & Instances \\
\midrule
Small  & 5  & 2 & 2 & 4 & 1 & 8 \\
Medium & 10 & 3 & 2 & 4 & 2 & 8 \\
Large  & 25 & 5 & 2 & 4 & 4 & 8 \\
\bottomrule
\end{tabular}
\end{table}
```

### Table 2: Baseline parameters

Include separate panels for physical parameters, graph generation, service windows, objective weights, and solver controls. This is clearer than embedding all settings in prose.

### Table 3: Small/medium validation

Aggregate by scale and method, reporting cases, zero-gap cases, mean runtime, mean nodes, and paired objective agreement. Keep the case-level data in an online appendix if page limits are tight.

### Table 4: Large paired results

Report all eight cases individually. Do not average statuses. Mark threshold-closed BPC cases and time-limited cases explicitly.

### Table 5: Large BPC proof-throughput diagnostics

Report root closure, pricing labels, pricing time, RMP time, SR time, worker utilization, dynamic splits, and open nodes. This table directly supports the paper's methodological claims about balanced process-based pricing and exact candidate management.

## 9. Artifact mapping

Use the following project artifacts as data sources.

### Small and medium cases

```text
numerical_experiments/ps_pc_bpc_compact_24case_1h_campaign/
```

Use only the `small_*` and `medium_*` rows from this folder. Its large rows are 20-customer historical cases and must not be combined with the current 25-customer large definition.

Relevant files:

- `campaign_manifest.json`: authoritative dimensions, seeds, weights, and configurations;
- `campaign_summary.csv`: concise run outcomes;
- `case_comparison.csv`: paired BPC/compact comparison;
- `bpc_full_stats.csv`: complete BPC diagnostics;
- `compact_full_stats.csv`: complete compact diagnostics;
- `bpc_pricing_calls.csv`: pricing-call diagnostics;
- `bpc_tree_evolution.csv`: root/post-root events; and
- `verification_report.json`: feasibility and configuration checks.

### Large cases

```text
numerical_experiments/ps_pc_large25_bpc_compact_8case_1h_campaign/
```

Use this folder for all 25-customer results. Do not substitute historical large20 or one-off tuning runs.

### Recommended publication rerun

For a final journal submission, run one unified 24-case campaign under a frozen code commit and write all small, medium, and large outputs to a new folder. The current usable evidence is split across two campaigns, and the solver has undergone correctness and scheduling revisions over time. A unified rerun avoids ambiguity about code version and ensures that timing comparisons use one implementation snapshot.

Record in the manuscript or online supplement:

- Git commit hash;
- complete command line;
- campaign manifest;
- Gurobi and Python versions;
- CPU and RAM;
- per-run time limit and thread/process settings; and
- SHA-256 inventory of result files.

## 10. Current evidence and publication cautions

### 10.1 Evidence currently suitable for descriptive use

- Paired configurations match for the stored campaigns.
- Stored BPC incumbents pass coverage, service-window, and payload checks.
- Stored compact incumbents pass service-window and payload checks.
- Small/medium paired objectives agree within numerical precision.
- All eight large BPC roots close under tolerance 0.01.
- Four large BPC trees close under tolerance 0.01 and four reach the one-hour limit.
- The corrected large PC seed-5 run has incumbent `0.1658971648`, lower bound `0.1619182503`, and gap `2.3984%`.

### 10.2 Unresolved large PC seed-8 inconsistency

The stored large PC seed-8 artifacts contain a formulation-equivalence contradiction:

- direct compact Gurobi reports an optimal objective of approximately `0.1189170326`;
- BPC reports a verified route-space objective of approximately `0.1097167853` on the matched instance configuration.

If both formulations and objective accounting are equivalent, a feasible BPC objective cannot be below a proven compact optimum. This case must be investigated before the paper claims compact/route-space equivalence over the complete large benchmark or reports a comparative win/loss count as definitive.

Until resolved:

- flag seed 8 in the large table;
- do not use it to claim BPC optimality or superiority;
- do not state that direct compact Gurobi proved the common problem optimum; and
- do not aggregate it into claims of formulation agreement.

This caution is not optional. It concerns formulation correctness, not merely solver performance.

### 10.3 Threshold-relative status

Every BPC result labeled `optimal` in the campaign uses pricing tolerance 0.01. Use “threshold-closed” or “optimal under the configured reduced-cost tolerance” in tables and prose.

### 10.4 Direct compact status

The compact wrapper field `status=success` is not a mathematical solve status. Read the underlying Gurobi status, bound, and gap. Seven large direct compact runs reached the one-hour limit; only PC seed 8 reported zero gap, and that case is currently inconsistent with the BPC objective.

## 11. Sensitivity and ablation experiments needed for the revised paper

The original objective-weight sensitivity figures cannot be reused because the current objective has three independently normalized terms and the baseline generator structurally requires drone service.

If a sensitivity section is retained, rerun one or more of the following designs.

### 11.1 Objective-weight sensitivity

Use weight triples that sum to one, for example:

```text
(0.6, 0.2, 0.2), (0.4, 0.3, 0.3), (0.2, 0.4, 0.4),
(0.4, 0.5, 0.1), (0.4, 0.1, 0.5).
```

Report delay, return time, operating cost, truck count, and route structure. Do not collapse the current objective into the original single `ObjWeight` parameter.

### 11.2 Service-window sensitivity

Vary the random offset interval while retaining the witness mechanism, for example `[15,60]`, `[30,90]`, and `[45,120]`. Report feasibility lifts, root closure time, pricing labels, and solution quality.

### 11.3 Drone-policy sensitivity

To support operational claims about drone adoption, compare:

- mandatory fraction 0 with optional arcs enabled;
- mandatory fraction 0.08;
- baseline mandatory fraction 0.16; and
- a higher mandatory fraction, provided the hub-access caps permit generation.

This separates the computational effect of drone-required customers from the endogenous value of drone service.

### 11.4 Algorithmic ablation

The current paper makes implementation claims about parallel pricing, dominance, reduced-cost pruning, active-SR coefficient caching, and incremental RMP construction. A defensible ablation should compare at least:

- serial forward pricing versus balanced 12-process pricing;
- dynamic splitting disabled versus enabled;
- physical-location dominance disabled versus enabled;
- deadline-aware pruning disabled versus enabled; and
- active-SR/incremental-RMP caching disabled versus enabled.

Use identical instances and report wall time, process-tree CPU time, labels, dominance tests, columns, root closure, and final lower bound. A component should be described as an efficiency improvement only if this controlled experiment supports the claim.

## 12. Recommended claims hierarchy

Structure conclusions in decreasing order of evidential strength.

1. **Implementation validation:** small and medium paired objectives agree with direct compact Gurobi.
2. **Scalability evidence:** the BPC method closes every 25-customer root under tolerance 0.01 and closes four full trees within one hour.
3. **Primal performance:** BPC obtains lower incumbents in six of eight stored large comparisons, subject to the seed-8 inconsistency caveat.
4. **Proof-throughput diagnosis:** time-limited cases can be characterized using root/post-root pricing, RMP, SR, and worker-utilization diagnostics.
5. **Operational structure:** generated feasible solutions integrate the required drone-service blocks while satisfying promised windows.

Do not claim that the experiments establish general superiority over Gurobi, universal large-scale exactness, or endogenous preference for drones.

## 13. Final revision checklist

- [ ] Insert Computational Experiments and Conclusions before `APPENDICES`.
- [ ] Replace 15-customer medium with 10-customer medium.
- [ ] Replace old fleet sizes with 2/3/5 trucks.
- [ ] Use PS seeds 1--4 and PC seeds 5--8.
- [ ] Remove mixed-layout claims unless new runs are produced.
- [ ] State depot location `(0,0)` and hub K-means placement.
- [ ] Use truck/drone speeds 40/100 and payloads 50/6.
- [ ] Use drone endurance 75 and costs 20/1.
- [ ] Use objective weights 0.4/0.3/0.3.
- [ ] Describe sparse arc probabilities 0.05/0.18.
- [ ] Describe the 16% mandatory-drone policy and deterministic feasibility witness.
- [ ] State that optional drone arcs are disabled.
- [ ] Describe random-absolute windows `[30,90]` with witness slack 3.
- [ ] Report full normalized objectives, including the constant shift.
- [ ] Use one-hour paired limits and one Gurobi thread.
- [ ] Report 12 process pricing workers and process-tree CPU utilization.
- [ ] Qualify BPC closure by pricing tolerance 0.01.
- [ ] Translate compact wrapper `success` into actual Gurobi termination status.
- [ ] Separate small/medium validation from large proof-throughput evidence.
- [ ] Do not reuse the original sensitivity figures.
- [ ] Resolve or explicitly quarantine large PC seed 8.
- [ ] Confirm RAM and freeze a code commit before the publication campaign.
- [ ] Prefer a unified 24-case rerun for final reported timing results.

