\ SCIP STATISTICS
\   Problem name     : RMP
\   Variables        : 3 (3 binary, 0 integer, 0 implicit integer, 0 continuous)
\   Constraints      : 4
Minimize
 Obj: +1676 z_0 +488 z_1 +2300 z_2
Subject to
 visit_customer_0: +1 z_0 = +1
 visit_customer_1: +1 z_1 = +1
 visit_customer_2: +1 z_2 = +1
 truck_fleet: +1 z_0 +1 z_1 +1 z_2 <= +2
Bounds
 0 <= z_0 <= 1
 0 <= z_1 <= 1
 0 <= z_2 <= 1
Binaries
 z_0 z_1 z_2
End
