\ SCIP STATISTICS
\   Problem name     : RMP
\   Variables        : 4 (4 binary, 0 integer, 0 implicit integer, 0 continuous)
\   Constraints      : 5
Minimize
 Obj: +1684 z_0 +487 z_1 +2306 z_2 +911 z_3
Subject to
 visit_customer_0: +1 z_0 = +1
 visit_customer_1: +1 z_1 = +1
 visit_customer_2: +1 z_2 = +1
 visit_customer_3: +1 z_3 = +1
 truck_fleet: +1 z_0 +1 z_1 +1 z_2 +1 z_3 <= +3
Bounds
 0 <= z_0 <= 1
 0 <= z_1 <= 1
 0 <= z_2 <= 1
 0 <= z_3 <= 1
Binaries
 z_0 z_1 z_2 z_3
End
