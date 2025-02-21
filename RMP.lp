\ SCIP STATISTICS
\   Problem name     : RMP
\   Variables        : 6 (6 binary, 0 integer, 0 implicit integer, 0 continuous)
\   Constraints      : 4
Minimize
 Obj: +1959 z_0 +1676 z_1 +488 z_2 +2300 z_3 +895 z_4 +556 z_5
Subject to
 visit_customer_0: +1 z_0 +1 z_1 = +1
 visit_customer_1: +1 z_0 +1 z_2 = +1
 visit_customer_2: +1 z_0 +1 z_3 = +1
 truck_fleet: +1 z_0 +1 z_1 +1 z_2 +1 z_3 +1 z_4 +1 z_5 <= +2
Bounds
 0 <= z_0 <= 1
 0 <= z_1 <= 1
 0 <= z_2 <= 1
 0 <= z_3 <= 1
 0 <= z_4 <= 1
 0 <= z_5 <= 1
Binaries
 z_0 z_1 z_2 z_3 z_4 z_5
End
