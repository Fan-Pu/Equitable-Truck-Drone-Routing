\ SCIP STATISTICS
\   Problem name     : RMP
\   Variables        : 7 (7 binary, 0 integer, 0 implicit integer, 0 continuous)
\   Constraints      : 5
Minimize
 Obj: +533 z_0 +106 z_1 +86 z_2 +119 z_3 +108 z_4 +85 z_5 +104 z_6
Subject to
 visit_customer_0: +1 z_0 +1 z_1 = +1
 visit_customer_1: +1 z_0 +1 z_2 = +1
 visit_customer_2: +1 z_0 +1 z_3 = +1
 visit_customer_3: +1 z_0 +1 z_4 = +1
 truck_fleet: +1 z_0 +1 z_1 +1 z_2 +1 z_3 +1 z_4 +1 z_5 +1 z_6 <= +2
Bounds
 0 <= z_0 <= 1
 0 <= z_1 <= 1
 0 <= z_2 <= 1
 0 <= z_3 <= 1
 0 <= z_4 <= 1
 0 <= z_5 <= 1
 0 <= z_6 <= 1
Binaries
 z_0 z_1 z_2 z_3 z_4 z_5 z_6
End
