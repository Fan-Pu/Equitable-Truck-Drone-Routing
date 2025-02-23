\ SCIP STATISTICS
\   Problem name     : RMP
\   Variables        : 9 (9 binary, 0 integer, 0 implicit integer, 0 continuous)
\   Constraints      : 6
Minimize
 Obj: +2442 z_0 +1665 z_1 +512 z_2 +2297 z_3 +902 z_4 +542 z_5 +1375 z_6 +751 z_7 +2003 z_8
Subject to
 visit_customer_0: +1 z_0 +1 z_1 = +1
 visit_customer_1: +1 z_0 +1 z_2 = +1
 visit_customer_2: +1 z_0 +1 z_3 = +1
 visit_customer_3: +1 z_0 +1 z_4 = +1
 visit_customer_4: +1 z_0 +1 z_5 = +1
 truck_fleet: +1 z_0 +1 z_1 +1 z_2 +1 z_3 +1 z_4 +1 z_5 +1 z_6 +1 z_7 +1 z_8 <= +2
Bounds
 0 <= z_0 <= 1
 0 <= z_1 <= 1
 0 <= z_2 <= 1
 0 <= z_3 <= 1
 0 <= z_4 <= 1
 0 <= z_5 <= 1
 0 <= z_6 <= 1
 0 <= z_7 <= 1
 0 <= z_8 <= 1
Binaries
 z_0 z_1 z_2 z_3 z_4 z_5 z_6 z_7 z_8
End
