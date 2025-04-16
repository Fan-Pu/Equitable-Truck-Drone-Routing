\ SCIP STATISTICS
\   Problem name     : RMP
\   Variables        : 8 (8 binary, 0 integer, 0 implicit integer, 0 continuous)
\   Constraints      : 9
Minimize
 Obj: +1681 z_0 +494 z_1 +2292 z_2 +913 z_3 +555 z_4 +1355 z_5 +731 z_6 +2024 z_7
Subject to
 visit_customer_0: +1 z_0 = +1
 visit_customer_1: +1 z_1 = +1
 visit_customer_2: +1 z_2 = +1
 visit_customer_3: +1 z_3 = +1
 visit_customer_4: +1 z_4 = +1
 visit_customer_5: +1 z_5 = +1
 visit_customer_6: +1 z_6 = +1
 visit_customer_7: +1 z_7 = +1
 truck_fleet: +1 z_0 +1 z_1 +1 z_2 +1 z_3 +1 z_4 +1 z_5 +1 z_6 +1 z_7 <= +5
Bounds
 0 <= z_0 <= 1
 0 <= z_1 <= 1
 0 <= z_2 <= 1
 0 <= z_3 <= 1
 0 <= z_4 <= 1
 0 <= z_5 <= 1
 0 <= z_6 <= 1
 0 <= z_7 <= 1
Binaries
 z_0 z_1 z_2 z_3 z_4 z_5 z_6 z_7
End
