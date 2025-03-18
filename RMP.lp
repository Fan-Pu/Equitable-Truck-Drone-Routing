\ SCIP STATISTICS
\   Problem name     : RMP
\   Variables        : 10 (10 binary, 0 integer, 0 implicit integer, 0 continuous)
\   Constraints      : 11
Minimize
 Obj: +1689 z_0 +485 z_1 +2285 z_2 +914 z_3 +537 z_4 +1375 z_5 +716 z_6 +2003 z_7 +689 z_8 +2569 z_9
Subject to
 visit_customer_0: +1 z_0 = +1
 visit_customer_1: +1 z_1 = +1
 visit_customer_2: +1 z_2 = +1
 visit_customer_3: +1 z_3 = +1
 visit_customer_4: +1 z_4 = +1
 visit_customer_5: +1 z_5 = +1
 visit_customer_6: +1 z_6 = +1
 visit_customer_7: +1 z_7 = +1
 visit_customer_8: +1 z_8 = +1
 visit_customer_9: +1 z_9 = +1
 truck_fleet: +1 z_0 +1 z_1 +1 z_2 +1 z_3 +1 z_4 +1 z_5 +1 z_6 +1 z_7 +1 z_8 +1 z_9 <= +4
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
 0 <= z_9 <= 1
Binaries
 z_0 z_1 z_2 z_3 z_4 z_5 z_6 z_7 z_8 z_9
End
