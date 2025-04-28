\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  69 z_45
Subject To
 visit_customer_0: = 1
 visit_customer_1: = 1
 visit_customer_2: = 1
 visit_customer_3: = 1
 visit_customer_4: = 1
 visit_customer_5: = 1
 visit_customer_6: = 1
 visit_customer_7: = 1
 truck_fleet_ub: z_45 <= 8
 truck_fleet_lb: z_45 >= 5
Bounds
End
