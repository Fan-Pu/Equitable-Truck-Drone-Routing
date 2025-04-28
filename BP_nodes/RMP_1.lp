\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  1684 z_0 + 487 z_1 + 2306 z_2 + 911 z_3 + 2653 z_4
Subject To
 visit_customer_0: z_0 = 1
 visit_customer_1: z_1 + z_4 = 1
 visit_customer_2: z_2 + z_4 = 1
 visit_customer_3: z_3 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 <= 4
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 >= 0
Bounds
End
