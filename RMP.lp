\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  1681 z_0 + 494 z_1 + 2292 z_2 + 913 z_3 + 555 z_4 + 1355 z_5 + 731 z_6
   + 2024 z_7
Subject To
 visit_customer_0: z_0 = 1
 visit_customer_1: z_1 = 1
 visit_customer_2: z_2 = 1
 visit_customer_3: z_3 = 1
 visit_customer_4: z_4 = 1
 visit_customer_5: z_5 = 1
 visit_customer_6: z_6 = 1
 visit_customer_7: z_7 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 <= 5
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 >= 0
Bounds
End
