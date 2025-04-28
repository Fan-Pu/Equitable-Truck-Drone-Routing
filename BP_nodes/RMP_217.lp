\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  1690 z_0 + 503 z_1 + 2288 z_2 + 906 z_3 + 549 z_4 + 1381 z_5 + 725 z_6
   + 2009 z_7 + 1698 z_29 + 2606 z_13 + 1643 z_12 + 1820 z_27 + 2427 z_23
   + 2661 z_22 + 2484 z_18
Subject To
 visit_customer_0: z_0 + z_13 + z_27 + z_22 = 1
 visit_customer_1: z_1 = 1
 visit_customer_2: z_2 + z_13 + z_12 + z_23 + z_18 = 1
 visit_customer_3: z_3 + z_13 + z_12 + z_23 + z_18 = 1
 visit_customer_4: z_4 = 1
 visit_customer_5: z_5 + z_29 + z_22 + z_18 = 1
 visit_customer_6: z_6 = 1
 visit_customer_7: z_7 + z_29 + z_27 + z_23 + z_22 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_29
   + z_13 + z_12 + z_27 + z_23 + z_22 + z_18 <= 8
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_29
   + z_13 + z_12 + z_27 + z_23 + z_22 + z_18 >= 0
Bounds
End
