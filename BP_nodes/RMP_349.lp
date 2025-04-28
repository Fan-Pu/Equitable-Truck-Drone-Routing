\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  1690 z_0 + 503 z_1 + 2288 z_2 + 906 z_3 + 549 z_4 + 1381 z_5 + 725 z_6
   + 2009 z_7 + 69 z_45 + 800 z_15 + 2718 z_14 + 1036 z_8 + 1765 z_10
   + 1755 z_48 + 2549 z_51 + 2539 z_49
Subject To
 visit_customer_0: z_0 + z_14 + z_8 + z_10 + z_51 = 1
 visit_customer_1: z_1 = 1
 visit_customer_2: z_2 + z_14 + z_48 + z_49 = 1
 visit_customer_3: z_3 + z_15 + z_10 + z_51 = 1
 visit_customer_4: z_4 = 1
 visit_customer_5: z_5 + z_14 + z_48 + z_49 = 1
 visit_customer_6: z_6 = 1
 visit_customer_7: z_7 + z_51 + z_49 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_45
   + z_15 + z_14 + z_8 + z_10 + z_48 + z_51 + z_49 <= 8
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_45
   + z_15 + z_14 + z_8 + z_10 + z_48 + z_51 + z_49 >= 5
Bounds
End
