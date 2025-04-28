\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  1690 z_0 + 503 z_1 + 2288 z_2 + 906 z_3 + 549 z_4 + 1381 z_5 + 725 z_6
   + 2009 z_7 + 800 z_15 + 1643 z_19 + 2427 z_20 + 1036 z_8 + 1765 z_10
   + 2606 z_16 + 914 z_11 + 1877 z_46 + 2484 z_18 + 2661 z_47 + 1755 z_48
   + 2539 z_49 + 69 z_45 + 2549 z_51
Subject To
 visit_customer_0: z_0 + z_8 + z_10 + z_16 + z_46 + z_47 + z_51 = 1
 visit_customer_1: z_1 = 1
 visit_customer_2: z_2 + z_11 + z_18 + z_48 + z_49 = 1
 visit_customer_3: z_3 + z_15 + z_19 + z_20 + z_10 + z_16 + z_18 + z_51
   = 1
 visit_customer_4: z_4 = 1
 visit_customer_5: z_5 + z_19 + z_20 + z_16 + z_46 + z_18 + z_47 + z_48
   + z_49 = 1
 visit_customer_6: z_6 = 1
 visit_customer_7: z_7 + z_20 + z_47 + z_49 + z_51 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_15
   + z_19 + z_20 + z_8 + z_10 + z_16 + z_11 + z_46 + z_18 + z_47 + z_48
   + z_49 + z_45 + z_51 <= 8
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_15
   + z_19 + z_20 + z_8 + z_10 + z_16 + z_11 + z_46 + z_18 + z_47 + z_48
   + z_49 + z_45 + z_51 >= 5
Bounds
End
