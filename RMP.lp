\ Model model
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  533 z_0 + 106 z_1 + 86 z_2 + 119 z_3 + 108 z_4 + 85 z_5 + 104 z_6
   + 176 z_7 + 200.9999999990675 z_8 + 134 z_9 + 130 z_10 + 130 z_11
Subject To
 visit_customer_0: z_0 + z_1 + z_7 + z_8 + z_9 = 1
 visit_customer_1: z_0 + z_2 + z_8 + z_9 + z_10 + z_11 = 1
 visit_customer_2: z_0 + z_3 + z_7 + z_11 = 1
 visit_customer_3: z_0 + z_4 + z_7 + z_8 + z_10 = 1
 truck_fleet1: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 >= 0
 truck_fleet2: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   + z_10 + z_11 <= 2
Bounds
End
