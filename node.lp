\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0xbe1b2132164d21d6
Minimize
  31.94213823051969 z_0 + 23.17569294821644 z_1 + 22.51049209865207 z_2
   + 25.58411985341755 z_3 + 29.82363418645728 z_4 + 31.59634715540057 z_5
   + 31.36698170428586 z_6 + 27.32747858545145 z_7 + 30.24794362353522 z_8
   + 23.55412849064118 z_9
Subject To
 visit_customer_0: z_0 = 1
 visit_customer_1: z_1 = 1
 visit_customer_2: z_2 = 1
 visit_customer_3: z_3 = 1
 visit_customer_4: z_4 = 1
 visit_customer_5: z_5 = 1
 visit_customer_6: z_6 = 1
 visit_customer_7: z_7 = 1
 visit_customer_8: z_8 = 1
 visit_customer_9: z_9 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   <= 2
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   >= 0
Bounds
End
