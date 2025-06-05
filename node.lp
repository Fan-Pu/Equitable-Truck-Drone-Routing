\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  33.51479429905829 z_0 + 22.4355854997464 z_1 + 23.26635918214082 z_2
   + 34.76972700570076 z_3 + 29.6824860428295 z_4 + 28.10290549335098 z_5
   + 36.42437420032132 z_6 + 33.3454818000891 z_7 + 27.08230752030633 z_8
   + 34.76237933811839 z_9 + 23.71214681535487 z_10
   + 32.73310880454923 z_11 + 31.0294000681175 z_12
   + 34.09058625291169 z_13 + 28.02576509891071 z_14
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
 visit_customer_10: z_10 = 1
 visit_customer_11: z_11 = 1
 visit_customer_12: z_12 = 1
 visit_customer_13: z_13 = 1
 visit_customer_14: z_14 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   + z_10 + z_11 + z_12 + z_13 + z_14 <= 4
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   + z_10 + z_11 + z_12 + z_13 + z_14 >= 0
Bounds
End
