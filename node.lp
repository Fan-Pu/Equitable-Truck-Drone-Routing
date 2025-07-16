\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0x72883e6f01274d6b
Minimize
  592.3910693355929 z_0 + 657.8751336756361 z_1 + 365.4233250271506 z_2
   + 391.3867803727544 z_3 + 279.9785322298223 z_4 + 389.3317026369341 z_5
   + 717.9368990287744 z_6 + 307.7245980575969 z_7 + 179.5505939346045 z_8
   + 183.8341424987696 z_9 + 172.7577910325518 z_10
   + 195.3618541576892 z_11 + 184.0638083836123 z_12
   + 167.1527236038434 z_13 + 149.6935460229637 z_14
   + 912.1675329265097 z_15 + 312.0264230121358 z_17
   + 627.6775629121813 z_18 + 597.2423434212478 z_19
Subject To
 visit_customer_0: z_0 + z_19 = 1
 visit_customer_1: z_1 + z_15 = 1
 visit_customer_2: z_2 = 1
 visit_customer_3: z_3 + z_19 = 1
 visit_customer_4: z_4 + z_15 = 1
 visit_customer_5: z_5 + z_18 = 1
 visit_customer_6: z_6 + z_18 = 1
 visit_customer_7: z_7 + z_17 = 1
 visit_customer_8: z_8 + z_17 = 1
 visit_customer_9: z_9 + z_15 = 1
 visit_customer_10: z_10 + z_15 = 1
 visit_customer_11: z_11 + z_15 = 1
 visit_customer_12: z_12 + z_15 = 1
 visit_customer_13: z_13 + z_17 = 1
 visit_customer_14: z_14 + z_15 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   + z_10 + z_11 + z_12 + z_13 + z_14 + z_15 + z_17 + z_18 + z_19 <= 5
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   + z_10 + z_11 + z_12 + z_13 + z_14 + z_15 + z_17 + z_18 + z_19 >= 0
 SR_('C1',_'C2',_'C5'): z_15 <= 1
 SR_('C1',_'C2',_'C11'): z_15 <= 1
 SR_('C1',_'C2',_'C12'): z_15 <= 1
 SR_('C1',_'C2',_'C10'): z_15 <= 1
 SR_('C1',_'C2',_'C4'): z_19 <= 1
 SR_('C1',_'C2',_'C15'): z_15 <= 1
 SR_('C1',_'C3',_'C4'): z_19 <= 1
 SR_('C1',_'C4',_'C6'): z_19 <= 1
 SR_('C1',_'C2',_'C13'): z_15 <= 1
 SR_('C1',_'C4',_'C5'): z_19 <= 1
Bounds
End
