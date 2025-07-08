\ Model RMP
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0x729f1da1f56d9f5e
Minimize
  20.51904304975404 z_0 + 20.32907659024284 z_1 + 20.74996639227064 z_2
   + 20.19615813377366 z_3 + 20.5265754397576 z_4 + 20.29575656767596 z_5
   + 20.75655365057085 z_6 + 20.11170021642177 z_7 + 20.57064379964572 z_8
   + 20.30328812059156 z_9 + 20.76892947660662 z_10
   + 20.12001174817001 z_11 + 20.59181991173848 z_12
   + 20.30045074695295 z_13 + 20.69070806419717 z_14
   + 20.09988567250458 z_15 + 20.55514196245476 z_16
   + 20.31941290369857 z_17 + 20.71453855601379 z_18
   + 20.11605829774144 z_19 + 21.59834930492535 z_20
   + 20.98667411274891 z_21 + 20.30545847572302 z_22
   + 21.55694261055246 z_24
Subject To
 visit_customer_0: z_0 + z_24 = 1
 visit_customer_1: z_1 + z_20 = 1
 visit_customer_2: z_2 + z_20 = 1
 visit_customer_3: z_3 + z_22 = 1
 visit_customer_4: z_4 + z_24 = 1
 visit_customer_5: z_5 + z_21 = 1
 visit_customer_6: z_6 + z_21 = 1
 visit_customer_7: z_7 + z_21 = 1
 visit_customer_8: z_8 + z_24 = 1
 visit_customer_9: z_9 + z_22 = 1
 visit_customer_10: z_10 + z_20 = 1
 visit_customer_11: z_11 + z_21 = 1
 visit_customer_12: z_12 + z_21 = 1
 visit_customer_13: z_13 + z_22 = 1
 visit_customer_14: z_14 + z_21 = 1
 visit_customer_15: z_15 + z_24 = 1
 visit_customer_16: z_16 + z_24 = 1
 visit_customer_17: z_17 + z_20 = 1
 visit_customer_18: z_18 + z_24 = 1
 visit_customer_19: z_19 + z_21 = 1
 truck_fleet_ub: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   + z_10 + z_11 + z_12 + z_13 + z_14 + z_15 + z_16 + z_17 + z_18 + z_19
   + z_20 + z_21 + z_22 + z_24 <= 5
 truck_fleet_lb: z_0 + z_1 + z_2 + z_3 + z_4 + z_5 + z_6 + z_7 + z_8 + z_9
   + z_10 + z_11 + z_12 + z_13 + z_14 + z_15 + z_16 + z_17 + z_18 + z_19
   + z_20 + z_21 + z_22 + z_24 >= 0
 SR_('C1',_'C2',_'C11'): z_20 <= 1
 SR_('C1',_'C2',_'C17'): z_24 <= 1
 SR_('C1',_'C2',_'C16'): z_24 <= 1
 SR_('C1',_'C2',_'C5'): z_24 <= 1
 SR_('C1',_'C2',_'C3'): z_20 <= 1
 SR_('C1',_'C2',_'C18'): z_20 <= 1
 SR_('C1',_'C2',_'C19'): z_24 <= 1
 SR_('C1',_'C3',_'C5'): z_24 <= 1
 SR_('C1',_'C3',_'C9'): z_24 <= 1
 SR_('C1',_'C2',_'C9'): z_24 <= 1
Bounds
End
