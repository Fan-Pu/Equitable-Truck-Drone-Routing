# Medium-Scale Crossover Study

| Customers | BPC median time | MIQP median time | BPC optimal | MIQP optimal | BPC gap | MIQP gap | Qualifies |
|---:|---:|---:|---:|---:|---:|---:|---|
| 15 | 15.453 | 1.928 | 4 | 4 | 0.000000 | 0.000000 | False |
| 20 | 54.168 | 317.282 | 4 | 2 | 0.000000 | 0.217269 | True |
| 25 | 132.400 | 600.193 | 4 | 0 | 0.000000 | 1.508876 | True |

Selected configuration: `{'num_customers': 20, 'num_trucks': 4, 'num_hubs': 3, 'drones_per_truck': 4}`.

Held-out final cases: `8`.
