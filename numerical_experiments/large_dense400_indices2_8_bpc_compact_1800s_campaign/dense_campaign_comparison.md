# Dense Large Indices 2–8 Solver Comparison

PC6 and PC8 are warm-start-filtered replacements. PC8 is additionally arc- and performance-filtered; these are not unbiased draws comparable to the other cases.

| Case | Realized seed | Filtered | Arcs | BPC status | BPC time | BPC gap | Compact status | Compact time | Compact gap |
|---|---:|---|---:|---|---:|---:|---|---:|---:|
| large_PS_seed2 | 2 | False | 404 | time_limit | 1800.477 | 0.018051 | time_limit_with_incumbent | 1800.319 | 1.852138 |
| large_PS_seed3 | 3 | False | 360 | optimal | 254.873 | 0.000000 | time_limit_with_incumbent | 1800.326 | 1.516296 |
| large_PS_seed4 | 4 | False | 346 | optimal | 114.081 | 0.000000 | time_limit_with_incumbent | 1800.316 | 1.385794 |
| large_PC_seed5 | 5 | False | 545 | optimal | 1086.791 | 0.000000 | time_limit_with_incumbent | 1800.398 | 2.604631 |
| large_PC_seed6 | 12000042 | True | 317 | optimal | 150.240 | 0.000000 | time_limit_with_incumbent | 1800.371 | 1.423834 |
| large_PC_seed7 | 7 | False | 317 | optimal | 128.725 | 0.000000 | time_limit_with_incumbent | 1800.358 | 0.842090 |
| large_PC_seed8 | 24000080 | True | 438 | time_limit | 1802.156 | 0.035842 | time_limit_with_incumbent | 1800.298 | 1.964463 |
