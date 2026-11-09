
### benes_128x128_flat contribution2: 20480 attempts, 23 s search, failed attempts 0 s (0 %)
| kind | budget | outcome | n | seconds | expansions |
|---|---|---|---|---|---|
| plain | 2000000 | routed | 20480 | 23 | 35.03 M |
successful first attempts: n=3264 median 0.00 M, p90 0.05 M, max 0.12 M, above 1 M: 0, above 0.5 M: 0

### benes_32x32_flat contribution1: 932 attempts, 305 s search, failed attempts 67 s (22 %)
| kind | budget | outcome | n | seconds | expansions |
|---|---|---|---|---|---|
| plain_retry | 10000000 | routed | 36 | 131 | 59.84 M |
| plain | 2000000 | routed | 860 | 106 | 55.34 M |
| plain | 2000000 | failed | 36 | 67 | 0.28 M |
retry after a failed first attempt: {'routed': 36}
successful first attempts: n=390 median 0.00 M, p90 0.57 M, max 1.50 M, above 1 M: 18, above 0.5 M: 48
successful retries: n=36 median 1.63 M, p90 3.46 M, max 5.03 M, above 2 M: 17, above 5 M: 1

### benes_32x32_flat lidar-pure: 948 attempts, 432 s search, failed attempts 106 s (25 %)
| kind | budget | outcome | n | seconds | expansions |
|---|---|---|---|---|---|
| plain_retry | 10000000 | routed | 52 | 222 | 113.09 M |
| plain | 2000000 | failed | 52 | 106 | 0.00 M |
| plain | 2000000 | routed | 844 | 104 | 58.42 M |
retry after a failed first attempt: {'routed': 52}
successful first attempts: n=374 median 0.00 M, p90 0.81 M, max 1.64 M, above 1 M: 28, above 0.5 M: 51
successful retries: n=52 median 2.17 M, p90 4.20 M, max 5.30 M, above 2 M: 30, above 5 M: 2

### multiportmmi_128x128 contribution2: 3745 attempts, 132 s search, failed attempts 4 s (3 %)
| kind | budget | outcome | n | seconds | expansions |
|---|---|---|---|---|---|
| plain | 2000000 | routed | 3626 | 121 | 72.69 M |
| plain | 30000000 | routed | 59 | 7 | 8.06 M |
| plain | 30000000 | failed | 40 | 4 | 4.96 M |
| post_ripup_plain | 30000000 | routed | 5 | 1 | 0.57 M |
| post_ripup_plain | 30000000 | failed | 6 | 1 | 0.74 M |
| post_ripup_plain | 2000000 | routed | 3 | 0 | 0.30 M |
| plain | 2000000 | failed | 3 | 0 | 0.02 M |
| plain_retry | 10000000 | failed | 3 | 0 | 0.02 M |
retry after a failed first attempt: {'failed': 3}
successful first attempts: n=590 median 0.13 M, p90 0.20 M, max 1.06 M, above 1 M: 4, above 0.5 M: 32

### multiportmmi_32x32 lidar-pure: 458 attempts, 169 s search, failed attempts 29 s (17 %)
| kind | budget | outcome | n | seconds | expansions |
|---|---|---|---|---|---|
| plain | 2000000 | routed | 436 | 84 | 50.55 M |
| plain_retry | 10000000 | routed | 11 | 56 | 30.04 M |
| plain | 2000000 | failed | 11 | 29 | 0.00 M |
retry after a failed first attempt: {'routed': 11}
successful first attempts: n=181 median 0.15 M, p90 0.74 M, max 1.62 M, above 1 M: 12, above 0.5 M: 27
successful retries: n=11 median 1.99 M, p90 4.52 M, max 6.99 M, above 2 M: 5, above 5 M: 1

### multiportmmi_64x64 contribution1: 1005 attempts, 476 s search, failed attempts 130 s (27 %)
| kind | budget | outcome | n | seconds | expansions |
|---|---|---|---|---|---|
| plain_retry | 10000000 | routed | 24 | 170 | 77.96 M |
| plain | 2000000 | routed | 855 | 100 | 68.17 M |
| plain | 2000000 | failed | 40 | 92 | 15.98 M |
| plain | 30000000 | routed | 50 | 70 | 34.46 M |
| plain_retry | 10000000 | failed | 16 | 38 | 17.07 M |
| post_ripup_crossing_free | 2000000 | routed | 15 | 4 | 3.14 M |
| post_ripup_guided | 2000000 | routed | 1 | 2 | 0.88 M |
| post_ripup_crossing_free | 30000000 | routed | 1 | 0 | 0.33 M |
| plain | 30000000 | failed | 3 | 0 | 0.11 M |
retry after a failed first attempt: {'failed': 16, 'routed': 24}
successful first attempts: n=439 median 0.09 M, p90 0.56 M, max 14.19 M, above 1 M: 20, above 0.5 M: 49
successful retries: n=24 median 3.42 M, p90 5.51 M, max 6.30 M, above 2 M: 20, above 5 M: 3

### multiportmmi_64x64 lidar-pure: 1068 attempts, 1462 s search, failed attempts 453 s (31 %)
| kind | budget | outcome | n | seconds | expansions |
|---|---|---|---|---|---|
| plain_retry | 10000000 | routed | 44 | 344 | 164.37 M |
| plain | 30000000 | routed | 94 | 335 | 152.41 M |
| plain | 2000000 | routed | 836 | 258 | 142.81 M |
| plain_retry | 10000000 | failed | 15 | 221 | 0.00 M |
| plain | 2000000 | failed | 59 | 185 | 0.00 M |
| plain | 30000000 | failed | 3 | 43 | 0.00 M |
| post_ripup_guided | 30000000 | routed | 1 | 38 | 13.59 M |
| post_ripup_plain | 2000000 | routed | 10 | 20 | 9.93 M |
| post_ripup_guided | 2000000 | routed | 4 | 10 | 4.42 M |
| post_ripup_plain | 30000000 | routed | 1 | 4 | 1.94 M |
| post_ripup_guided | 2000000 | failed | 1 | 4 | 0.00 M |
retry after a failed first attempt: {'failed': 15, 'routed': 44}
successful first attempts: n=464 median 0.34 M, p90 1.26 M, max 14.84 M, above 1 M: 78, above 0.5 M: 170
successful retries: n=44 median 3.47 M, p90 6.33 M, max 7.73 M, above 2 M: 37, above 5 M: 11

### ALL CELLS: 42572 attempts, 3210 s search, failed attempts 807 s (25 %)
| kind | budget | outcome | n | seconds | expansions |
|---|---|---|---|---|---|
| plain_retry | 10000000 | routed | 189 | 956 | 463.72 M |
| plain | 2000000 | routed | 41791 | 955 | 592.57 M |
| plain | 2000000 | failed | 231 | 496 | 17.47 M |
| plain | 30000000 | routed | 216 | 413 | 196.19 M |
| plain_retry | 10000000 | failed | 42 | 260 | 18.22 M |
| plain | 30000000 | failed | 47 | 47 | 5.07 M |
| post_ripup_guided | 30000000 | routed | 2 | 38 | 13.64 M |
| post_ripup_plain | 2000000 | routed | 15 | 20 | 10.23 M |
| post_ripup_guided | 2000000 | routed | 5 | 12 | 5.30 M |
| post_ripup_plain | 30000000 | routed | 6 | 5 | 2.51 M |
| post_ripup_guided | 2000000 | failed | 1 | 4 | 0.00 M |
| post_ripup_crossing_free | 2000000 | routed | 20 | 4 | 3.41 M |
| post_ripup_plain | 30000000 | failed | 6 | 1 | 0.74 M |
| post_ripup_crossing_free | 30000000 | routed | 1 | 0 | 0.33 M |
retry after a failed first attempt: {'routed': 189, 'failed': 42}
successful first attempts: n=9164 median 0.00 M, p90 0.16 M, max 14.84 M, above 1 M: 180, above 0.5 M: 430
successful retries: n=189 median 2.31 M, p90 4.99 M, max 7.73 M, above 2 M: 113, above 5 M: 18
