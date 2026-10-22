# ndt3d replay: rates of the status rule (threshold 0.65, verification 15 updates)

## 1. Groups, threshold 0.65

| Runs | updates | OK | not verified | low match | OK while > 1 m off (max in a row) | OK while > 0.3 m off | low match while < 0.15 m off |
|---|---|---|---|---|---|---|---|
| healthy, exact map; 0.5x to 3x speed, sparser lidars (24 runs) | 4084 | 89.5 % | 8.8 % | 1.6 % | 0.0 % | 2.3 % | 0.1 % |
| healthy, 2 or 5 cm noise on the prior map (8 runs) | 984 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.2 % | 0.0 % |
| healthy, scene differs from the exact map (24 runs) | 2952 | 74.7 % | 12.1 % | 13.2 % | 0.0 % | 3.2 % | 11.6 % |
| the same with 2 cm of noise on the prior map (24 runs) | 2952 | 87.7 % | 12.2 % | 0.1 % | 0.0 % | 0.8 % | 0.1 % |
| kidnapped, exact map, every update after the kidnap (8 runs) | 584 | 8.4 % | 0.0 % | 91.6 % | 2.7 % (4) | 7.0 % | 0.0 % |
| kidnapped, 2 or 5 cm map noise (12 runs) | 876 | 24.1 % | 0.0 % | 75.9 % | 6.8 % (15) | 21.5 % | 0.0 % |
| /initialpose 3.6 m and 1 rad off, exact map (4 runs) | 492 | 0.8 % | 8.9 % | 90.2 % | 0.0 % | 0.8 % | 0.0 % |
| the same, 2 or 5 cm map noise (8 runs) | 984 | 58.3 % | 10.5 % | 31.2 % | 0.0 % | 9.8 % | 0.2 % |
| /initialpose 0.3 to 1.0 m off (28 runs) | 3444 | 86.8 % | 12.2 % | 1.0 % | 0.0 % | 4.7 % | 0.0 % |
| pose off the map (4 runs) | 200 | 0.0 % | 2.0 % | 98.0 % | 0.0 % | 0.0 % | 0.0 % |
| base_height 5 or 10 cm wrong, pose fine (8 runs) | 984 | 0.0 % | 0.8 % | 99.2 % | 0.0 % | 0.0 % | 82.2 % |

## 2. The same groups at threshold 0.70

| Runs | updates | OK | not verified | low match | OK while > 1 m off (max in a row) | OK while > 0.3 m off | low match while < 0.15 m off |
|---|---|---|---|---|---|---|---|
| healthy, exact map; 0.5x to 3x speed, sparser lidars (24 runs) | 4084 | 88.9 % | 8.8 % | 2.3 % | 0.0 % | 2.1 % | 0.1 % |
| healthy, 2 or 5 cm noise on the prior map (8 runs) | 984 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.2 % | 0.0 % |
| healthy, scene differs from the exact map (24 runs) | 2952 | 48.1 % | 11.3 % | 40.6 % | 0.0 % | 2.0 % | 36.1 % |
| the same with 2 cm of noise on the prior map (24 runs) | 2952 | 87.2 % | 12.0 % | 0.8 % | 0.0 % | 0.8 % | 0.8 % |
| kidnapped, exact map, every update after the kidnap (8 runs) | 584 | 4.5 % | 0.0 % | 95.5 % | 0.9 % (1) | 3.1 % | 0.0 % |
| kidnapped, 2 or 5 cm map noise (12 runs) | 876 | 13.2 % | 0.0 % | 86.8 % | 4.6 % (10) | 10.6 % | 0.0 % |
| /initialpose 3.6 m and 1 rad off, exact map (4 runs) | 492 | 0.0 % | 3.5 % | 96.5 % | 0.0 % | 0.0 % | 0.0 % |
| the same, 2 or 5 cm map noise (8 runs) | 984 | 50.5 % | 7.0 % | 42.5 % | 0.0 % | 2.2 % | 0.3 % |
| /initialpose 0.3 to 1.0 m off (28 runs) | 3444 | 86.2 % | 12.2 % | 1.6 % | 0.0 % | 4.6 % | 0.0 % |
| pose off the map (4 runs) | 200 | 0.0 % | 2.0 % | 98.0 % | 0.0 % | 0.0 % | 0.0 % |
| base_height 5 or 10 cm wrong, pose fine (8 runs) | 984 | 0.0 % | 0.8 % | 99.2 % | 0.0 % | 0.0 % | 82.2 % |

Without the "not verified" state (verification 0), threshold 0.65:

| Runs | updates | OK | not verified | low match | OK while > 1 m off (max in a row) | OK while > 0.3 m off | low match while < 0.15 m off |
|---|---|---|---|---|---|---|---|
| healthy, exact map; 0.5x to 3x speed, sparser lidars (24 runs) | 4084 | 98.4 % | 0.0 % | 1.6 % | 0.0 % | 7.2 % | 0.1 % |
| healthy, 2 or 5 cm noise on the prior map (8 runs) | 984 | 100.0 % | 0.0 % | 0.0 % | 0.0 % | 1.7 % | 0.0 % |
| healthy, scene differs from the exact map (24 runs) | 2952 | 86.8 % | 0.0 % | 13.2 % | 0.0 % | 10.8 % | 11.6 % |
| the same with 2 cm of noise on the prior map (24 runs) | 2952 | 99.9 % | 0.0 % | 0.1 % | 0.0 % | 3.1 % | 0.1 % |
| kidnapped, exact map, every update after the kidnap (8 runs) | 584 | 8.4 % | 0.0 % | 91.6 % | 2.7 % (4) | 7.0 % | 0.0 % |
| kidnapped, 2 or 5 cm map noise (12 runs) | 876 | 24.1 % | 0.0 % | 75.9 % | 6.8 % (15) | 21.5 % | 0.0 % |
| /initialpose 3.6 m and 1 rad off, exact map (4 runs) | 492 | 9.8 % | 0.0 % | 90.2 % | 8.5 % (11) | 9.8 % | 0.0 % |
| the same, 2 or 5 cm map noise (8 runs) | 984 | 68.8 % | 0.0 % | 31.2 % | 10.1 % (15) | 20.2 % | 0.2 % |
| /initialpose 0.3 to 1.0 m off (28 runs) | 3444 | 99.0 % | 0.0 % | 1.0 % | 0.0 % | 11.2 % | 0.0 % |
| pose off the map (4 runs) | 200 | 2.0 % | 0.0 % | 98.0 % | 2.0 % (1) | 2.0 % | 0.0 % |
| base_height 5 or 10 cm wrong, pose fine (8 runs) | 984 | 0.8 % | 0.0 % | 99.2 % | 0.0 % | 0.6 % | 82.2 % |

## 3. False alarms and mean-of-5 ranges

Low match per changed-scene case on the exact map (share of the updates of the 4 runs):

| Case | updates | low match | low match while < 0.15 m off |
|---|---|---|---|
| H_dyn15 | 492 | 34.6 % | 26.6 % |
| H_chg5add1drop | 492 | 20.7 % | 20.7 % |
| H_range5 | 492 | 19.9 % | 18.3 % |
| H_chg2add | 492 | 3.0 % | 3.0 % |
| H_dyn6 | 492 | 1.0 % | 1.0 % |
| H_range3 | 492 | 0.0 % | 0.0 % |
| H_dyn15_mapnoise2 | 492 | 0.4 % | 0.4 % |
| H_chg5add1drop_mapnoise2 | 492 | 0.0 % | 0.0 % |
| H_range5_mapnoise2 | 492 | 0.0 % | 0.0 % |
| H_chg2add_mapnoise2 | 492 | 0.0 % | 0.0 % |
| H_dyn6_mapnoise2 | 492 | 0.0 % | 0.0 % |
| H_range3_mapnoise2 | 492 | 0.0 % | 0.0 % |

`match_fraction` of single updates, and its mean of 5 (the value the rule uses; just after a kidnap the window still holds
healthy values, which is why its maximum is higher):

| Updates | count | single update, min - max | mean of 5, min - max |
|---|---|---|---|
| kidnapped, exact map, estimate > 1 m off | 423 | 0.306 - 0.734 | 0.321 - 0.745 |
| kidnapped, 2 or 5 cm map noise, estimate > 1 m off | 623 | 0.320 - 0.768 | 0.337 - 0.891 |
| healthy, exact map (6 cases) | 4060 | 0.525 - 0.948 | 0.562 - 0.916 |
| healthy, 2 or 5 cm map noise | 976 | 0.881 - 1.000 | 0.888 - 1.000 |
| /initialpose 3.6 m off, estimate > 1 m off, exact map | 391 | 0.199 - 0.716 | 0.208 - 0.705 |
| /initialpose 3.6 m off, estimate > 1 m off, 2 or 5 cm map noise | 285 | 0.218 - 0.755 | 0.225 - 0.741 |

Kidnapped, longest stretch of OK updates while > 1 m off, in seconds (updates are about 0.6 s apart):

| Case | runs with such an update | longest stretch (updates) | longest stretch (s) |
|---|---|---|---|
| K_kidnap | 4 of 4 | 2 | 0.6 |
| K_kidnap_shift8 | 4 of 4 | 4 | 2.1 |
| K_kidnap_mapnoise2 | 4 of 4 | 14 | 8.0 |
| K_kidnap_mapnoise5 | 4 of 4 | 15 | 7.0 |
| K_kidnap_shift8_mapnoise2 | 4 of 4 | 2 | 0.6 |

## 4. /initialpose 0.3 to 1.0 m off: OK while off by more than 0.5 m

| Case | runs | runs with an OK update > 0.5 m off | OK updates > 0.5 m off | largest error of those (m) | longest stretch (s) |
|---|---|---|---|---|---|
| W_init_0.3 | 4 | 0 | 0 | 0.00 | 0.0 |
| W_init_0.5 | 4 | 2 | 19 | 0.83 | 6.7 |
| W_init_1.0 | 4 | 2 | 18 | 0.87 | 4.7 |
| W_init_1.0y | 4 | 0 | 0 | 0.00 | 0.0 |
| W_init_5deg | 4 | 1 | 13 | 0.78 | 4.7 |
| W_init_mid | 4 | 1 | 9 | 0.74 | 4.7 |
| W_init_1.0_mapnoise2 | 4 | 1 | 7 | 0.76 | 3.6 |
| all | 28 | 7 | | | |

## 4b. /initialpose 3.6 m and 1 rad off: judged updates before the first low match

| Case | seed | judged updates with the match not yet low | estimate error over those (m) |
|---|---|---|---|
| W_init_3_2_1 | 0 | 10 | 1.40 - 2.23 |
| W_init_3_2_1 | 1 | 11 | 0.96 - 2.07 |
| W_init_3_2_1 | 2 | 9 | 1.70 - 2.46 |
| W_init_3_2_1 | 3 | 10 | 1.55 - 2.28 |
| W_init_3_2_1_mapnoise2 | 0 | 10 | 1.44 - 2.23 |
| W_init_3_2_1_mapnoise2 | 1 | 12 | 0.92 - 2.07 |
| W_init_3_2_1_mapnoise2 | 2 | 10 | 1.57 - 2.48 |
| W_init_3_2_1_mapnoise2 | 3 | 10 | 1.49 - 2.27 |
| W_init_3_2_1_mapnoise5 | 0 | 13 | 1.09 - 2.30 |
| W_init_3_2_1_mapnoise5 | 1 | 15 | 0.83 - 2.05 |
| W_init_3_2_1_mapnoise5 | 2 | 12 | 1.27 - 2.47 |
| W_init_3_2_1_mapnoise5 | 3 | 14 | 1.04 - 1.78 |

All 12 runs: 9 to 15 judged updates, estimate 0.83 to 2.48 m off.

## 5. Every case of the batch (4 runs each)

| Case | runs | updates | OK | not verified | low match | OK while > 1 m off | OK while > 0.3 m off | low match while < 0.15 m off |
|---|---|---|---|---|---|---|---|---|
| B_height-0.05 | 4 | 492 | 0.0 % | 0.8 % | 99.2 % | 0.0 % | 0.0 % | 82.1 % |
| B_height-0.1 | 4 | 492 | 0.0 % | 0.8 % | 99.2 % | 0.0 % | 0.0 % | 82.3 % |
| B_height-0.1_mapnoise2 | 4 | 492 | 57.1 % | 11.8 % | 31.1 % | 0.0 % | 0.0 % | 28.9 % |
| B_height-0.2 | 4 | 492 | 0.0 % | 0.8 % | 99.2 % | 0.0 % | 0.0 % | 82.1 % |
| B_height-0.2_mapnoise2 | 4 | 492 | 31.9 % | 11.0 % | 57.1 % | 0.0 % | 0.0 % | 54.1 % |
| B_height-0.5 | 4 | 492 | 0.0 % | 0.8 % | 99.2 % | 0.0 % | 0.0 % | 85.0 % |
| B_height-0.5_mapnoise2 | 4 | 492 | 0.2 % | 5.3 % | 94.5 % | 0.0 % | 0.0 % | 90.2 % |
| B_height-0.5_mapnoise5 | 4 | 492 | 7.1 % | 7.7 % | 85.2 % | 0.0 % | 0.0 % | 85.2 % |
| H_base | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 1.4 % | 0.0 % |
| H_chg2add | 4 | 492 | 84.8 % | 12.2 % | 3.0 % | 0.0 % | 3.3 % | 3.0 % |
| H_chg2add_mapnoise2 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.8 % | 0.0 % |
| H_chg5add1drop | 4 | 492 | 67.1 % | 12.2 % | 20.7 % | 0.0 % | 2.4 % | 20.7 % |
| H_chg5add1drop_mapnoise2 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.6 % | 0.0 % |
| H_dyn15 | 4 | 492 | 53.7 % | 11.8 % | 34.6 % | 0.0 % | 1.8 % | 26.6 % |
| H_dyn15_mapnoise2 | 4 | 492 | 87.4 % | 12.2 % | 0.4 % | 0.0 % | 0.0 % | 0.4 % |
| H_dyn6 | 4 | 492 | 86.8 % | 12.2 % | 1.0 % | 0.0 % | 4.9 % | 1.0 % |
| H_dyn6_mapnoise2 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 1.6 % | 0.0 % |
| H_edge_crop10 | 4 | 492 | 7.3 % | 0.8 % | 91.9 % | 0.0 % | 7.3 % | 0.0 % |
| H_edge_crop14 | 4 | 492 | 45.1 % | 0.8 % | 54.1 % | 0.0 % | 19.3 % | 13.0 % |
| H_edge_crop7 | 4 | 492 | 0.0 % | 0.8 % | 99.2 % | 0.0 % | 0.0 % | 0.0 % |
| H_mapnoise2 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.4 % | 0.0 % |
| H_mapnoise5 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.0 % | 0.0 % |
| H_pitch0.02 | 4 | 492 | 2.8 % | 9.3 % | 87.8 % | 0.0 % | 0.0 % | 74.6 % |
| H_pitch0.05 | 4 | 492 | 0.0 % | 0.8 % | 99.2 % | 0.0 % | 0.0 % | 77.6 % |
| H_range3 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 3.5 % | 0.0 % |
| H_range3_mapnoise2 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 1.8 % | 0.0 % |
| H_range5 | 4 | 492 | 67.9 % | 12.2 % | 19.9 % | 0.0 % | 3.0 % | 18.3 % |
| H_range5_mapnoise2 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.0 % | 0.0 % |
| H_sparse_4x90 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 7.1 % | 0.0 % |
| H_sparse_8x180 | 4 | 492 | 85.0 % | 12.2 % | 2.8 % | 0.0 % | 5.1 % | 0.0 % |
| H_speed0.5 | 4 | 408 | 85.3 % | 14.7 % | 0.0 % | 0.0 % | 0.0 % | 0.0 % |
| H_speed2 | 4 | 900 | 87.4 % | 6.7 % | 5.9 % | 0.0 % | 1.6 % | 0.3 % |
| H_speed3 | 4 | 1300 | 95.4 % | 4.6 % | 0.0 % | 0.0 % | 0.8 % | 0.0 % |
| K_kidnap (from the kidnap) | 4 | 292 | 1.7 % | 0.0 % | 98.3 % | 1.7 % | 1.7 % | 0.0 % |
| K_kidnap_mapnoise2 (from the kidnap) | 4 | 292 | 8.6 % | 0.0 % | 91.4 % | 8.6 % | 8.6 % | 0.0 % |
| K_kidnap_mapnoise5 (from the kidnap) | 4 | 292 | 9.2 % | 0.0 % | 90.8 % | 9.2 % | 9.2 % | 0.0 % |
| K_kidnap_shift8 (from the kidnap) | 4 | 292 | 15.1 % | 0.0 % | 84.9 % | 3.8 % | 12.3 % | 0.0 % |
| K_kidnap_shift8_mapnoise2 (from the kidnap) | 4 | 292 | 54.5 % | 0.0 % | 45.5 % | 2.7 % | 46.6 % | 0.0 % |
| O_offmap | 4 | 200 | 0.0 % | 2.0 % | 98.0 % | 0.0 % | 0.0 % | 0.0 % |
| R_reinit_after_kidnap | 4 | 496 | 58.7 % | 23.6 % | 17.7 % | 1.0 % | 3.2 % | 0.2 % |
| R_reinit_wrong_after_kidnap | 4 | 496 | 34.7 % | 12.9 % | 52.4 % | 6.5 % | 7.9 % | 0.0 % |
| W_init_0.3 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.4 % | 0.0 % |
| W_init_0.5 | 4 | 492 | 84.6 % | 12.2 % | 3.3 % | 0.0 % | 9.3 % | 0.0 % |
| W_init_1.0 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 6.5 % | 0.0 % |
| W_init_1.0_mapnoise2 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 2.8 % | 0.0 % |
| W_init_1.0y | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 0.0 % | 0.0 % |
| W_init_3_2_1 | 4 | 492 | 0.8 % | 8.9 % | 90.2 % | 0.0 % | 0.8 % | 0.0 % |
| W_init_3_2_1_mapnoise2 | 4 | 492 | 37.0 % | 9.3 % | 53.7 % | 0.0 % | 18.3 % | 0.0 % |
| W_init_3_2_1_mapnoise5 | 4 | 492 | 79.7 % | 11.6 % | 8.7 % | 0.0 % | 1.2 % | 0.4 % |
| W_init_5deg | 4 | 492 | 83.9 % | 12.2 % | 3.9 % | 0.0 % | 7.9 % | 0.0 % |
| W_init_mid | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 5.7 % | 0.0 % |
| W_init_mid_mapnoise2 | 4 | 492 | 87.8 % | 12.2 % | 0.0 % | 0.0 % | 6.9 % | 0.0 % |
