# Task 1 results (simulator, sim_world_tables.yaml)

Bag: map_sim_20261020_201131 (simulator mapping drive, 48 MB, kept outside the repository)

Maps side by side: [sim_maps.png](sim_maps.png) (black LiDAR map, orange cells added by the camera layer)

## Maps vs reference (map_compare, tol 0.10 m)

| map | precision | recall | F1 | occupied cells | phantom cells (> 0.10 m from any object) |
|---|---|---|---|---|---|
| lidar only | 0.982 | 0.681 | 0.804 | 1016 | 16 |
| lidar + camera layer (mapping run) | 0.841 | 0.909 | 0.874 | 2353 | 48 |
| lidar + camera layer (rebuilt) | 0.862 | 0.904 | 0.882 | 2188 | 17 |

## Share of each object outline found in the map

| object | lidar only | lidar + camera layer (mapping run) | lidar + camera layer (rebuilt) |
|---|---|---|---|
| walls | 0.97 | 1.00 | 1.00 |
| desk_a | 0.25 | 0.96 | 0.95 |
| desk_b | 0.25 | 1.00 | 0.99 |
| table_c | 0.26 | 0.53 | 0.51 |
| shelf | 0.21 | 0.61 | 0.57 |
| low_a | 0.00 | 1.00 | 1.00 |
| low_b | 0.00 | 1.00 | 1.00 |
| cabinet | 0.58 | 0.58 | 0.58 |
| bin | 0.82 | 0.82 | 0.82 |
| east_box | 0.91 | 1.00 | 1.00 |
| south_box | 1.00 | 1.00 | 1.00 |
| pillar | 0.88 | 0.88 | 0.94 |

## Navigation goals: lidar only

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 44.6 | 3.74 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | FAILED | 152.1 | 3.66 | 0 | 2 | 0 | 0 |
| 3 | [6.0, 4.5] | OK | 28.0 | 2.94 | 0 | 0 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.1 | 3.76 | 0 | 3 | 0 | 15 |
| 5 | [7.0, 1.0] | FAILED | 152.1 | 0.84 | 0 | 3 | 0 | 21 |
| 6 | [5.3, 1.0] | OK | 117.3 | 2.24 | 0 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.0 | 2.61 | 0 | 1 | 0 | 6 |
| 8 | [1.5, 3.3] | FAILED | 152.0 | 2.21 | 0 | 1 | 0 | 0 |
| 9 | [3.3, 1.2] | OK | 28.6 | 4.57 | 0 | 1 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 77.6 | 6.97 | 1 | 0 | 0 | 0 |
| **total** | | **5/10** | 1056 | 33.5 | 1 | 12 | 0 | 42 |

## Navigation goals: lidar + camera layer (mapping run)

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 43.1 | 3.71 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | FAILED | 152.1 | 3.56 | 0 | 2 | 0 | 0 |
| 3 | [6.0, 4.5] | OK | 33.0 | 2.87 | 0 | 0 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.1 | 4.43 | 0 | 3 | 0 | 0 |
| 5 | [7.0, 1.0] | OK | 73.8 | 2.34 | 0 | 1 | 0 | 0 |
| 6 | [5.3, 1.0] | OK | 28.5 | 2.01 | 0 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.0 | 1.52 | 0 | 1 | 2 | 0 |
| 8 | [1.5, 3.3] | FAILED | 41.8 | 0.22 | 0 | 0 | 0 | 0 |
| 9 | [3.3, 1.2] | OK | 17.9 | 3.32 | 0 | 0 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 101.9 | 7.22 | 0 | 1 | 0 | 0 |
| **total** | | **6/10** | 796 | 31.2 | 0 | 9 | 2 | 0 |

## Navigation goals: lidar + camera layer (rebuilt)

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 42.0 | 3.74 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | FAILED | 152.1 | 3.68 | 0 | 2 | 0 | 0 |
| 3 | [6.0, 4.5] | OK | 33.5 | 2.88 | 0 | 0 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.1 | 3.84 | 0 | 3 | 0 | 3 |
| 5 | [7.0, 1.0] | OK | 77.1 | 2.36 | 0 | 1 | 0 | 0 |
| 6 | [5.3, 1.0] | FAILED | 152.1 | 1.67 | 0 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.1 | 2.22 | 0 | 1 | 1 | 0 |
| 8 | [1.5, 3.3] | OK | 98.7 | 7.55 | 0 | 1 | 0 | 0 |
| 9 | [3.3, 1.2] | OK | 18.0 | 3.41 | 0 | 0 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 77.5 | 7.12 | 0 | 0 | 0 | 0 |
| **total** | | **6/10** | 955 | 38.5 | 0 | 9 | 1 | 3 |

## Repeat of the navigation goals

Same maps and the same navigation configuration, an earlier run.

### Navigation goals (repeat): lidar only

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 40.1 | 3.76 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | FAILED | 152.1 | 3.87 | 0 | 1 | 18 | 3 |
| 3 | [6.0, 4.5] | OK | 49.9 | 2.86 | 0 | 2 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.0 | 1.49 | 3 | 1 | 0 | 0 |
| 5 | [7.0, 1.0] | FAILED | 152.0 | 1.11 | 1 | 1 | 0 | 16 |
| 6 | [5.3, 1.0] | OK | 94.1 | 2.06 | 1 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.1 | 2.59 | 0 | 1 | 1 | 11 |
| 8 | [1.5, 3.3] | FAILED | 152.1 | 1.81 | 0 | 1 | 0 | 3 |
| 9 | [3.3, 1.2] | OK | 55.4 | 3.83 | 0 | 1 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 94.4 | 7.35 | 2 | 1 | 0 | 0 |
| **total** | | **5/10** | 1094 | 30.7 | 7 | 10 | 19 | 33 |

### Navigation goals (repeat): lidar + camera layer (mapping run)

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 41.6 | 3.75 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | FAILED | 152.0 | 3.89 | 0 | 1 | 0 | 0 |
| 3 | [6.0, 4.5] | OK | 58.4 | 3.05 | 0 | 1 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.1 | 4.37 | 1 | 3 | 0 | 0 |
| 5 | [7.0, 1.0] | OK | 56.3 | 2.23 | 0 | 0 | 0 | 0 |
| 6 | [5.3, 1.0] | OK | 76.4 | 2.17 | 0 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.1 | 1.58 | 0 | 1 | 5 | 0 |
| 8 | [1.5, 3.3] | FAILED | 152.0 | 3.92 | 0 | 1 | 0 | 0 |
| 9 | [3.3, 1.2] | OK | 22.5 | 3.34 | 0 | 0 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 81.6 | 7.19 | 0 | 1 | 0 | 0 |
| **total** | | **6/10** | 945 | 35.5 | 1 | 9 | 5 | 0 |

### Navigation goals (repeat): lidar + camera layer (rebuilt)

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 44.6 | 3.75 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | OK | 143.7 | 4.19 | 0 | 2 | 0 | 0 |
| 3 | [6.0, 4.5] | OK | 12.7 | 2.77 | 0 | 0 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.1 | 4.13 | 0 | 4 | 0 | 0 |
| 5 | [7.0, 1.0] | OK | 55.1 | 2.25 | 0 | 0 | 0 | 0 |
| 6 | [5.3, 1.0] | OK | 29.3 | 2.08 | 0 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.0 | 2.1 | 0 | 1 | 21 | 0 |
| 8 | [1.5, 3.3] | OK | 148.9 | 4.06 | 0 | 1 | 0 | 0 |
| 9 | [3.3, 1.2] | OK | 17.9 | 3.35 | 0 | 0 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 83.0 | 7.11 | 0 | 1 | 0 | 0 |
| **total** | | **8/10** | 839 | 35.8 | 0 | 10 | 21 | 0 |

## Ablation: camera layer in the global costmap only

Same maps; the camera layer was not loaded in the local costmap. The robot then hit low_b while turning next to it at goal 4 (blind zone of the camera).

### Navigation goals (no camera layer in the local costmap): lidar only

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 41.9 | 3.78 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | FAILED | 152.0 | 3.56 | 0 | 2 | 0 | 0 |
| 3 | [6.0, 4.5] | OK | 12.7 | 2.89 | 0 | 0 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.1 | 1.95 | 4 | 0 | 0 | 0 |
| 5 | [7.0, 1.0] | OK | 56.0 | 2.28 | 0 | 0 | 0 | 0 |
| 6 | [5.3, 1.0] | OK | 84.9 | 1.94 | 0 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.0 | 2.16 | 0 | 2 | 0 | 42 |
| 8 | [1.5, 3.3] | FAILED | 152.1 | 1.03 | 1 | 1 | 0 | 8 |
| 9 | [3.3, 1.2] | OK | 21.4 | 4.04 | 0 | 0 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 104.2 | 9.52 | 0 | 2 | 0 | 0 |
| **total** | | **6/10** | 929 | 33.1 | 5 | 8 | 0 | 50 |

### Navigation goals (no camera layer in the local costmap): lidar + camera layer (mapping run)

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 39.9 | 3.75 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | FAILED | 152.1 | 3.8 | 0 | 2 | 0 | 0 |
| 3 | [6.0, 4.5] | OK | 27.5 | 2.91 | 0 | 0 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.0 | 4.31 | 0 | 4 | 0 | 12 |
| 5 | [7.0, 1.0] | OK | 88.3 | 2.45 | 0 | 0 | 0 | 0 |
| 6 | [5.3, 1.0] | OK | 29.9 | 2.12 | 0 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.1 | 2.11 | 0 | 1 | 3 | 0 |
| 8 | [1.5, 3.3] | FAILED | 152.1 | 3.98 | 0 | 1 | 0 | 0 |
| 9 | [3.3, 1.2] | OK | 17.7 | 3.37 | 0 | 0 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 76.6 | 7.14 | 0 | 0 | 0 | 0 |
| **total** | | **6/10** | 888 | 35.9 | 0 | 9 | 3 | 12 |

### Navigation goals (no camera layer in the local costmap): lidar + camera layer (rebuilt)

Goals in sequence from the previous goal. After a failed goal the robot is put on that goal (sim teleport, AMCL re-seeded), so each goal is measured on its own.

| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |
|---|---|---|---|---|---|---|---|---|
| 1 | [3.3, 3.35] | OK | 40.6 | 3.74 | 0 | 1 | 0 | 0 |
| 2 | [3.3, 5.3] | FAILED | 152.1 | 3.77 | 0 | 2 | 0 | 0 |
| 3 | [6.0, 4.5] | OK | 29.0 | 2.89 | 0 | 0 | 0 | 0 |
| 4 | [6.0, 2.5] | FAILED | 152.1 | 3.44 | 0 | 3 | 0 | 14 |
| 5 | [7.0, 1.0] | OK | 57.7 | 2.25 | 0 | 0 | 0 | 0 |
| 6 | [5.3, 1.0] | OK | 30.0 | 2.07 | 0 | 0 | 0 | 0 |
| 7 | [4.8, 2.6] | FAILED | 152.1 | 2.12 | 0 | 1 | 3 | 0 |
| 8 | [1.5, 3.3] | FAILED | 152.0 | 2.75 | 0 | 1 | 0 | 0 |
| 9 | [3.3, 1.2] | OK | 18.6 | 3.25 | 0 | 0 | 0 | 0 |
| 10 | [7.0, 4.5] | OK | 84.0 | 7.12 | 0 | 1 | 0 | 0 |
| **total** | | **6/10** | 868 | 33.4 | 0 | 9 | 3 | 14 |
