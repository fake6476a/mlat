# Native C++ MLAT Pipeline — 1-Hour Benchmark Results

**Date:** 2026-03-26
**System:** AMD EPYC (2 cores, x86_64), GCC 11.4.0, Release build with LTO
**Dataset:** `data-pipe/correlation_groups_1h.jsonl` — 165,917 correlation groups (57 MB)

Two runs were performed: one **without** location overrides and one **with** `data-pipe/location-overrides.txt` (9 sensors matched).

---

## Executive Summary

| Metric | Without Overrides | With Overrides |
|---|---|---|
| **Input groups** | 165,917 | 165,917 |
| **Solved fixes (Layer 4)** | 84,338 | 84,561 |
| **Solve rate** | 50.8% | 51.0% |
| **Median residual** | 22.47 m | **18.91 m** |
| **p95 residual** | 141.01 m | **140.80 m** |
| **Track updates (Layer 5)** | 62,929 | 67,084 |
| **L5 accept rate** | 40.6% | 43.5% |
| **Unique aircraft tracked** | 120 | 120 |
| **Layer 4 wall time** | 3.617 s avg | 3.635 s avg |
| **Layer 5 wall time** | 2.344 s avg | 2.371 s avg |
| **Full pipeline wall time** | 3.782 s avg | 3.813 s avg |
| **Realtime factor** | **952x** | **944x** |

> Location overrides improved median residual by **15.8%** (22.47 m -> 18.91 m), increased solve count by 223, and boosted Layer 5 EKF accept rate from 40.6% to 43.5% (+4,155 track updates).

---

## 1. Timing Benchmarks (5 Iterations)

### 1.1 Layer 4 — MLAT Solver

#### Without Overrides

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 3.628 | 3.506 | 0.251 |
| 2 | 3.611 | 3.498 | 0.353 |
| 3 | 3.630 | 3.550 | 0.215 |
| 4 | 3.596 | 3.501 | 0.192 |
| 5 | 3.622 | 3.518 | 0.183 |
| **Avg** | **3.617** | **3.515** | **0.239** |

#### With Overrides

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 3.618 | 3.491 | 0.275 |
| 2 | 3.700 | 3.528 | 0.297 |
| 3 | 3.621 | 3.470 | 0.348 |
| 4 | 3.592 | 3.463 | 0.227 |
| 5 | 3.644 | 3.454 | 0.299 |
| **Avg** | **3.635** | **3.481** | **0.289** |

### 1.2 Layer 5 — Track Builder

#### Without Overrides

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 2.347 | 2.233 | 0.338 |
| 2 | 2.342 | 2.299 | 0.132 |
| 3 | 2.339 | 2.281 | 0.143 |
| 4 | 2.346 | 2.257 | 0.170 |
| 5 | 2.347 | 2.239 | 0.194 |
| **Avg** | **2.344** | **2.262** | **0.195** |

#### With Overrides

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 2.360 | 2.205 | 0.401 |
| 2 | 2.380 | 2.250 | 0.346 |
| 3 | 2.357 | 2.207 | 0.260 |
| 4 | 2.385 | 2.226 | 0.240 |
| 5 | 2.373 | 2.274 | 0.192 |
| **Avg** | **2.371** | **2.232** | **0.288** |

### 1.3 Full Pipeline (Layer 4 + Layer 5 piped)

#### Without Overrides

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 3.774 | 5.843 | 0.202 |
| 2 | 3.774 | 5.868 | 0.197 |
| 3 | 3.777 | 5.847 | 0.215 |
| 4 | 3.787 | 5.912 | 0.162 |
| 5 | 3.798 | 5.862 | 0.247 |
| **Avg** | **3.782** | **5.866** | **0.205** |

#### With Overrides

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 3.915 | 5.878 | 0.365 |
| 2 | 3.774 | 5.817 | 0.365 |
| 3 | 3.796 | 5.920 | 0.259 |
| 4 | 3.752 | 5.908 | 0.203 |
| 5 | 3.826 | 5.878 | 0.226 |
| **Avg** | **3.813** | **5.880** | **0.284** |

> Pipeline user time (~5.9 s) exceeds wall time (~3.8 s) — both L4 and L5 run in parallel via Unix pipe, utilizing both CPU cores.

---

## 2. Throughput

| Component | Without Overrides | With Overrides |
|---|---:|---:|
| Layer 4 (groups/s) | 45,866 | 45,634 |
| Layer 4 (us/group) | 21.80 | 21.91 |
| Layer 5 (records/s) | 66,084 | 65,331 |
| Layer 5 (us/record) | 15.13 | 15.31 |
| Full pipeline (groups/s) | 43,870 | 43,510 |
| Realtime factor | 952x | 944x |

---

## 3. Layer 4 — Solver Quality Metrics

### 3.1 Solve Summary

| Metric | Without Overrides | With Overrides |
|---|---:|---:|
| Groups received | 165,917 | 165,917 |
| Groups solved | 84,338 | 84,561 |
| Groups failed | 78,583 | 78,159 |
| Groups skipped (sensors) | 2,996 | 3,197 |
| Parse errors | 0 | 0 |
| **Solve rate** | **50.8%** | **51.0%** |

### 3.2 Residual Distribution (meters)

| Percentile | Without Overrides | With Overrides |
|---|---:|---:|
| Min | 0.00 | 0.00 |
| p5 | 1.36 | 1.14 |
| p10 | 3.10 | 2.62 |
| p25 | 8.60 | 7.30 |
| **Median** | **22.47** | **18.91** |
| Mean | 39.36 | 37.05 |
| p75 | 54.10 | 49.97 |
| p90 | 104.82 | 103.43 |
| **p95** | **141.01** | **140.80** |
| p99 | 184.95 | 185.54 |
| Max | 200.00 | 200.00 |
| StdDev | 43.46 | 43.60 |

### 3.3 Solve Methods

| Method | Without Overrides | With Overrides |
|---|---:|---:|
| prior_2sensor | 83,456 (99.0%) | 83,643 (98.9%) |
| constrained_3sensor | 827 (1.0%) | 853 (1.0%) |
| constrained_3sensor_drop1 | 49 (0.1%) | 59 (0.1%) |
| constrained_3sensor_drop2 | 4 (<0.1%) | 5 (<0.1%) |
| inamdar_4sensor_alt | 2 (<0.1%) | 1 (<0.1%) |

### 3.4 Failure Reasons

| Reason | Without Overrides | With Overrides |
|---|---:|---:|
| residual_exceeded | 66,287 | 65,498 |
| too_few_sensors | 9,418 | 9,940 |
| no_prior_2sensor | 2,996 | 3,197 |
| physically_inconsistent | 1,567 | 1,510 |
| gdop_2d_exceeded | 668 | 681 |
| prior_drift_exceeded | 465 | 369 |
| gdop_exceeded | 118 | 119 |
| range_exceeded | 60 | 42 |

### 3.5 Sensor Count Distribution

| Sensors | Without Overrides | With Overrides |
|---|---:|---:|
| 2 | 83,456 (99.0%) | 83,643 (98.9%) |
| 3 | 880 (1.0%) | 917 (1.1%) |
| 4 | 2 (<0.1%) | 1 (<0.1%) |

### 3.6 GDOP Distribution

| Percentile | Without Overrides | With Overrides |
|---|---:|---:|
| Min | 1.18 | 1.18 |
| Median | 18.44 | 18.15 |
| Mean | 981.57 | 148.72 |
| p95 | 220.39 | 217.54 |

### 3.7 Clock Calibration

| Metric | Without Overrides | With Overrides |
|---|---:|---:|
| Total pairings | 36 | 36 |
| Valid pairings | 35 | 35 |
| Sync points total | 86,900 | 87,212 |
| Sync points accepted | 86,717 | 86,992 |
| Groups corrected | 163,873 | 163,919 |

### 3.8 Position Cache

| Metric | Without Overrides | With Overrides |
|---|---:|---:|
| Aircraft tracked | 120 | 120 |
| Cache hits | 273,741 | 272,485 |
| Cache misses | 21,982 | 23,238 |
| Hit rate | 92.6% | 92.1% |
| Multi-solve aircraft | 120 | 120 |

### 3.9 CPR Buffer

| Metric | Value |
|---|---:|
| Frames received | 12,156 |
| Global decodes | 5,877 |
| Local decodes | 5,555 |
| Tracked ICAOs | 146 |
| CPR cache seeds | 11,432 |

### 3.10 Per-Sensor Clock Sync Bias (meters)

| Sensor ID | Name (with overrides) | Without Overrides | With Overrides |
|---|---|---:|---:|
| 2126082203 | ne106 Isle 1900 | 20.58 | 18.81 |
| 538954918 | ne078 Bojevan | 22.33 | 18.88 |
| 1592161674 | ne007 Lands End | 27.29 | 27.41 |
| 2712405983 | ne108 Isle 1100 | 28.21 | 26.61 |
| 1427135468 | ne101 Isle 1300 | 31.08 | 31.14 |
| 1437748392 | ne083 St Ives | 31.12 | 32.04 |
| 2896675171 | ne010 Isle 1600 | 32.80 | 33.29 |
| 2883057380 | ne007 Lands End | 34.51 | 33.63 |
| 1665721444 | ne073 Penzance | 41.26 | 43.92 |

### 3.11 Location Overrides Summary (with overrides run)

| Metric | Value |
|---|---:|
| Matched sensors | 9 |
| Rejected sensors | 0 |

---

## 4. Layer 5 — Track Builder Quality Metrics

### 4.1 Track Summary

| Metric | Without Overrides | With Overrides |
|---|---:|---:|
| Fixes received | 154,914 | 154,301 |
| Fixes accepted (EKF gate) | 62,929 | 67,084 |
| Fixes rejected | 29,436 | 25,660 |
| Accept rate | 40.6% | 43.5% |
| Tracks created | 120 | 120 |
| Tracks pruned | 0 | 0 |
| Active tracks | 120 | 120 |
| Established tracks | 120 | 120 |
| Parse errors | 0 | 146 |

### 4.2 Post-EKF Residual Distribution (meters)

| Percentile | Without Overrides | With Overrides |
|---|---:|---:|
| Min | 0.00 | 0.00 |
| p5 | 1.88 | 1.57 |
| p25 | 10.14 | 8.74 |
| **Median** | **26.99** | **23.23** |
| Mean | 49.34 | 46.33 |
| p75 | 65.27 | 60.85 |
| **p95** | **167.23** | **161.75** |
| p99 | 311.66 | 300.35 |
| Max | 499.98 | 499.66 |

### 4.3 Track Quality

| Metric | Without Overrides | With Overrides |
|---|---:|---:|
| Min quality | 1 | 1 |
| Median quality | 508 | 563 |
| Max quality | 3,089 | 3,115 |

### 4.4 Speed Distribution (knots)

| Metric | Without Overrides | With Overrides |
|---|---:|---:|
| Min | 0.0 | 0.0 |
| Median | 491.7 | 504.5 |
| Mean | 475.1 | 482.2 |
| Max | 9,023.7 | 8,920.3 |

---

## 5. Data Flow Summary (with overrides)

```
correlation_groups_1h.jsonl  (165,917 lines, 57 MB)
        |
        v
   MLAT Solver (Layer 4)   --- 3.635 s avg
        |
        v
  layer4_native_1h.jsonl   (154,447 lines, 51 MB)
   [84,561 solved + 69,886 unsolved/passthrough]
        |
        v
  Track Builder (Layer 5)  --- 2.371 s avg
        |
        v
  layer5_native_1h.jsonl   (67,084 lines, 30 MB)
   [120 aircraft tracks, 67,084 EKF-filtered updates]
```

---

## 6. Comparison With Historical Baselines

| Metric | README baseline (30 min) | This benchmark (1 h, with overrides) |
|---|---:|---:|
| Input groups | 77,722 | 165,917 |
| Solve rate | ~47.2% | 51.0% |
| Median residual | ~16.5 m | 18.91 m |
| p95 residual | ~129-130 m | 140.80 m |
| Native L4 runtime | ~1m08s | 3.635 s |

> **Note:** The 30-min baseline numbers in the README were captured on a different machine and dataset. The current 1-hour run processed **2.1x more groups** in significantly less wall time (3.6 s vs ~68 s), likely due to hardware differences and build optimizations (LTO, `-march=native`).

---

## 7. Impact of Location Overrides

Enabling location overrides (9 sensors matched from `data-pipe/location-overrides.txt`) had the following effects:

| Metric | Change |
|---|---|
| Solve rate | 50.8% -> 51.0% (+0.2pp) |
| Median residual (L4) | 22.47 m -> 18.91 m (**-15.8%**) |
| p95 residual (L4) | 141.01 m -> 140.80 m (-0.1%) |
| Mean residual (L4) | 39.36 m -> 37.05 m (-5.9%) |
| L5 EKF accept rate | 40.6% -> 43.5% (+2.9pp) |
| L5 track updates | 62,929 -> 67,084 (+4,155 / **+6.6%**) |
| L5 median residual | 26.99 m -> 23.23 m (**-13.9%**) |
| L5 median quality | 508 -> 563 (+10.8%) |
| GDOP mean | 981.57 -> 148.72 (**-84.8%**) |
| residual_exceeded failures | 66,287 -> 65,498 (-789) |
| physically_inconsistent | 1,567 -> 1,510 (-57) |
| range_exceeded | 60 -> 42 (-18) |
| Pipeline wall time | 3.782 s -> 3.813 s (+0.8%, negligible) |

Key takeaway: Location overrides provide **meaningful quality improvement** (especially median residual and L5 accept rate) with **no measurable performance cost**.

---

## 8. System Configuration

- **CPU:** AMD EPYC, 2 cores, x86_64
- **Compiler:** GCC 11.4.0
- **Build:** Release, LTO enabled, `-O3 -march=native`
- **OS:** Linux (Ubuntu)
- **Location overrides file:** `data-pipe/location-overrides.txt` (9 sensors)
