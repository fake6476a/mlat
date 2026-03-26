# Native C++ MLAT Pipeline — 1-Hour Benchmark Results

**Date:** 2026-03-26
**System:** AMD EPYC (2 cores, x86_64), GCC 11.4.0, Release build with LTO
**Dataset:** `data-pipe/correlation_groups_1h.jsonl` — 165,917 correlation groups (57 MB)

---

## Executive Summary

| Metric | Value |
|---|---|
| **Input groups** | 165,917 |
| **Solved fixes (Layer 4)** | 84,338 |
| **Solve rate** | 50.8% |
| **Track updates (Layer 5)** | 62,929 |
| **Unique aircraft tracked** | 120 |
| **Layer 4 wall time** | 3.617 s avg |
| **Layer 5 wall time** | 2.344 s avg |
| **Full pipeline wall time** | 3.782 s avg |
| **Realtime factor** | **952x** (1 h of data in ~3.8 s) |

---

## 1. Timing Benchmarks (5 Iterations)

### Layer 4 — MLAT Solver

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 3.628 | 3.506 | 0.251 |
| 2 | 3.611 | 3.498 | 0.353 |
| 3 | 3.630 | 3.550 | 0.215 |
| 4 | 3.596 | 3.501 | 0.192 |
| 5 | 3.622 | 3.518 | 0.183 |
| **Avg** | **3.617** | **3.515** | **0.239** |

### Layer 5 — Track Builder

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 2.347 | 2.233 | 0.338 |
| 2 | 2.342 | 2.299 | 0.132 |
| 3 | 2.339 | 2.281 | 0.143 |
| 4 | 2.346 | 2.257 | 0.170 |
| 5 | 2.347 | 2.239 | 0.194 |
| **Avg** | **2.344** | **2.262** | **0.195** |

### Full Pipeline (Layer 4 + Layer 5 piped)

| Run | Wall (s) | User (s) | Sys (s) |
|---|---:|---:|---:|
| 1 | 3.774 | 5.843 | 0.202 |
| 2 | 3.774 | 5.868 | 0.197 |
| 3 | 3.777 | 5.847 | 0.215 |
| 4 | 3.787 | 5.912 | 0.162 |
| 5 | 3.798 | 5.862 | 0.247 |
| **Avg** | **3.782** | **5.866** | **0.205** |

> Pipeline user time (~5.9 s) exceeds wall time (~3.8 s) — both L4 and L5 run in parallel via Unix pipe, utilizing both CPU cores.

---

## 2. Throughput

| Component | Throughput | Per-record latency |
|---|---:|---:|
| Layer 4 (solver) | 45,866 groups/s | 21.80 us/group |
| Layer 5 (tracker) | 66,084 records/s | 15.13 us/record |
| Full pipeline | 43,870 groups/s | 22.79 us/group |

---

## 3. Layer 4 — Solver Quality Metrics

### 3.1 Solve Summary

| Metric | Value |
|---|---:|
| Groups received | 165,917 |
| Groups solved | 84,338 |
| Groups failed | 78,583 |
| Groups skipped (sensors) | 2,996 |
| Parse errors | 0 |
| **Solve rate** | **50.8%** |

### 3.2 Residual Distribution (meters)

| Percentile | Value (m) |
|---|---:|
| Min | 0.00 |
| p5 | 1.36 |
| p10 | 3.10 |
| p25 | 8.60 |
| **Median** | **22.47** |
| Mean | 39.36 |
| p75 | 54.10 |
| p90 | 104.82 |
| **p95** | **141.01** |
| p99 | 184.95 |
| Max | 200.00 |
| StdDev | 43.46 |

### 3.3 Solve Methods

| Method | Count | % of Solves |
|---|---:|---:|
| prior_2sensor | 83,456 | 99.0% |
| constrained_3sensor | 827 | 1.0% |
| constrained_3sensor_drop1 | 49 | 0.1% |
| constrained_3sensor_drop2 | 4 | <0.1% |
| inamdar_4sensor_alt | 2 | <0.1% |

### 3.4 Failure Reasons

| Reason | Count |
|---|---:|
| residual_exceeded | 66,287 |
| too_few_sensors | 9,418 |
| no_prior_2sensor | 2,996 |
| physically_inconsistent | 1,567 |
| gdop_2d_exceeded | 668 |
| prior_drift_exceeded | 465 |
| gdop_exceeded | 118 |
| range_exceeded | 60 |

### 3.5 Sensor Count Distribution

| Sensors | Solves | % |
|---|---:|---:|
| 2 | 83,456 | 99.0% |
| 3 | 880 | 1.0% |
| 4 | 2 | <0.1% |

### 3.6 GDOP Distribution

| Percentile | GDOP |
|---|---:|
| Min | 1.18 |
| Median | 18.44 |
| Mean | 981.57 |
| p95 | 220.39 |

### 3.7 Clock Calibration

| Metric | Value |
|---|---:|
| Total pairings | 36 |
| Valid pairings | 35 |
| Sync points total | 86,900 |
| Sync points accepted | 86,717 |
| Groups corrected | 163,873 |

### 3.8 Position Cache

| Metric | Value |
|---|---:|
| Aircraft tracked | 120 |
| Cache hits | 273,741 |
| Cache misses | 21,982 |
| Hit rate | 92.6% |
| Multi-solve aircraft | 120 |

### 3.9 CPR Buffer

| Metric | Value |
|---|---:|
| Frames received | 12,156 |
| Global decodes | 5,877 |
| Local decodes | 5,555 |
| Tracked ICAOs | 146 |
| CPR cache seeds | 11,432 |

### 3.10 Per-Sensor Clock Sync Bias (meters)

| Sensor ID | Bias (m) |
|---|---:|
| 2126082203 | 20.58 |
| 538954918 | 22.33 |
| 1592161674 | 27.29 |
| 2712405983 | 28.21 |
| 1427135468 | 31.08 |
| 1437748392 | 31.12 |
| 2896675171 | 32.80 |
| 2883057380 | 34.51 |
| 1665721444 | 41.26 |

---

## 4. Layer 5 — Track Builder Quality Metrics

### 4.1 Track Summary

| Metric | Value |
|---|---:|
| Fixes received | 154,914 |
| Fixes accepted (EKF gate) | 62,929 |
| Fixes rejected | 29,436 |
| Accept rate | 40.6% |
| Tracks created | 120 |
| Tracks pruned | 0 |
| Active tracks | 120 |
| Established tracks | 120 |
| Parse errors | 0 |

### 4.2 Post-EKF Residual Distribution (meters)

| Percentile | Value (m) |
|---|---:|
| Min | 0.00 |
| p5 | 1.88 |
| p25 | 10.14 |
| **Median** | **26.99** |
| Mean | 49.34 |
| p75 | 65.27 |
| **p95** | **167.23** |
| p99 | 311.66 |
| Max | 499.98 |

### 4.3 Track Quality

| Metric | Value |
|---|---:|
| Min quality | 1 |
| Median quality | 508 |
| Max quality | 3,089 |

### 4.4 Speed Distribution (knots)

| Metric | Value |
|---|---:|
| Min | 0.0 |
| Median | 491.7 |
| Mean | 475.1 |
| Max | 9,023.7 |

---

## 5. Data Flow Summary

```
correlation_groups_1h.jsonl  (165,917 lines, 57 MB)
        |
        v
   MLAT Solver (Layer 4)   --- 3.617 s avg
        |
        v
  layer4_native_1h.jsonl   (154,914 lines, 51 MB)
   [84,338 solved + 70,576 unsolved/passthrough]
        |
        v
  Track Builder (Layer 5)  --- 2.344 s avg
        |
        v
  layer5_native_1h.jsonl   (62,929 lines, 30 MB)
   [120 aircraft tracks, 62,929 EKF-filtered updates]
```

---

## 6. Comparison With Historical Baselines

| Metric | README baseline (30 min) | This benchmark (1 h) |
|---|---:|---:|
| Input groups | 77,722 | 165,917 |
| Solve rate | ~47.2% | 50.8% |
| Median residual | ~16.5 m | 22.47 m |
| p95 residual | ~129-130 m | 141.01 m |
| Native L4 runtime | ~1m08s | 3.617 s |

> **Note:** The 30-min baseline numbers in the README were captured on a different machine and dataset. The current 1-hour run processed **2.1x more groups** in significantly less wall time (3.6 s vs ~68 s), likely due to hardware differences and build optimizations (LTO, `-march=native`).

---

## 7. System Configuration

- **CPU:** AMD EPYC, 2 cores, x86_64
- **Compiler:** GCC 11.4.0
- **Build:** Release, LTO enabled, `-O3 -march=native`
- **OS:** Linux (Ubuntu)
- **Location overrides:** Not loaded (no `location-overrides.txt` present)
