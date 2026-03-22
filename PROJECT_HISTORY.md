# MLAT Solver — Complete Project History

A chronological record of every change attempted across the MLAT solver project, what worked, what didn't, and why. All results measured on real Cornwall SDR sensor data.

---

## Phase 1: Baseline (PI Controller Era)

**Dataset:** 101,405 correlation groups from Cornwall SDR network (9 sensors, cheap dongles, ~15–20μs clock jitter).

**Starting point:**
- Clock sync: PI controller (`KP=0.05`, `KI=0.01`) with 5σ outlier gate
- Solver: Frisch TOA + Inamdar algebraic init
- No position cache seeding, no ADS-B integration
- `MAX_PAIR_VARIANCE_US2 = 400 μs²`

**Results:**
- Solves: ~6,847 (6.8%)
- Median residual: ~700m
- P95 residual: ~2,400m

---

## Phase 2: Parameter Tuning (PI Controller)

**Goal:** Squeeze more solves from the existing architecture.

### Tried and kept:
| Change | Impact |
|--------|--------|
| `MAX_PAIR_VARIANCE_US2`: 400 → 600 | +239 solves (+3.5%) — sweet spot for SDR noise |
| `prediction_weight = 8.0` | Optimal for 2-sensor prior-aided solves |

### Tried and failed:
| Change | Why it failed |
|--------|--------------|
| TOA-only residual | Assumed GPS-synced Jetvision sensors; our SDRs are too noisy |
| EMA velocity smoothing | Same — designed for precise sensors |
| Altitude σ = 30m constraint | Over-constrained with SDR timing errors |
| Cornwall grid search | Marginal gain, not worth complexity |
| 2D GDOP gate | Neutral to slightly negative |
| Clock drift regression | Unstable with SDR noise levels |
| `MAX_PAIR_VARIANCE_US2 = 800` | Lets in too much noise |

**Key lesson:** Most "textbook" improvements assume GPS-synced receivers. Our cheap SDR dongles break those assumptions.

**Final Phase 2 results:**
- Solves: 7,086 (7.0%)
- Median residual: 704m
- P95 residual: 2,399m

---

## Phase 3: ADS-B CPR Cache Seeding (Breakthrough)

**Goal:** Break the chicken-and-egg problem — solver needs position priors, but has none at startup.

### Changes made:
1. **NEW** `mlat-solver/adsb_decoder.py` — CPR global/local decoder + DF17 field extraction (~370 lines)
2. `modes-decoder/decoder.py` — Added DF17 TC 9-18 altitude extraction via `pms.adsb.altitude()`
3. `mlat-solver/main.py` — CPR frame buffering, cache seeding, DF17 altitude extraction, cached altitude propagation, clock cal feeding, DF17 velocity decode
4. `mlat-solver/position_cache.py` — Added `update_velocity_from_adsb()` for ADS-B velocity integration

### Results (101K dataset):
| Metric | Before | After | Change |
|--------|--------|-------|--------|
| Solves | 7,086 (7.0%) | 43,777 (43.2%) | **6.2× improvement** |
| Median residual | 704m | 35.69m | **19.7× better** |
| P95 residual | 2,399m | 808m | **3× better** |
| CPR cache seeds | 0 | 6,503 | 3,362 global + 3,141 local |
| DF17 altitude | 0 | 4,755 groups | Direct extraction |
| Cached altitude | 0 | 65,696 groups | DF11/DF21 borrowing |
| Clock sync points | ~3K | 47K+ | From solved position feedback |
| constrained_3sensor | 57 | 311 | 5.5× from better clock cal |

**Key lesson:** The single biggest lever was feeding the solver with known positions from ADS-B, which simultaneously seeds the position cache AND provides clock calibration references.

---

## Phase 4: Kalman Filter Replaces PI Controller

**Goal:** Replace the heuristic PI clock controller with a proper 2-state Kalman filter.

### Changes made:
1. `clock_sync.py` — `ClockPairing` rewritten with 2-state Kalman `[offset, drift]`, constant-velocity process noise `Q = q_drift * [[dt³/3, dt²/2], [dt²/2, dt]]`
2. `OUTLIER_SIGMA` tightened from 5.0 → 3.5 (works with Kalman, hurts with PI)
3. Warm-up detection via high initial covariance (replaces explicit warm-up counter)
4. `MAX_PAIR_VARIANCE_US2`: 600 → 50.0 μs² to match Kalman variance scale
5. Dead code removed: `SIGMA_ALT_M`, `_toa_only_rms`, `EMA_VELOCITY_ALPHA`, `_cornwall_grid_search`
6. `frisch.py` — `max_nfev` parameter fix (was hardcoded)
7. `main.py` — Processing loop extracted into `_process_group()`

### Tried and reverted:
| Change | Why it failed |
|--------|--------------|
| Multi-pass solver | Only +23 solves, not worth complexity |
| Cache age 300s | -1,380 solves due to velocity consistency rejections |
| 3.5σ outlier gate alone (without Kalman) | -2,188 solves — PI controller can't handle tight gate |

### Results (101K dataset):
| Metric | Before (PI) | After (Kalman) | Change |
|--------|-------------|----------------|--------|
| Solves | 43,777 | 45,780 (45.1%) | +2,003 (+4.6%) |
| Median residual | 35.69m | 46.50m | Worse (dilution from marginal solves) |
| P95 residual | 808m | 739m | **8.5% better** |
| Pair variances | — | 1.2–13.5 μs² | Tight convergence |

---

## Phase 5: Kalman Filter Fine-Tuning (Q1/Q2/Q7/Q8/Q9)

**Goal:** Optimize the Kalman clock filter parameters and fix subtle bugs.

### Changes made:
1. **Q1** (`clock_sync.py`, `main.py`) — Fixed dt to use message timestamps instead of `time.monotonic()`. Tuned `_Q_DRIFT = 1e-18` (swept 1e-18 to 1e-12).
2. **Q9** (`main.py`) — Gated solver→clock feedback with `result.residual_m < 200.0`. Optimal: recovers 46K sync points vs 7.7K with strict ≥3-sensor gate.
3. **Q8** (`clock_sync.py`) — Joseph form covariance update `P = (I-KH)P(I-KH)' + KRK'` for numerical stability.
4. **Q7** (`position_cache.py`, `main.py`) — Uncertainty-aware speed gating using `residual_m`. Subtracts `2*(cached + new residual)` from distance before speed check.
5. **Q2** (`solver.py`, `main.py`) — Failure reason tracking. `solve_group()` returns `(result, fail_reason)` tuple with histogram.

### Parameter tuning results:
- `_Q_DRIFT`: 1e-18 optimal (swept 1e-18 to 1e-12)
- `MAX_PAIR_VARIANCE_US2`: 50 optimal (25–500 all equivalent, 10 too tight)
- Q9 gate: `residual_m < 200.0` optimal (beats ≥3 sensors AND <100m)

### Results (101K dataset):
| Metric | Before | After | Change |
|--------|--------|-------|--------|
| Solves | 45,780 (45.1%) | 45,848 (45.2%) | +68 (+0.1%) |
| Median residual | 46.50m | 31.61m | **32% better** |
| P95 residual | 739m | 739.83m | ~same |
| Sync points | — | 46,505 accepted | Tight convergence |
| Pair variances | 1.2–13.5 μs² | 0.2–6.4 μs² | Much tighter |

### Failure breakdown (101K dataset):
```
residual_exceeded:       38,075  (dominant — clock noise limits)
too_few_sensors:          9,911
no_prior_2sensor:         3,937
solver_diverged:          2,558
physically_inconsistent:    802
prior_drift_exceeded:       124
gdop_2d_exceeded:           108
gdop_exceeded:               34
range_exceeded:                8
```

---

## Phase 6: Synthesis Report Implementation (R-series)

**Goal:** Implement recommendations from a full technical synthesis comparing our system to the mutability/mlat-server reference.

**Dataset:** 121,807 correlation groups (larger/harder dataset than previous 101K).

### Baseline on 121K dataset (before any R-series changes):
- Solves: 34,227 (28.1%)
- Median: 18.89m, P95: 549.94m
- `pair_variance_exceeded`: 6,700

### Changes made and kept:

**R3 — Outlier counter + filter reset** (`clock_sync.py`)
- After `MAX_CONSECUTIVE_OUTLIERS = 5` consecutive rejections, reset the Kalman filter state
- Handles clock steps (GPS lock changes) that previously caused permanent rejection
- Inspired by mutability/mlat-server's outlier counter approach
- **Impact: +2,096 solves, `pair_variance_exceeded` 6,700 → 8**

**R10 — Convergence check on solver→clock feedback** (`main.py`)
- Added `clock_cal.has_any_calibration(receptions)` gate
- Prevents early noisy solver positions from corrupting freshly initializing clock pairs
- Only feeds back when at least one clock pair in the group is already converged
- **Impact: +741 additional solves on top of R3**

**R5+R8 — Solver-covariance-weighted EKF measurement noise** (`ekf.py`, `tracker.py`)
- EKF `update()` now accepts optional `measurement_noise_m` parameter
- Noise derived from `residual_m × gdop`, clamped to [50, 2000]m
- Replaces fixed 200m isotropic assumption
- High-quality solves (low residual, low GDOP) get stronger weight
- **Impact: +3,088 track fixes accepted (+13.6%)**

**R7 — EKF-covariance-scaled prediction weight** (`tracker.py`)
- For 2-sensor prediction-aided solves, `prediction_weight = 500 / pred_uncertainty`
- Clamped to [1.0, 12.0]
- Tight EKF prediction → strong anchor; uncertain prediction → let geometry dominate
- **Impact: Improved 2-sensor prediction-aided solve quality**

### Tried and reverted:

**R1 — Adaptive `_R` estimation from innovation EMA** (`clock_sync.py`)
- Tracked EMA of innovation² per clock pair
- Estimated `R_hat = EMA(ν²) - P⁻₀₀` to adapt measurement noise per pair
- **Result: -787 solves, destabilized clock pairs (`pair_variance_exceeded` 23 → 1,390)**
- Problem: During bootstrap, large innovations inflated the EMA, causing R to jump to ceiling when adaptive mode activated at n=10
- Dead code removed after testing

### Final results (121K dataset):
| Metric | Baseline | Final (R3+R10+R5/R8+R7) | Change |
|--------|----------|--------------------------|--------|
| **Solver solves** | 34,227 (28.1%) | 37,064 (30.4%) | **+2,837 (+8.3%)** |
| Median residual | 18.89m | 20.94m | Slight dilution (more marginal solves) |
| **P95 residual** | 549.94m | 495.71m | **10% better** |
| pair_variance_exceeded | 6,700 | 23 | **99.7% reduction** |
| **Track fixes accepted** | 22,754 (22.2%) | 25,842 (25.2%) | **+3,088 (+13.6%)** |

---

## Cumulative Progress (101K dataset reference)

| Phase | Solves | Rate | Median | P95 | Key Lever |
|-------|--------|------|--------|-----|-----------|
| 1 — Baseline | ~6,847 | 6.8% | ~700m | ~2,400m | — |
| 2 — Parameter tuning | 7,086 | 7.0% | 704m | 2,399m | Variance filter loosening |
| 3 — **CPR seeding** | 43,777 | 43.2% | 35.69m | 808m | **ADS-B position/altitude injection** |
| 4 — Kalman clock sync | 45,780 | 45.1% | 46.50m | 739m | Formal Kalman filter |
| 5 — KF fine-tuning | 45,848 | 45.2% | 31.61m | 740m | Message-ts dt, Joseph form, gated feedback |

*Phase 6 measured on different 121K dataset — not directly comparable.*

---

## Key Technical Lessons

1. **SDR dongles ≠ GPS receivers.** Most MLAT literature assumes GPS-disciplined timestamps. Our ~15–20μs jitter SDRs require fundamentally different parameter tuning — tight process noise (`_Q_DRIFT = 1e-18`), generous measurement noise (`_R = 25e-12`), and soft outlier gates.

2. **The biggest lever is data, not algorithms.** ADS-B CPR seeding gave a 6.2× improvement. All subsequent algorithmic improvements combined added ~15% on top of that.

3. **Feedback loops are powerful but dangerous.** Feeding solver positions back to clock calibration (Q9) was critical (+46K sync points), but needs careful gating (`< 200m` residual + convergence check) to avoid positive feedback bias.

4. **Adaptive estimation is fragile.** Both adaptive `_R` (R1) and EMA-based approaches struggled with the bootstrap problem — early noisy observations corrupt the estimates. Fixed, well-tuned parameters outperformed adaptive ones.

5. **Joseph form matters.** The numerically stable covariance update prevented subtle divergence that accumulated over thousands of updates.

6. **Clock steps need explicit handling.** Without the outlier counter reset (R3), clock steps caused permanent rejection of affected pairs, silently degrading calibration. This was the single biggest win in the final phase.

---

## Files Modified (Cumulative)

| File | Changes |
|------|---------|
| `mlat-solver/clock_sync.py` | Kalman filter (replaces PI), Joseph form, outlier counter reset, 3.5σ gate |
| `mlat-solver/main.py` | CPR seeding, clock feedback gating, convergence check, failure tracking, location overrides |
| `mlat-solver/solver.py` | Failure reason returns, 2D GDOP gate, outlier sensor rejection |
| `mlat-solver/frisch.py` | max_nfev fix, soft_l1 loss, atmospheric correction |
| `mlat-solver/position_cache.py` | ADS-B velocity integration, uncertainty-aware speed gating |
| `mlat-solver/adsb_decoder.py` | **NEW** — CPR decoder, DF17 extraction |
| `mlat-solver/gdop.py` | 2D GDOP function, pre-check GDOP |
| `mlat-solver/inamdar.py` | Algebraic TDOA init (5-sensor, 4-sensor+alt) |
| `mlat-solver/atmosphere.py` | Markochev refraction model |
| `mlat-solver/geo.py` | WGS84↔ECEF conversions |
| `track-builder/ekf.py` | Adaptive measurement noise parameter, Joseph form |
| `track-builder/tracker.py` | Residual×GDOP noise weighting, covariance-scaled prediction weight |
| `track-builder/main.py` | Layer 5 entry point |
| `correlation-engine/correlator.py` | Time-windowed grouping |
| `modes-decoder/decoder.py` | DF17 altitude extraction |
