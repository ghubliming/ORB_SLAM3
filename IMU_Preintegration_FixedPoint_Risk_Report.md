# IMU Preintegration Fixed-Point Conversion Risk Report

## 1. Scope

This report analyses what can go wrong if ORB-SLAM3 IMU preintegration is converted from floating-point (`float`/`double`) to fixed-point arithmetic.

The analysis covers:

1. **Code-level failure modes** in the current implementation (`ImuTypes`, `Tracking`, `G2oTypes`, `Optimizer` paths)
2. **Real SLAM workflow impact** (tracking stability, map consistency, inertial BA convergence, gravity/scale initialization)

The focus is risk discovery, not implementation details.

---

## 2. Why this conversion is high risk in this codebase

Current IMU preintegration uses:

- non-linear rotation updates (`SO3::exp`, `LogSO3`, normalization),
- repeated covariance/Jacobian matrix propagation,
- long accumulation chains over many IMU samples,
- and tight coupling to optimization residuals.

These are exactly the places where fixed-point overflow, quantization, rounding bias, and accumulation drift are most dangerous.

---

## 3. Where floating-point is relied on today

### 3.1 IMU integration core (`src/ImuTypes.cc`)

`Preintegrated::IntegrateNewMeasurement(...)` repeatedly updates:

- `dR`, `dV`, `dP`,
- Jacobians `JRg`, `JVg`, `JVa`, `JPg`, `JPa`,
- covariance/information terms `C`, `Info`,
- total time `dT`.

This path contains chained multiply-add operations and matrix products per IMU sample. Any quantization error is reinjected at every step.

### 3.2 Tracking-side streaming integration (`src/Tracking.cc`)

`Tracking::PreintegrateIMU()` can integrate many samples between frames and then uses `dT`, `GetDeltaRotation/Velocity/Position(...)` for IMU prediction (`PredictStateIMU()`).

Errors in preintegration directly perturb frame-to-frame pose/velocity priors before visual correction.

### 3.3 Optimization residual construction (`src/G2oTypes.cc`)

`EdgeInertial` / `EdgeInertialGS` consume preintegration outputs and Jacobians in error and Jacobian terms.

Any fixed-point clipping/wrapping in preintegration states contaminates optimizer residuals and can destabilize inertial BA.

---

## 4. Core fixed-point failure modes and how they appear here

### 4.1 Bit growth in multiplication (silent precision loss if width is not promoted)

Fixed-point multiplication increases required bit-width. If intermediate results are cast back to narrow storage too early, high bits or fractional bits are dropped.

In this codebase, this affects:

- covariance propagation (`A * C * A^T`, `B * Nga * B^T`),
- delta updates (`dP = dP + dV*dt + 0.5*dR*acc*dt*dt`),
- Jacobian propagation (multiple chained matrix products).

**Result:** numerically plausible but systematically degraded deltas and covariance, hard to detect by inspection.

### 4.2 Rounding accumulation drift over long sequences

Single-step quantization error can be tiny, but preintegration loops apply updates many times (high-rate IMU).

In practice:

- orientation drift accumulates through repeated `dR` updates,
- velocity and position drift amplify via repeated integration,
- Jacobian drift mis-calibrates first-order bias correction.

**Result:** bias-corrected deltas become inconsistent with true motion; optimizer starts from poorer priors and may need more iterations or diverge.

### 4.3 Range-analysis miss -> overflow in edge-case motion

Bit-width selection often uses logged “typical” sequences. Real deployments include shocks, fast turns, dropped frames, and long `dt` tails.

If fixed-point range is under-provisioned:

- intermediate or stored states overflow,
- default wrap-around can produce sign flips and discontinuities,
- no NaN/exception appears to trigger obvious fault handling.

**Result:** silent corruption that appears as sporadic tracking failures or unexplained map collapse.

### 4.4 Overflow mode: wrap-around vs saturation

Wrap-around is catastrophic for state-estimation math (e.g., large positive value becomes negative). Saturation is safer for debugging but still physically wrong.

In this pipeline, either mode can break consistency:

- wrap-around: optimizer receives impossible inertial deltas/Jacobians,
- saturation: motion is clipped, causing persistent model mismatch.

### 4.5 Rounding mode bias

Truncation introduces one-sided bias. For long integration windows this becomes directional drift (especially velocity/position).

Using symmetric rounding reduces bias but raises hardware/resource cost.

---

## 5. Real SLAM workflow impact (end-to-end)

### Stage A: IMU sample ingestion and preintegration

`Tracking::GrabImuData()` -> `PreintegrateIMU()` -> `IntegrateNewMeasurement()`

- fixed-point quantization starts immediately,
- repeated update chain amplifies error,
- `dT` + deltas + Jacobians already biased before optimization.

### Stage B: Frame prediction

`PredictStateIMU()` applies `dT`, delta rotation, delta position, delta velocity to predict next state.

- bad `dR` biases attitude prediction,
- bad `dV`/`dP` biases translation and speed priors,
- visual tracking starts farther from truth and becomes less robust in low-texture / fast-motion intervals.

### Stage C: Inertial optimization edges

`EdgeInertial` / `EdgeInertialGS` consume the preintegrated outputs.

- residual linearization quality degrades,
- Jacobian inconsistency increases,
- gravity/scale initialization is vulnerable if `dT` and inertial deltas are quantized aggressively.

### Stage D: Bias update / reintegration loop

After BA updates biases, reintegration is expected to restore consistency.

If fixed-point already clipped/wrapped earlier measurements, reintegration cannot recover lost information; it only recomputes from already-degraded numeric representation.

---

## 6. Most dangerous practical failure

The highest operational risk is:

1. range sizing based on non-worst-case logs,
2. edge-case motion causes overflow,
3. silent wrap-around corrupts preintegration,
4. corruption propagates into prediction and BA,
5. failure appears as intermittent tracking/map instability instead of obvious numeric exception.

This is difficult to debug because outputs remain finite and syntactically valid.

---

## 7. Hotspots to treat as “must-not-overflow”

If fixed-point conversion is attempted, the following quantities require explicit dynamic-range proof with worst-case scenarios:

- `dR`, `dV`, `dP`, `dT`
- Jacobians `JRg`, `JVg`, `JVa`, `JPg`, `JPa`
- covariance/information blocks used by inertial edges
- all matrix multiply intermediates in propagation and residual formation

Additionally, all fixed-point assignment points must explicitly define:

- intermediate type width,
- rounding mode,
- overflow mode (development should avoid silent wrap-around behavior).

---

## 8. Validation requirements before trusting fixed-point

Minimum validation should include:

1. **Worst-case synthetic stress**: high angular rates, high linear acceleration, long frame gaps, bursty timestamps.
2. **Long-horizon drift tests**: many consecutive integrations to expose accumulation bias.
3. **A/B replay** against floating-point baseline:
   - preintegration deltas/Jacobians/covariance,
   - tracking success rate,
   - BA convergence behavior,
   - final trajectory/map scale.
4. **Overflow instrumentation**: counters/logging for clipping/wrap events at every critical state and intermediate.

Without this level of validation, fixed-point conversion is likely to fail in real deployment despite passing short “normal” logs.

---

## 9. Conclusion

Converting ORB-SLAM3 IMU preintegration to fixed-point is feasible only with strict numeric design and stress validation.  
The main hazards are **overflow and rounding accumulation**, and the main system-level consequence is **silent corruption of inertial priors/residuals** that destabilizes tracking and optimization.

In other words: the risk is less “compile breaks” and more “SLAM still runs but becomes intermittently wrong.”
