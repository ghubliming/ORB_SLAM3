# IMU Pre-integration Module: Dependency Analysis and Decomposition Guide

## 1. Overview

This document analyses the IMU pre-integration module inside ORB-SLAM3, maps every dependency relationship it has with the rest of the system, and proposes a concrete step-by-step strategy for extracting it into a standalone, reusable library.

ORB-SLAM3 treats IMU pre-integration as a first-class citizen: the `IMU::Preintegrated` object is created during tracking, carried all the way into the map, and consumed again during graph optimisation. Because these three roles (sensing, mapping, optimisation) each touch the same object, a naive extraction attempt immediately reveals three distinct coupling surfaces that must each be handled separately.

---

## 2. Source Files That Form the Module

| File | Role |
|------|------|
| `include/ImuTypes.h` | Public API — all five core classes plus free Lie-algebra functions |
| `src/ImuTypes.cc` | Implementation — integration numerics, bias correction, serialisation helpers |

The two files together are **self-contained with respect to business logic**: every mathematical operation (rotation integration, covariance propagation, Jacobian updates, bias-linearised corrections) lives here and nowhere else.

### 2.1 Direct Third-party Headers Included by `ImuTypes.h`

```
<vector>          (std)
<mutex>           (std)
<opencv2/core/core.hpp>     — used only for cv::Point3f constructor overload of IMU::Point
<Eigen/Core>
<Eigen/Geometry>
<Eigen/Dense>
<sophus/se3.hpp>            — Sophus::SE3<float> in IMU::Calib; Sophus::SO3f in maths
<boost/serialization/…>    — Boost.Serialization for save/load
"SerializationUtils.h"     — thin wrapper that serialises Sophus types with Boost
```

The only ORB-SLAM3 header pulled in by `ImuTypes.h` is `SerializationUtils.h`. That file contains only template helpers and has **no transitive dependency on any other ORB-SLAM3 class**.

`src/ImuTypes.cc` additionally includes:

```
"Converter.h"        — used for nothing in the current compiled code (legacy include)
"GeometricTools.h"   — not directly used; was presumably needed in an earlier version
```

Both of these includes in `ImuTypes.cc` are **unused** by any symbol in that translation unit and can be removed without breaking anything.

---

## 3. Class Catalogue

### 3.1 `IMU::Point`
Single raw sensor reading: 3-D acceleration `a`, angular velocity `w`, timestamp `t`.  
Constructor overloads accept either `float` scalars or `cv::Point3f` values.  
The `cv::Point3f` overload is the **only** reason `<opencv2/core/core.hpp>` is in `ImuTypes.h`.

### 3.2 `IMU::Bias`
Six floats (`bax bay baz bwx bwy bwz`).  
Implements Boost serialisation and a streaming `operator<<`.  
No dependency beyond Eigen and Boost.

### 3.3 `IMU::Calib`
Stores the camera-IMU extrinsic `mTbc` / `mTcb` (as `Sophus::SE3<float>`) plus the two diagonal noise covariance matrices `Cov` and `CovWalk`.  
Constructed from a single `Sophus::SE3<float>` and four scalar noise parameters.  
Full Boost serialisation.

### 3.4 `IMU::IntegratedRotation`
Helper for one rotation integration step: computes the Rodrigues rotation matrix `deltaR` and the right Jacobian `rightJ` from a single gyroscope reading.  
Uses `Sophus::SO3f::hat()` for the skew-symmetric matrix.

### 3.5 `IMU::Preintegrated` (core class)
Accumulates all IMU measurements between two keyframes.  
Tracks the 15-D covariance matrix `C`, the 15-D information matrix `Info`, delta rotation/velocity/position `(dR, dV, dP)`, five bias-sensitivity Jacobians `(JRg, JVg, JVa, JPg, JPa)`, original bias `b`, updated bias `bu`, and bias delta vector `db`.  
Stores all raw measurements internally in `mvMeasurements` to allow `Reintegrate()`.  
Protected by `std::mutex mMutex` for thread safety.

**Key methods:**

| Method | Purpose |
|--------|---------|
| `IntegrateNewMeasurement(a, w, dt)` | Append one IMU reading and propagate state |
| `SetNewBias(bu_)` | Record an updated bias; does NOT reintegrate |
| `Reintegrate()` | Re-run integration from scratch with `bu` as the new origin bias |
| `MergePrevious(pPrev)` | Absorb a predecessor's measurements (keyframe marginalisation) |
| `GetDeltaRotation/Velocity/Position(b_)` | First-order bias-linearised corrections for an arbitrary query bias |
| `GetUpdatedDelta*()` | Same corrections using the stored `bu` |
| `GetOriginalDelta*()` | Raw integrated values without bias correction |

### 3.6 Free Lie-Algebra Functions

```cpp
Eigen::Matrix3f RightJacobianSO3(x, y, z);
Eigen::Matrix3f RightJacobianSO3(v);
Eigen::Matrix3f InverseRightJacobianSO3(x, y, z);
Eigen::Matrix3f InverseRightJacobianSO3(v);
Eigen::Matrix3f NormalizeRotation(R);
```

These live in namespace `ORB_SLAM3::IMU` and are used both inside `ImuTypes.cc` and inside `G2oTypes.cc`.

---

## 4. Dependency Graph

### 4.1 What the IMU Module Depends On (inbound arrows TO `ImuTypes`)

```
ImuTypes.h / ImuTypes.cc
    ├── Eigen3             (linear algebra — mandatory)
    ├── Sophus             (SE3 / SO3 — mandatory)
    ├── Boost.Serialization (save/load — mandatory if persistence is needed)
    ├── OpenCV core        (cv::Point3f — one constructor overload only)
    └── SerializationUtils.h (ORB-SLAM3 header, but template-only, no symbols)
```

### 4.2 What Depends On the IMU Module (outbound arrows FROM `ImuTypes`)

```
Frame.h / Frame.cc
    ├── IMU::Calib  mImuCalib
    ├── IMU::Bias   mImuBias, mPredBias
    ├── IMU::Preintegrated*  mpImuPreintegrated
    └── IMU::Preintegrated*  mpImuPreintegratedFrame

KeyFrame.h / KeyFrame.cc
    ├── IMU::Preintegrated*  mpImuPreintegrated
    ├── IMU::Preintegrated   mBackupImuPreintegrated   (value, for serialisation)
    ├── IMU::Calib           mImuCalib
    ├── IMU::Bias            mImuBias, mBiasGBA, mBiasMerge
    └── KeyFrame*            mPrevKF / mNextKF         (linked list for integration span)

Tracking.h / Tracking.cc
    ├── GrabImuData(IMU::Point)
    ├── ParseIMUParamFile()    → constructs IMU::Calib
    └── UpdateFrameIMU()       → passes IMU::Bias to frames

G2oTypes.h / G2oTypes.cc
    ├── EdgeInertial(IMU::Preintegrated*)
    │       — 9-residual multi-edge connecting pose_i, v_i, bg_i, ba_i, pose_j, v_j
    │       — reads dR, dV, dP, JRg, JVg, JVa, JPg, JPa, b, Info from Preintegrated
    └── EdgeInertialGS(IMU::Preintegrated*)
            — variant that also optimises gravity direction and scale

Optimizer.cc
    └── (indirect) — builds EdgeInertial graphs in LocalBundleAdjustment,
                     FullInertialBA, InertialOptimization, etc.

System.h / System.cc
    └── IMU::Calib  — passed to Tracking on construction
```

### 4.3 Dependency Matrix (✓ = uses, — = no direct use)

| Consumer \ IMU type | `Point` | `Bias` | `Calib` | `IntegratedRotation` | `Preintegrated` | Free functions |
|---|---|---|---|---|---|---|
| `Frame` | ✓ | ✓ | ✓ | — | ✓ | — |
| `KeyFrame` | — | ✓ | ✓ | — | ✓ | — |
| `Tracking` | ✓ | ✓ | ✓ | — | ✓ | — |
| `G2oTypes` | — | ✓ | — | — | ✓ | ✓ |
| `Optimizer` | — | ✓ | — | — | ✓ | — |
| `System` | — | — | ✓ | — | — | — |

---

## 5. Coupling Analysis

### 5.1 Coupling Surface 1 — Data Ownership (Frame / KeyFrame)

`Frame` and `KeyFrame` store raw pointers to `IMU::Preintegrated` objects. Ownership is informal: Tracking allocates `Preintegrated` objects and hands the pointer to Frame; Frame hands it to KeyFrame on promotion. There is no smart pointer, no documented destructor responsibility.

**Coupling tightness: HIGH.** To decouple, the allocation and lifetime of `Preintegrated` must be managed explicitly (e.g., `std::unique_ptr` in Frame, move-on-promote into KeyFrame).

### 5.2 Coupling Surface 2 — Streaming Integration (Tracking)

`Tracking::GrabImuData()` receives raw `IMU::Point` objects and buffers them in `mlQueueImuData`. On every call to `Track()`, the buffered measurements are fed into the current frame's `mpImuPreintegrated` via `IntegrateNewMeasurement()`. The bias used for integration is taken from the previous keyframe's `mImuBias`.

**Coupling tightness: MEDIUM.** The integration call itself is a clean function call. The coupling arises from Tracking reaching directly into Frame's `mpImuPreintegrated` pointer instead of going through an interface.

### 5.3 Coupling Surface 3 — Optimisation Residuals (G2oTypes / Optimizer)

`EdgeInertial` holds a raw `IMU::Preintegrated*` and accesses eight public members (`dR`, `dV`, `dP`, `JRg`, `JVg`, `JVa`, `JPg`, `JPa`, `b`, `Info`) directly. This is the tightest coupling: the optimisation edge is essentially reading the internal state of `Preintegrated`.

**Coupling tightness: HIGH.** The public data layout of `Preintegrated` is part of the de-facto API of `EdgeInertial`. Changing field names or packing would break the optimiser.

### 5.4 Coupling Surface 4 — Serialisation (KeyFrame / Boost)

`KeyFrame` serialises `mBackupImuPreintegrated` (a value, not a pointer) using Boost. `Preintegrated` exposes its own `serialize()` method. The private `struct integrable` inside `Preintegrated` also has a `serialize()` method, making the raw measurement buffer part of the persisted state.

**Coupling tightness: LOW** (serialisation is already well-encapsulated inside `Preintegrated::serialize()`).

### 5.5 Coupling Surface 5 — OpenCV Type in Public API

The `IMU::Point` constructor accepting `cv::Point3f` forces a dependency on OpenCV in any code that includes `ImuTypes.h`, even if no image processing is done.

**Coupling tightness: LOW** (a single constructor overload).

---

## 6. Proposed Decomposition Strategy

The goal is to produce a standalone static or shared library `imu_preintegration` that:

1. Has no dependency on any ORB-SLAM3 application class (Frame, KeyFrame, Tracking, Optimizer).
2. Depends only on Eigen, Sophus, Boost.Serialization, and optionally OpenCV.
3. Can be included in ORB-SLAM3 by adding it as a CMake sub-target, preserving identical behaviour.
4. Can also be compiled and tested independently.

### 6.1 Step 1 — Remove Unused Includes from `ImuTypes.cc`

`src/ImuTypes.cc` currently includes `"Converter.h"` and `"GeometricTools.h"`, neither of which is actually used by any symbol in that file. Remove both lines.

```diff
-#include "Converter.h"
-#include "GeometricTools.h"
```

This eliminates transitive pull-ins of `Frame.h`, `KeyFrame.h`, OpenCV mat types, etc. from the IMU module's own translation unit.

### 6.2 Step 2 — Remove the OpenCV Constructor from `IMU::Point`

The `cv::Point3f`-based constructor is syntactic sugar used in a handful of call sites in `Tracking.cc` and example code. Replace those call sites with the scalar constructor and remove the OpenCV overload from `ImuTypes.h`.

**Before (ImuTypes.h):**
```cpp
#include <opencv2/core/core.hpp>
// …
Point(const cv::Point3f Acc, const cv::Point3f Gyro, const double &timestamp):
    a(Acc.x,Acc.y,Acc.z), w(Gyro.x,Gyro.y,Gyro.z), t(timestamp){}
```

**After (ImuTypes.h):**
```cpp
// No opencv include needed
Point(const float &acc_x, const float &acc_y, const float &acc_z,
      const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
      const double &timestamp)
    : a(acc_x,acc_y,acc_z), w(ang_vel_x,ang_vel_y,ang_vel_z), t(timestamp){}
```

**Call site fix (Tracking.cc, wherever `IMU::Point(cv::Point3f, …)` is used):**
```cpp
// Before
IMU::Point(Acc, Gyro, t)
// After
IMU::Point(Acc.x, Acc.y, Acc.z, Gyro.x, Gyro.y, Gyro.z, t)
```

After this change `ImuTypes.h` has zero dependency on OpenCV.

### 6.3 Step 3 — Extract `SerializationUtils.h` into the New Library

`ImuTypes.h` includes `"SerializationUtils.h"`. This file contains only `serializeSophusSE3` template helpers and has no ORB-SLAM3 dependencies. Move it alongside `ImuTypes.h` in the new library's include directory so the IMU module remains self-contained.

### 6.4 Step 4 — Create a New CMake Sub-directory

Create the following layout:

```
imu_preintegration/
├── CMakeLists.txt
├── include/
│   ├── ImuTypes.h          (moved / symlinked from top-level include/)
│   └── SerializationUtils.h
└── src/
    └── ImuTypes.cc
```

**`imu_preintegration/CMakeLists.txt`:**
```cmake
cmake_minimum_required(VERSION 2.8)
project(imu_preintegration)

find_package(Eigen3 3.1.0 REQUIRED)

# Sophus is header-only; point to the vendored copy
set(SOPHUS_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/Thirdparty/Sophus)

add_library(imu_preintegration STATIC
    src/ImuTypes.cc
)

target_include_directories(imu_preintegration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

target_link_libraries(imu_preintegration PUBLIC
    ${EIGEN3_LIBS}
    -lboost_serialization
)
```

### 6.5 Step 5 — Update the Top-level `CMakeLists.txt`

Replace the inline compilation of `ImuTypes.cc` in the monolithic `ORB_SLAM3` target with a link to the new sub-library:

```cmake
# Add the standalone IMU library
add_subdirectory(imu_preintegration)

# In the ORB_SLAM3 shared library target, REMOVE src/ImuTypes.cc
# and ADD a link to the new library
add_library(ORB_SLAM3 SHARED
    # … all other sources, but NOT src/ImuTypes.cc …
)

target_link_libraries(ORB_SLAM3
    imu_preintegration   # <-- new sub-library
    # … rest of existing links …
)
```

### 6.6 Step 6 — Fix Data Ownership in Frame and KeyFrame

Currently `Frame` and `KeyFrame` hold raw pointers to `IMU::Preintegrated`. This makes it impossible to reason about lifetime from outside the module. Introduce explicit ownership:

```cpp
// Frame.h — change raw pointer to unique_ptr
std::unique_ptr<IMU::Preintegrated> mpImuPreintegrated;

// On KeyFrame promotion (KeyFrame constructor), transfer ownership:
// KeyFrame.cc
mpImuPreintegrated = std::move(pFrame->mpImuPreintegrated);
```

Because `IMU::Preintegrated` stores its measurements and all state internally, this change requires no modification to the `Preintegrated` class itself.

### 6.7 Step 7 — Introduce an Accessor Interface for the Optimiser

`EdgeInertial` currently reads eight public data members directly from `Preintegrated`. Introduce a lightweight data-transfer struct so the optimiser edge does not depend on the internal layout:

```cpp
// In ImuTypes.h — add to IMU::Preintegrated
struct PreintegratedData {
    float    dT;
    Eigen::Matrix3f dR;
    Eigen::Vector3f dV, dP;
    Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
    Bias     b;
    Eigen::Matrix<float,15,15> Info;
};

PreintegratedData GetData() const;
```

`EdgeInertial` then calls `pInt->GetData()` once in its constructor and stores the struct, removing the direct member access. This also makes `EdgeInertial` safe to use after the `Preintegrated` object has been destroyed (e.g., after keyframe culling).

### 6.8 Step 8 — Remove the Bi-directional KeyFrame Linked List Dependency

`KeyFrame` stores `mPrevKF` and `mNextKF` specifically to allow `MergePrevious()` to walk the chain and re-integrate across keyframe boundaries. This means `IMU::Preintegrated::MergePrevious()` is called from inside `KeyFrame` management code, creating a circular dependency:

```
KeyFrame  →  IMU::Preintegrated  →  (conceptually needs KeyFrame chain)
```

To break this, make `MergePrevious()` entirely data-driven: the caller is responsible for passing the ordered list of `Preintegrated*` objects to merge, rather than the function walking `mPrevKF` itself. (The current implementation already does this — it takes a `Preintegrated*` argument — so the linked-list walking is in the KeyFrame code, not in `Preintegrated`. No change to `ImuTypes.cc` is needed; the documentation should simply make this boundary explicit.)

---

## 7. Summary of All Coupling Points and How to Break Them

| # | Coupling point | File(s) affected | Severity | Resolution |
|---|---|---|---|---|
| 1 | Unused `#include "Converter.h"` in `ImuTypes.cc` | `src/ImuTypes.cc` | Low | Remove include |
| 2 | Unused `#include "GeometricTools.h"` in `ImuTypes.cc` | `src/ImuTypes.cc` | Low | Remove include |
| 3 | `cv::Point3f` constructor in `IMU::Point` forces OpenCV dependency | `include/ImuTypes.h` | Low | Remove overload; update call sites in `Tracking.cc` |
| 4 | `Frame` and `KeyFrame` hold raw `Preintegrated*` pointers (no ownership) | `include/Frame.h`, `include/KeyFrame.h` | High | Switch to `std::unique_ptr`; move on KeyFrame promotion |
| 5 | `EdgeInertial` directly reads eight public fields of `Preintegrated` | `include/G2oTypes.h`, `src/G2oTypes.cc` | High | Add `GetData()` accessor returning a plain struct |
| 6 | `ImuTypes.cc` compiled as part of the monolithic `ORB_SLAM3` library | `CMakeLists.txt` | Medium | Extract into `imu_preintegration` sub-library |
| 7 | `SerializationUtils.h` lives in the top-level `include/` folder | `include/SerializationUtils.h` | Low | Copy/symlink into `imu_preintegration/include/` |

### 7.1 Extra Coupling Risks in the BA Path (important for safe extraction)

The items above describe compile-time coupling. There are also **runtime coupling risks** in the optimisation path that should be addressed in the same extraction effort:

| # | Where | Runtime risk | Severity | Recommended fix |
|---|---|---|---|---|
| 8 | `G2oTypes.cc` (`EdgeInertial`) | Edge stores a live `IMU::Preintegrated*` and may read it while map/keyframe operations modify or replace the object | Critical | Snapshot immutable data at edge construction (`PreintegratedData`) instead of retaining pointer |
| 9 | `Optimizer.cc` write-back path | BA can update `KeyFrame::mImuBias` while `Preintegrated` still carries old linearisation bias until reintegration | Critical | Enforce bias update + reintegration ordering under a consistent lock/scheduling policy |
| 10 | `G2oTypes.cc` (`EdgeInertialGS`) | Gravity/scale residual uses `dT`; omitting `dT` in exported snapshot silently breaks initialisation | High | Keep `dT` in `PreintegratedData` contract |

---

## 8. Resulting Architecture After Decomposition

```
┌────────────────────────────────────────────────────────────┐
│                    imu_preintegration (static lib)          │
│                                                            │
│  ImuTypes.h / ImuTypes.cc                                  │
│  SerializationUtils.h                                      │
│                                                            │
│  External deps: Eigen3, Sophus (header-only),              │
│                 Boost.Serialization                         │
└────────────┬───────────────────────────────────────────────┘
             │  links to
             ▼
┌────────────────────────────────────────────────────────────┐
│                    ORB_SLAM3 (shared lib)                   │
│                                                            │
│  Frame.cc / KeyFrame.cc   → own Preintegrated via          │
│                              unique_ptr                     │
│  Tracking.cc              → feeds Point objects in         │
│  G2oTypes.cc / Optimizer  → consumes via GetData()         │
│                              accessor struct                │
│                                                            │
│  External deps: OpenCV, Pangolin, DBoW2, g2o, + above      │
└────────────────────────────────────────────────────────────┘
```

The key property of this architecture is that `imu_preintegration` can be compiled, unit-tested, and versioned entirely independently of ORB-SLAM3. The only contract between the two layers is the `ImuTypes.h` API and the `PreintegratedData` accessor struct.

### 8.1 Contract Note for `PreintegratedData`

To support both `EdgeInertial` and `EdgeInertialGS`, the snapshot struct must include:

```cpp
struct PreintegratedData {
    float dT; // required by gravity/scale terms
    Eigen::Matrix3f dR;
    Eigen::Vector3f dV, dP;
    Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
    Bias b;
    Eigen::Matrix<float,15,15> Info;
};
```

When using this contract, `EdgeInertial` should store `PreintegratedData` **by value** in its constructor and never dereference a live `IMU::Preintegrated*` during iterative solve.

---

## 9. Minimal Standalone Test for the Extracted Module

Once the library is extracted, the following self-contained test (no ORB-SLAM3 required) validates the core numerical behaviour:

```cpp
// test_imu_preintegration.cpp
#include "ImuTypes.h"
#include <cassert>
#include <cmath>

int main() {
    using namespace ORB_SLAM3::IMU;

    // 1. Identity: zero rotation with zero gyro input
    Sophus::SE3<float> Tbc; // identity
    Calib calib(Tbc, 1e-3f, 1e-3f, 1e-5f, 1e-5f);
    Bias zeroBias;

    Preintegrated pint(zeroBias, calib);

    // Gravity-aligned constant acceleration, zero angular velocity
    Eigen::Vector3f acc(0.f, 0.f, 9.81f);
    Eigen::Vector3f gyro(0.f, 0.f, 0.f);

    for (int i = 0; i < 100; ++i)
        pint.IntegrateNewMeasurement(acc, gyro, 0.01f);   // 1 s total

    // After 1 s of pure free-fall (no bias, identity rotation):
    // dR should be identity
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    assert((pint.GetOriginalDeltaRotation() - I).norm() < 1e-4f);

    // dV should be (0, 0, ~9.81) m/s
    Eigen::Vector3f dV = pint.GetOriginalDeltaVelocity();
    assert(std::abs(dV.z() - 9.81f) < 0.05f);

    // dP should be ~0.5 * 9.81 * 1^2 = ~4.905 m
    Eigen::Vector3f dP = pint.GetOriginalDeltaPosition();
    assert(std::abs(dP.z() - 4.905f) < 0.05f);

    // 2. Bias update and first-order correction
    Bias newBias(0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // small accel bias
    pint.SetNewBias(newBias);
    Eigen::Vector3f dP_corrected = pint.GetUpdatedDeltaPosition();
    // Corrected position should differ from original
    assert((dP_corrected - dP).norm() > 1e-6f);

    return 0;
}
```

Add it to `imu_preintegration/CMakeLists.txt`:

```cmake
add_executable(test_imu_preintegration
    test/test_imu_preintegration.cpp
)
target_link_libraries(test_imu_preintegration imu_preintegration)
```

---

## 10. Checklist for Implementation

- [ ] Remove `#include "Converter.h"` and `#include "GeometricTools.h"` from `src/ImuTypes.cc`
- [ ] Remove `cv::Point3f` constructor overload from `IMU::Point`; remove `#include <opencv2/core/core.hpp>` from `ImuTypes.h`
- [ ] Update all `cv::Point3f`-based `IMU::Point` construction sites in `Tracking.cc` to use the scalar constructor
- [ ] Create `imu_preintegration/` directory with `CMakeLists.txt`, `include/`, `src/`
- [ ] Copy `ImuTypes.h`, `SerializationUtils.h`, `ImuTypes.cc` into the new directory
- [ ] Update top-level `CMakeLists.txt`: add `add_subdirectory(imu_preintegration)`, remove `src/ImuTypes.cc` from `ORB_SLAM3` sources, add `imu_preintegration` to `target_link_libraries(ORB_SLAM3 …)`
- [ ] Change `Frame::mpImuPreintegrated` and `Frame::mpImuPreintegratedFrame` to `std::unique_ptr<IMU::Preintegrated>`
- [ ] Move ownership to `KeyFrame` on construction via `std::move`
- [ ] Add `IMU::Preintegrated::GetData()` returning a `PreintegratedData` struct
- [ ] Update `EdgeInertial` and `EdgeInertialGS` in `G2oTypes` to use `GetData()` instead of direct member access
- [ ] Write and pass the standalone test in `imu_preintegration/test/`
- [ ] Verify the full ORB-SLAM3 build still succeeds
