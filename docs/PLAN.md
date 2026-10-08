# Magnetic-only detumble implementation

The project now focuses on a 3U CubeSat using one three-axis magnetometer and three magnetic rods to slow an uncontrolled tumble. Completion means low rotation, with continued damping. Earth/Sun pointing, gyroscopes, Sun sensors, and reaction wheels are outside this mission.

## 1. Establish the magnetic baseline

- [x] Retain quaternion rigid-body dynamics, prescribed orbit, magnetometer errors, coil blanking, and limited magnetic torques.
- [x] Remove Sun/gyro/wheel simulation, assets, modes, telemetry, and tests.
- [x] Keep filtered B-dot as the default controller.
- [x] Score true rate below 0.5 deg/s continuously for 30 s; verify the final 300 s stay below 0.7 deg/s.
- [x] Provide magnetometer-only `BOOT`, `DETUMBLE`, `LOW_RATE`, and `SAFE` modes.

## 2. Add a dated Earth-field model

- [x] Bundle the official IGRF-14 table with checksum, attribution, and a reproducible header generator.
- [x] Implement degree-13 geocentric synthesis, 2025–2030 secular variation, and ECI/ECEF rotation.
- [x] Retain the tilted dipole for simpler and mismatched-model experiments.
- [x] Validate independent north/east/down fixtures against the official Fortran program, including both poles.

## 3. Estimate attitude and rate without a gyro

- [x] Use known orbit/time references and magnetic history without truth initialization.
- [x] Initialize multiple possible orientations and rates through a deterministic batch fit.
- [x] Estimate quaternion attitude, angular velocity, and magnetometer bias with a multiplicative EKF.
- [x] Preserve uncertainty when orientation/rate are ambiguous; gate inconsistent measurements.
- [x] Test arbitrary seeded orientations, initially field-parallel rotation, static-field unobservability, outliers, and realistic sensor errors.

## 4. Compare controllers

- [x] Add optional estimated-rate magnetic damping.
- [x] Require 30 s of sustained rate confidence before handoff; fall back when confidence is lost.
- [x] Use an uncertainty margin and dwell before confirming `LOW_RATE`.
- [x] Record convergence/handoff, rate and attitude error, saturation, dipole effort, and false low-rate entries.
- [x] Support paired randomized runs, campaign slicing, and reproducible CSV analysis.

## 5. Present the mission

- [x] Route CLI, campaign runner, and viewer through the same `DetumbleMission` step.
- [x] Simplify the viewer to three scenario buttons, fixed 20× playback, pause/reset, readable rate/goal status, and simple smoothed X/Y/Z strength bars in a permanent quarter-width sidebar.
- [x] Use NASA's ICECube exterior in orbit and a project-authored body-frame hardware schematic in the sidebar; keep geometry separate from physical inertia.
- [x] Keep component positions and dimensions in hover tooltips, with a short magnetometer legend and no central reference cross.
- [x] Verify the desktop viewer visually and update its screenshot.
- [x] Update the public README, canonical reference, magnetic-field walkthrough, attribution, and learning notes to match the current mission and viewer.

Validation evidence, campaign configuration, and measured controller comparisons are recorded in `docs/validation/README.md`. Defaults remain a 0.02 s timestep, 10 Hz measurements, B-dot gain 50,000, and a 0.2 s derivative filter. The estimator is enabled in shadow mode for the default B-dot mission.
