# Detumble Technical Reference

This document is the canonical reference for Detumble's implemented architecture, build system, conventions, and technical assumptions. Update it whenever a significant implementation or architecture decision changes.

## Current Status

Phases 0 through 15 are complete for the `v0.1.0` portfolio release. Detumble has a validated autonomous detumble and Sun-pointing mission with rigid-body attitude dynamics, deterministic initial conditions, a circular orbit, magnetic and Sun environments, seeded realistic sensor errors, eclipse detection, TRIAD attitude determination with gyro propagation, saturated magnetorquers, quaternion PD pointing control, a four-reaction-wheel pyramid, autonomous flight modes, configurable headless CSV runs, Monte Carlo robustness analysis, and a desktop viewer driven by the same simulation core. The viewer uses a custom, dimensionally consistent 3U model with configuration-driven ADCS component locations, environment overlays, deterministic scenario presets, and in-application guidance. Current macOS, Ubuntu, and Windows GitHub Actions runners all build the application and pass its `103` tests.

## Repository Layout

```text
apps/                   Application entry points
  cli_main.cpp          Headless simulator entry point
  monte_carlo_main.cpp  Headless randomized validation campaign
  viewer_main.cpp       Desktop viewer entry point
assets/models/          Project-authored runtime 3D asset
assets/scripts/         Reproducible 3D-asset generation scripts
cmake/                  Shared CMake helpers
docs/
  media/                Portfolio screenshot and mission demo
  PLAN.md               Sequential development roadmap
  REFERENCE.md          Canonical repository reference
include/detumble/       Public C++ headers
src/                    Simulation library implementation
tests/                  Unit and integration tests
THIRD_PARTY.md          Dependency and asset provenance notes
.github/workflows/      Cross-platform continuous integration
```

`docs/CONCEPTS.md` is a private learning journal and is intentionally excluded from version control.

## Build Architecture

The project uses C++20 and CMake 3.25 or newer. Ninja is the standard local and continuous-integration generator. Debug and release workflows are defined in `CMakePresets.json`.

Targets:

- `detumble_core` is the rendering-independent static library. Other targets may access it through the `detumble::core` alias.
- `detumble_cli` is the headless executable and is emitted with the filename `detumble`.
- `detumble_monte_carlo` runs randomized native-backend mission campaigns and writes per-run and aggregate metrics.
- `detumble_viewer` is the raylib desktop viewer with Dear ImGui controls, status, and mission progress.
- `detumble_tests` contains tests discovered and run through CTest.

The simulation core must never depend on viewer or user-interface code. The CLI and viewer may both link to the core.

## Dependencies

Dependencies are fetched by CMake and pinned to release archives and SHA-256 hashes:

- Eigen 5.0.1 provides vector, matrix, quaternion, and decomposition primitives.
- Catch2 3.15.3 provides the test framework and CTest discovery.
- raylib 6.0 provides the desktop window, camera, rendering, input, and GLB loader.
- Dear ImGui 1.92.7 provides immediate-mode viewer controls.
- rlImGui commit `3bc5731c4216bb8caa67fbea24aa85ce80d57ccb` bridges raylib 6.0 and Dear ImGui 1.92.7.

Visualization dependencies are only fetched when `DETUMBLE_BUILD_VIEWER` is enabled. Dear ImGui and rlImGui do not provide the CMake targets needed here, so Detumble defines small static-library targets from their official source files. Third-party targets do not inherit Detumble's project-warning policy.

## Compiler Policy

The project enables C++20 without compiler-specific language extensions. Project targets use strict warnings on Clang, GCC, and MSVC. Warnings are not treated as errors so that compiler upgrades do not unexpectedly block development, but project code should build without warnings.

## Testing and Continuous Integration

Catch2 tests are exposed through CTest. GitHub Actions configures, builds, and tests the debug preset on current hosted macOS, Linux, and Windows runners. The Linux job installs raylib's required audio, OpenGL, and X11 development packages before configuration.

The smoke test verifies that an executable can link against `detumble_core` and call its public API. Frame tests lock down state defaults, quaternion storage, normalization, integration, known-axis rotations, inverse frame transformations, and rotation-composition order. Dynamics tests cover conservation and known torque cases. Viewer tests cover deterministic batching and the Eigen-to-raylib rotation convention. Orbit and environment tests cover periodicity, inclination, magnetic-field behavior, Sun-frame conversion, Earth occultation, and orbital eclipse transitions. Sensor, actuator, estimator, controller, and flight-software tests cover sample timing, seeded error replay, bias, noise, quantization, saturation, Sun-sensor visibility, coil-off sampling, B-dot control, gyro measurements, TRIAD geometry, gyro attitude propagation, eclipse behavior, shortest quaternion errors, PD torque limiting, reaction-wheel allocation and failures, pointing dwell logic, mode transitions, mission metrics, and full seeded detumble and autonomous-mission scenarios. A regression campaign case ends in eclipse after successful acquisition, ensuring endpoint lighting is not mistaken for mission failure. Spacecraft-visual tests cover physical dimensions, component axes and normals, reaction-wheel geometry, and component frame transformation.

## Technical Conventions

These conventions apply before any physics implementation is added:

- Use SI units internally.
- Include units in identifiers when ambiguity would otherwise be likely.
- Use a deterministic seed for every randomized simulation.
- Advance simulation state with a fixed timestep independent of rendering rate.
- Keep truth state inaccessible to flight software, sensors, estimators, and controllers except through explicit simulated measurements.
- Document the reference frame of every physical vector.

## Reference Frames

Every physical vector must identify the frame in which its components are expressed. Function and member names use suffixes such as `_body` and `_inertial` when the type alone cannot communicate the frame. All frames are right-handed.

### Earth-Centered Inertial Frame

The Earth-centered inertial frame, abbreviated ECI, has its origin at Earth's center:

- `+Z` follows Earth's mean north rotation axis.
- `+X` is a fixed reference direction in Earth's equatorial plane.
- `+Y` completes the right-handed frame.

The simplified simulator does not require the precision of a dated astronomical frame. Simulation time zero defines the orbit and rotating-dipole phases; it is not tied to a real calendar epoch.

### Local Orbital Frame

The local orbital frame uses the radial, transverse, normal convention, abbreviated RTN:

- `+R` points from Earth's center toward the spacecraft.
- `+N` points along the orbit angular momentum vector `r × v`.
- `+T = N × R` points along track for the planned circular prograde orbit.

This definition ensures `R × T = N`. The code will use `radial`, `transverse`, and `normal` terminology rather than the ambiguous term LVLH.

### Spacecraft Body Frame

The body frame is fixed to the generic 3U CubeSat:

- Its origin is the modeled center of mass.
- `+Z` follows the long axis from the deployer/aft end toward the forward end.
- `+X` points through a designated side panel.
- `+Y = Z × X` completes the right-handed frame.

The custom visual model marks these axes and preserves this orientation.

## Attitude State

`AttitudeState` contains:

- `body_to_inertial`, the spacecraft attitude quaternion.
- `angular_velocity_body_rad_s`, the angular velocity of the body relative to the inertial frame, expressed in body coordinates and measured in radians per second.

The default state is identity attitude with zero angular velocity.

## Quaternion Convention

`body_to_inertial` is an active Hamilton quaternion that maps body-frame vector components into ECI components:

```text
vector_inertial = body_to_inertial * vector_body
```

The reverse transform uses the normalized quaternion's conjugate. Positive rotations follow the right-hand rule.

Quaternion multiplication composes rotations right to left. Given a body-to-intermediate rotation and an intermediate-to-inertial rotation:

```text
body_to_inertial = intermediate_to_inertial * body_to_intermediate
```

Eigen's quaternion constructor accepts components as `(w, x, y, z)`, while `coeffs()` exposes them in `(x, y, z, w)` order. Code must use named accessors or explicitly document the order when serializing. A quaternion and its negation represent the same physical attitude.

## Attitude Normalization and Propagation

Attitude operations normalize their input quaternion. A quaternion with a non-finite norm or norm below `1e-12` is rejected because it cannot represent a valid rotation.

`integrate_attitude` assumes body-frame angular velocity is constant during one nonnegative timestep. For angular speed `|ω|`, timestep `dt`, angle `θ = |ω|dt`, and unit axis `u = ω/|ω|`, it constructs:

```text
increment = [cos(θ/2), u sin(θ/2)]
attitude_next = normalize(attitude * increment)
```

Right multiplication is required because angular velocity is expressed in body coordinates. This exact constant-rate helper establishes the convention and remains useful for focused tests. The coupled RK4 propagator uses the equivalent quaternion derivative while angular velocity changes.

## Generic 3U Rigid Body

The initial spacecraft model is a uniform rectangular cuboid:

- Mass: `4.0 kg`
- Body dimensions: `0.10 × 0.10 × 0.34 m`
- Center of mass: body-frame origin
- Principal axes: aligned with the body axes
- Principal moments: `[0.0418667, 0.0418667, 0.00666667] kg·m²`

The moments use the uniform-cuboid formulas:

```text
Ixx = m/12 (y² + z²)
Iyy = m/12 (x² + z²)
Izz = m/12 (x² + y²)
```

This is a transparent baseline rather than a claim about a particular flight vehicle. Component-level mass properties can replace it later without changing the dynamics interface.

## Rigid-Body Dynamics

Angular acceleration follows Euler's equation in principal body coordinates:

```text
I ω_dot = torque - ω × (Iω)
```

`angular_acceleration_body_rad_s2` accepts a body-frame angular velocity and a body-frame external torque. It includes the gyroscopic coupling term and rejects non-finite inputs or nonphysical body properties.

## Coupled RK4 Propagation

`propagate_rigid_body_rk4` advances angular velocity and the body-to-inertial quaternion together with fixed-step classical fourth-order Runge-Kutta integration. Applied body torque is held constant during a step.

For body-frame angular velocity, quaternion kinematics are:

```text
q_dot = 1/2 q * [0, ω]
```

RK4 intermediate quaternions are not normalized because they are temporary derivative samples. The final quaternion is normalized once after the weighted RK4 update. A zero-duration step returns a normalized copy of the input state.

## Initial Conditions

`random_attitude_state` generates:

- A quaternion distributed uniformly over three-dimensional orientations.
- An angular-velocity direction distributed uniformly over the unit sphere.
- An angular-speed magnitude uniformly distributed between configured bounds.

The generator uses `std::mt19937_64` with an explicit 53-bit conversion to a unit-interval double. Every scenario records a 64-bit seed so it can be reproduced.

## Shared Simulation Runner

`Simulation` is the rendering-independent owner of one scenario's configuration, rigid-body properties, true attitude state, and completed-step count. Its default configuration is the Phase 2 torque-free case: seed `42`, timestep `0.01 s`, and an initial angular speed between 5 and 15 degrees per second.

`step()` advances exactly one RK4 timestep. `reset()` regenerates the initial state from the recorded seed. Elapsed simulation time is calculated as `step_count × timestep` instead of accumulated separately. This keeps the CLI and viewer on one deterministic code path and avoids time drift between two clocks.

The runner accepts an optional body-frame torque per step so later actuators and controllers can use it without introducing viewer dependencies into the core.

## Circular Orbit Model

`CircularOrbitConfig` defines a two-body circular orbit with these defaults:

- Mean Earth radius: `6,371,000 m`
- Altitude: `500,000 m`
- Earth gravitational parameter: `3.986004418e14 m^3/s^2`
- Inclination: `51.6 deg`
- Initial argument of latitude: `0 rad`

The orbit radius is `r = Earth radius + altitude`. Mean motion and period are:

```text
n = sqrt(mu / r^3)
period = 2 pi / n
```

At simulation time zero, position is on inertial `+X`. The orbit plane is produced by rotating an equatorial circular orbit about inertial `+X` by the configured inclination. A positive inclination gives a northbound velocity at the initial equator crossing.

`circular_orbit_state` calculates inertial position and velocity analytically from time. It does not numerically integrate translation, so the radius remains constant and the state is exactly periodic apart from floating-point roundoff. The default period is approximately `5,668.14 s`, or `94.47 min`.

This is an environment generator rather than a mission-trajectory model. It intentionally omits drag, oblateness, precession, and other orbital perturbations.

## Tilted-Dipole Magnetic Field

`TiltedDipoleConfig` defines a centered Earth magnetic dipole with these defaults:

- Reference radius: `6,371,000 m`
- Equatorial surface magnitude: `31.2 microtesla`
- Dipole tilt: `11 deg`
- Earth rotation rate: `7.2921150e-5 rad/s`

The dipole-moment axis points mostly toward inertial `-Z`. This sign makes the modeled northern polar field point inward toward Earth and the equatorial field point generally northward. Its horizontal tilt rotates around inertial `+Z` with Earth.

For radial unit vector `r_hat`, unit dipole axis `m_hat`, reference radius `R`, and position magnitude `r`, the field is:

```text
B = B_equator (R / r)^3 [3 (m_hat dot r_hat) r_hat - m_hat]
```

At the reference surface, an untilted model has magnitude `B_equator` at the equator and `2 B_equator` at the poles. Inputs below the modeled Earth surface are rejected. This model is suitable for B-dot development but is not a replacement for IGRF or flight-quality geomagnetic software.

## Environment Sampling

`EnvironmentConfig` combines the orbit, magnetic-field, and Sun configurations. `sample_environment` returns:

- Inertial spacecraft position and velocity
- Magnetic field expressed in ECI coordinates
- The same magnetic field transformed into body coordinates using `body_to_inertial`
- Unit Sun direction expressed in ECI and body coordinates
- Whether Earth currently occults the Sun

The CLI and viewer use the body-frame field to calculate magnetorquer torque, so the environment affects attitude dynamics whenever a nonzero dipole is applied.

## Sun and Eclipse Environment

`SunEnvironmentConfig` defaults to a fixed inertial `+X` unit direction from the spacecraft toward the Sun. Treating the Sun as fixed is appropriate for the current short-duration LEO scenarios and avoids introducing calendar ephemeris data before it is useful. `sample_environment` normalizes the configured vector and transforms it into body coordinates using the attitude quaternion.

The eclipse model treats the Sun as infinitely distant and Earth as a sphere, producing a cylindrical umbra. For spacecraft position `r` and unit direction toward the Sun `s`, the closest point on the forward ray is found from `r + t s`. Earth occults the Sun only when that closest point lies ahead of the spacecraft and within the configured Earth radius. The model has no atmosphere, penumbra, or finite-Sun taper.

The default orbit begins sunlit on inertial `+X` and passes through the center of the shadow half an orbit later on inertial `-X`.

## Simulated Magnetometer

`IdealMagnetometer` samples all three body-frame magnetic-field components at a configurable rate. The class name is retained for API compatibility, but its configuration can now add seeded bias, Gaussian noise, quantization, and saturation. The default low-level configuration remains exact for focused unit tests. The applications use the realistic profile. The sample rate is `10 Hz`, with the first sample due at simulation time zero.

The sample schedule advances from its previous target time instead of from render time. Calls must use nondecreasing simulation time. A due sample is rejected if the caller reports that the magnetorquers are energized, which makes the coil-off requirement explicit and testable.

## Magnetorquers and Magnetic Torque

The actuator model has three orthogonal coils aligned with body `+X`, `+Y`, and `+Z`. Their three signed scalar commands form one combined body-frame magnetic-dipole vector. Each axis is independently clamped to its configured magnitude limit; the default is `0.2 A m^2` per axis. The control output retains the requested, saturation-limited, and instantaneous applied vectors. The instantaneous vector is zero during coil-off sampling steps.

Applied magnetic torque is:

```text
tau_body = m_body x B_body
```

Magnetic dipole `m` is measured in ampere-square-meters, magnetic field `B` in tesla, and torque `tau` in newton-meters. The cross product makes torque perpendicular to both the applied dipole and local field. A magnetorquer therefore cannot create torque parallel to the field at that instant.

## Magnetic Control Cycle

`MagneticControlCycle` coordinates sensing and actuation. On a due magnetometer sample, it sets the applied dipole and torque to zero, then records the configured field measurement. Between samples it independently saturates the three commanded coil dipoles and calculates their torque from the current body-frame field.

With the default `0.01 s` simulation step and `10 Hz` magnetometer, the coils are disabled for the complete step beginning at `0.0`, `0.1`, `0.2 s`, and so on. This simple one-step blanking model prevents simulated self-field contamination without adding sub-step switching. The Phase 6 flight software supplies commands automatically, while the viewer can switch back to Phase 5 manual commands.

## Simulated Gyroscope

`IdealGyroscope` samples body angular velocity at `10 Hz`. Like the magnetometer class, it supports the Phase 13 error model while preserving zero-error defaults for unit tests. It uses the same nondecreasing, simulation-time-based scheduling pattern as the magnetometer. The detumble flight software and attitude estimator consume only its timestamped measurement.

## Simulated Coarse Sun Sensors

`IdealCoarseSunSensorArray` contains six sensors aligned with body `+/-X`, `+/-Y`, and `+/-Z`. These definitions are shared with the visual configuration, keeping rendered face normals consistent with measurement geometry. The default sample rate is `10 Hz` and the default field-of-view half-angle is `90 deg`.

For unit body-frame Sun direction `s_body` and sensor normal `n_body`, an in-view sensor reports the ideal cosine response:

```text
illumination = max(0, n_body dot s_body)
```

Responses outside the configured field of view are zero. The normalized sum of `illumination * n_body` reconstructs the measured body-frame Sun direction. The default opposing-axis layout and hemispherical fields of view reproduce an ideal input direction within floating-point tolerance. A sampled eclipse record contains zero response on all six sensors and no valid Sun-direction value.

## Sensor Error Model

`VectorSensorErrorModel` applies errors at the truth-to-measurement boundary in this order:

1. Draw one fixed three-axis bias when the sensor is constructed or reset.
2. Add an independent Gaussian noise draw to each sample.
3. Round it to the configured digital quantization step.
4. Clamp the result to the configured sensor range.

Every sensor has its own seed derived from the scenario seed. Resetting a sensor replays its bias and complete noise sequence. A local Box-Muller transform avoids implementation-defined `std::normal_distribution` sequences. This makes failures reproducible without sharing a random-number stream between sensors.

The application-level realistic profile uses these educational, order-of-magnitude values:

| Sensor | White noise (1 sigma) | Run bias (1 sigma) | Quantization | Saturation |
|---|---:|---:|---:|---:|
| Gyroscope, each axis | `0.005 deg/s` | `0.02 deg/s` | `0.001 deg/s` | `+/-250 deg/s` |
| Magnetometer, each axis | `100 nT` | `300 nT` | `10 nT` | `+/-100 uT` |
| Coarse Sun sensor, each face | `0.005` response | `0.01` response | `1/4095` | `0` to `1` |

These are not specifications for a selected flight part. They are transparent simulation assumptions chosen to exercise estimation and control. Eclipse measurements stay explicitly invalid rather than turning dark-current noise into a false Sun vector.

## Attitude Estimator Interface

`AttitudeEstimatorInput` is a standalone timestamped flight-software input. It contains measured body angular velocity, a gyro-valid flag, and optional magnetic and Sun vector observations. Each observation contains a measured body-frame vector, its known inertial reference, and an explicit validity flag. It has no simulation, environment, viewer, or true-attitude member.

The inertial Sun direction is the simplified onboard ephemeris reference. The inertial magnetic vector comes from the orbit and magnetic model; it does not depend on true attitude. Only simulated sensors receive truth-transformed body vectors. Application telemetry may compare an estimate with truth, but that result is never fed back into estimation or control.

`AttitudeEstimate` contains its timestamp, body-to-inertial quaternion, latest measured body rate, validity, correction status, and accepted-correction count. It becomes valid after the first accepted absolute correction and remains valid during gyro-only propagation.

## TRIAD and Gyro Correction

TRIAD uses two nonparallel vector pairs. For the magnetic and Sun vectors, it creates orthonormal body and inertial bases from the primary vector, the second vector projected perpendicular to it, and their cross product. The rotation is:

```text
R_body_to_inertial = inertial_basis * transpose(body_basis)
```

Non-finite or near-zero vectors are invalid. Pairs whose normalized cross-product magnitude is below `0.05` are rejected as nearly parallel because they do not define attitude reliably.

At each `10 Hz` gyro sample, the estimator first propagates its quaternion with measured body rate. When same-time magnetic and Sun observations are valid, it calculates TRIAD and uses quaternion spherical interpolation to move the propagated attitude toward the absolute solution. The default correction gain is `0.25`. During eclipse, the invalid Sun observation prevents TRIAD correction, so the estimate continues with gyro propagation and resumes corrections when sunlight returns.

`attitude_error_angle_rad` calculates the shortest rotation between estimated and true quaternions for simulation scoring. It uses the absolute quaternion scalar component so opposite quaternion signs report zero physical error.

## B-dot Estimation and Filtering

`BdotEstimator` calculates the finite difference between consecutive magnetometer samples:

```text
raw_B_dot = (B[k] - B[k-1]) / (t[k] - t[k-1])
```

The first measurement initializes the estimator and produces no derivative. Later measurements must have strictly increasing timestamps.

A first-order low-pass filter reduces abrupt derivative changes:

```text
previous_weight = exp(-sample_interval / time_constant)
filtered_B_dot = previous_weight * previous_filtered
               + (1 - previous_weight) * raw_B_dot
```

The default time constant is `0.2 s`. A zero time constant disables filtering. The filter operates only on sampled body-frame field measurements; it does not access truth angular velocity.

## B-dot Controller

The controller uses the explicitly tested negative sign convention:

```text
commanded_dipole_body = -gain * filtered_B_dot_body
```

The default gain is `50,000 A m^2 s/T`. Each command component is then limited to the default magnetorquer range of `+/-0.2 A m^2`. For a mostly attitude-driven field derivative, the negative sign produces torque that opposes rotation perpendicular to the local magnetic field. Orbital field variation prevents this simplified relationship from being exact at very low rates.

## Flight Software and Completion Logic

`DetumbleFlightSoftware` receives only timestamped magnetometer and gyro measurements. It begins in `BOOT` with zero command. The first field sample initializes B-dot; the second produces a derivative, changes the mode to `DETUMBLE`, and enables commands.

Detumble completes when measured angular-speed magnitude remains at or below `0.5 deg/s` for `30 s`. If speed rises to `0.7 deg/s`, the dwell timer resets. Values inside that `0.2 deg/s` hysteresis band preserve the timer, avoiding rapid resets near the boundary. The reusable detumble controller then sets its command to zero; `AutonomousFlightSoftware` owns the mission-level transition out of `DETUMBLE`.

## Desired Sun-Pointing Attitude

The selected Sun-pointing direction is the normalized body vector `[1, 1, 0]`, halfway between the `+X` and `+Y` long-side solar arrays. The desired attitude maps this power axis onto the inertial direction toward the Sun, illuminating both adjacent arrays at about `0.707` projected area each. That provides about `1.414` times the ideal projected area of pointing one equal-size side array directly at the Sun. Pointing one axis leaves roll undefined, so the spacecraft's long body `+Z` axis is also aligned with the projection of inertial `+Z` onto the plane perpendicular to the Sun. The default fixed inertial `+X` Sun direction is not parallel to this roll reference. The simulation does not yet calculate electrical power.

`desired_sun_pointing_attitude` builds orthonormal body and inertial bases from these two axis definitions and converts their rotation matrix to the project's body-to-inertial quaternion convention. Body pointing and roll axes must be orthogonal, finite, and nonzero.

## Quaternion PD Sun Controller

`sun_pointing_command` has a narrow flight-software interface: validated configuration, estimated attitude, measured body rate, and desired attitude. It has no access to true attitude, environment state, simulation types, or the viewer.

The error quaternion is `estimated.conjugate() * desired`. Its sign is selected so the scalar component is nonnegative, giving the shortest rotation of at most `180 deg`. The controller converts it to a body-frame rotation vector and requests:

```text
torque_body = Kp * attitude_error_body - Kd * measured_rate_body
```

Default per-axis proportional gains are `0.0015 N m/rad`. Derivative gains are `[0.016, 0.016, 0.0065] N m s/rad`, reflecting the lower inertia about the long body `Z` axis. Each requested body-torque component is limited to `+/-0.00002 N m` in `SUN_ACQUIRE`, producing a slower visible slew, and `+/-0.001 N m` in `SUN_POINT`, retaining enough authority for accurate tracking. These are flight-software limits on the same physical wheel cluster; the viewer never changes simulation speed automatically. Torque becomes zero inside both the `0.1 deg` attitude and `0.01 deg/s` rate deadbands to avoid command chatter.

The same `SunPointingControllerConfig` stores gains, limits, deadbands, tracking thresholds, hysteresis, and dwell in one validated structure. `SunPointingTracker` separately applies the acquisition and steady-pointing timing: the acquisition threshold is `10 deg`, steady threshold is `2 deg`, hysteresis is `2 deg`, and dwell is `10 s`. Keeping timing logic separate leaves the PD command function deterministic and easy to test.

`SunPointingCommand` distinguishes unconstrained requested torque from controller-limited torque. The latter is an actuator request, not assumed applied torque; the reaction-wheel model reports the body torque that the plant actually receives.

## Reaction-Wheel Cluster

`ReactionWheelCluster` is defined entirely by a list of normalized body-frame spin axes and per-wheel rotor inertia, motor-torque limit, speed limit, and initial speed. The default cluster uses four `45 deg` tilted axes:

```text
[+sqrt(1/2), 0,          +sqrt(1/2)]
[0,          +sqrt(1/2), +sqrt(1/2)]
[-sqrt(1/2), 0,          +sqrt(1/2)]
[0,          -sqrt(1/2), +sqrt(1/2)]
```

For active-wheel axis matrix `A`, requested spacecraft torque `tau`, and wheel motor torques `u`, equal and opposite torque gives `tau = -A u`. Eigen's complete orthogonal decomposition solves the least-norm allocation. Failed wheels are removed from `A`; one failed wheel leaves three independent axes in the default pyramid.

Each motor command is limited to `+/-0.001 N m`. Rotor speed integrates from `omega_dot = u / J` with default inertia `1e-5 kg m^2` and a `6,000 rpm` limit. A wheel at its speed limit may still torque in the unloading direction. Telemetry separates requested and applied spacecraft torque, allocation error, stored cluster momentum, individual motor commands, applied motor torque, wheel speeds and angles, saturation flags, and failure flags. The current model captures momentum exchange but omits rotor gyroscopic coupling and automatic momentum unloading.

## Autonomous Mission Flight Software

`AutonomousFlightSoftware` composes the reusable B-dot logic, attitude estimator, Sun-pointing controller, and pointing tracker without depending on the simulation plant or viewer. It receives a timestamped gyro sample, optional same-time magnetometer and coarse Sun-sensor measurements, inertial magnetic and Sun reference vectors, and eclipse status. True attitude is not part of its input.

The operational modes are:

- `BOOT`: wait for enough valid magnetic samples to form the first B-dot estimate.
- `DETUMBLE`: enable only the magnetorquer request until the rate threshold and dwell are satisfied.
- `SUN_ACQUIRE`: enable only the reaction-wheel request and turn toward the Sun.
- `SUN_POINT`: maintain the acquired attitude with reaction wheels.
- `SAFE`: command both actuator groups off while the Sun is unavailable or navigation is invalid.

Rate and pointing thresholds use hysteresis and dwell. Binary sensor faults and recovery use configurable dwell timers. Actuator commands are gated immediately when a required estimate or Sun measurement is unavailable, even while a fault transition is still dwelling. `ActuatorCommand` is the single narrow output containing timestamped magnetorquer and reaction-wheel requests plus enable flags.

Every completed mode change appends a `ModeTransitionEvent` containing time, old mode, new mode, and a typed reason. Reset clears state, timers, estimators, trackers, commands, and event history. This narrow interface keeps the flight software independent of the plant and viewer.

Each application step follows this order:

1. Sample the environment from current simulation time and attitude.
2. Sample the independent gyro and coarse Sun-sensor schedules when due.
3. Apply the previously computed flight-software dipole through the magnetic control cycle.
4. Pass same-time sensor samples and inertial reference vectors into the autonomous flight-software update.
5. Apply the returned reaction-wheel request through the wheel allocator and dynamics.
6. Propagate rigid-body state with the sum of instantaneous magnetic torque and applied reaction-wheel spacecraft torque.

This one-sample command delay represents a simple discrete flight-software loop and prevents the controller from using a measurement before it exists.

## Seeded Detumble Validation

The automated validation uses a `0.02 s` fixed step, default `10 Hz` magnetometer, default controller settings, and a `12,000 s` timeout. Each seed fixes a random attitude and tumble direction while the magnitude is explicitly selected:

| Seed | Initial rate | Completion time | Final rate | Final / initial energy |
|---:|---:|---:|---:|---:|
| 7 | 5 deg/s | 3,733.82 s | 0.487 deg/s | 1.625% |
| 42 | 10 deg/s | 2,291.72 s | 0.481 deg/s | 0.500% |
| 2026 | 15 deg/s | 4,278.02 s | 0.476 deg/s | 0.122% |

All three scenarios satisfy the `0.5 deg/s` threshold for the full `30 s` dwell. Final energy is far below initial energy in every case. Individual short intervals may increase because the filtered, sampled controller acts in an orbit-varying field; monotonic energy decrease is not required at every step.

## Rotational Diagnostics

`rotational_metrics` calculates:

- Body-frame angular momentum `Iω`
- Inertial-frame angular momentum
- Rotational kinetic energy `1/2 ωᵀIω`
- Quaternion norm

Torque-free tests use these quantities to verify conservation rather than relying only on visually plausible motion.

The Phase 2 asymmetric-body validation propagates ten seconds with a `0.001 s` timestep. Its absolute error limits are `1e-12 J` for rotational energy, `1e-11 kg·m²/s` per inertial angular-momentum component, and `1e-10` for quaternion norm relative to one.

## Attitude and Pointing Validation

Known ideal TRIAD cases recover body-to-inertial attitude within `1e-12 rad`. A slowly rotating case using the default `0.25` correction gain converges below `1e-8 rad`. A ten-second ideal-gyro eclipse interval remains below `1e-7 rad` and returns below `1e-12 rad` on its first valid post-eclipse correction.

Three initially detumbled attitudes, including a `170 deg` error, are propagated for `180 s` with reaction-wheel torque. Each finishes below `0.25 deg` attitude error and `0.02 deg/s` body rate without unstable oscillation. Orthogonal and four-wheel allocations reproduce achievable body torque to within `1e-12 N m`; the pyramid also retains exact small-command allocation after one failed wheel.

With the realistic sensor profile, the default seed-42 `6,000 s` mission records four deterministic transitions: `BOOT -> DETUMBLE` at `0.2 s`, `DETUMBLE -> SAFE` at `2,107.2 s` because detumble completes during eclipse, `SAFE -> SUN_ACQUIRE` at `3,906.9 s` after Sun and attitude availability recover, and `SUN_ACQUIRE -> SUN_POINT` at `4,120.4 s`. It finishes in steady pointing at `0.385 deg` estimated pointing error, `0.0132 deg/s` true body rate, `2.032 deg` estimate-versus-truth error, negligible final wheel-allocation error, and a peak wheel speed of `979 rpm` below the `6,000 rpm` limit. The ideal profile remains available for zero-error comparisons.

## Current CLI Scenario

The CLI runs the complete autonomous recovery mission for `6,000 s` by default with seed `42`, a `0.01 s` fixed step, and exactly `10 deg/s` initial angular-speed magnitude on a seeded random axis and attitude. It reports initial and final angular speed and energy, final mode, detumble completion, estimate validity and error, Sun-pointing state, wheel allocation, and the full timestamped mode-event history.

Options are `--duration`, `--time-step`, `--seed`, `--initial-rate-deg-s`, `--sensor-profile`, and `--output`. `--sensor-profile realistic` is the application default; `ideal` is available for mathematical comparison and prior deterministic baselines. When supplied, `--output` creates missing parent directories and writes full-step CSV telemetry containing true and estimated attitude, estimate validity and error, correction status, body rate, rotational energy, magnetic field, Sun directions, eclipse and Sun-sensor state, magnetorquer commands and torque, Sun-pointing requested and controller-limited torque, applied wheel body torque, allocation error, per-wheel speed, command, saturation and failure state, tracking state, autonomous flight mode, and detumble completion. Full-step output preserves the coil-off sampling pulses but can create large files for long runs. The ignored `runs/` directory is the recommended destination.

## Monte Carlo Robustness Validation

`run_native_mission` is a rendering-independent mission runner. It records whether detumble, Sun acquisition, and steady pointing were ever achieved; their first achievement times; final rate; true and estimated pointing errors; attitude-estimation error; wheel speed and saturation; and a typed failure reason. A callback can save reduced per-step telemetry without coupling file output to the mission core.

`detumble_monte_carlo` derives a reproducible run seed from a master seed and randomizes:

- the full initial attitude and tumble axis;
- initial angular speed uniformly from `5` through `15 deg/s`;
- initial orbit argument of latitude uniformly through `360 deg`;
- each sensor's fixed bias and sample noise sequence.

The validated envelope uses `25` runs, master seed `2026`, an `8,000 s` duration, a `0.02 s` fixed step, the `500 km` and `51.6 deg` circular orbit, the realistic sensor profile above, and the nominal healthy actuator configuration. A run succeeds when it detumbles, acquires the Sun, reaches steady pointing, finishes below `0.1 deg/s`, and has no more than `2 deg` RMS true Sun-pointing error while sunlit in `SUN_POINT`. Ending in eclipse and `SAFE` is allowed after those achievements because no valid Sun measurement exists then.

The current two-panel power-axis campaign with gentle acquisition torque observed `25/25` successes. Mean detumble time was `4,055.08 s`, the slowest detumble was `5,468.24 s`, the latest first acquisition was `6,365.34 s`, mean sunlit RMS true pointing error was `0.922 deg`, worst per-run RMS was `1.555 deg`, and maximum final body rate was `0.0314 deg/s`. The maximum wheel speed was `1,557 rpm`; no wheel reached the `6,000 rpm` limit.

The tuning campaign replayed the first `15` scenarios while comparing attitude correction gains `0.10`, `0.25`, and `0.50`, B-dot filter time constants `0.20 s` and `0.50 s`, and pointing gain scales `0.75`, `1.0`, and `1.25`. All candidates passed. Their mean RMS pointing errors differed by less than `0.009 deg`, and the `0.20 s` B-dot filter detumbled about `1.6 s` sooner than `0.50 s`. The project therefore retains the established `0.25` correction gain, `0.20 s` B-dot filter, and unscaled PD gains instead of overfitting this small campaign.

Failed cases are kept interpretable. An early endpoint metric incorrectly marked five otherwise successful runs as failures because they ended in eclipse; inspecting their mode and telemetry led to the sunlit RMS criterion above. A separate `0.05 s` step sensitivity run passed only `22/25`: one case exceeded the RMS pointing target, one did not detumble, and one did not reacquire within `8,000 s`. Therefore the performance claim is limited to the documented `0.02 s` timestep. A deterministic eclipse-ending regression seed and a deliberate short-duration timeout case protect both behaviors in tests.

Run the documented campaign with:

```bash
./build/release/detumble_monte_carlo --runs 25 --duration 8000 \
  --time-step 0.02 --seed 2026 \
  --results runs/phase13_runs.csv \
  --aggregate runs/phase13_aggregate.csv
```

`--results` contains one row per run. `--aggregate` is a separate compact metrics file. `--telemetry-dir` optionally writes reduced step telemetry for investigation; it is intentionally off by default so a large campaign does not create hundreds of large files.

## Desktop Viewer

The viewer uses raylib for its resizable desktop window, camera, input, rendering, and GLB loading. Dear ImGui provides the compact controls, status, actuator schematic, and context-aware mission-progress gauge.

Viewer behavior:

- The nominal scenario begins at exactly `10 deg/s` from seed `42`, uses realistic sensors and automatic B-dot control, and defaults to `50x` simulation speed. Gentle `5 deg/s` and fast `15 deg/s` seeded presets cover the design envelope without exposing a large configuration editor. Sun acquisition is intentionally slower because flight software limits reaction-wheel torque during the slew; the viewer always honors the requested playback speed.
- A compact control panel provides pause/resume, reset, scenario selection, logarithmic `0.1x` to `100x` speed control, camera reset, and help. Single-step and replay controls are intentionally omitted from the final presentation UI.
- One consolidated mission-status panel reports flight mode, seed, whole-second simulation time, sunlight availability, and the active actuator group. It also contains the current goal gauge and spacecraft-internals schematic. Scenario selection remains only in Controls.
- The viewer displays only the current live simulation state. Because each scenario and sensor sequence is deterministic, Reset reproduces a run without maintaining a separate replay buffer or frame-scrubbing interface.
- The viewer has one orbital presentation. `O` smoothly resets the camera; right-mouse dragging changes azimuth and elevation, and the mouse wheel changes distance.
- The orbit scene shows a simple blue Earth with a transparent wireframe shell, an unlabeled bright-orange Sun marker, inclined orbit path, fading recent-position trail, deterministic starfield, and the current spacecraft position and attitude. Sunlight state appears only in Mission status; the eclipse volume and diagnostic vectors are intentionally not rendered.
- The complete custom GLB is drawn in solid mode. Coarse Sun-sensor markers are orange when illuminated, dark cyan when not viewing the Sun, and gray during eclipse. The viewer reports `SUNLIT` or `ECLIPSE` continuously.
- The mission-progress gauge uses the current displayed state rather than a time-history plot. It shows the remaining angular-speed or pointing-error magnitude, shrinking toward zero as the spacecraft approaches its goal. Pointing error uses a `0` to `10 deg` presentation scale so sub-degree behavior remains visible. The two-decimal readout is low-pass filtered and refreshed at `4 Hz` to prevent realistic sensor jitter from becoming unreadable; this display filtering never feeds back into control. During eclipse it says that the sensors are waiting to detect the Sun. Once steady pointing meets its dwell requirement, a mission-goal-achieved message remains latched until Reset or scenario selection.
- The spacecraft-internals section uses two adjacent subsystem cards so the actuators do not overlap. The magnetorquer card retains the configuration-defined rod locations inside a 3U outline. The reaction-wheel card separates the four configured wheel axes into a clear two-by-two top view; individual wheel labels are unnecessary. Rod brightness follows the limited dipole command with a short visual fade, avoiding coil-blanking flicker. Wheel spokes use simulated speed direction and magnitude but cap their display speed to prevent aliasing at accelerated playback. The exploded presentation does not alter actuator physics or configuration.
- `Space` pauses or resumes, `R` resets, `O` resets the camera, and `H` opens concise help.

The Earth, orbital altitude, spacecraft, and Sun marker use deliberately different display scales so all remain visible together. These presentation choices never change physical simulation values.
Rendering time never enters the equations of motion. Each display frame contributes wall time multiplied by playback speed to an accumulator. The viewer repeatedly calls `Simulation::step()` while at least one complete `0.01 s` interval is due. Display frame rate can change how smooth the motion looks, but it cannot change the sequence of simulated states.

The determinism test advances one simulation continuously and another in uneven viewer-like batches. Their final attitude, angular velocity, and elapsed time match exactly after 1,000 steps.

## Custom 3U Visual Model

`assets/models/detumble_3u.glb` is Detumble's runtime spacecraft model. It is generated by `assets/scripts/create_detumble_3u.py`, which uses Blender primitives and project-authored materials rather than NASA geometry or textures. Its exterior includes aluminum rails and end plates, segmented body-mounted solar cells, gold panel traces, fasteners, six coarse Sun sensors, and a short magnetometer boom. From the repository root, regenerate it with:

```bash
blender --background --python assets/scripts/create_detumble_3u.py
```

The generator uses meters, keeps the body-frame origin at the modeled center of mass, exports without Y-up conversion, and makes body `+Z` the 3U long axis. Before export it checks required component names, the root origin, and the long-axis orientation. CMake copies the resulting GLB beside the viewer executable. raylib also checks and reports its loaded bounds; a generated cube remains the load-failure fallback.

The C++ component configurations are authoritative. `SpacecraftVisualConfig` defines the `0.10 x 0.10 x 0.34 m` body, three orthogonal magnetorquer rods, sensor locations, an isolated `+X` magnetometer location, and four reaction wheels with normalized pyramid axes. Coarse Sun sensor names and outward normals come from the same definitions used by `IdealCoarseSunSensorArray`. The viewer transforms these body-frame locations and directions using the tested `rotate_body_to_inertial` helper. The Blender script mirrors this configuration only to create the visible mesh; neither the GLB nor the visual configuration changes mass properties or dynamics.

The temporary NASA model used during the first viewer prototype has been removed. The shipped GLB contains only project-authored geometry and materials, which avoids carrying an unused runtime asset or implying that its geometry defines the simulation.

## Build Commands

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Replace `debug` with `release` for an optimized build. Build output is contained under `build/`.

Run applications from the repository root:

```bash
./build/debug/detumble
./build/debug/detumble_viewer
```
