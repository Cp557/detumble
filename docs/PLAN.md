# Detumble Development Plan

Build Detumble as a sequence of small, validated milestones. Complete each phase's validation gate before moving to the next phase. Prefer a correct, understandable approximation over a more sophisticated model that cannot be clearly explained or tested.

## Final Product

Detumble will be a cross-platform C++ desktop simulation of a generic 3U CubeSat recovering from an uncontrolled tumble in a 500 km circular Low Earth Orbit. The spacecraft will use magnetorquers for B-dot detumbling, estimate its attitude from realistic sensor measurements, acquire the Sun, and maintain a Sun-pointing attitude with four reaction wheels.

The desktop viewer will provide one focused orbital camera, compact controls, and one consolidated mission-status panel containing the current goal and actuator schematic.

The simulator and flight-software algorithms will run through one native C++ path. This keeps the application fast, understandable, and portable across macOS, Linux, and Windows.

## Scope Boundary

Included:

- Three-dimensional rigid-body attitude dynamics
- Quaternion attitude representation
- Simplified circular-orbit propagation
- Tilted-dipole Earth magnetic-field model
- Simplified Sun direction and eclipse detection
- Gyroscope, magnetometer, and coarse Sun sensors
- Sensor sample rates, bias, noise, saturation, and field of view
- Magnetorquers and B-dot detumbling
- TRIAD attitude determination with gyro propagation and correction
- Quaternion Sun-pointing control
- Four-reaction-wheel pyramid with torque and speed limits
- Autonomous flight modes and safe transitions
- Monte Carlo validation and recorded simulation runs
- C++ desktop visualization of the spacecraft and environment

Excluded:

- Translational orbit perturbations and mission trajectory design
- High-order geomagnetic models such as IGRF
- Star trackers or camera-image processing
- Extended Kalman filters
- Detailed electrical, thermal, structural, or power-system simulation
- Flexible-body dynamics and vibration
- Detailed reaction-wheel motor electronics or bearing models
- Flight-qualified software or certification claims
- Deployment to physical spacecraft hardware
- RTEMS, VxWorks, or hardware-in-the-loop integration

## Proposed Technical Stack

- C++20 for the simulator, controllers, estimator, command-line tools, and viewer
- CMake and CMake presets for macOS, Linux, and Windows builds
- Eigen for vectors, matrices, quaternions, and wheel-allocation math
- Catch2 for unit and integration tests
- raylib for the desktop window, camera, rendering, and model loading
- Dear ImGui for controls, status, and the mission-progress gauge
- Blender for authoring a custom 3U CubeSat and exporting it as GLB
- CSV for human-readable telemetry output
- GitHub Actions for cross-platform builds and tests

The Phase 3 compatibility prototype included ImPlot while exploring live telemetry. The focused final viewer no longer needs a plotting dependency. It pins raylib 6.0, Dear ImGui 1.92.7, and a raylib-6-compatible rlImGui revision.

## Engineering Rules

- [x] Define every vector's reference frame and use SI units internally.
- [x] Keep the simulation core independent of rendering and user-interface code.
- [x] Keep the true simulation state private from flight software and controllers.
- [x] Make every randomized run reproducible from a recorded seed.
- [x] Use fixed simulation timesteps and explicit sensor and controller sample rates.
- [x] Add tests before increasing a model's physical complexity.
- [x] Record important assumptions, coordinate conventions, and architecture in `docs/REFERENCE.md`.
- [x] Add personal explanations and lessons to the ignored `docs/CONCEPTS.md` throughout development.
- [x] Keep control and estimation algorithms independent of rendering, networking, and simulation truth.
- [x] Never expose unavailable truth-state data to flight-software algorithms.

## Phase 0 — Repository and Build Foundation

- [x] Initialize the local Git repository with `main` as the default branch.
- [x] Choose a permissive open-source license.
- [x] Create the initial CMake project using C++20.
- [x] Create separate `detumble_core`, `detumble_cli`, `detumble_viewer`, and test targets.
- [x] Add strict compiler warnings for Clang, GCC, and MSVC.
- [x] Add debug and release CMake presets.
- [x] Add Eigen and Catch2 with pinned dependency versions.
- [x] Add a smoke test that links and runs against `detumble_core`.
- [x] Add GitHub Actions builds for macOS, Linux, and Windows.
- [x] Document how to configure, build, and test the empty project.
- [x] Create the GitHub repository, keeping it private during development.

Validation gate:

- [x] All three operating systems configure, compile, and pass the smoke test.

Validation status: local macOS debug and release builds pass. GitHub Actions also passes the debug build and smoke test on macOS, Linux, and Windows.

## Phase 1 — Frames, Math, and Conventions

- [x] Define the Earth-centered inertial, orbital, and spacecraft body frames.
- [x] Choose and document right-handed axis directions.
- [x] Define whether the attitude quaternion maps body coordinates to inertial coordinates or the reverse.
- [x] Define quaternion storage and multiplication conventions.
- [x] Add strongly named state structures with units included in member names where helpful.
- [x] Implement small helpers for vector-frame transformations.
- [x] Implement quaternion normalization and attitude integration using Eigen primitives.
- [x] Test identity, known-axis rotations, inverse rotations, composition order, and normalization.
- [x] Explain vectors, frames, angular velocity, rotation matrices, and quaternions in `docs/CONCEPTS.md`.

Validation gate:

- [x] Known vectors rotate between body and inertial frames with the expected direction and sign.

## Phase 2 — Rigid-Body Attitude Dynamics

- [x] Define approximate mass, dimensions, center of mass, and diagonal inertia for a generic 3U CubeSat.
- [x] Implement Euler's rigid-body rotational equation.
- [x] Propagate angular velocity and attitude with a fixed-step RK4 integrator.
- [x] Renormalize the attitude quaternion at a documented point in the integration loop.
- [x] Support zero torque and externally supplied body torque.
- [x] Add deterministic initial attitude and angular-velocity generation from a seed.
- [x] Add a minimal CLI run that prints final attitude, angular velocity, and rotational energy.
- [x] Test spherical-body motion, asymmetric torque-free motion, and constant applied torque.
- [x] Track angular momentum, rotational energy, and quaternion norm for validation.
- [x] Explain inertia, torque, angular momentum, and rigid-body dynamics in `docs/CONCEPTS.md`.

Validation gate:

- [x] Torque-free runs conserve angular momentum and rotational energy within documented numerical tolerances.
- [x] Quaternion norm remains within its documented tolerance.

## Phase 3 — First Visual Slice

- [x] Prototype raylib, Dear ImGui, ImPlot, and GLB loading in one cross-platform viewer.
- [x] Pin compatible visualization dependency versions.
- [x] Render a simple 3U rectangular placeholder using the propagated attitude.
- [x] Add an orbit camera and body-axis overlay.
- [x] Add pause, resume, reset, and simulation-speed controls.
- [x] Plot body rates and quaternion norm live.
- [x] Keep simulation updates fixed-step and independent of rendering frame rate.

Validation gate:

- [x] The same seeded run produces the same final state with and without the viewer.
- [x] The visible body-axis rotations agree with the frame-convention tests.

Validation status: the full viewer stack builds and launches on macOS, the prototype GLB loader works, and all early viewer-enabled tests passed. The viewer and CLI use the same fixed-step simulation runner. Body-axis endpoints and the 3U outline use the tested body-to-inertial frame transform directly, and a raylib integration test confirms the rendered model uses the same positive-rotation convention. The temporary NASA prototype model was later replaced by the project-authored 3U model and removed from the repository.

## Phase 4 — Orbit and Magnetic Environment

- [x] Implement a simplified 500 km circular orbit with configurable inclination.
- [x] Produce spacecraft position in the inertial frame as a function of simulation time.
- [x] Implement a centered, tilted-dipole Earth magnetic-field model.
- [x] Transform the local magnetic field from inertial coordinates into body coordinates.
- [x] Validate magnetic-field magnitude and direction at selected orbital positions.
- [x] Record position and magnetic-field telemetry.
- [x] Add optional orbit-path and magnetic-field-vector overlays to the viewer.
- [x] Explain Low Earth Orbit, orbital frames, and Earth's magnetic field in `docs/CONCEPTS.md`.

Validation gate:

- [x] One complete orbit is periodic and the local magnetic field varies smoothly through a plausible range.

Validation status: the default 500 km, 51.6-degree orbit returns to its initial inertial position and velocity after one 94.47-minute period within numerical tolerance. Across 720 samples per orbit, the tilted-dipole field remains between 20 and 60 microtesla and changes smoothly by less than 1 microtesla between adjacent samples.

## Phase 5 — Magnetometer and Magnetorquers

- [x] Implement an ideal three-axis magnetometer with a configurable sample rate.
- [x] Implement three orthogonal magnetorquers with configurable dipole limits.
- [x] Convert commanded coil dipoles into a combined body-frame magnetic dipole.
- [x] Compute magnetic torque using the commanded dipole and local magnetic field.
- [x] Model a control cycle that disables the torquers before magnetometer sampling.
- [x] Enforce actuator saturation and record commanded versus applied dipole.
- [x] Test that magnetic torque is perpendicular to the magnetic field.
- [x] Test that no magnetorquer command exceeds its configured limit.
- [x] Explain magnetometers, magnetorquers, magnetic dipole, and magnetic torque in `docs/CONCEPTS.md`.

Validation gate:

- [x] Known magnetic-dipole and field vectors produce the expected torque direction and magnitude.

Validation status: a `+X` dipole of `1 A m^2` crossed with a `+Z` field of `20 microtesla` produces `-Y` torque of `20 micronewton-meters`. Tests also verify that torque is perpendicular to both input vectors, commands remain within each configured limit, magnetometer samples occur at the configured rate with the coils off, and applied magnetic torque changes the simulated attitude dynamics.

## Phase 6 — B-dot Detumble Milestone

- [x] Estimate the body-frame magnetic-field derivative from sampled measurements.
- [x] Add a simple configurable low-pass filter for the derivative estimate.
- [x] Implement a saturated B-dot controller with an explicitly tested sign convention.
- [x] Add ideal gyro rate measurements for transition detection and telemetry.
- [x] Define a detumble threshold, hysteresis band, and dwell time.
- [x] Add `BOOT` and `DETUMBLE` flight modes.
- [x] Add CLI configuration for duration, timestep, seed, initial rate, and output path.
- [x] Export attitude, body rate, energy, field, dipole, torque, and flight mode to CSV.
- [x] Plot rate, energy, dipole command, and control mode in the viewer.
- [x] Test B-dot against representative fixed and randomized initial states.
- [x] Explain B-dot control, gain, filtering, saturation, hysteresis, and dwell time in `docs/CONCEPTS.md`.

Validation gate:

- [x] A documented batch of seeded scenarios reduces body rates below the detumble threshold for the required dwell time.
- [x] Rotational kinetic energy trends downward across the batch despite short local increases.

Milestone result: a complete, validated headless and visual B-dot detumble simulation.

Validation status: seeds `7`, `42`, and `2026` begin at fixed magnitudes of `5`, `10`, and `15 deg/s` with seeded random attitudes and tumble axes. All three remain below `0.5 deg/s` for the required `30 s` dwell and complete within `4,279 s`. Their final rotational energies are between `0.12%` and `1.63%` of their initial values. The validation uses a `0.02 s` fixed step; the applications default to `0.01 s`.

## Phase 7 — Custom 3U Spacecraft and Environment Visualization

- [x] Learn the minimal Blender workflow needed to create, name, and export components.
- [x] Create a dimensionally consistent generic 3U CubeSat model.
- [x] Place and name three orthogonal magnetorquer rods.
- [x] Add exterior coarse Sun sensors and an isolated magnetometer location.
- [x] Add four reaction-wheel placeholders in a pyramid arrangement.
- [x] Align the GLB origin, scale, and axes with the simulation body frame.
- [x] Keep component positions and orientations in simulation configuration rather than relying on the mesh as physics data.
- [x] Add a solid custom 3U spacecraft rendering mode.
- [x] Visualize angular velocity, magnetic field, magnetic dipole, and torque with distinct vector arrows.
- [x] Document the visual model's provenance and any third-party dependencies.

Validation gate:

- [x] Every visual component and vector agrees with the configured body-frame orientation.

Validation status: the reproducible Blender generator checks the body-frame origin, named components, dimensions, and long `+Z` axis before export. C++ tests verify the 3U dimensions, orthogonal magnetorquers, outward sensor normals, normalized reaction-wheel pyramid axes, and body-to-inertial component transformation. The viewer loads the custom GLB at its authored meter scale and reports bounds of `0.1153 x 0.1045 x 0.3445 m`, including exterior sensors and the magnetometer boom.

## Phase 8 — Sun Environment and Sensors

- [x] Implement a simplified inertial Sun direction.
- [x] Implement Earth-occultation logic for eclipse detection.
- [x] Implement a three-axis gyroscope interface with a configurable sample rate.
- [x] Implement coarse Sun sensors using face normals, field of view, and illumination.
- [x] Produce no valid Sun measurement during eclipse.
- [x] Add Sun direction, sensor visibility, and eclipse state to telemetry.
- [x] Render Earth, sunlight direction, orbit path, eclipse state, and an orbital overview camera.
- [x] Explain gyroscopes, coarse Sun sensors, sensor field of view, and eclipse in `docs/CONCEPTS.md`.

Validation gate:

- [x] Sun-sensor readings agree with known spacecraft attitudes and disappear correctly during eclipse.

Validation status: known body-frame Sun directions activate the expected `+/-X`, `+/-Y`, and `+/-Z` sensors and reconstruct the original direction within floating-point tolerance. The default orbit is sunlit at its initial `+X` position and eclipsed half an orbit later behind Earth. The same orbital eclipse state removes the sensor array's valid Sun vector and sets all six illumination responses to zero.

## Phase 9 — Attitude Determination

- [x] Keep the true attitude inaccessible to the estimator and flight controller.
- [x] Define standalone timestamped estimator inputs containing only sensor measurements and validity state.
- [x] Define a standalone attitude-estimate output that does not depend on simulation or viewer types.
- [x] Implement TRIAD using measured magnetic-field and Sun vectors.
- [x] Detect and reject invalid or nearly parallel vector pairs.
- [x] Propagate the estimated quaternion between corrections using gyro measurements.
- [x] Add a simple tunable quaternion correction toward the TRIAD solution.
- [x] Continue gyro propagation without Sun corrections during eclipse.
- [x] Track attitude-estimation error using truth data available only to simulation telemetry.
- [x] Test known attitudes, slowly rotating cases, eclipse intervals, and invalid measurements.
- [x] Plot estimated-versus-true attitude error.
- [x] Explain attitude determination, TRIAD, gyro propagation, and sensor fusion in `docs/CONCEPTS.md`.

Validation gate:

- [x] With ideal sensors, the estimator converges to a documented attitude-error tolerance.
- [x] During a representative eclipse, estimation error remains bounded and recovers after Sun measurements return.

Validation status: TRIAD recovers known attitudes to within `1e-12 rad`, and the default `0.25` quaternion correction gain converges a slowly rotating ideal-sensor case below `1e-8 rad`. A ten-second simulated eclipse uses gyro-only propagation, remains below `1e-7 rad`, and returns below `1e-12 rad` on the first valid post-eclipse TRIAD update. The default seed-42 application run finishes with `0.000830 deg` estimated-versus-true attitude error.

## Phase 10 — Sun Acquisition with Ideal Control Torque

- [x] Choose and document the spacecraft face and body axis that point toward the Sun.
- [x] Keep the pointing controller interface limited to estimated attitude, measured rate, desired attitude, and validated configuration.
- [x] Store controller gains, limits, thresholds, and dwell settings in one validated structure.
- [x] Compute a desired Sun-pointing quaternion while preserving a defined roll reference.
- [x] Compute quaternion attitude error with the shortest-rotation convention.
- [x] Implement a proportional-derivative attitude controller.
- [x] Apply the requested body torque directly before introducing reaction-wheel mechanics.
- [x] Add torque limits and anti-chatter behavior near the pointing target.
- [x] Define acquisition and steady-pointing error thresholds with dwell times.
- [x] Test large-angle commands, small-angle settling, and quaternion sign equivalence.
- [x] Explain quaternion error and PD attitude control in `docs/CONCEPTS.md`.

Validation gate:

- [x] From several detumbled attitudes, ideal control torque acquires and maintains the Sun-pointing target without unstable oscillation.

Validation status: three detumbled initial attitudes, including a `170 deg` case, settle below `0.25 deg` attitude error and `0.02 deg/s` rate within `180 s`. The seed-42 `6,000 s` application scenario completes B-dot detumbling, reaches steady Sun pointing, and finishes at `0.0668 deg` pointing error and `0.00922 deg/s` angular speed using the estimator rather than true attitude.

## Phase 11 — Four-Wheel Pyramid

- [x] Implement a general reaction-wheel cluster defined by wheel-axis vectors.
- [x] Define standalone wheel-command and wheel-telemetry structures independent of the viewer.
- [x] Validate the cluster first with three orthogonal wheels.
- [x] Configure four tilted wheels in a pyramidal arrangement.
- [x] Allocate requested body torque across the wheels using Eigen matrix operations.
- [x] Model equal and opposite spacecraft and wheel torques.
- [x] Integrate individual wheel speeds.
- [x] Enforce wheel torque and speed limits.
- [x] Report allocation error and saturation in telemetry.
- [x] Add optional single-wheel failure scenarios after nominal behavior works.
- [x] Report saturated or failed wheels in the viewer diagnostics.
- [x] Explain reaction wheels, momentum exchange, allocation matrices, saturation, and redundancy in `docs/CONCEPTS.md`.

Validation gate:

- [x] The four-wheel cluster tracks achievable three-axis torque commands within tolerance.
- [x] The Sun-pointing controller remains stable using wheel-generated torque instead of ideal torque.

Validation status: orthogonal and four-wheel configurations reproduce achievable body-torque commands to within `1e-12 N m`. The four-wheel cluster retains three-axis allocation after one failed wheel and reports torque, speed, and allocation saturation. Three detumbled attitudes remain stable for `180 s` using only wheel-generated torque. The nominal seed-42 mission finishes with zero allocation error and all wheel speeds below the `6,000 rpm` limit.

## Phase 12 — Complete Autonomous Mission

- [x] Define `SUN_ACQUIRE`, `SUN_POINT`, and `SAFE` flight modes.
- [x] Transition automatically from deployment through detumble, acquisition, and pointing.
- [x] Add hysteresis and dwell times to every mode transition.
- [x] Define behavior when the Sun is unavailable during eclipse.
- [x] Define behavior when estimation becomes invalid.
- [x] Prevent controllers from issuing commands outside their active modes.
- [x] Pass all actuator requests through one narrow command interface.
- [x] Record every mode transition as a timestamped event containing the old mode, new mode, and reason.
- [x] Display the current mode and transition history in the viewer.
- [x] Add an end-to-end deterministic mission test.

Validation gate:

- [x] A seeded mission autonomously detumbles, acquires the Sun when visible, and maintains Sun pointing without reading truth-state attitude.

Milestone result: the complete autonomous recovery and Sun-pointing mission.

Validation status: the seed-42 `6,000 s` mission transitions `BOOT -> DETUMBLE -> SAFE -> SUN_ACQUIRE -> SUN_POINT`. It safely waits through eclipse, slews for about `214 s` with reduced reaction-wheel torque, then finishes in steady Sun pointing at `0.385 deg` estimated pointing error and `0.0132 deg/s` body rate while aiming the two-panel power axis toward the Sun. The estimator and controllers receive only timestamped sensor measurements and inertial reference vectors; true attitude is used only by the plant and validation score.

## Phase 13 — Sensor Realism and Robustness

- [x] Add configurable Gaussian noise to gyro, magnetometer, and Sun-sensor measurements.
- [x] Add reproducible per-run sensor biases.
- [x] Add sensor quantization and saturation where meaningful.
- [x] Tune estimator correction, B-dot filtering, and controllers using documented criteria.
- [x] Build a headless Monte Carlo runner for randomized attitude, body rates, orbit position, noise, and bias.
- [x] Define success, failure, detumble-time, acquisition-time, and pointing-error metrics.
- [x] Save aggregate results separately from per-run telemetry.
- [x] Investigate and document representative failed cases rather than hiding them.
- [x] Add regression seeds for important edge cases.
- [x] Explain sensor error, Monte Carlo testing, and robustness metrics in `docs/CONCEPTS.md`.

Validation gate:

- [x] The mission meets documented success-rate and pointing-performance targets across a documented scenario envelope.

Validation status: the documented master-seed-2026 campaign passed `25/25` randomized `8,000 s` missions at a `0.02 s` step with `5` to `15 deg/s` initial tumble, randomized attitude and orbit phase, and seeded gyro, magnetometer, and coarse-Sun-sensor errors. With the two-panel power axis and gentle acquisition torque, mean sunlit RMS true pointing error was `0.922 deg`, the worst per-run RMS was `1.555 deg`, maximum final rate was `0.0314 deg/s`, and no wheel reached its speed limit. Endpoint-in-eclipse false failures and a `0.05 s` step sensitivity failure set are documented in `docs/REFERENCE.md`; an eclipse-ending seed is locked into regression coverage.

## Phase 14 — Finish the Application

- [x] Finalize a focused orbital camera view.
- [x] Replace telemetry plots with one context-aware goal gauge for angular speed or Sun-pointing error.
- [x] Consolidate mission progress and actuator internals into the mission-status panel.
- [x] Use a gentle physical reaction-wheel torque limit during Sun acquisition and show a latched mission-goal-achieved state.
- [x] Smooth and rate-limit the displayed pointing error without changing simulation telemetry.
- [x] Add a concise scenario panel with seed, time, mode, rate, pointing error, eclipse state, and actuator status.
- [x] Keep the presentation focused on restartable deterministic live scenarios.
- [x] Add sensible default scenarios without turning the viewer into a large configuration editor.
- [x] Add keyboard and mouse help inside the application.
- [x] Add a deterministic starfield and a simple blue Earth with a transparent wireframe shell.
- [x] Add a fading recent-orbit trail and smooth camera transitions.
- [x] Remove the reference grid and separate visual scale from physical state.
- [x] Keep the default orbit scene uncluttered by omitting angular-velocity and eclipse-volume overlays.
- [x] Remove engineering diagnostics and optional vector controls from the presentation UI.
- [x] Add an uncluttered actuator schematic showing the configured magnetorquer rods and animated four-wheel pyramid.
- [x] Verify clean native builds on current macOS, Linux, and Windows environments.
- [x] Profile accelerated simulation and viewer performance.

Application acceptance criteria:

- [x] The application visibly progresses from tumble through detumble to stable Sun pointing.
- [x] The viewer clearly distinguishes truth, measurements, estimates, and commands.
- [x] Automated tests validate the main mathematical and physical assumptions.
- [x] Monte Carlo results support the project's performance claims.

Validation status: debug and release builds are clean on macOS and all `103` tests pass. The documented 25-run release campaign also passes. GitHub Actions configured, compiled, and passed all tests on current macOS, Ubuntu, and Windows runners on 2026-09-12.

## Phase 15 — Portfolio Documentation and Open-Source Release

- [x] Add final screenshots and a demo GIF to the README.
- [x] Add an architecture diagram, equations, validation results, and build instructions to the README.
- [x] Complete `docs/REFERENCE.md` with architecture, data flow, frames, units, models, assumptions, and configuration.
- [x] Add a third-party dependency and asset attribution file.
- [x] Record a short demonstration showing tumble, detumble, Sun acquisition, and stable pointing.
- [x] Tag a reproducible `v0.1.0` release.
- [ ] Make the GitHub repository public as a portfolio project.

Release acceptance criteria:

- [x] A new user can build and run a default mission using the documented commands; all targets also build and test in the three-platform CI matrix.
- [x] Every major approximation and limitation is documented and explainable.

## Optional Extensions After the Final Product

- [ ] Magnetorquer-based reaction-wheel momentum unloading.
- [ ] Residual magnetic dipole, aerodynamic drag, and gravity-gradient disturbances.
- [ ] Improved Sun ephemeris or geomagnetic field models.
- [ ] Alternative attitude estimators, including a multiplicative extended Kalman filter.
- [ ] Additional fault injection and recovery behavior.
- [ ] Exported videos or presentation-ready mission reports.
