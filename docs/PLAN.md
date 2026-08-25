# Detumble Development Plan

Build Detumble as a sequence of small, validated milestones. Complete each phase's validation gate before moving to the next phase. Prefer a correct, understandable approximation over a more sophisticated model that cannot be clearly explained or tested.

## Final Product

Detumble will be a cross-platform C++ desktop simulation of a generic 3U CubeSat recovering from an uncontrolled tumble in a 500 km circular Low Earth Orbit. The spacecraft will use magnetorquers for B-dot detumbling, estimate its attitude from realistic sensor measurements, acquire the Sun, and maintain a Sun-pointing attitude with four reaction wheels.

The desktop viewer will provide close-up and orbital camera views, live telemetry, flight-mode status, and optional overlays for reference frames, angular velocity, magnetic field, Sun direction, actuator commands, and control torque.

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

## Proposed Technical Stack

- C++20 for the simulator, controllers, estimator, command-line tools, and viewer
- CMake and CMake presets for macOS, Linux, and Windows builds
- Eigen for vectors, matrices, quaternions, and wheel-allocation math
- Catch2 for unit and integration tests
- raylib for the desktop window, camera, rendering, and model loading
- Dear ImGui with ImPlot for controls and telemetry plots
- CLI11 for headless simulation arguments
- nlohmann/json for scenario configuration and run metadata
- Blender for authoring a custom 3U CubeSat and exporting it as GLB
- CSV for human-readable telemetry output
- GitHub Actions for cross-platform builds and tests

Dependency versions will be pinned after small compatibility prototypes confirm the complete stack works together.

## Engineering Rules

- [ ] Define every vector's reference frame and use SI units internally.
- [ ] Keep the simulation core independent of rendering and user-interface code.
- [ ] Keep the true simulation state private from flight software and controllers.
- [ ] Make every randomized run reproducible from a recorded seed.
- [ ] Use fixed simulation timesteps and explicit sensor and controller sample rates.
- [ ] Add tests before increasing a model's physical complexity.
- [ ] Record important assumptions, coordinate conventions, and architecture in `docs/REFERENCE.md`.
- [ ] Add personal explanations and lessons to the ignored `docs/CONCEPTS.md` throughout development.

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

- [ ] Define the Earth-centered inertial, orbital, and spacecraft body frames.
- [ ] Choose and document right-handed axis directions.
- [ ] Define whether the attitude quaternion maps body coordinates to inertial coordinates or the reverse.
- [ ] Define quaternion storage and multiplication conventions.
- [ ] Add strongly named state structures with units included in member names where helpful.
- [ ] Implement small helpers for vector-frame transformations.
- [ ] Implement quaternion normalization and attitude integration using Eigen primitives.
- [ ] Test identity, known-axis rotations, inverse rotations, composition order, and normalization.
- [ ] Explain vectors, frames, angular velocity, rotation matrices, and quaternions in `docs/CONCEPTS.md`.

Validation gate:

- [ ] Known vectors rotate between body and inertial frames with the expected direction and sign.

## Phase 2 — Rigid-Body Attitude Dynamics

- [ ] Define approximate mass, dimensions, center of mass, and diagonal inertia for a generic 3U CubeSat.
- [ ] Implement Euler's rigid-body rotational equation.
- [ ] Propagate angular velocity and attitude with a fixed-step RK4 integrator.
- [ ] Renormalize the attitude quaternion at a documented point in the integration loop.
- [ ] Support zero torque and externally supplied body torque.
- [ ] Add deterministic initial attitude and angular-velocity generation from a seed.
- [ ] Add a minimal CLI run that prints final attitude, angular velocity, and rotational energy.
- [ ] Test spherical-body motion, asymmetric torque-free motion, and constant applied torque.
- [ ] Track angular momentum, rotational energy, and quaternion norm for validation.
- [ ] Explain inertia, torque, angular momentum, and rigid-body dynamics in `docs/CONCEPTS.md`.

Validation gate:

- [ ] Torque-free runs conserve angular momentum and rotational energy within documented numerical tolerances.
- [ ] Quaternion norm remains within its documented tolerance.

## Phase 3 — First Visual Slice

- [ ] Prototype raylib, Dear ImGui, ImPlot, and GLB loading in one cross-platform viewer.
- [ ] Pin compatible visualization dependency versions.
- [ ] Render a simple 3U rectangular placeholder using the propagated attitude.
- [ ] Add an orbiting close-up camera and body-axis overlay.
- [ ] Add pause, resume, reset, single-step, and playback-speed controls.
- [ ] Plot body rates and quaternion norm live.
- [ ] Keep simulation updates fixed-step and independent of rendering frame rate.

Validation gate:

- [ ] The same seeded run produces the same final state with and without the viewer.
- [ ] The visible body-axis rotations agree with the frame-convention tests.

## Phase 4 — Orbit and Magnetic Environment

- [ ] Implement a simplified 500 km circular orbit with configurable inclination.
- [ ] Produce spacecraft position in the inertial frame as a function of simulation time.
- [ ] Implement a centered, tilted-dipole Earth magnetic-field model.
- [ ] Transform the local magnetic field from inertial coordinates into body coordinates.
- [ ] Validate magnetic-field magnitude and direction at selected orbital positions.
- [ ] Record position and magnetic-field telemetry.
- [ ] Add optional orbit-path and magnetic-field-vector overlays to the viewer.
- [ ] Explain Low Earth Orbit, orbital frames, and Earth's magnetic field in `docs/CONCEPTS.md`.

Validation gate:

- [ ] One complete orbit is periodic and the local magnetic field varies smoothly through a plausible range.

## Phase 5 — Magnetometer and Magnetorquers

- [ ] Implement an ideal three-axis magnetometer with a configurable sample rate.
- [ ] Implement three orthogonal magnetorquers with configurable dipole limits.
- [ ] Convert commanded coil dipoles into a combined body-frame magnetic dipole.
- [ ] Compute magnetic torque using the commanded dipole and local magnetic field.
- [ ] Model a control cycle that disables the torquers before magnetometer sampling.
- [ ] Enforce actuator saturation and record commanded versus applied dipole.
- [ ] Test that magnetic torque is perpendicular to the magnetic field.
- [ ] Test that no magnetorquer command exceeds its configured limit.
- [ ] Explain magnetometers, magnetorquers, magnetic dipole, and magnetic torque in `docs/CONCEPTS.md`.

Validation gate:

- [ ] Known magnetic-dipole and field vectors produce the expected torque direction and magnitude.

## Phase 6 — B-dot Detumble Milestone

- [ ] Estimate the body-frame magnetic-field derivative from sampled measurements.
- [ ] Add a simple configurable low-pass filter for the derivative estimate.
- [ ] Implement a saturated B-dot controller with an explicitly tested sign convention.
- [ ] Add ideal gyro rate measurements for transition detection and telemetry.
- [ ] Define a detumble threshold, hysteresis band, and dwell time.
- [ ] Add `BOOT` and `DETUMBLE` flight modes.
- [ ] Add CLI configuration for duration, timestep, seed, initial rate, and output path.
- [ ] Export attitude, body rate, energy, field, dipole, torque, and flight mode to CSV.
- [ ] Plot rate, energy, dipole command, and control mode in the viewer.
- [ ] Test B-dot against representative fixed and randomized initial states.
- [ ] Explain B-dot control, gain, filtering, saturation, hysteresis, and dwell time in `docs/CONCEPTS.md`.

Validation gate:

- [ ] A documented batch of seeded scenarios reduces body rates below the detumble threshold for the required dwell time.
- [ ] Rotational kinetic energy trends downward across the batch despite short local increases.

Milestone result: a complete, validated headless and visual B-dot detumble simulation.

## Phase 7 — Custom 3U Spacecraft and Environment Visualization

- [ ] Learn the minimal Blender workflow needed to create, name, and export components.
- [ ] Create a dimensionally consistent generic 3U CubeSat model.
- [ ] Place and name three orthogonal magnetorquer rods.
- [ ] Add exterior coarse Sun sensors and an isolated magnetometer location.
- [ ] Add four reaction-wheel placeholders in a pyramid arrangement.
- [ ] Align the GLB origin, scale, and axes with the simulation body frame.
- [ ] Keep component positions and orientations in simulation configuration rather than relying on the mesh as physics data.
- [ ] Add solid, transparent, and cutaway rendering modes.
- [ ] Visualize angular velocity, magnetic field, magnetic dipole, and torque with distinct vector arrows.
- [ ] Credit any NASA textures or reference assets in a third-party attribution file.

Validation gate:

- [ ] Every visual component and vector agrees with the configured body-frame orientation.

## Phase 8 — Sun Environment and Sensors

- [ ] Implement a simplified inertial Sun direction.
- [ ] Implement Earth-occultation logic for eclipse detection.
- [ ] Implement a three-axis gyroscope interface with a configurable sample rate.
- [ ] Implement coarse Sun sensors using face normals, field of view, and illumination.
- [ ] Produce no valid Sun measurement during eclipse.
- [ ] Add Sun direction, sensor visibility, and eclipse state to telemetry.
- [ ] Render Earth, sunlight direction, orbit path, eclipse state, and an orbital overview camera.
- [ ] Explain gyroscopes, coarse Sun sensors, sensor field of view, and eclipse in `docs/CONCEPTS.md`.

Validation gate:

- [ ] Sun-sensor readings agree with known spacecraft attitudes and disappear correctly during eclipse.

## Phase 9 — Attitude Determination

- [ ] Keep the true attitude inaccessible to the estimator and flight controller.
- [ ] Implement TRIAD using measured magnetic-field and Sun vectors.
- [ ] Detect and reject invalid or nearly parallel vector pairs.
- [ ] Propagate the estimated quaternion between corrections using gyro measurements.
- [ ] Add a simple tunable quaternion correction toward the TRIAD solution.
- [ ] Continue gyro propagation without Sun corrections during eclipse.
- [ ] Track attitude-estimation error using truth data available only to simulation telemetry.
- [ ] Test known attitudes, slowly rotating cases, eclipse intervals, and invalid measurements.
- [ ] Plot estimated-versus-true attitude error.
- [ ] Explain attitude determination, TRIAD, gyro propagation, and sensor fusion in `docs/CONCEPTS.md`.

Validation gate:

- [ ] With ideal sensors, the estimator converges to a documented attitude-error tolerance.
- [ ] During a representative eclipse, estimation error remains bounded and recovers after Sun measurements return.

## Phase 10 — Sun Acquisition with Ideal Control Torque

- [ ] Choose and document the spacecraft face and body axis that point toward the Sun.
- [ ] Compute a desired Sun-pointing quaternion while preserving a defined roll reference.
- [ ] Compute quaternion attitude error with the shortest-rotation convention.
- [ ] Implement a proportional-derivative attitude controller.
- [ ] Apply the requested body torque directly before introducing reaction-wheel mechanics.
- [ ] Add torque limits and anti-chatter behavior near the pointing target.
- [ ] Define acquisition and steady-pointing error thresholds with dwell times.
- [ ] Test large-angle commands, small-angle settling, and quaternion sign equivalence.
- [ ] Explain quaternion error and PD attitude control in `docs/CONCEPTS.md`.

Validation gate:

- [ ] From several detumbled attitudes, ideal control torque acquires and maintains the Sun-pointing target without unstable oscillation.

## Phase 11 — Four-Wheel Pyramid

- [ ] Implement a general reaction-wheel cluster defined by wheel-axis vectors.
- [ ] Validate the cluster first with three orthogonal wheels.
- [ ] Configure four tilted wheels in a pyramidal arrangement.
- [ ] Allocate requested body torque across the wheels using Eigen matrix operations.
- [ ] Model equal and opposite spacecraft and wheel torques.
- [ ] Integrate individual wheel speeds.
- [ ] Enforce wheel torque and speed limits.
- [ ] Report allocation error and saturation in telemetry.
- [ ] Add optional single-wheel failure scenarios after nominal behavior works.
- [ ] Animate wheel spin and highlight saturated or failed wheels in the cutaway view.
- [ ] Explain reaction wheels, momentum exchange, allocation matrices, saturation, and redundancy in `docs/CONCEPTS.md`.

Validation gate:

- [ ] The four-wheel cluster tracks achievable three-axis torque commands within tolerance.
- [ ] The Sun-pointing controller remains stable using wheel-generated torque instead of ideal torque.

## Phase 12 — Complete Autonomous Mission

- [ ] Define `SUN_ACQUIRE`, `SUN_POINT`, and `SAFE` flight modes.
- [ ] Transition automatically from deployment through detumble, acquisition, and pointing.
- [ ] Add hysteresis and dwell times to every mode transition.
- [ ] Define behavior when the Sun is unavailable during eclipse.
- [ ] Define behavior when estimation becomes invalid.
- [ ] Prevent controllers from issuing commands outside their active modes.
- [ ] Record every mode transition and its reason.
- [ ] Display the current mode and transition history in the viewer.
- [ ] Add an end-to-end deterministic mission test.

Validation gate:

- [ ] A seeded mission autonomously detumbles, acquires the Sun when visible, and maintains Sun pointing without reading truth-state attitude.

Milestone result: the complete autonomous recovery and Sun-pointing mission.

## Phase 13 — Sensor Realism and Robustness

- [ ] Add configurable Gaussian noise to gyro, magnetometer, and Sun-sensor measurements.
- [ ] Add reproducible per-run sensor biases.
- [ ] Add sensor quantization and saturation where meaningful.
- [ ] Tune estimator correction, B-dot filtering, and controllers using documented criteria.
- [ ] Build a headless Monte Carlo runner for randomized attitude, body rates, orbit position, noise, and bias.
- [ ] Define success, failure, detumble-time, acquisition-time, and pointing-error metrics.
- [ ] Save aggregate results separately from per-run telemetry.
- [ ] Investigate and document representative failed cases rather than hiding them.
- [ ] Add regression seeds for important edge cases.
- [ ] Explain sensor error, Monte Carlo testing, and robustness metrics in `docs/CONCEPTS.md`.

Validation gate:

- [ ] The mission meets documented success-rate and pointing-performance targets across a documented scenario envelope.

## Phase 14 — Final Application and Portfolio Polish

- [ ] Finalize close-up, cutaway, and orbital camera views.
- [ ] Finalize telemetry plots, units, legends, colors, and vector-arrow scaling.
- [ ] Add a concise scenario panel with seed, time, mode, rate, pointing error, eclipse state, and actuator status.
- [ ] Support live runs and playback of recorded runs.
- [ ] Add sensible default scenarios without turning the viewer into a large configuration editor.
- [ ] Add keyboard and mouse help inside the application.
- [ ] Verify clean builds on current macOS, Linux, and Windows environments.
- [ ] Profile accelerated simulation and viewer performance.
- [ ] Add screenshots, an architecture diagram, equations, validation results, and build instructions to the README.
- [ ] Complete `docs/REFERENCE.md` with architecture, data flow, frames, units, models, assumptions, and configuration.
- [ ] Add a third-party dependency and asset attribution file.
- [ ] Record a short demonstration showing tumble, detumble, Sun acquisition, and stable pointing.
- [ ] Tag a reproducible release.
- [ ] Make the GitHub repository public when it is ready to use as a portfolio project.

Final acceptance criteria:

- [ ] A new user can build and run a default mission using the documented commands.
- [ ] The application visibly progresses from tumble through detumble to stable Sun pointing.
- [ ] The viewer clearly distinguishes truth, measurements, estimates, and commands.
- [ ] Automated tests validate the main mathematical and physical assumptions.
- [ ] Monte Carlo results support the project's performance claims.
- [ ] Every major approximation and limitation is documented and explainable.

## Optional Extensions After the Final Product

- [ ] Magnetorquer-based reaction-wheel momentum unloading.
- [ ] Residual magnetic dipole, aerodynamic drag, and gravity-gradient disturbances.
- [ ] Improved Sun ephemeris or geomagnetic field models.
- [ ] Alternative attitude estimators, including a multiplicative extended Kalman filter.
- [ ] Additional fault injection and recovery behavior.
- [ ] Exported videos or presentation-ready mission reports.
