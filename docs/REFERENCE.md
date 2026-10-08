# Detumble technical reference

This is the canonical reference for the magnetic-only detumble simulator, version 0.2.0. Internal values use SI units. Application output converts rates and angular errors to degrees for readability.

For a plain-language introduction, start with [the README](../README.md) and
[the magnetic-field walkthrough](MAGNETIC_FIELD.md). Recorded experiments and
reproduction commands are in [validation](validation/README.md).

## Build portability

GitHub Actions builds the viewer and runs the test suite on Linux, macOS, and
Windows. GNU/MinGW builds enable the assembler's large COFF object format for
the core library because Eigen's estimator templates exceed the normal section
limit in debug builds. This changes object-file packaging, not simulation behavior.

## Mission and boundary

A generic rigid 3U CubeSat begins in an arbitrary seeded attitude with a seeded tumble axis. Three orthogonal magnetorquers damp rotation using one three-axis magnetometer. The mission goal is angular-speed magnitude at or below 0.5 deg/s continuously for 30 s. Damping continues afterward; there is no target pointing direction.

A completed headless mission additionally needs a full final 300 s window whose maximum true speed is at or below 0.7 deg/s. A short run cannot pass this verification. The goal badge is latched after the true-rate dwell, while flight-software `LOW_RATE` is reversible and based on estimates. These are deliberately separate measurements of progress.

## Frames and rigid-body dynamics

- **ECI:** Earth-centered inertial coordinates, +Z along Earth's spin axis.
- **ECEF:** Earth-fixed coordinates, coincident with ECI at simulation time zero; rotate about +Z as Earth turns.
- **RTN:** Radial, along-track, orbit-normal frame.
- **Body:** CubeSat-fixed right-handed coordinates, long dimension along Z.

The unit quaternion `body_to_inertial` rotates a body vector into ECI. Its inverse is the conjugate. Quaternion construction is `(w,x,y,z)`; Eigen's raw `coeffs()` storage is `(x,y,z,w)`. Quaternion signs `q` and `-q` represent the same attitude.

Euler's equation is

`I omega_dot = torque - omega × (I omega)`.

The plant propagates quaternion and body-frame angular velocity with fixed-step RK4 and normalizes the quaternion after each step. Body inertia comes from `generic_3u_cubesat()` in `dynamics.cpp`; visual geometry is not the inertia source. Magnetic torque is held over each physical step. The default timestep is 0.02 s.

The circular orbit has Earth radius 6,371 km, altitude 500 km, inclination 51.6 degrees, and gravitational parameter 3.986004418e14 m³/s². Position and velocity are analytic functions of elapsed time; attitude does not affect the orbit. Randomized missions vary the initial argument of latitude.

## Earth's magnetic field

`MagneticFieldConfig` selects `igrf14` (default) or `dipole`. `Environment` samples the true field at the prescribed orbital position and transforms it into the true body frame for the sensor/plant. A separate navigation `Environment` computes the estimator's ECI reference from known orbit and time without using true attitude.

This is a local main-field calculation, not a full magnetosphere or solar-wind
simulation. The viewer's globe and wire grid depict Earth and its rendering
geometry; they do not show magnetic field lines. Display scales do not feed back
into the physical field calculation.

### IGRF-14

The bundled official IAGA coefficient table is `assets/geomagnetic/igrf14coeffs.txt`. Its SHA-256 is `8f8d88403028fc4ee92c4f38d97b46e0a87e2cfc496045b43c9e26c1d6b0903c`. The stdlib-only `assets/scripts/generate_igrf14_coefficients.py` verifies it and regenerates `src/igrf14_coefficients.hpp`. No field data download or filesystem read is needed at runtime.

The implementation synthesizes the internal main field using degree-13 Schmidt quasi-normal spherical harmonics, the official 6,371.2 km reference radius, and coefficients in nanotesla. The 2025 main field is advanced with its secular-variation coefficients (nonzero through degree 8). Results are converted to tesla.

The field is the negative gradient of a scalar magnetic potential,
`B = -gradient(V)`. Each degree/order term combines a radial factor,
Schmidt-normalized associated Legendre functions, and longitude dependence
weighted by the dated Gauss coefficients `g(n,m)` and `h(n,m)`. The degree-1
terms describe the dipole contribution; higher degrees add spatial detail.
Coefficient evolution is linear: `g(t) = g(2025) + (t - 2025) * dg`, with the
same rule for `h`, where `t` is decimal year.

The scenario epoch must be a valid `YYYY-MM-DDTHH:MM:SSZ` date between 2025-01-01 and 2030-01-01. Elapsed seconds advance the decimal year using the actual length of each calendar year. Evaluation outside that interval is rejected. The default epoch is 2025-01-01T00:00:00Z, preserving reproducibility across dates on which the program is run.

At elapsed `t`, ECI position rotates by `-earth_rotation_rate*t` into ECEF. Geocentric radius, colatitude, and longitude feed the synthesis. Its north/east/down components transform into ECEF Cartesian coordinates, then back to ECI with the positive rotation. Both poles have explicit finite limiting evaluations.

The ECI/ECEF zero alignment is a scenario convention, not a Greenwich sidereal-time calculation. The spherical/geocentric orbit does not require geodetic latitude or ellipsoid conversion. This model includes the internal main field, not space-weather or externally driven magnetic disturbances.

Six independent fixtures check north/east/down against the official `igrf14syn` Fortran implementation within 0.02 nT, covering dates, longitude, altitude, and both poles. This tolerance measures agreement with the reference implementation, not accuracy relative to the real geomagnetic field. Source attribution is in `THIRD_PARTY.md` and `assets/geomagnetic/README.md`.

### Dipole and mismatch experiments

The previous centered tilted dipole is retained. Its axis rotates with Earth; field falls with inverse radius cubed. The CLI can set truth and reference models independently. `--reference-field-model dipole` with IGRF truth tests reference mismatch. Default matching references represent ideal navigation/model knowledge, rather than a guaranteed real spacecraft condition.

## Measurement and actuation

The magnetometer returns timestamped body-frame vectors at 10 Hz. The realistic profile applies seeded constant bias, independent Gaussian noise, quantization, and component-wise saturation:

| Quantity | Default per axis |
| --- | ---: |
| Noise standard deviation | 100 nT |
| Bias standard deviation | 300 nT |
| Quantization step | 10 nT |
| Saturation limit | 100 µT |

The ideal profile removes those errors. Sensor random streams are seeded independently of initial-state generation. Reset restores the same initial state, bias, and noise sequence.

Each rod is limited to ±0.2 A m². The magnetic control cycle samples with all rods off for that physical step; rods operate during the remaining steps. This is discrete ideal coil blanking, not a coil-current decay or interference model. At the validated default timestep, five steps span a sample interval. Changing timestep also changes this simplified blanking duty cycle, so timestep comparisons can include actuator-duty effects.

At the defaults, this gives 20 ms off for sampling and up to 80 ms actuation in
each 100 ms interval. During actuation, the signed dipole can vary in magnitude
and reverse direction. The simulator commands dipole directly; electrical PWM,
current regulation, and power consumption are outside the model.

`torque = applied_dipole × true_field`.

No magnetic command can create instantaneous torque parallel to the field. Field direction changes along the orbit, creating opportunities to damp different rate components over time.

## B-dot baseline

`BdotEstimator` differences consecutive measurements at their actual sample times, then applies a first-order low-pass derivative filter with time constant 0.2 s. The command is

`m = saturate(-k * filtered_B_dot)`.

Default `k` is 50,000 A m² s/T. The same per-axis limits apply to all controllers. B-dot needs two fresh valid samples and no attitude or angular-rate estimate. Orbital field variation remains in its measured derivative; it can sustain small residual motion near the end of detumbling.

With a slowly varying inertial field, `B_dot_body ≈ B_body × omega`. This makes the damping sign consistent with the optional rate-based command below. The approximation becomes weaker when body rotation is small relative to orbital field variation.

## Magnetometer-only estimator

`AttitudeEstimator` is a multiplicative extended Kalman filter with nominal quaternion attitude, body rate, and body-fixed magnetic bias. Its 9-component error state is `[small_body_rotation, rate_error, bias_error]` with a 9×9 covariance. It never receives true attitude, true rate, true sensor bias, or a gyro.

Inputs are timestamped magnetic measurements, ECI field references computed from navigation, and known applied rod dipoles. Those dipoles are the actual limited/blanked commands, not an unconstrained request. The estimator uses the same documented inertia model as the plant.

### Initialization and observability

A single magnetic vector leaves orientation about that vector undetermined. Startup therefore collects 60 s of magnetic history and fits attitude, initial angular velocity, and constant bias jointly. Twelve rolls about the first reference vector and five field-parallel rates seed 60 deterministic nonlinear fits. Initial transverse rate comes from early field differences. Fits extend through 2 s, 10 s, and the full window to reduce rate aliases. The optimizer uses scaled finite-difference Jacobians and bounded Levenberg–Marquardt steps.

The initialization search seeds field-parallel rates up to ±20 deg/s. This is a search envelope, not a hard physical limit or a supplied true-rate prior. The validated mission range is 5–15 deg/s.

Average applied dipole between observations and intermediate reference fields drive candidate dynamics. Sensor variance and a zero-mean bias prior weight residuals. Fits with excessive normalized residual cost are rejected; the window slides by half its width and retries. A cheap field-magnitude consistency check avoids fitting grossly mismatched references.

Plausible distinct fits within 25 cost units of the best remain candidates. Effectively identical fits are deduplicated. Initial covariance comes from the fit's information matrix, with large uncertainty retained along nearly unobservable directions, and is transported to the end of the history.

Static reference-field tests must remain uncertain. Magnetic-only full attitude relies on time-varying references and known dynamics; it is not instantly observable or guaranteed for every geometry. An optimizer can miss hypotheses, and covariance is a local approximation. These limitations remain even when nominal campaigns pass.

### Prediction and correction

Nominal state uses RK4 rigid-body prediction with modeled magnetic torque. The local error transition is a second-order approximation to the linearized attitude/rate dynamics. Process covariance adds rate diffusion from white acceleration noise (1e-5 rad/s/√s) and bias random walk (1e-10 T/√s). Rate variance grows with elapsed time; its coupled angle variance grows with the cube of elapsed time.

The measurement equation is

`B_measured = q_inverse * B_reference + bias + noise`.

The observation Jacobian uses `[cross_matrix(predicted_field), 0, Identity]`. Measurement variance combines configured sensor noise, quantization variance, and a 100 nT model-error allowance. The update injects a small quaternion rotation, rate correction, and bias correction, uses Joseph covariance form, and applies the multiplicative attitude-error reset. Quaternions stay normalized.

Normalized innovation is residualᵀ × residual_covariance_inverse × residual. Values above 16.3 reject that sample. Rejection removes controller confidence; 20 consecutive rejected samples discard a candidate. Losing all candidates restarts magnetic-history collection.

Published status is `collecting magnetic history`, `orientation/rate ambiguous`, `tracking`, or `measurement rejected`. Rate uncertainty combines the largest rate-covariance standard deviation across plausible candidates with the spread of their rates. Confidence requires valid finite covariance, an accepted latest measurement, and rate uncertainty below 0.05 deg/s. Attitude ambiguity does not necessarily prevent a useful rate estimate.

## Flight software and estimated-rate damping

`DetumbleFlightSoftware` owns B-dot, the estimator, commands, and transition events. It takes measured field/reference inputs, never simulation truth.

| Mode | Behavior |
| --- | --- |
| `BOOT` | Collect measurements; rods off until two valid samples exist |
| `DETUMBLE` | Magnetic damping, B-dot by default |
| `LOW_RATE` | Estimated low-rate dwell confirmed; damping continues |
| `SAFE` | Invalid/stale magnetic data; rods off, estimator and derivative history reset |

Invalid measurements include nonfinite vectors, implausibly small magnitude, saturation at the configured sensor limit, nonchronological timestamps, and a sample timestamp inconsistent with the update time. A gap longer than 0.3 s enters `SAFE`. Two subsequent valid samples permit damping again.

`--controller estimated-rate` initially uses B-dot. After 30 s of sustained estimator confidence it commands

`m = saturate(k * omega_estimated × B_measured)`.

Loss of confidence immediately restores B-dot at the next measurement update. Invalid/stale magnetic data instead enter `SAFE`. Handoff applies to rate confidence and does not require precise full attitude.

Flight-software completion requires `norm(omega_estimated) + 3*rate_std ≤ 0.5 deg/s` for 30 s. `LOW_RATE` exits when confidence is lost or the upper rate bound exceeds 0.7 deg/s. With the estimator disabled, damping works but flight software cannot assert low rotation. Truth scoring can still show mission success.

## Shared mission architecture and timing

`DetumbleMission` owns plant, true environment, independent navigation environment, magnetic cycle, flight software, telemetry, and scoring. `step()` performs:

1. Use the field at the beginning of the physical step and independently compute the navigation reference.
2. Apply the preceding commanded dipole through limits/measurement blanking; obtain any new sample.
3. Update flight software, producing the command for the next step. If `SAFE`, remove applied torque immediately.
4. Predict the estimator with the known applied dipole, then propagate the plant.
5. Sample endpoint truth/environment and update telemetry and validation metrics.

Telemetry contains endpoint truth and estimate, the command/applied torque from the preceding interval, and the explicitly recorded timestamp of the most recent sensor sample. A command can differ from the applied dipole because of limiting, blanking, or the one-step command delay.

`run_native_mission()` wraps this exact step loop for CLI/campaigns. The viewer advances the same runner with a wall-time accumulator. A per-frame step budget keeps rendering responsive without discarding pending simulated time. Playback is fixed at 20×; pause and rendering frame count do not change the trajectory. Step grouping and reset/replay are tested.

## Applications, telemetry, and reproducibility

`detumble` exports full-step CSV with scenario/model identifiers, true and estimated quaternions/rates, magnetic samples, bias, rate uncertainty, innovation, commanded/applied dipoles, torque, mode, coil blanking, and the goal badge. Unavailable sample/estimate metrics use NaN or an explicit sentinel; validity columns describe usability.

`detumble_monte_carlo` draws a mission seed, rate uniformly from 5–15 deg/s, and orbit phase from 0–360 degrees from a master seed. The mission seed determines arbitrary attitude, tumble axis, and sensor errors. Each scenario consumes exactly three master-generator draws. `--compare` runs both controllers with identical scenario parameters. `--start-run` skips complete scenarios so independently processed slices merge without changing cases.

Per-run CSV records model/epoch, sensor profile, duration/timestep, seed/rate/orbit phase, success/failure, detumble time, final-window rate, estimator convergence, first handoff, estimation errors, saturation steps, false `LOW_RATE` entries, and integrated squared applied dipole. The convergence timestamp records the first confident estimate, not independently verified accuracy or sustained confidence. Aggregate CSV reports success count, times, rates, effort, and estimator/controller counts. Optional `--telemetry-dir` writes 1 Hz traces. Neither integrated dipole squared nor saturation count is a battery-power model.

Failures distinguish no qualifying dwell (`detumble timeout`), too little final-window history (`insufficient verification`), and excessive final-window rotation (`residual rate exceeded`). A false `LOW_RATE` event means an entry into that mode while the true endpoint rate exceeds 0.5 deg/s.

`assets/scripts/analyze_campaign.py` validates paired cases, detects duplicates, merges slices, and calculates mean/median/95th-percentile/maximum detumble times. Recorded configuration and compact results live in `docs/validation/`.

## Visualization and model

The focused viewer uses raylib and Dear ImGui, retaining the orbital camera, Earth, stars, orbit trail, and NASA ICECube exterior GLB. Playback is fixed at 20×. A permanent, full-height sidebar occupies the left quarter of the window, with telemetry and rod activity at the top. Controls are anchored at the bottom: three equal-width scenario buttons (Gentle, Nominal, Fast), then two equal-width Pause/Play and Reset buttons. Both rows use the same 32-pixel button height. Shortcut hints and the O overview shortcut are removed; right-drag orbit, scroll zoom, Space pause, and R reset remain available. The sidebar font is 15.3 pixels, one pixel larger than the previous size. Its light slate palette uses the same dark navy color for all text, including subtitles, help, status, axis labels, and selected buttons. Selection changes the button's blue background, not its text color. Activity bars and hardware highlights retain red/teal/blue accents. The title bar is omitted. It shows simulated rotation to one decimal, elapsed time to whole seconds, and the true mission goal. Numeric readouts refresh twice per real second.

The orbital scene renders into a texture sized to the remaining three quarters of the window. This gives its camera the correct aspect ratio and centers Earth within that area. The texture is recreated when the window changes size and released at shutdown. The minimum window size is 1200 × 720 so sidebar controls remain readable.

`apps/rod_view.cpp` draws a procedural orthographic hardware schematic in a separate render texture, downsampled from twice its display resolution. A wireframe body envelope replaces the imported CAD assembly. The body origin is at its center; no reference-axis cross is drawn. Rod centers, axes, lengths, radii, body dimensions, and sensor placement/size come directly from `generic_3u_visual_config()`. The layout is the generic simulated arrangement, not ICECube's actual interior or a mechanically verified assembly. The simulator applies the net body dipole; rod positions are visual layout parameters and do not change torque.

The X/Y/Z callouts identify each rod. Three simple colored strength bars replace the history strips and ON/OFF text. Rod-center coordinates and dimensions are available on hover; the bar tooltip explains what the strength means. A short pink-magnetometer legend identifies the sensor. Rod sleeves use their axis color with shade indicating recent activity, and fade to gray when that activity decays to zero. The pink cube is the magnetometer. These colors are display annotations, not physical light emission. The schematic reaches 460 pixels at the default window size and reduces its height in shorter windows.

Bars and rod highlights show an exponential average of absolute applied dipole, normalized to the 0.2 A m² actuator limit. The averaging time constant is 10 simulated seconds, or half a real second at 20× playback. Averaging includes coil-off measurement steps and avoids rapid flicker; it describes recent applied magnetic strength, not instantaneous ON/OFF state or electrical power. Only physical steps update the averages, so pause freezes them and reset/scenario changes clear them. Numeric telemetry still refreshes twice per real second. Physics, flight software, and exported telemetry retain their full update rate.

The old SUCHAI-II cutaway and regeneration scripts remain as reference assets with attribution in `assets/models/README.md`; they are no longer loaded or packaged by the viewer. CMake packages the NASA exterior, the project-authored fallback, and the NASA attribution notice.

`assets/scripts/prepare_icecube.py` decodes the NASA source, bakes its transforms, centers the mesh, and uniformly scales its Z length to 0.34 m. It preserves exterior proportions and materials, with body +Z as the long axis and meter coordinates. This is a visual asset; simulation physics still describe the generic detumbling spacecraft. Missing or corrupt ICECube assets fall back to the project-authored GLB, then a generated cuboid.

`assets/scripts/create_detumble_3u.py` regenerates the fallback GLB with Blender. The model contains structural panels, three magnetic rods, and a boom-mounted magnetometer. No wheel or Sun-sensor components remain. C++ visual configuration defines component locations/axes; model bounds are approximately 0.115 × 0.104 × 0.3425 m. Rendering scale does not change physical inertia.

## Validation and practical limits

Automated tests and reproducible campaigns are described in `docs/validation/README.md`. Independent IGRF reference fixtures validate physics separately from the controller. Seeded realistic-sensor tests cover arbitrary orientations and initially field-parallel rotation; constant-field tests retain uncertainty; outliers cannot produce an accepted correction.

The default matched reference field and inertia are favorable modeling assumptions. Confidence has not been statistically calibrated across arbitrary navigation, inertia, bias drift, or external-field errors. The advanced controller remains optional and B-dot remains the default. A holdout campaign is evidence for its stated envelope, not flight qualification.

There is no orbit perturbation, residual spacecraft dipole, environmental disturbance torque, detailed sensor calibration/electronics, coil decay, power/thermal model, flexible body, gyro, Sun sensor, reaction wheel, or target-pointing controller.
