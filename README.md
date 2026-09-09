# Detumble

Detumble is a C++ spacecraft attitude-control simulation focused on autonomous CubeSat recovery after deployment or an attitude-control failure.

The simulated CubeSat begins in Low Earth Orbit with a randomized 3-axis tumble. Its flight software must autonomously reduce its angular velocity, stabilize the spacecraft, acquire the Sun, and eventually maintain a Sun-pointing attitude.

## Core Goals

* Model 3D rigid-body rotational dynamics in C++
* Represent spacecraft attitude using quaternions
* Simulate randomized initial angular velocity and orientation
* Implement autonomous detumbling using magnetorquers
* Model Earth's magnetic field sufficiently for B-dot control
* Implement a B-dot detumble controller
* Track angular velocity, attitude, actuator commands, and control modes
* Visualize the spacecraft's attitude and recovery process
* Add reaction-wheel-based Sun pointing after successful detumbling

## Initial Mission Scenario

The spacecraft is a small CubeSat in an approximately 500 km circular Low Earth Orbit around Earth.

The initial recovery sequence is:

1. CubeSat begins with an uncontrolled 3-axis tumble
2. Magnetometer measures Earth's magnetic field
3. B-dot controller commands the magnetorquers
4. Angular velocity is reduced below a defined threshold
5. TRIAD and gyroscope measurements estimate spacecraft attitude
6. A quaternion PD controller requests Sun-pointing torque
7. A four-wheel pyramid exchanges angular momentum with the spacecraft
8. Autonomous flight modes wait safely through eclipse, acquire the Sun, and maintain pointing

## Scope

This is not intended to become a general-purpose spacecraft simulator.

The project should remain focused on:

* attitude dynamics
* spacecraft control
* sensors and actuators
* autonomous mode transitions
* environmental effects relevant to ADCS

Orbital mechanics should only be modeled to the level necessary to provide the spacecraft's position and surrounding environment.

## Development Philosophy

Build incrementally and validate the physics at each stage.

The first milestone is intentionally narrow:

> Given a CubeSat with randomized initial attitude and angular velocity, use a C++ attitude simulation and B-dot controller to reduce its body rates below a defined detumble threshold.

The validated project now includes visualization, realistic seeded sensor errors, attitude estimation, reaction-wheel Sun pointing, eclipse-safe autonomous modes, timestamped mode events, and Monte Carlo robustness analysis. Additional physical disturbances and broader fault recovery remain later layers.

## Development

Detumble requires a C++20 compiler, CMake 3.25 or newer, and Ninja. CMake downloads the pinned math, testing, rendering, and user-interface dependencies during the first configuration.

On Debian or Ubuntu, install raylib's desktop build prerequisites first:

```bash
sudo apt-get install libasound2-dev libgl1-mesa-dev libx11-dev \
  libxcursor-dev libxi-dev libxinerama-dev libxrandr-dev
```

Configure, build, and test the debug preset:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Run the autonomous detumble/Sun-pointing simulation and live desktop viewer:

```bash
./build/debug/detumble
./build/debug/detumble_viewer
```

The applications use reproducible sensor noise, per-run bias, quantization, and saturation by default. Use `--sensor-profile ideal` with the CLI when comparing against the zero-error mathematical baseline.

Configure a headless run and export full-rate telemetry with:

```bash
./build/debug/detumble --duration 6000 --time-step 0.01 \
  --seed 42 --initial-rate-deg-s 10 --output runs/detumble.csv
```

The viewer starts the same deterministic tumbling 3U CubeSat in a simplified 500 km orbit. B-dot magnetorquers detumble it, TRIAD plus gyro propagation estimate its attitude, and a four-reaction-wheel pyramid points body `+Z` toward the Sun. Autonomous `BOOT`, `DETUMBLE`, `SAFE`, `SUN_ACQUIRE`, and `SUN_POINT` modes gate the controllers and wait safely through eclipse. The custom model includes magnetorquers, coarse Sun sensors, an isolated magnetometer, and animated reaction wheels. Live status and telemetry distinguish truth scoring, measurements, estimates, commands, applied actuator torque, mode events, eclipse state, and pointing performance.

Run the documented 25-case robustness campaign with the optimized build:

```bash
./build/release/detumble_monte_carlo --runs 25 --duration 8000 \
  --time-step 0.02 --seed 2026 \
  --results runs/phase13_runs.csv \
  --aggregate runs/phase13_aggregate.csv
```

This envelope randomizes attitude, tumble axis and speed, initial orbit position, sensor biases, and sensor noise. The Phase 13 baseline passed `25/25` runs with a mean sunlit RMS true pointing error of `0.961 deg`; the full assumptions and limitations are in [`docs/REFERENCE.md`](docs/REFERENCE.md).

Use the equivalent `release` presets for an optimized build. Build artifacts are written beneath `build/` and are ignored by Git.

The project is organized as separate targets:

- `detumble_core`: rendering-independent simulation library
- `detumble_cli`: headless command-line simulation, emitted as `detumble`
- `detumble_monte_carlo`: randomized headless mission validation
- `detumble_viewer`: desktop visualization application
- `detumble_tests`: automated tests

Third-party model attribution is recorded in [`THIRD_PARTY.md`](THIRD_PARTY.md).

See [`docs/PLAN.md`](docs/PLAN.md) for the incremental roadmap and [`docs/REFERENCE.md`](docs/REFERENCE.md) for the canonical technical reference.

## License

Detumble is available under the [MIT License](LICENSE).
