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
5. Spacecraft transitions out of detumble mode
6. Future versions acquire the Sun and use reaction wheels to establish Sun-pointing attitude

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

Visualization, Sun acquisition, reaction wheels, sensor noise, disturbances, Monte Carlo analysis, and fault recovery can be layered on after the basic detumble simulation is working correctly.

## Development

Detumble requires a C++20 compiler, CMake 3.25 or newer, and Ninja. CMake downloads the pinned Eigen and Catch2 dependencies during the first configuration.

Configure, build, and test the debug preset:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Run the current command-line and viewer placeholders:

```bash
./build/debug/detumble
./build/debug/detumble_viewer
```

Use the equivalent `release` presets for an optimized build. Build artifacts are written beneath `build/` and are ignored by Git.

The project is organized as separate targets:

- `detumble_core`: rendering-independent simulation library
- `detumble_cli`: headless command-line application, emitted as `detumble`
- `detumble_viewer`: desktop visualization application
- `detumble_tests`: automated tests

See [`docs/PLAN.md`](docs/PLAN.md) for the incremental roadmap and [`docs/REFERENCE.md`](docs/REFERENCE.md) for the canonical technical reference.

## License

Detumble is available under the [MIT License](LICENSE).
