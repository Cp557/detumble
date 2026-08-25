# Detumble Technical Reference

This document is the canonical reference for Detumble's implemented architecture, build system, conventions, and technical assumptions. Update it whenever a significant implementation or architecture decision changes.

## Current Status

Phase 0 is complete. Debug and release builds pass locally on macOS, and the GitHub Actions debug build and smoke test pass on hosted macOS, Linux, and Windows runners. The current executables are intentionally small placeholders; spacecraft math and simulation behavior begin in Phase 1.

## Repository Layout

```text
apps/                   Application entry points
  cli_main.cpp          Headless simulator entry point
  viewer_main.cpp       Desktop viewer entry point
cmake/                  Shared CMake helpers
docs/
  PLAN.md               Sequential development roadmap
  REFERENCE.md          Canonical repository reference
include/detumble/       Public C++ headers
src/                    Simulation library implementation
tests/                  Unit and integration tests
.github/workflows/      Cross-platform continuous integration
```

`docs/CONCEPTS.md` is a private learning journal and is intentionally excluded from version control.

## Build Architecture

The project uses C++20 and CMake 3.25 or newer. Ninja is the standard local and continuous-integration generator. Debug and release workflows are defined in `CMakePresets.json`.

Targets:

- `detumble_core` is the rendering-independent static library. Other targets may access it through the `detumble::core` alias.
- `detumble_cli` is the headless executable and is emitted with the filename `detumble`.
- `detumble_viewer` is the desktop viewer executable. It currently has a placeholder entry point.
- `detumble_tests` contains tests discovered and run through CTest.

The simulation core must never depend on viewer or user-interface code. The CLI and viewer may both link to the core.

## Dependencies

Dependencies are fetched by CMake and pinned to release archives and SHA-256 hashes:

- Eigen 5.0.1 provides vector, matrix, quaternion, and decomposition primitives.
- Catch2 3.15.3 provides the test framework and CTest discovery.

Future visualization and application dependencies will be added only after a small cross-platform compatibility prototype.

## Compiler Policy

The project enables C++20 without compiler-specific language extensions. Project targets use strict warnings on Clang, GCC, and MSVC. Warnings are not treated as errors so that compiler upgrades do not unexpectedly block development, but project code should build without warnings.

## Testing and Continuous Integration

Catch2 tests are exposed through CTest. GitHub Actions configures, builds, and tests the debug preset on current hosted macOS, Linux, and Windows runners.

The initial smoke test verifies that an executable can link against `detumble_core` and call its public API. Mathematical and physical validation tests will replace this minimal coverage as later phases are implemented.

## Technical Conventions

These conventions apply before any physics implementation is added:

- Use SI units internally.
- Include units in identifiers when ambiguity would otherwise be likely.
- Use a deterministic seed for every randomized simulation.
- Advance simulation state with a fixed timestep independent of rendering rate.
- Keep truth state inaccessible to flight software, sensors, estimators, and controllers except through explicit simulated measurements.
- Document the reference frame of every physical vector.

Frame directions, quaternion mapping, quaternion multiplication order, and integration conventions will be selected and documented during Phase 1.

## Build Commands

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Replace `debug` with `release` for an optimized build. Build output is contained under `build/`.
