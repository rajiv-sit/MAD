# MAD (Magnetic Anomaly Detection)

MAD is a C++ framework for magnetic anomaly detection with a modular pipeline, multiple filter backends, and a real-time ImGui visualizer. It is designed to be OS/compiler independent, uses Conan for third-party libraries, and builds with CMake.

## Goals

- Run real data through multiple filters in parallel
- Visualize measurements, estimates, and magnetic residual metrics in real time
- Keep the core model and filtering logic decoupled from the UI
- Make it easy to add filters, metrics, and data sources

## Features

- EKF/UKF and multiple particle filter variants
- Config-driven model and filter settings (JSON)
- Two observation models: `dipole` and `body`
- Live metrics without ground truth (residual, NIS, RMSE, runtime, ESS)
- Multi-filter GUI selection for side-by-side comparison

## Repo Layout

- `MAD/apps/` CLI and GUI entrypoints
- `MAD/include/` public headers (`.hpp`)
- `MAD/src/` core implementation
- `MAD/config/` JSON config files
- `MAD/data/` datasets
- `MAD/bindings/` ImGui backend sources (GLFW/OpenGL)
- `MAD/tests/` unit tests

## Requirements

- CMake 3.20+
- Conan 2.x (Conan 1.x fallback in `conanfile.py`)
- C++20 compiler (MSVC, clang, gcc)
- OpenGL + GLFW (via Conan)

## Data

Place real data at:

- `MAD/data/raw/mad_data.asc`

The ASCII loader expects 10 columns:

1. time
2. magnetic (TFC)
3. aircraft lat
4. aircraft lon
5. aircraft height (ft)
6. aircraft heading (deg)
7. ship lat
8. ship lon
9. ship height (ft)
10. ship heading (deg)

The loader converts magnetic values from nT to Tesla and aircraft position to ECEF.

## Build (Windows)

Debug build and run:

```
BuildDebug.bat
```

Release build and run:

```
BuildRelease.bat
```

The batch scripts:
- run `conan install`
- generate the toolchain
- build the project
- launch `mad_gui.exe`

## Run (GUI)

The GUI is the primary entrypoint. It starts immediately, lets you select filters, and streams data in real time.

From the build folder:

```
cd MAD\build_debug
conanrun.bat
Debug\mad_gui.exe
```

## Run (CLI)

The CLI runs a single filter with the selected model:

```
mad_cli --config config/default.json --dataset data/raw/mad_data.asc
mad_cli --filter sir --obs-model dipole
mad_cli --filter ukf --obs-model body
```

## Filters

Supported filter types:

- `ekf`, `ukf`
- `sir`, `sis`, `apf`, `rpf`, `adaptive`, `robust`, `gmf`
- `ekf-pf`, `ukf-pf`

## Configuration

Main config: `MAD/config/default.json`

Key fields:

- `dataset.path`: input dataset
- `model.earthField`: Earth field vector (Tesla)
- `model.processNoiseVar`: process noise variance
- `model.measurementNoiseVar`: measurement noise variance
- `model.observationModel`: `dipole` or `body`
- `filter.type`: filter selection
- `filter.params`: particle filter settings

Example:

```json
{
  "dataset": { "path": "data/raw/mad_data.asc" },
  "model": {
    "earthField": [2.5e-5, -3.4e-6, 3.6e-5],
    "processNoiseVar": 0.05,
    "measurementNoiseVar": 1.0e-18,
    "observationModel": "dipole"
  },
  "filter": {
    "type": "ukf",
    "params": {
      "numParticles": 1000,
      "resamplePolicy": "adaptive",
      "essThresholdRatio": 0.5
    }
  }
}
```

## Visualizer Panels

- **Measurement**: raw magnetic field and sensor position
- **Estimates**: per-filter state values and residual statistics
- **Magnetic**: measured vs predicted magnetic values and ESS
- **Metrics**: NIS, RMSE, mean absolute residual, runtime

You can enable multiple filters at once for direct comparison.

## Tests

Run tests from the build folder:

```
ctest --test-dir build_debug -C Debug
```

Coverage (MSVC + OpenCppCoverage):

```
cmake -S . -B build_debug -DMAD_ENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build_debug --config Debug
cmake --build build_debug --config Debug --target coverage
```

Tests use the real dataset path in `MAD/data/raw/mad_data.asc`.

## Troubleshooting

- **GUI opens but no updates**: ensure `MAD/data/raw/mad_data.asc` exists and the config path resolves correctly from the build folder.
- **Missing DLLs**: run `conanrun.bat` before launching `mad_gui.exe`.
- **GLFW/GLAD errors**: verify your GPU drivers and OpenGL support.

## Architecture

Detailed architecture and data flow: `MAD/architecture/architecture.md`.

## License

MIT (placeholder). Update as needed.
