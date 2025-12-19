# MAD (Magnetic Anomaly Detection)

Mad is a cross-platform C++ framework that reimplements the MATLAB MAD prototype with the same 10D ship state, magnetic dipole model, and real Bahamian flight data. The focus is on modular pipelines, multiple filters running side-by-side, and an ImGui visualizer that keeps the measurement flow, tracker panel, and performance metrics visible at all times.

## Repository Layout

- `apps/` – entry points: `mad_gui` for the ImGui visualizer and `mad_cli` for headless experiments.
- `bindings/` – ImGui/GLFW/OpenGL glue so the GUI remains platform agnostic.
- `config/` – JSON defaults plus `imgui.ini` for saving UI layouts.
- `data/raw/` – `mad_data.asc` is the canonical real dataset (ASCII format, 10 columns, nT → Tesla + ENU conversion).
- `include/`, `src/` – core libraries (`mad::MadModel`, `mad::IFilter`, `mad::ParticleFilterBase`, `mad::Visualizer`, etc.).
- `tests/` – GTest-based suites covering filters, models, data loading, and performance metrics.
- `sound/` – alert beep played when predictions drift from the Earth field baseline.
- `journal/` – scenario notes (10-state meanings, Earth field, filtering strategy).
- `cmake/` – reusable macros.
- `conanfile.py` & `CMakeLists.txt` wire Conan + CMake to every supported compiler.

## Dependencies & Build

1. Install Conan 2.x and a C++20 toolchain (MSVC, clang, or gcc).
2. From the repo root run:

   ```bat
   MAD\BuildDebug.bat
   ```

   The script:
   - Runs `conan install` with the list of dependencies (GLFW, GLEW, OpenGL/Glu, GLM, ImGui, Eigen, fmt, spdlog, nlohmann::json, glad, GoogleTest, etc.).
   - Configures CMake (`CMAKE_CXX_STANDARD=20`).
   - Builds `mad_gui`.
   - Launches the GUI once the build succeeds.

3. For a release build replace the script with `MAD\BuildRelease.bat`.
4. The GUI layout persists in `config/imgui.ini`; delete it to reset the arrangement.

## Data & Scenario

- **Dataset**: `data/raw/mad_data.asc`, renamed from the legacy `Feb19_2008_time_TF_ACnav_shipnav` file. Every row follows MATLAB's 10-column layout (time, TFC, aircraft LLA+heading, ship LLA+heading).
- **Measurement loader**: `mad::AsciiDataSource` mirrors `MEas_Retrieval` by converting positions into a local ENU frame, deriving per-scan truth states, and seeding velocities from the first two rows.
- **State vector (10D)**: `[x, vx, y, vy, ML, MT, MV, CLW, CTW, CVW]`, the same as the MATLAB filters. Models rely on this ordering for Jacobians and particle propagation.
- **Earth field**: `[25000, -3400, 36000] × 1e‑9 T` (Bahamas). The GUI uses this constant as the `truth` baseline, and any deviation beyond `viz.alert.thresholdTesla` triggers the alert beep.
- **Truth data**: The loader stores ship truth positions/moments and exposes them to the tracker panel and the `PerformanceMetrics` (NEES/RMSE) computations.
- **Journal**: `journal/` contains supporting notes should you need the math background for the MAD scenario.

## Filters & Model Mapping

| MATLAB Script | C++ Equivalent | Notes |
| --- | --- | --- |
| `EKF_MAD_real_case` | `mad::EKF` | Standard EKF with the 10D constant-velocity + magnetic dipole model. |
| `UKF_MAD_real_case` | `mad::UKF` | Unscented propagation via MadModel. |
| `RegularPF`, `RegularizedPF`, `AuxiliaryPF` | derived from `mad::ParticleFilterBase` (`SIRParticleFilter`, `AuxiliaryParticleFilter`, `RegularizedParticleFilter`) | Particle filters share the same base class and proposal/resampling hooks. |
| `PF_EKF_MAD_real_case`, `PF_UKF_MAD_real_case` | `mad::EkfParticleFilter`, `mad::UkfParticleFilter` | EKF/UKF used inside the proposal. |
| `mad::ParticleFilterBase` derivatives | `sis`, `adaptive`, `robust`, `gmf` | Strategy pattern lets you swap proposals, resampling policies, and likelihoods without touching the pipeline. |

- **MadModel** (`mad::MadModel`): encapsulates process noise, dipole/body observation models, Jacobians, and measurement scale handling. The model is shared between `mad::IFilter` objects (Strategy pattern) and the GUI.
- **Complexity note**: EKF/UKF cost `O(stateDim^3)` per step due to matrix operations, while particle filters run in `O(numParticles × stateDim)` with additional resampling overhead. The data loader and visualizer process `O(#scans)` per run to maintain responsiveness.

## Visualizer & Controls

Run from `MAD\build_debug`:
```
conanrun.bat
Debug\mad_gui.exe
```

Controls:
- Run/Pause/Step/Reset
- Filter toggles; map/tracker dropdowns select which filter feeds each panel
- Max Scans slider + Unlimited toggle (binds `dataset.maxScans`)
- Performance panel toggles (RMSE/NEES/Residual graphs)
- Logging enable + level; logs go to `logs/mad.log` and per-class logs under `logs/classes`
- Tracker Zoom slider to zoom the tracker panel (X/Y in meters)
- Alert settings (threshold/cooldown/scope) driven by `viz.alert`

Panels:
- Measurement summary; state table (10D); magnetic table (measured/predicted/ESS)
- Magnetic maps: measured, predicted, residual, uniform truth baseline (`|B|`)
- Trajectories 2D/3D
- Tracker: truth vs estimate (2D) with zoom; axis labels in meters
- Performance Metrics window: RMSE/NEES/Residual histories per filter

<img width="1919" height="1029" alt="image" src="https://github.com/user-attachments/assets/1e5672d3-1680-463b-a70e-9d147216daf9" />

## Configuration

`config/default.json` is the single source of truth. Key sections:

```json
{
  "dataset": {
    "path": "data/raw/mad_data.asc",
    "format": "ascii",
    "maxScans": 500
  },
  "model": {
    "earthField": [2.5e-5, -3.4e-6, 3.6e-5],
    "processNoiseVar": 0.05,
    "measurementNoiseVar": 1.0e-18,
    "measurementScale": 0.0,
    "observationModel": "dipole"
  },
  "filter": {
    "type": "ekf",
    "params": {
      "numParticles": 500,
      "stateDimension": 10,
      "resamplePolicy": "adaptive",
      "essThresholdRatio": 0.5
    }
  },
  "logging": {
    "enabled": true,
    "level": "debug"
  },
  "viz": {
    "alert": {
      "enabled": true,
      "thresholdTesla": 5e-9,
      "cooldownMs": 800,
      "soundPath": "sound/button-11.wav",
      "scope": "map"
    }
  }
}
```

- `filter.params` controls the particle filters (resample policy, ESS threshold, mixture components, etc.).
- `model.measurementScale` = 1.0 by default. Set to 0 to auto-scale from the first truth sample (same logic the MATLAB bootstrap used).
- `viz.alert` governs the audible warning when the magnetic anomaly leaves the constant Earth field `truth` area.
- `config/imgui.ini` stores ImGui window positions; it is now tracked so team members share the same layout. Remove it to reset.

## Logging & Alerts

- Logger: `mad::Logger` wraps `spdlog`. Use the GUI toggle or `logging.enabled`/`logging.level` in the JSON to mute or filter messages. The GUI emits detailed debug events once per 50 steps for any filter with truth data so you can diagnose drift.
- Alerts: `appState.alertBaselineTesla` is the Earth field norm (`|B| ≈ 3.6e-5`). `main` checks each selected filter's predicted measurement and plays `sound/button-11.wav` whenever the anomaly exceeds the configured threshold.

## Testing

- Run unit tests after building:

  ```
  cd MAD/build_debug
  ctest --output-on-failure -C Debug
  ```

- Coverage (MSVC + OpenCppCoverage):

  ```
  cmake -S . -B build_debug -DMAD_ENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug
  cmake --build build_debug --config Debug
  cmake --build build_debug --config Debug --target coverage
  ```

- Tests rely on `MAD_REAL_DATA_PATH` (set to `data/raw/mad_data.asc`) so they exercise the real dataset. Additional deterministic fixtures such as small subsets may live under `tests/data/` if you add them later.
- Unit suites validate the 10-state output for every filter, `MadModel` Jacobians, data parsing, and performance metrics to keep parity with the MATLAB reference.

## Architecture & Next Steps

- See `architecture/architecture.md` for the full block diagram, responsibilities, and how the config → data → core → pipeline → viz layers interact.
- The visualizer drives the pipeline: `stepOnce` pulls measurements, updates each enabled filter, updates `PerformanceMetrics`, and feeds the map/tracker panels while respecting the `maxScans` slider.
- The tracker and performance panels are independent ImGui windows so you can move them around and the new `config/imgui.ini` keeps the layout between runs.
- The `bindings/` folder ensures ImGui runs on GLFW + OpenGL; Conan delivers the same deps across compilers.
- You can extend `mad::FilterFactory` with new filters or trackers without touching the GUI; a `mad::IFilter` implementation only needs to supply `predict`/`update`.
- Logger, measurement scaling, ESS-based resampling, and the alert system are designed to stay modular so the C++ code can consume the same real data that the MATLAB scripts used.

## Tips

- Start the GUI first; it automatically populates the tracker and performance windows with the `mad_data.asc` flight.
- Use the filter checkboxes and map/tracker dropdowns to compare `ekf`, `ukf`, and the new SIR/SIS/APF/RPF variants in parallel.
- Toggle the performance panel to see RMSE, NEES, and residual histories for every active filter.
- The tracker panel plots truth vs estimate in 2D. The truth track is derived from the dataset (ship positions converted to ENU), so discrepancies highlight modeling drift.
- Keep an eye on the alert beep (sound path in `sound/button-11.wav`) whenever predicted magnetic values leave the constant Earth field area; it’s the same “beep on anomaly” behavior described in the MATLAB version.
