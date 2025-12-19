# MAD Architecture

The MAD architecture keeps the data ingestion, filtering, tracking, and visualization layers separated so each piece can evolve without breaking the others. Configuration, logging, and scripting glue are intentionally kept near the edges so the core C++ library stays independent of imposition details (OS, GUI, tests). Here is the high-level flow:

```
 Config (.json)         Real Data (mad_data.asc)
 +---------+            +------------------+
 |  dataset| ---------->|  AsciiDataSource |--> Measurement_t (ENU + truth)
 |  model  |            +------------------+
 |  filter |                         |
 |  viz    |                         v
 +---------+        +----------+    +----------------------------+
                     |  MadModel|<--|  Filter Layer              |
                     +----------+    |  mad::IFilter (EKF/UKF)    |
                         ^           |  ParticleFilterBase + PFs   |
                         |           |  (SIR, SIS, APF, RPF, etc.) |
              +----------+-----------+----------------------------+
              |                      |  PerformanceMetrics & Logger |
              v                      +----------------------------+
 Pipeline (apps/mad_gui/main loop)          |           |
 updates filters -> metrics -> viz state    |           |
                                            v           v
                                    Visualizer (ImGui)  Tracker Panel
                                    + maps, state tables,  + truth vs estimate
                                    + magnetic/residual    + 2D/3D trajectories
                                    + Performance window   + alerts/beeps
```

## Modules & Responsibilities

- **Configuration (`config/default.json`, `config/imgui.ini`)**
  - Defines the dataset path, Earth field, noise parameters, filter selection, logging level, and visualization preferences.
  - `imgui.ini` keeps window layout between runs (the GUI sets `ImGuiIO::IniFilename` before rendering and calls `ImGui::SaveIniSettingsToDisk()` on exit).
  - Any change in the JSON propagates through `AppState_t`, so the pipeline automatically respects new filters, slider limits, and logging flags.

- **Data Extraction (`mad::AsciiDataSource`)**
  - Mirrors the MATLAB `MEas_Retrieval` function: converts the `.asc` file into ENU sensor/ship positions, magnetic values (converted from nT to Tesla), and our 10-state truth vector.
  - Samples a fixed `T = 1/32 s` period, stores velocities derived from the first two rows, and returns structured `Measurement_t` objects.
  - Complexity: O(scanCount); each scan does a few vector conversions and writes to history buffers.

- **Model (`mad::MadModel`)**
  - Encapsulates the process model (constant velocity on `x/y`, static magnetic moments) and two observation modes (`dipole` and `body`).
  - Offers Jacobians for filters/tests, propagates noise (`processNoiseVar`), and scales measurements via `measurementScale`.
  - Complexity per call: O(stateDim) for propagation, O(stateDim) for measurement, plus O(stateDim^2) for Jacobians.

- **Filter Layer**
  - `mad::IFilter` is the Strategy interface consumed by the pipeline/GUI.
  - `mad::ParticleFilterBase` is a template-resampling facility that concrete filters extend (SIR, SIS, APF, RPF, Adaptive, Robust, Gaussian Mixture, EKF-PF, UKF-PF).
  - `mad::EKF` and `mad::UKF` implement classical Kalman updates.
  - Each filter outputs a 10D state estimate; tests verify this and the measurement Jacobians.
  - Complexity: EKF/UKF are O(stateDim^3) per update; particle filters are O(numParticles × stateDim) plus resampling (O(numParticles)).

- **Pipeline & Tracking (`apps/mad_gui/Main.cpp`, `mad::Pipeline`)**
  - `stepOnce` (GUI) drives the loop: fetch measurement, advance each filter, compute metrics (residual, NEES, RMSE, runtime), feed the visualizer, and sound alerts according to `appState.alert` rules.
  - The same pipeline logic could be used in CLI mode (`mad_cli`) for unattended experiments; the GUI simply wraps this loop with ImGui frames.
  - `PerformanceMetrics` accumulates residual sums, truth RMSE, and NEES even when the ground truth is used only for diagnostics.

- **Visualization (`mad::Visualizer`)**
  - ImGui windows: Measurement summary, state/magnetic tables, magnetic/residual maps, trajectories (2D/3D), tracker window, performance panel.
  - Performance window plots RMSE, NEES, and residual histories stored per filter (`metricHistories`), allowing multi-filter comparison.
  - Tracker panel shows truth vs estimate in a dedicated view; truth is derived from the dataset, so any divergence becomes immediately visible.
  - The magnetic map also renders `Bn/Be/Bv` values and a uniform truth baseline (`earthField.norm()`).

- **Logging & Alerts**
  - `mad::Logger` wraps `spdlog` and behaves like a singleton: enable/disable via JSON or GUI, set levels (`trace`, `debug`, `info`, etc.).
  - Logs emit debug statements every 50 steps for synced performance checks; faults/alerts log warnings.
  - Alerts compare predicted magnetic values against the Earth field baseline and play `sound/button-11.wav` if the anomaly threshold is exceeded.

- **Tests (`tests/unit`, `tests/integration`)**
  - GoogleTest suites validate data loading, `MadModel` behavior, filter outputs, and pipeline interactions.
  - The `MAD_REAL_DATA_PATH` macro points to `data/raw/mad_data.asc`, so the tests exercise the real dataset.
  - Coverage targets ~80% via the provided scripts (`ctest`, `coverage` target).

- **Bindings & External Dependencies**
  - `bindings/` contains `imgui_impl_glfw.cpp` and `imgui_impl_opengl3.cpp`, keeping the ImGui renderer/platform layers inside the repo.
  - `conanfile.py` defines the third-party stack (GLFW, GLEW, GLM, ImGui, Eigen, fmt, spdlog, nlohmann_json, glad, glu/opengl, GoogleTest).
  - CMake simply links the `mad_core`, `mad_data`, `mad_pipeline`, and `mad_viz` libraries together.

## Design Patterns & Extensibility

- **Strategy Pattern**: `mad::IFilter` allows runtime selection of EKF, UKF, or any particle filter; `FilterFactory` instantiates filters via JSON.
- **Template/Hook Pattern**: `ParticleFilterBase` defines `predict`/`update`, and derived classes override `propose`, `afterResample`, and `logLikelihood` without reimplementing resampling/normalization.
- **Observer/Sink**: `PerformanceMetrics` receives residuals and truth updates; `Visualizer` consumes the metrics for real-time graphs.
- **Singleton**: `mad::Logger` holds a single `spdlog` logger instance configurable from GUI or config files.
- **Adapter**: `AsciiDataSource` adapts the raw MATLAB-style `.asc` file to `Measurement_t` objects used everywhere else.

## Complexity Summary

- **Data path**: linear in the number of scans. Each measurement writes to history buffers and updates the sensor/target tracks (memory ~O(maxHistory)).
- **Filtering**: 10D vector ensures state propagation is cheap. Particle filters scale with particle count (default 500) while EKF/UKF cost matrix operations; choose the right filter depending on the trade-off between accuracy and time.
- **Visualization**: `Visualizer` keeps only ~300 history points per filter per plot, preventing unbounded memory growth while keeping the metrics window responsive.

## Next Steps

- Add new datasets or processed versions under `data/processed` and hook them into `config/default.json`.
- Extend `FilterFactory` if new MAT-derived filters are required (e.g., GLMB-PF or PHD-PF).
- Plug in a tracker implementation if you want to manage track lifetimes beyond the single-target pipeline (`mad::tracking` namespace is ready for it).
- Improve alerting (multiple sound scopes, visual cues) by extending `viz.alert`.
