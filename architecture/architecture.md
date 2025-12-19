# MAD Architecture

## Overview

MAD is a modular C++ pipeline for magnetic anomaly detection. It is organized as small, testable libraries that are composed by the CLI and GUI applications. The design supports multiple filters running at the same time, real-time measurement visualization, and performance metrics without ground truth.

## System Diagram

```
                                  +------------------------+
                                  |     config/*.json      |
                                  |  dataset + model + PF  |
                                  +-----------+------------+
                                              |
                                              v
+-----------------------+      +--------------+--------------+
|     data/raw/*.asc    | ---> |      Data Source Layer      |
|  (real flight data)   |      | AsciiDataSource -> Measurement|
+-----------------------+      +--------------+--------------+
                                              |
                                              v
                                  +-----------+------------+
                                  |     Core / Model       |
                                  |  MadModel + Noise      |
                                  |  dipole | body         |
                                  +-----------+------------+
                                              |
                                              v
+-----------------------+      +--------------+--------------+
|     Filter Factory    | ---> |      Filter Layer           |
| createFilter(config)  |      | EKF/UKF/SIR/SIS/APF/RPF/... |
+-----------------------+      +--------------+--------------+
                                              |
                                              v
                                  +-----------+------------+
                                  |     Pipeline Layer     |
                                  | predict/update/track   |
                                  +-----------+------------+
                                              |
                   +--------------------------+--------------------------+
                   |                                                     |
                   v                                                     v
+------------------------+                                 +------------------------+
|   PerformanceMetrics   |                                 |      Tracking Layer    |
|  residual/NIS/RMSE/ESS |                                 |  ITracker (optional)   |
+-----------+------------+                                 +-----------+------------+
            |                                                            |
            +--------------------------+---------------------------------+
                                       v
                         +-------------+--------------+
                         |       Visualizer GUI       |
                         |  Measurements + Estimates  |
                         |  Magnetic + Metrics panels |
                         +----------------------------+
```

## Execution Flow (GUI)

1. GUI starts and loads `config/default.json`.
2. Dataset path is resolved (absolute or relative to the config file).
3. The data source streams measurements from `data/raw/mad_data.asc`.
4. For each selected filter:
   - `predict(dt)` then `update(measurement)`
   - magnetic prediction is generated from `MadModel`
   - `PerformanceMetrics` updates residual, NIS, RMSE, runtime, ESS
5. Visualizer shows live measurements, filter states, magnetic predictions, and metrics.

## Modules and Responsibilities

- **Data** (`mad/data`)
  - `AsciiDataSource`: parses ASCII rows into `Measurement_t`.
  - Responsible for unit conversion (nT -> Tesla) and ECEF position.

- **Core** (`mad/core`)
  - `MadModel`: process and observation model, supports `dipole` and `body` modes.
  - `IFilter`: common filter interface.
  - `ParticleFilterBase`: resampling/proposal logic and ESS computation.
  - `FilterFactory`: builds filters from JSON config.
  - `PerformanceMetrics`: accumulates residual-based metrics without ground truth.

- **Pipeline** (`mad/pipeline`)
  - Orchestrates data -> model -> filter -> tracking.
  - Keeps the integration logic separated from UI.

- **Tracking** (`mad/tracking`)
  - `ITracker`: optional layer to manage tracks if multi-target logic is added later.

- **Visualization** (`mad/viz`)
  - ImGui UI showing measurements, estimates, magnetic info, and metrics.
  - Supports multiple filters simultaneously.

## Key Design Decisions

- **Config-driven**: all runtime settings live in JSON (`config/default.json`).
- **Filter extensibility**: implement `IFilter` or derive from `ParticleFilterBase`.
- **Model flexibility**: observation model can switch between `dipole` and `body`.
- **No truth required**: performance is based on residual statistics and runtime.
- **GUI-first**: visualizer starts first and drives the update loop.

## Extending the System

- Add new datasets: implement `IDataSource` and update the config path.
- Add new filters: register in `FilterFactory` and GUI list.
- Add new metrics: extend `PerformanceMetrics` and `Visualizer` tables.
- Add tracking logic: implement `ITracker` and plug into the pipeline.
