# Project CARAC

**Adaptive Automation Platform for Aerodynamic Drone Characterization**

A comprehensive automation platform for conducting aerodynamic experiments on morphing drones under controlled windy conditions. This project was developed as part of a Master's thesis at EPFL by Marcus Cemes.

## Overview

Project CARAC integrates multiple hardware systems into a fully automated experimental environment. Built on a client-server architecture, it orchestrates complex experiments involving robotic manipulation, precision force measurement, controlled wind generation, and high-precision motion tracking.

The **Server** (`carac.exe`), written in Rust, provides the core orchestration and data acquisition engine. The **Client** is a suite of Python packages for experiment design, execution, data processing, and analysis.

## Key Features

- **Client-Server Architecture**: Robust Rust-based server for hardware control and a flexible Python client for experiment scripting and analysis.
- **Multi-Hardware Integration**: Seamlessly connects and controls diverse hardware systems (Robot Arm, Load Cell, Wind Tunnel, Motion Capture).
- **Experiment Orchestration**: Automated experiment execution with precise timing and coordination via a simple HTTP API.
- **High-Performance Data Collection**: Real-time data streaming and buffering from multiple sources into a custom, high-efficiency binary format.
- **Comprehensive Python Toolkit**:
  - **SDK**: Easy-to-use `async` Python interface for experiment design.
  - **Pipelines**: Tools for post-processing, applying analytical models, and force correction.
  - **Analysis**: Modules for report generation and machine learning.
- **CLI Tools**: Dedicated utilities for serving, data export (CSV/Parquet), file inspection, and payload measurement.
- **Real-time Visualization**: PlotJuggler integration for live data monitoring.

## Architecture

Project CARAC is split into two main components: a Rust server and a Python client.

### Server (Rust Backend)

The server is the central hub that runs on a dedicated machine connected to the hardware. It consists of:

- **Orchestration Engine**: Manages hardware, executes instruction sequences, and handles experiment state.
- **DataSink System**: Collects timestamped data into efficient in-memory buffers.
- **Hardware Contexts**: Abstracted, asynchronous interfaces for each piece of hardware.
- **HTTP & UDP APIs**: Exposes control endpoints for the Python client and generic devices.

### Client (Python Frontend)

The client is a collection of Python packages used to interact with the server and process the resulting data.

- **`carac`**: An SDK providing an `Orchestrator` client to communicate with the server's HTTP API. It includes definitions, helpers, and constants for scripting experiments.
- **`experiments`**: A set of runnable Python modules that define specific experimental procedures (e.g., `python -m experiments.coupled_axis`).
- **`pipeline`**: Scripts for post-processing recorded data, such as applying force corrections or analytical models.
- **`report`**: Code for generating plots, figures, and summaries from processed data.
- **`training`**: Tools for applying machine learning models to the collected datasets.

## Hardware Support

### Primary Hardware Components

- **Stäubli TX2-90 Robot Arm**: 6-DOF industrial robot for precise drone positioning.
- **ATI NANO25-E Load Cell**: High-precision force/torque sensor for aerodynamic measurements.
- **WindShape Wind Generator**: Controlled wind generation system.
- **OptiTrack Motion Capture**: Sub-millimetre precision tracking for position and orientation.

### Additional Hardware Support

The platform supports custom hardware devices through a generic UDP-based protocol for bidirectional command and state communication.

## Getting Started

### Prerequisites

- Rust (latest stable version)
- Python 3.10+
- Hardware systems configured and connected to the network.

### Server Installation (Rust)

The server provides two binaries: `carac.exe` for orchestration and `carac-kit.exe` for utilities.

```bash
# Clone the repository
git clone <repository-url>
cd project-carac/server

# Build the binaries
cargo build --release
```

### Client Installation (Python)

The client is structured as a Python project with multiple packages.

```bash
cd ../client

# Install the project in editable mode with all dependencies (managed by pyproject.toml)
pip install -e .
```

### Configuration

Create a `config.yaml` file to specify your hardware setup. The server will load this file on startup.

```yaml
hardware:
  robot_arm:
    ip: "192.168.1.100"
    port: 20000

load_cell:
    ip: "192.168.1.101"
    configure_device: true

motion_capture:
    ip: "192.168.1.102"
    rigid_bodies: ["drone"]

  wind_shape:
    ip: "192.168.1.103"

  additional_devices:
    - name: "drone"
      ip: "192.168.1.200"
      port: 21000
      channels: ["throttle", "sweep_l", "sweep_r", "servo_1", "servo_2"]

sink:
  session_path: "./data"
```

## Running an Experiment (Example Workflow)

1.  **Start the orchestration server:**

    ```bash
    # From the server directory, with optional config path (default is working directory)
    .\\target\\release\\carac.exe --config path/to/config.yaml
    ```

2.  **Execute an experiment script from the client:**

    ```bash
    # From the client directory
    python -m experiments.coupled_axis
    ```

3.  **Process and analyze the data:**
    Data is saved in the `session_path` defined in your config. Use `carac-kit.exe` to export it.

    ```bash
    # From the /server directory
    .\\target\\release\\carac-kit.exe export --format parquet .\\data

    # View available options
    .\\target\\release\\carac-kit.exe export --help
    ```

## Command-Line Tools

### `carac.exe` (Server & Measurement)

- `carac (server) --config <path>`: (Default) Starts the orchestration server.
- `carac measure --config <path>`: Initiates a guided procedure to measure the mass and center of mass of the attached payload.

### `carac-kit.exe` (Data Utilities)

- `carac-kit export --format <csv|parquet> <session>`: Exports a session to CSV/Parquet
- `carac-kit view <experiment>`: Inspects the header and stream information of a binary recording file.
- `carac-kit plot <session>`: Generates stream/channel plots for an entire session

## Data Format & Session Structure

Data is stored in a session-based structure. When a new experiment is started, a new session folder is created.

- **Session Directory**: Contains all recordings for an experiment.
- **`meta.json`**: A top-level file in the session directory that defines all data streams and their channels (e.g., `load_cell/Fx`, `robot/joint_1`).
- **Binary Files**: Each recording is saved to a custom binary format optimized for high-frequency time-series data with microsecond precision.

## API Reference

### HTTP Endpoints (Server)

The server exposes a simple HTTP API for the client.

- `POST /new-experiment`: Creates a new experiment session.
- `POST /save-experiment`: Closes and saves the current session.
- `POST /execute`: Executes a sequence of instructions without recording data.
- `POST /record`: Executes a sequence of instructions while recording data.
- `POST /start-recording`: Manually starts data recording.
- `POST /stop-recording`: Manually stops data recording and saves the file.
- `GET /progress`: Get the robot trajectory progress (move ID)
- `GET /status`: Returns the system health status.

### Python SDK (`carac` package)

The `Orchestrator` client provides a high-level async interface for writing experiments.

```python
# Example from an experiment script
from carac.prelude import *

async def main():
    # Connect to the server
    async with Orchestrator() as o:
        # 1. Initialize hardware and move to a starting point
        await init(o)

        # 2. Define and run the main experimental procedure
        await o.new_experiment("my_awesome_experiment")

        # The entire batch of instructions are executed in sequence and recorded
        # with precise timing on the orchestrator server
        await o.record([
            Robot(Move(MotionLinear(target_pose))),
            Sleep(2.0),
            Robot(Move(MotionLinear(start_pose))),
        ])

        await o.save_experiment()

        # 3. Return hardware to a safe state
        await finalise(o)
```

## Project Structure

```
project-carac/
├── client/
│   ├── carac/          # Core Python SDK (Orchestrator client, helpers)
│   ├── experiments/    # Runnable experiment scripts
│   ├── pipeline/       # Data processing and correction scripts
│   ├── report/         # Plotting and report generation
│   ├── training/       # Machine learning models
│   └── pyproject.toml  # Project definition and dependencies
│
└── server/
    ├── src/
    │   ├── bin/        # Entrypoints for `carac` and `carac-kit`
    │   ├── config.rs   # Configuration structures
    │   ├── data/       # Data handling, formats, DataSink, Orchestrator logic
    │   └── hardware/   # Hardware abstraction layer
    └── Cargo.toml      # Rust project definition
```
