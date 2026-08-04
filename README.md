# STM32 CAN Telemetry Logger

A modular two-node STM32 CAN telemetry system for IMU data acquisition, RTC timestamping, microSD CSV logging, and MATLAB offline orientation replay.

This project demonstrates a complete embedded telemetry pipeline from physical motion sensing to CAN communication, local data storage, and offline visualisation.

## Project Overview

The system consists of two STM32-based nodes connected through a physical CAN bus:

- **Vehicle Dynamics Node**  
  Reads MPU-6050 accelerometer and gyroscope data over I2C and transmits the measurements over CAN.

- **Logger Node**  
  Receives CAN frames, decodes and pairs accelerometer/gyroscope data, applies DS3231 RTC timestamps, and logs complete samples to `LOG.CSV` on a microSD card.

- **MATLAB Post-Processing**  
  Imports the logged CSV file, converts raw IMU values into physical units, plots telemetry data, estimates roll/pitch, and replays the estimated board orientation using an offline 3D dashboard.

## Key Features

- STM32 NUCLEO-F446RE development boards
- MPU-6050 6-axis IMU acquisition
- SN65HVD230 CAN transceivers
- 500 kbit/s CAN communication
- Standard 11-bit CAN identifiers
- DS3231 RTC timestamping
- microSD/FatFS CSV logging
- MATLAB telemetry plotting
- Offline 3D orientation replay dashboard
- Approximately 10 logged IMU samples per second

## System Architecture

The project is built around a modular telemetry architecture:

```text
MPU-6050 IMU
    ↓ I2C
Vehicle Dynamics Node
    ↓ CAN frames
SN65HVD230 CAN bus
    ↓ CAN frames
Logger Node
    ↓ RTC timestamp + CSV logging
microSD card / LOG.CSV
    ↓
MATLAB post-processing and 3D replay
```

## CAN Message Format

Each complete IMU sample is transmitted using two CAN frames.

| CAN ID | Message Type | Payload |
|---|---|---|
| `0x120` | Accelerometer frame | Accel X, Accel Y, Accel Z, sequence, status |
| `0x121` | Gyroscope frame | Gyro X, Gyro Y, Gyro Z, sequence, status |

Each frame uses an 8-byte payload:

| Byte | Meaning |
|---|---|
| 0 | X high byte |
| 1 | X low byte |
| 2 | Y high byte |
| 3 | Y low byte |
| 4 | Z high byte |
| 5 | Z low byte |
| 6 | Sequence number |
| 7 | Status/reserved |

Sensor values are transmitted as signed 16-bit integers in high-byte-first order.

## Repository Structure

```text
stm32-can-telemetry-logger/
├── README.md
├── firmware/
│   ├── Vehicle_Dynamics_Node/
│   └── Logger_Node/
├── matlab/
│   ├── Basic_PLot.m
│   ├── Offline_BoardDisplay.m
│   └── LOG.CSV
├── results/
│   ├── sample_LOG.CSV
│   ├── replay_video/
│   ├── matlab_plots/
│   └── uart_outputs/
├── images/
├── report/
└── datasheets/
```

## Firmware

The firmware is split into two STM32CubeIDE projects:

| Folder | Description |
|---|---|
| `firmware/Vehicle_Dynamics_Node/` | MPU-6050 acquisition and CAN transmission firmware. |
| `firmware/Logger_Node/` | CAN reception, RTC timestamping, FatFS, and microSD CSV logging firmware. |

Each firmware folder includes its own `README.md` with node-specific details.

## MATLAB Analysis

The `matlab/` folder contains scripts for analysing the logged telemetry data.

| File | Description |
|---|---|
| `Basic_PLot.m` | Plots accelerometer and gyroscope telemetry from `LOG.CSV`. |
| `Offline_BoardDisplay.m` | Runs the offline 3D orientation replay dashboard. |
| `LOG.CSV` | Sample telemetry file included so the scripts can be run directly. |

## Results

The `results/` folder contains test evidence from the final system, including:

- sample telemetry CSV data
- MATLAB-generated plots
- UART output screenshots
- physical board vs MATLAB replay comparison video

The replay comparison provides qualitative validation of the full telemetry chain:

```text
physical board movement
→ MPU-6050 measurement
→ CAN transmission
→ Logger Node reception
→ DS3231 timestamping
→ microSD CSV logging
→ MATLAB post-processing
→ offline 3D orientation replay
```

## Report

The full technical report is available in:

```text
/report/Technical_Report.pdf
```

The report documents the system architecture, hardware design, firmware implementation, CAN protocol, data logging method, MATLAB processing, testing, limitations, and future work.

## Status

Version 1.0 is a bench-tested prototype.

The system successfully demonstrates the full embedded telemetry pipeline from IMU sensing to CAN transmission, RTC timestamping, SD card logging, and MATLAB visualisation and achieves 4.4 rows written/s.

Version 1.1 introduces faster logging of data.
The logging rate improved after replacing per-row `f_open()` / `f_close()` operations with a persistent file handle and periodic `f_sync()`.

## Limitations

This project was developed as a controlled bench prototype rather than vehicle-qualified hardware.

Current limitations include:

- breadboard and jumper-wire construction
- qualitative orientation replay rather than precision-calibrated attitude estimation
- no absolute yaw estimation due to the lack of a magnetometer
- intentionally limited logging rate for reliable SD card writing
- no real vehicle vibration or long-duration environmental testing

## Future Work

Possible future improvements include:

- PCB or soldered stripboard implementation
- additional CAN-based sensor nodes
- thermal, wheel speed, current, or GPS telemetry
- magnetometer-assisted yaw estimation
- longer-duration testing under vibration and electrical noise
