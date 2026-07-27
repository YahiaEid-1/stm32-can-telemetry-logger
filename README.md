# STM32 CAN Telemetry Logger

This project is a modular two-node STM32 CAN telemetry system for IMU data acquisition, RTC timestamping, microSD CSV logging, and MATLAB offline orientation replay.

## System Overview

The system consists of:

- Vehicle Dynamics Node: reads MPU-6050 accelerometer and gyroscope data over I2C and transmits it over CAN.
- Logger Node: receives CAN frames, applies DS3231 RTC timestamps, and logs complete samples to LOG.CSV on a microSD card.
- MATLAB analysis: imports the CSV file, converts raw IMU values, plots telemetry data, and replays estimated roll/pitch orientation using a 3D dashboard.

## Key Features

- STM32 NUCLEO-F446RE development boards
- MPU-6050 IMU acquisition
- SN65HVD230 CAN transceivers
- 500 kbit/s CAN communication
- DS3231 RTC timestamping
- microSD/FatFS CSV logging
- MATLAB offline orientation replay
- Approximately 10 logged IMU samples/s

## CAN Message Format

| CAN ID | Message | Payload |
|---|---|---|
| 0x120 | Accelerometer | X, Y, Z, sequence, status |
| 0x121 | Gyroscope | X, Y, Z, sequence, status |

## Report

The full technical report is available in `/report/Technical_Report.pdf`.

## Status

Version 1.0 is a bench-tested prototype. It demonstrates the full telemetry chain from IMU sensing to CAN transmission, RTC timestamping, SD card logging, and MATLAB visualisation.
