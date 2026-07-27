# Vehicle Dynamics Node Firmware

This folder contains the STM32 firmware for the Vehicle Dynamics Node of the STM32 CAN telemetry system.

The Vehicle Dynamics Node is responsible for acquiring inertial measurement data from an MPU-6050 IMU and transmitting the measurements over CAN to the Logger Node.

## Hardware Used

- STM32 NUCLEO-F446RE
- MPU-6050 accelerometer and gyroscope module
- SN65HVD230 CAN transceiver
- CAN bus connection to Logger Node
- UART over ST-LINK Virtual COM Port for debugging

## Main Functions

The firmware performs the following tasks:

1. Initialises STM32 peripherals using HAL.
2. Configures the MPU-6050 over I2C.
3. Uses the MPU-6050 data-ready interrupt to detect new samples.
4. Reads raw accelerometer and gyroscope data.
5. Packs the IMU readings into CAN frames.
6. Transmits accelerometer and gyroscope frames over CAN.
7. Prints debug information over UART.

## Peripheral Usage

| Peripheral | Purpose |
|---|---|
| I2C1 | Communication with MPU-6050 |
| CAN1 | Transmission of IMU telemetry frames |
| GPIO/EXTI | MPU-6050 data-ready interrupt |
| USART2 | UART debug output through ST-LINK VCP |

## MPU-6050 Configuration

The MPU-6050 is configured for normal telemetry operation using register-level I2C writes.

Final configuration:

| Setting | Value |
|---|---|
| Accelerometer range | ±8 g |
| Gyroscope range | ±250 °/s |
| Sample rate divider | 4 |
| Approximate IMU sample rate | 200 Hz |
| Digital low-pass filter | Enabled |
| Data-ready interrupt | Enabled |

The final firmware transmits one complete IMU sample every 20 data-ready events. This gives an effective transmitted/logged rate of approximately 10 complete IMU samples per second.

## CAN Message Format

The Vehicle Dynamics Node transmits one complete IMU sample using two CAN frames.

| CAN ID | Message Type | Payload |
|---|---|---|
| 0x120 | Accelerometer frame | Accel X, Accel Y, Accel Z, sequence, status |
| 0x121 | Gyroscope frame | Gyro X, Gyro Y, Gyro Z, sequence, status |

Each frame uses an 8-byte payload.

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

All sensor axis values are transmitted as signed 16-bit integers in high-byte-first order.

## CAN Configuration

| Parameter | Value |
|---|---|
| CAN mode | Normal mode |
| Identifier type | Standard 11-bit |
| Frame type | Data frame |
| Bitrate | 500 kbit/s |
| Transceiver | SN65HVD230 |

## Important Source Files

| File | Purpose |
|---|---|
| `Core/Src/main.c` | Main firmware logic |
| `Core/Inc/main.h` | GPIO definitions and project header |
| `.ioc` file | STM32CubeMX project configuration |

## Validation

The Vehicle Dynamics Node was tested in stages:

1. MPU-6050 I2C communication test.
2. MPU-6050 self-test.
3. Normal accelerometer and gyroscope UART output.
4. CAN loopback transmission and reception.
5. Final physical CAN transmission to the Logger Node.

The final integration test confirmed that live IMU readings were transmitted over CAN and received by the Logger Node for timestamped SD card logging.

## Notes

This firmware is part of a bench-tested prototype. It is not intended as production or vehicle-qualified firmware. A more robust implementation would require stronger error handling, watchdog behaviour, better buffering, and hardware designed for vibration and electrical noise.
