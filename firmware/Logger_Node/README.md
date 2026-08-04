# Logger Node Firmware

This folder contains the STM32 firmware for the Logger Node of the STM32 CAN telemetry system.

The Logger Node receives IMU telemetry frames over CAN, decodes and pairs accelerometer and gyroscope data, timestamps each complete sample using a DS3231 real-time clock, and stores the result on a microSD card in CSV format.

## Hardware Used

- STM32 NUCLEO-F446RE
- SN65HVD230 CAN transceiver
- DS3231 RTC module
- microSD card breakout board
- microSD card formatted as FAT32
- UART over ST-LINK Virtual COM Port for debugging

## Main Functions

The firmware performs the following tasks:

1. Initialises STM32 peripherals using HAL.
2. Configures CAN reception.
3. Receives accelerometer and gyroscope CAN frames.
4. Decodes signed 16-bit IMU values from CAN payloads.
5. Pairs accelerometer and gyroscope frames into complete samples.
6. Reads real-world timestamps from the DS3231 RTC.
7. Mounts the microSD card filesystem using FatFS.
8. Appends complete telemetry rows to `LOG.CSV`.
9. Prints debug information over UART.

## Peripheral Usage

| Peripheral | Purpose |
|---|---|
| CAN1 | Receives IMU telemetry frames |
| I2C1 | Communicates with DS3231 RTC |
| SPI1 | Communicates with microSD card |
| GPIO | microSD chip select and card-detect |
| USART2 | UART debug output through ST-LINK VCP |

## CAN Reception

The Logger Node receives CAN frames from the Vehicle Dynamics Node.

| CAN ID | Message Type | Payload |
|---|---|---|
| 0x120 | Accelerometer frame | Accel X, Accel Y, Accel Z, sequence, status |
| 0x121 | Gyroscope frame | Gyro X, Gyro Y, Gyro Z, sequence, status |

Each received frame contains three signed 16-bit axis values, one sequence byte, and one status/reserved byte.

The Logger Node waits until both an accelerometer frame and a gyroscope frame have been received before treating the data as one complete IMU sample.

## CAN Configuration

| Parameter | Value |
|---|---|
| CAN mode | Normal mode |
| Identifier type | Standard 11-bit |
| Frame type | Data frame |
| Bitrate | 500 kbit/s |
| Transceiver | SN65HVD230 |
| Receive FIFO | FIFO0 |

## RTC Timestamping

The DS3231 RTC is used to provide real-world timestamps for logged samples.

The timestamp format written to the CSV file is:

```text
YYYY-MM-DD HH:MM:SS
```

Example:

```text
2026-07-12 16:30:05
```

The STM32 system tick is also logged as `time_ms`, giving a relative millisecond timestamp from `HAL_GetTick()`.

## microSD and FatFS

The microSD card is accessed over SPI using a custom low-level SD card driver connected to FatFS.

The firmware:

1. Checks whether a microSD card is inserted.
2. Mounts the FAT32 filesystem.
3. Opens or creates `LOG.CSV`.
4. Writes the CSV header if the file is empty.
5. Appends a new telemetry row for each complete IMU sample.
6. Closes the file after writing.

Closing the file after each row was chosen to prioritise reliability during prototype testing.

## CSV Output Format

The final logged file is named:

```text
LOG.CSV
```

CSV header:

```csv
timestamp,time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,status
```

Example row:

```csv
2026-07-12 16:30:05,1234,1532,-212,15890,40,-12,3,OK
```

## Important Source Files

| File | Purpose |
|---|---|
| `Core/Src/main.c` | Main Logger Node firmware logic |
| `Core/Inc/main.h` | GPIO definitions and project header |
| `Core/Src/sd_spi.c` | Custom SPI SD card driver |
| `Core/Inc/sd_spi.h` | SD SPI driver header |
| `FATFS/App/fatfs.c` | FatFS application bridge |
| `FATFS/App/fatfs.h` | FatFS application header |
| `FATFS/Target/user_diskio.c` | FatFS low-level disk I/O interface |
| `FATFS/Target/user_diskio.h` | Disk I/O interface header |
| `.ioc` file | STM32CubeMX project configuration |

## Validation

The Logger Node was tested in stages:

1. DS3231 RTC timestamp reading over I2C.
2. microSD card detection.
3. FAT32 filesystem mounting.
4. Basic file write test.
5. CSV write test using fixed sample values.
6. CAN reception and frame decoding.
7. Final two-node CAN-to-SD integration.

The final test confirmed that the Logger Node could receive live IMU data over CAN, timestamp the samples, and write them successfully to `LOG.CSV`.

## Final Logging Rate

The optimised Logger Node firmware achieved approximately:

| Parameter | Value |
|---|---|
| Complete IMU samples | ~10 samples/s |
| CAN frames received | ~20 frames/s |
| CSV rows written | ~10 rows/s |
| Long-duration test | ~30,000 rows in 50 minutes |

The logging rate improved after replacing per-row `f_open()` / `f_close()` operations with a persistent file handle and periodic `f_sync()`.

## Notes

This firmware is part of a bench-tested prototype. It is not intended as production or vehicle-qualified firmware. Future improvements could include RAM buffering, keeping the CSV file open during acquisition, improved error recovery, watchdog handling, and a more robust PCB-based hardware platform.
