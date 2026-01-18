# Environmental Black Box: Embedded Flight Recorder

> **A pocket-sized environmental telemetry system that continuously samples air quality metrics, compresses data into binary format, and detects environmental anomalies in real-time.**

## 📖 Project Overview [Work currently in progress...]

The **Environmental Black Box** is a resource-constrained data logger powered by the **ESP32-S3**. Unlike standard hobbyist weather stations, this system functions as a "black box" for human environments—recording critical telemetry (temperature, pressure, humidity) with millisecond precision to detect environmental shifts that impact cognitive performance.

This project demonstrates **systems-level engineering**: moving beyond high-level libraries to control hardware at the register level, manage real-time concurrency with **FreeRTOS**, and optimize memory usage on bare-metal architecture.

## ⚙️ Engineering Architecture

The system is decomposed into three distinct subsystems modeling the device's physics, time, and memory.

```text
[ BME280 ] --(I2C)--> [ I2C Driver ] --(Raw Data)--> [ Compensation Engine ]
                                                             |
                                                       (Calibrated Data)
                                                             v
[ WiFi Task ] <--(Queue)-- [ Scribe Task ] <--(Struct)-- [ Observer Task ]
      |                          |
(Telemetry)                (Ring Buffer)
                                 |
                          [ Flash Storage ]
```

### 1. The Physics Driver (Bare-Metal I/O)

**Goal:** Achieve raw data acquisition without external dependencies (no `Adafruit_BME280.h`).

- **Protocol:** Custom I2C driver implementation with direct register manipulation
- **Signal Processing:** Implementation of the Bosch BME280 compensation formula using 32-bit and 64-bit integer math to convert raw ADC values into calibrated physical units (Pascals, Celsius)
- **Validation:** Logic analyzer used to verify I2C start/stop conditions and ACK/NACK responses

### 2. The Time Lord (Real-Time Concurrency)

**Goal:** Deterministic system behavior using FreeRTOS—no "super loops."

**Task Decomposition:**
- **Observer (High Priority):** Hard real-time sensor sampling at 10Hz
- **Scribe (Medium Priority):** Data serialization and queue management
- **Broadcaster (Low Priority):** Wi-Fi telemetry and burst transmission

**IPC (Inter-Process Communication):** Uses `xQueue` for thread-safe data passing and task notifications to minimize CPU cycles.

### 3. The Memory Palace (Storage Optimization)

**Goal:** Efficient non-volatile storage on limited flash memory (8MB).

- **Data Structure:** Circular buffer (ring buffer) implementation to handle continuous logging without stop-the-world garbage collection
- **Serialization:** Binary packing (structs) instead of text formats (JSON/CSV) to reduce write cycles and storage overhead
- **Wear Leveling:** Utilizes ESP32 partition tables to manage storage sectors

## 🧠 Algorithmic Logic: "The Elevator Detective"

The system runs a background anomaly detection algorithm based on barometric pressure deltas.

- **Vertical Velocity Detection:** Distinguishes between weather patterns (slow pressure drop) and mechanical elevation changes (rapid drop >12Pa/sec)
- **Cognitive Density Metrics:** Calculates relative air density and heat index to flag high-humidity/low-pressure environments known to reduce cognitive focus

## 🛠️ Hardware & Tech Stack

- **MCU:** Espressif ESP32-S3 (Xtensa LX7)
- **Sensor:** Bosch BME280 (Temperature, Humidity, Pressure)
- **Framework:** ESP-IDF v5.3 (C/C++)
- **OS:** FreeRTOS
- **Build System:** CMake / Ninja
- **Dev Environment:** Neovim (custom LSP configuration with Clangd integration) and my so loved plugins ;)

## 🚀 Getting Started

### Prerequisites

- ESP-IDF v5.3 toolchain installed
- BME280 connected via I2C (SDA/SCL defined in `config.h`)

### Build Instructions

**Clone the Repository**
```bash
git clone https://github.com/trkBytes/ESP32-S3-N8R8.git
cd ESP32-S3-N8R8
```

**Configure Environment**
```bash
get_idf  # Source your export.sh alias
idf.py set-target esp32s3
```

**Build & Flash**
```bash
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## 📈 Future Roadmap

- [ ] DMA (Direct Memory Access) for sensor readings to offload CPU
- [ ] MQTT integration for cloud telemetry
- [ ] Low power "deep sleep" implementation for multi-day battery life

---

**Created by Tarik Hireche - Winter 2026 - last edit: 17/01/2026**
