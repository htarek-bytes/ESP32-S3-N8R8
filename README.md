# The Environmental Black Box: Embedded Flight Recorder

> **A pocket-sized life telemetry system. It continuously samples the fluid we live in (Air), compresses reality into binary, and detects environmental anomalies in real-time.**

## 📖 Project Overview
The **Environmental Black Box** is a resource-constrained data logger powered by the **ESP32-S3**. Unlike standard hobbyist weather stations, this system functions as a "Black Box" for human environments—recording critical telemetry (Temperature, Pressure, Humidity) with millisecond precision to detect environmental shifts that impact cognitive performance.

This project was built to demonstrate **systems-level engineering**: moving beyond high-level libraries to control hardware at the register level, manage real-time concurrency with **FreeRTOS**, and optimize memory usage on bare-metal architecture.

## ⚙️ Engineering Architecture

The system is decomposed into three distinct subsystems, designed to model the "Physics, Time, and Memory" of the device.

### 1. The Physics Driver (Bare-Metal I/O)
**Goal:** Achieve raw data acquisition without external dependencies (No `Adafruit_BME280.h`).
* **Protocol:** Custom I2C driver implementation (Direct register manipulation).
* **Signal Processing:** Implementation of the **Bosch BME280 Compensation Formula** (32-bit and 64-bit integer math) to convert raw ADC garbage values into calibrated physical units (Pascals, Celsius).
* **Validation:** Logic Analyzer used to verify I2C start/stop conditions and ACK/NACK responses.

### 2. The "Time Lord" (Real-Time Concurrency)
**Goal:** Deterministic system behavior using **FreeRTOS**. No "Super Loops."
* **Task Decomposition:**
    * **Observer (High Priority):** Hard real-time sensor sampling (10Hz).
    * **Scribe (Medium Priority):** Data serialization and queue management.
    * **Broadcaster (Low Priority):** Wi-Fi telemetry and burst transmission.
* **IPC (Inter-Process Communication):** Uses `xQueue` for thread-safe data passing and Task Notifications to minimize CPU cycles.

### 3. The "Memory Palace" (Storage Optimization)
**Goal:** Efficient non-volatile storage on limited Flash memory (8MB).
* **Data Structure:** **Circular Buffer (Ring Buffer)** implementation to handle continuous logging without "Stop-the-world" garbage collection.
* **Serialization:** Binary packing (Structs) instead of text (JSON/CSV) to reduce write cycles and storage overhead.
* **Wear Leveling:** Utilizes ESP32 Partition Tables to manage storage sectors.

## 🧠 Algorithmic Logic: "The Elevator Detective"
The system runs a background anomaly detection algorithm based on barometric pressure deltas.
* **Vertical Velocity Detection:** Distinguishes between weather patterns (slow pressure drop) and mechanical elevation changes (rapid drop >12Pa/sec).
* **Cognitive Decline Zone:** Monitors CO2/Humidity correlations to flag environments detrimental to human focus ("Brain Fog" threshold).

## 🛠️ Hardware & Tech Stack
* **MCU:** Espressif ESP32-S3 (Xtensa LX7)
* **Sensor:** Bosch BME280 (Temperature, Humidity, Pressure)
* **Framework:** ESP-IDF v5.3 (C/C++)
* **OS:** FreeRTOS
* **Build System:** CMake / Ninja

## 🚀 Getting Started

### Prerequisites
* ESP-IDF v5.3 Toolchain installed.
* BME280 connected via I2C (SDA/SCL defined in `config.h`).

### Build Instructions
1.  **Clone the Repository**
    ```bash
    git clone [https://github.com/YOUR_USERNAME/Environmental-Black-Box.git](https://github.com/YOUR_USERNAME/Environmental-Black-Box.git)
    cd Environmental-Black-Box
    ```

2.  **Configure Environment**
    ```bash
    get_idf  # Source your export.sh alias
    idf.py set-target esp32s3
    ```

3.  **Build & Flash**
    ```bash
    idf.py build
    idf.py -p /dev/ttyACM0 flash monitor
    ```

## 📈 Future Roadmap
* [ ] DMA (Direct Memory Access) for sensor readings to offload CPU.
* [ ] MQTT Integration for cloud telemetry.
* [ ] Low Power "Deep Sleep" implementation for multi-day battery life.

---
*Created by Tarik Hireche - Winter 2026*
