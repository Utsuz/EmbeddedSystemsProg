# Adaptive Temperature Datalogger (Dual Pico W + BMP388 + NTP)

This project implements an **adaptive temperature datalogger** using two Raspberry Pi Pico W boards:

- A **Slave Pico** connected to a **BMP388 temperature sensor** which:
  - Samples temperature at a **slow baseline period**, but
  - Automatically speeds up sampling during **temperature excursions** (outside a configured comfort range).
  - Stores data compactly in on-board flash using a custom **compact-bit encoding**.

- A **Master Pico** which:
  - Connects to Wi-Fi and synchronises time via **NTP**.
  - Provides **accurate activation time** to the Slave.
  - Securely receives the Slave’s compact logs over **UART** and stores them in its own flash.
  - Dumps the stored logs to a PC via **USB**, where a Python script decodes them to a clean CSV file.

Security is provided with **HMAC-SHA256** (via mbedTLS) over the compact data stream to detect tampering/corruption.

---

## 1. Repository Structure

> Folder names assume the structure implied by your `CMakeLists.txt` files.

### Top Level

- **`CMakeLists.txt`**
  - Root CMake project for `TempSensor`.
  - Imports:
    - `pico_sdk_import.cmake`
    - `pico_extras_import_optional.cmake`
    - `FreeRTOS_Kernel_import.cmake`
  - Sets board to `pico_w`, configures Wi-Fi credentials via `secrets.cmake`.
  - Adds all subdirectories:
    - `bmp388/`
    - `ntp_driver/`
    - `activation_driver/`
    - `master_pico/`
    - `slave_pico/`
    - `usb/`
    - `uart_driver/`
    - `HMAC_SHA256/`

- **`decode.py`**
  - PC-side **Python decoder** for the compact binary stream.
  - Workflow:
    1. Waits for an ASCII line from Master:
       - `TIME YYYY-MM-DD HH:MM:SS`
    2. After that, reads raw binary from USB serial.
    3. Decodes the compact format:
       - Handles `TS` markers, excursion markers (`E5`), and `END` markers (`EE`).
       - Reconstructs timestamps and temperatures.
    4. Outputs a **clean CSV**:  
       `datetime,temp_C,excursion`.
  - Default serial port: `COM6` (change as needed).

- **`lwipopts_examples_common.h`**
  - Shared **LWIP configuration** used by Pico W Wi-Fi code:
    - Enables IPv4, DHCP, DNS, TCP, UDP, ICMP.
    - Sets memory sizes, TCP windows, and debug options.

- **`FreeRTOSConfig_examples_common.h`**
  - Example/common FreeRTOS config (SMP-capable, 2 cores).
  - Not directly used by master/slave builds but kept as reference.

- **Import / Secrets files** (not inspected, but referenced):
  - `pico_sdk_import.cmake`
  - `pico_extras_import_optional.cmake`
  - `FreeRTOS_Kernel_import.cmake`
  - `secrets.cmake`
    - Defines `WIFI_SSID` and `WIFI_PASSWORD` used by Wi-Fi/NTP.

---

## 2. BMP388 Sensor & Storage Core (`bmp388/`)

**Purpose:** Low-level BMP388 driver + flash logging + compact bitstream encoding.

- **`CMakeLists.txt`**
  - Builds **`bmp388_core`** library from:
    - `sensor_driver.c`
    - `storage_driver.c`
    - `flash_helpers.c`
    - `bmp388_driver.h`
    - `flash_helpers.h`
  - Links `bmp388_core` with:
    - `pico_stdlib`, `hardware_i2c`, `hardware_adc`, `hardware_flash`
  - Builds `bmp388` executable (standalone logger demo) linking:
    - `bmp388_core`, `pico_stdlib`
  - Defines:
    - `LOG_REGION_SIZE=8192` (small test log region)
    - `SAMPLE_PERIOD_MS=200` (fast sampling for testing)

- **`bmp388.c`**
  - Standalone **BMP388 Logger (Dual-Region)** demo.
  - Sets up BMP388, storage, and three buttons:
    - `GP20` – (currently used only via serial command `S` for compact backup).
    - `GP21` – dumps the **ACTIVE** log in compact-bit format over USB.
    - `GP22` – toggles sensor ON/OFF.
  - Serial commands:
    - `S` – `bmp388_backup_compact_save()` (save compact blob to flash).
    - `B` – `dump_backup_compact_raw()` (dump compact blob from backup region).
    - `A` – `dump_active_compact_bit()` (dump current ACTIVE log).
  - Periodically samples temperature, prints it, and appends to **ACTIVE** region.

- **`bmp388_driver.h`**
  - Public interface for BMP388 sensor + logging + compact encoding:
    - **Sensor API:**
      - `int bmp388_init(int i2c_port, uint32_t sda_pin, uint32_t scl_pin, uint8_t i2c_addr);`
      - `int bmp388_read(bmp388_sample_t *out);`
      - `float bmp388_last_temp(void);`
      - `void bmp388_sensorStart(void);`
      - `void bmp388_sensorStop(void);`
    - **ACTIVE storage API:**
      - `int bmp388_storage_init(void);`
      - `uint32_t bmp388_storage_count(void);`
      - `int bmp388_storage_append(uint32_t time_ms, float temperature_c);`
      - `int bmp388_storage_read(uint32_t index, uint32_t *out_ms, float *out_temp);`
      - `void bmp388_storage_erase_all(void);`
    - **Compact bit encoder:**
      - `size_t bmp388_compact_encode(cbit_emit_fn emit, void *ctx);`
      - `void dump_active_compact_bit(void);`
      - `void bmp388_backup_compact_save(void);`
      - `void dump_backup_compact_raw(void);`
    - **Excursion helpers:**
      - `void bmp388_excursion_config(float t_low_c, float t_high_c, uint32_t stable_samples);`
      - `bool bmp388_excursion_state(void);`
      - `void bmp388_excursion_reset(void);`
    - **Compact header in flash:**
      - `typedef struct compact_hdr_t { uint32_t magic; uint32_t length; ... }`
      - `const compact_hdr_t *xip_hdr_compact(void);`
      - `const uint8_t *xip_data_compact(void);`
      - `extern const uint32_t CB_MAGIC;`  (`'CBIT'` magic).

- **`sensor_driver.c`**
  - **BMP388 sensor driver with excursion-aware ODR:**
    - Configures I²C and verifies CHIP ID.
    - Reads temperature calibration registers and applies compensation.
    - Converts raw BMP388 temperature into calibrated °C with an offset adjustment.
  - **Excursion logic:**
    - Configurable comfort range: `g_t_low_c`, `g_t_high_c`.
    - Tracks whether current temperature is in excursion (`g_in_excursion`).
    - Requires multiple stable samples back inside the range to exit excursion.
  - **ODR control:**
    - Normal (baseline) ODR vs **FAST** ODR during excursion.
    - Automatically switches ODR and prints status when state changes.
  - **Control functions:**
    - `bmp388_sensorStart()` / `bmp388_sensorStop()` to enable/disable sampling.

- **`storage_driver.c`**
  - **ACTIVE Flash Region:**
    - Stores 4-byte records: `[delta_time_10ms | temp_c_centi]`.
    - Maintains header with magic `'LOG1'` and count.
    - Uses page buffering and safe flash programming.
  - **RAM Mirror (ramlog):**
    - Maintains the *most recent* `RAMLOG_CAP` samples in RAM:
      - `t_ms`, temperature, and excursion flag.
    - This avoids costly flash reads for encoding.
  - **Compact-bit Encoder (`bmp388_compact_encode`):**
    - Single **source of truth** for the compact wire/storage format.
    - Emits:
      - Fixed header bytes.
      - Initial timestamp (`OPC_TS`).
      - Temperature buckets (0.5 °C steps from 22–24 °C, clamped).
      - Excursion toggle markers (`OPC_E`).
      - End marker (`OPC_END`).
    - Called by:
      - `dump_active_compact_bit()` – streams over UART/USB.
      - `bmp388_backup_compact_save()` – stores a blob into COMPACT region.
  - **COMPACT Flash Region:**
    - Stores compact blob prefixed with `compact_hdr_t {magic='CBIT', length=...}`.
    - `dump_backup_compact_raw()` replays the stored blob raw.

- **`flash_helpers.c` / `flash_helpers.h`**
  - Thin wrappers for flash operations:
    - `flash_program_block(...)`
    - `flash_erase_sectors(...)`
  - Disable interrupts around flash operations for safety.

---

## 3. NTP Time Driver (`ntp_driver/`)

- **`CMakeLists.txt`**
  - Builds:
    - Static library `ntp_driver` (for Master).
    - Optional standalone `ntp_driver_test` executable.
  - Links against:
    - `pico_cyw43_arch_lwip_sys_freertos`
    - `FreeRTOS-Kernel-Heap4`
    - `pico_stdlib`
  - Injects Wi-Fi credentials via `WIFI_SSID`, `WIFI_PASSWORD`.

- **`ntp_driver.h`**
  - Public API:
    - `void ntp_init(void);` (Wi-Fi setup for NTP).
    - `time_t ntp_get_time(void);` (blocking NTP query).

- **`ntp_driver.c`**
  - **NTP Client:**
    - Uses LWIP (`lwip/sockets.h`, DNS, UDP) to send an NTP request to `pool.ntp.org`.
    - Converts NTP timestamp (since 1900) to Unix time (since 1970).
    - Applies fixed UTC+8 offset (Singapore).
  - **Test Mode (`BUILD_NTP_TEST`):**
    - FreeRTOS task:
      - Initializes Wi-Fi.
      - Syncs time once using `ntp_get_time()`.
      - Prints time via USB.

---

## 4. Master Pico Firmware (`master_pico/`)

- **`CMakeLists.txt`**
  - Builds `master_pico` executable.
  - Links:
    - `pico_stdlib`
    - `usb_driver`
    - `uart_driver`
    - `ntp_driver`
    - `activation_driver`
    - `pico_cyw43_arch_lwip_sys_freertos`
    - `FreeRTOS-Kernel-Heap4`
    - `bmp388_core` (for shared compact header types)
    - `hmac_sha256`
  - Enables USB stdio, disables UART stdio.
  - Passes Wi-Fi / LWIP compile definitions.

- **`FreeRTOSConfig.h`**
  - Master’s FreeRTOS configuration (single core, 128 KB heap, timers enabled, mutexes/semaphores enabled, etc.).

- **`lwipopts.h`**
  - Overrides LWIP options for FreeRTOS + sockets usage on Master.

- **`master_pico.c`**
  - Uses FreeRTOS with **two main tasks**:
    1. `wifi_ntp_task`  
       - Initializes Wi-Fi on `pico_w`.
       - Retries NTP connection up to `WIFI_MAX_RETRIES`.
       - Periodically calls `ntp_get_time()` (every 10 minutes).
       - Stores:
         - `last_ntp_time` (time_t).
         - `last_sync_time` (absolute_time_t).
       - Protects them with `ntp_mutex`.

    2. `uart_activation_task`  
       - Runs UART **binary protocol** with Slave.
       - States:
         - `STATE_HANDSHAKE`
         - `STATE_IDLE`
         - `STATE_RECEIVING`
       - Buttons:
         - `GP20` – request data (`CMD_REQ_DATA`).
         - `GP21` – dump saved compact data from Master flash to PC via USB.
         - `GP22` – wipe Master’s dump flash region.
       - Behaviours:
         - On GP20:
           - Clears any pending UART bytes.
           - Sends `CMD_REQ_DATA` to Slave.
           - Enters `STATE_RECEIVING`.
         - Continuously calls `handle_time_request()` to:
           - Respond to Slave’s `CMD_GET_TIME` with `CMD_TIME_RSP` + 8-byte Unix time.
           - Maintain handshake `CMD_HELLO`/`CMD_HI_ACK`.
         - In `STATE_RECEIVING`:
           - Reads binary header `CMD_SEND_DATA_HDR` + 2-byte length.
           - Reads payload (8-byte Unix time + compact blob).
           - Reads `DONE_CHAR` as completion.
           - Extracts **activation start time** (Unix) and stores as `last_dump_unix_time`.
           - Verifies data integrity with **HMAC-SHA256** using shared `HMAC_KEY`.
           - Calls `master_save_compact_dump()` to write **compact blob only** to Master’s flash region.
  - **Master Flash Layout:**
    - Master keeps its own **dump region** at the end of flash:
      - Header: `master_dump_hdr_t` with magic (`MASTER_DUMP_MAGIC`), length, and reserved area where activation time is stored.
      - Data: compact blob copied from Slave.
    - `master_save_compact_dump()`:
      - Erases region.
      - Stores header and compact data.
      - Stores last activation Unix time into header’s `reserved[0..7]`.

  - **Dumping to PC:**
    - `master_dump_saved_data()`:
      - Validates magic and length.
      - Resolves base time in priority:
        1. Time stored in header’s `reserved[0..7]`.
        2. `last_dump_unix_time` from last transfer.
        3. Derived from `last_ntp_time + uptime delta`.
      - Prints a single ASCII line:
        - `TIME YYYY-MM-DD HH:MM:SS`
      - Flushes, then sends **only** the raw compact blob (no extra headers).
      - Finally prints `[DUMP_END]` for humans (ignored by decoder).

  - **Wipe:**
    - `master_wipe_saved_data()` fully erases the Master dump region and resets header.

---

## 5. Slave Pico Firmware (`slave_pico/`)

- **`CMakeLists.txt`**
  - Builds `slave_pico` executable.
  - Links:
    - `pico_stdlib`
    - `uart_driver`
    - `activation_driver`
    - `bmp388_core`
    - `pico_cyw43_arch_none`
    - `hmac_sha256`
  - Includes `bmp388/`, `uart_driver`, and `activation_driver` headers.
  - Enables USB stdio.

- **`slave_pico.c`**
  - Controls the **BMP388 logger** on the Slave Pico, with:
    - I²C settings for BMP388.
    - Buzzer pin.
    - UART1 for communication with Master.
    - Buttons:
      - `GP20` – request time from Master.
      - `GP21` – dump compact to USB (`'A'` key or GP21).
      - `GP22` – start/stop sensor sampling.
  - Maintains state:
    - `STATE_HANDSHAKE`, `STATE_IDLE`, `STATE_WAIT_TIME`.
    - `initial_unix_time` – time received from Master at activation.
    - `sensor_active` – sampling flag.
  - Core functions:
    - Periodic sampling (`periodic_sampling_and_log()`):
      - Uses `bmp388_read()` and `bmp388_storage_append()`.
      - Sampling interval adapts based on `bmp388_excursion_state()` (fast vs slow).
    - LED/Buzzer (`handle_led_and_buzzer_state()`):
      - Buzzer ON and fast blinking LED during excursion.
      - LED steady when normal, OFF when inactive.
    - Buttons & USB (`handle_button_polling()`):
      - `A`/`a` over USB, or GP21 → dump compact (`dump_active_compact_bit()`).
      - GP22 → toggle sensor ON/OFF.
    - UART State Machine (`handle_uart_state_machine()`):
      - `STATE_HANDSHAKE`: responds to `CMD_HELLO` with `CMD_HI_ACK`.
      - `STATE_IDLE`:
        - On GP20: sends `CMD_GET_TIME`, enters `STATE_WAIT_TIME`.
        - On `CMD_REQ_DATA` from Master:
          - Stops sensor.
          - Calls `bmp388_backup_compact_save()` to save compact blob to flash.
          - Calls `dump_compact_to_uart()` to send compact blob plus activation time.
      - `STATE_WAIT_TIME`:
        - Waits for `CMD_TIME_RSP` + 8-byte Unix time from Master.
        - Stores `initial_unix_time`.
        - Starts sensor and moves to `STATE_IDLE`.

  - **Binary Transfer to Master (`dump_compact_to_uart()`):**
    - Reads compact header from flash (`xip_hdr_compact`).
    - Validates magic and length.
    - Computes HMAC over compact data with shared `HMAC_KEY`.
    - Sends:
      1. `CMD_SEND_DATA_HDR` (0x20).
      2. 2-byte payload length (`8 bytes Unix time + compact blob`).
      3. 8-byte `initial_unix_time` (little-endian).
      4. Raw compact blob bytes.
      5. `DONE_CHAR` (0x06).

---

## 6. USB Driver (`usb/`)

- **`usb.c`**
  - `usb_init()` – initializes stdio and prints “USB initialized.”
  - `usb_send(const char *message)` – send ASCII over USB.
  - `usb_send_raw(const uint8_t *data, size_t length)` – send raw binary bytes.
  - `usb_is_connected()` – check if USB host is present.
  - Standalone test mode (`USB_STANDALONE_TEST`) includes a small demo `main()` that sends text and a raw binary sample.

- **`usb.h`**
  - Public declarations for the functions above.

- **`CMakeLists.txt`**
  - Builds:
    - `usb_test` executable (optional standalone test).
    - `usb_driver` library for reuse.

---

## 7. UART Driver (`uart_driver/`)

- **`uart_driver.c`**
  - Low-level UART abstraction:
    - `uart_driver_init(uart_num, tx, rx, baudrate)`
    - `uart_driver_write(data, len)`
    - `uart_driver_available()`
    - `uart_driver_read_byte()`
    - `uart_driver_read(buf, len)` (blocking)
    - `uart_driver_read_timed(buf, len, timeout_ms)`:
      - Resets deadline on activity to support long transfers (e.g., compact blobs).
      - Short sleeps to play nice with FreeRTOS.

- **`uart_driver.h`**
  - Public API definitions.

- **`uart_driver_test.c`**
  - Simple test tool:
    - `GP20` button sends a “Hello N from Pico!” message.
    - Prints any received UART bytes to USB console.

- **`CMakeLists.txt`**
  - Builds `uart_driver` library and `uart_driver_test` executable.

---

## 8. Activation Driver (`activation_driver/`)

- **`activation_driver.c`**
  - Slightly higher-level wrapper over `uart_driver`:
    - `activation_driver_init(...)` – calls `uart_driver_init()`.
    - `activation_send(const char *message)` – send string over UART.
    - `activation_receive()` – light buffered receive.
    - `activation_receive_line()` – line-buffered receive terminated by `\n`.

- **`activation_driver.h`**
  - Public header.

- **`CMakeLists.txt`**
  - Builds `activation_driver` library (links `uart_driver` & `pico_stdlib`).

---

## 9. HMAC / Crypto (`HMAC_SHA256/`)

- **`CMakeLists.txt`**
  - Builds `hmac_sha256` library from `hmac_sha256.c`.
  - Includes custom `mbedtls_config.h` for minimal mbedTLS.
  - Links `pico_mbedtls` + `pico_stdlib`.
  - Optional standalone `hmac_test` executable.

- **`hmac_sha256.c`**
  - Implements **HMAC-SHA256** using mbedTLS SHA256:
    - Manual ipad/opad logic.
    - Single function: `hmac_sha256(key, key_len, msg, msg_len, out[32])`.
  - `HMAC_STANDALONE_TEST` mode:
    - Computes and prints a fixed HMAC for verification.

- **`hmac_sha256.h`**
  - Declares `hmac_sha256(...)`.

- **`mbedtls_config.h`**
  - Minimal configuration enabling SHA256 and MD, disabling filesystem/threading/network options.

---

## 10. FreeRTOS Config Files

You have multiple FreeRTOS configs:

- **`master_pico/FreeRTOSConfig.h`**  
- **`ntp_driver/FreeRTOSConfig.h` (if present)**  
- **`FreeRTOSConfig_examples_common.h`** (top-level example)

The main project uses the single-core config in the Master’s folder with 128 KB heap and standard options.

---

## 11. How to Compile

### 11.1. Prerequisites

- **Raspberry Pi Pico SDK** installed.
- **CMake ≥ 3.12**, **Ninja** or `make`, and an ARM GCC toolchain.
- **FreeRTOS-Kernel**
  ```
    git clone https://github.com/FreeRTOS/FreeRTOS.git
  ```
- `secrets.cmake` with your Wi-Fi credentials:
  ```cmake
  set(WIFI_SSID "YourSSID")
  set(WIFI_PASSWORD "YourPassword") 
  ```
 ### 11.2. Configure & Build
 - **From the repo root:

 ```
 mkdir build
 cd build
  ```
 ```cmake .. \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DPICO_TOOLCHAIN_PATH=/path/to/arm-none-eabi \
  -G "Ninja"
```
### This will build all targets, including:
  - `bmp388`
  - `master_pico`
  - `slave_pico`
  - `uart_driver_test`
  - `usb_test` (if enabled)
  - `ntp_driver_test` (if BUILD_NTP_TEST=ON)
  - `hmac_test` (if HMAC_TEST=ON)
  - 
### You can also build specific targets, e.g.:
```
ninja master_pico
ninja slave_pico
```

### The `.uf2` files will appear in the appropriate build subdirectories (e.g. `master_pico/master_pico.uf2`).

## 12. How to Run
### 12.1. Hardware Setup
  - Slave Pico:
    - Raspberry Pi Pico W (or Pico).
    - BMP388 sensor connected via I²C:
      - `EX_I2C_PORT=0`, `SDA=GP4`, `SCL=GP5`, address `0x77`.
    - UART1 to Master:
      - `TX=GP8`, `RX=GP9`.
    - Buzzer:
      - `BUZZER_PIN=GP18`.
    - Buttons:
      - `GP20` – Request time from Master.
      - `GP21` – Dump compact via USB / request dump.
      - `GP22` – Toggle sensor.
  - Master Pico:
    - Raspberry Pi Pico W.
    - UART1 to Slave (cross-over pins, common GND):
      - GP8 ↔ GP9 (TX↔RX).
    - Buttons:
      - `GP20` – Request data from Slave.
      - `GP21` – Dump saved compact data over USB to PC.
      - `GP22` – Wipe saved data region.

### 12.2. Flashing

  - Slave Pico
    - Copy `slave_pico.uf2` onto the Slave board (BOOTSEL method).
  - Master Pico
    - Copy `master_pico.uf2` onto the Master board.
  - Optional Test Binaries
    - `bmp388.uf2` – stand-alone logging test.
    - `uart_driver_test.uf2` – UART wiring test.
    - `usb_test.uf2` – USB serial test.
    - `ntp_driver_test.uf2` – NTP/Wi-Fi sanity test.
    - `hmac_test.uf2` – HMAC correctness test.

## 13. System Operation
### 13.1. Time Synchronisation

  - Power both boards.
  - Master:
    - Wi-Fi + NTP task connects to Wi-Fi and starts syncing time.
  - On Slave:
    - Press **GP20** (“Request TIME”).
    - Slave sends `CMD_GET_TIME` to Master.
  - Master:
    - Responds with `CMD_TIME_RSP` + 8-byte Unix time.
    - Saves this as activation time (`last_activation_unix_time`).
  - Slave:
    - Receives time, prints human readable time.
    - Starts sensor sampling with excursion-aware rates.

### 13.2. Logging & Excursion
  - Slave samples temperature periodically:
    - Baseline period: `SAMPLE_PERIOD_MS` (e.g. 5000 ms).
    - Excursion period: `EXCURSION_PERIOD_MS` (e.g. 200 ms) when temperature goes outside [t_low, t_high].
  - Excursion detection:
    - If temperature < `t_low` or > `t_high` → excursion ON.
    - Returns to normal after several stable in-range samples.

### 13.3. Requesting Data (Master ← Slave)

  - On Master, press GP20:
    - Master sends `CMD_REQ_DATA` to Slave.
  - On Slave:
    - Stops sensor.
    - Saves compact blob to flash (`bmp388_backup_compact_save()`).
  - Sends:
    - Binary header (`CMD_SEND_DATA_HDR` + length).
    - 8-byte activation Unix time.
    - Compact blob.
    - `DONE_CHAR`.
  - On Master:
    - Reads header & payload.
    - Extracts activation time.
    - Computes HMAC-SHA256 over compact blob.
    - Stores compact blob (excl. time) to its own flash dump region and saves activation time in header.

### 13.4. Dumping to PC

  - Connect Master Pico to PC via USB.
  - Open a terminal to check logs if needed.
  - Press GP21 on Master:
    - Master:
      - Looks up activation time (from header or last dump).
      - Prints `TIME YYYY-MM-DD HH:MM:SS`.
      - Flushes.
      - Sends raw compact blob via USB.
  - On PC, run:
    ```
      pip install pyserial
      python decode.py
    ```
  -  Ensure `PORT` in `decode.py` matches your Master Pico COM port.
  -  Script will:
    -  Wait for `TIME` line.
    -  Capture following binary blob.'
    -  Decode to decoded.csv with:
      -  `datetime`
      -  `temp_C`
      -  `excursion` (0 or 1).

## 14. How to Test
### 14.1. Unit / Module Tests
  - UART Driver Test
    - Build `uart_driver_test`.
    - Flash to a Pico, connect UART loopback or second Pico.
    - Press GP20:
      - Expect “Hello N from Pico!” on the receiving side.
      - Any incoming UART data should echo to USB console.
  - USB Test
    - Build `usb_test` (if enabled).
    - Flash to a Pico.
    - Connect via USB serial:
      - Expect “USB Serial Ready!” and sample lines.
      - Confirm the host sees both text and optional raw binary bursts.
  - NTP Driver Test
    - Enable `BUILD_NTP_TEST` in `ntp_driver/CMakeLists.txt` and build.
    - Flash ntp_driver_test to Pico W.
    - Monitor USB serial:
      - Should connect to Wi-Fi and print `NTP time: ....`
  - HMAC Test
    - Enable `HMAC_TEST` in `HMAC_SHA256/CMakeLists.txt`.
    - Flash `hmac_test` and check printed HMAC against expected value (for given key/message).
  - BMP388 Standalone Logger
    - Flash `bmp388` to a board with BMP388 wired.
    - Observe temperature logs, sensor toggling via GP22, and compact dumps via serial commands.

### 14.2. End-to-End System Test
  - Configure Wi-Fi (`secrets.cmake`), rebuild, and flash.
  - Start both Master and Slave Picos.
  - On Slave, request time (GP20) and confirm:
    - Slave prints human-readable time.
    - Sensor starts logging.
  - Allow system to run and trigger excursions (e.g., warm/cool the sensor).
  - On Master, press GP20 to request data.
  - On Master, press GP21 to dump to PC.
  - Run `decode.py` and inspect `decoded.csv`:
    - Timestamps must be in correct order.
    - Temperatures should be plausible.
    - Excursion flag should reflect your temperature manipulations.

## 15. Summary

### This project combines:

  - Embedded C on dual Pico W boards (Master + Slave).
  - BMP388 sensor driver with adaptive sampling.
  - Custom compact-bit encoding that greatly compresses temperature + excursion data.
  - FreeRTOS + LWIP + NTP for accurate timekeeping.
  - HMAC-SHA256 integrity checking.
  - USB + Python tooling for exporting clean CSV logs
