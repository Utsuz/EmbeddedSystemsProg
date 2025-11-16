---
title: Adaptive Temperature Datalogger (Master–Slave Pico System)
description: Dual-Pico BMP388 temperature logging system with compact flash storage, UART transfer, and USB export.
---

# Adaptive Temperature Datalogger (Master–Slave Pico System)

A dual-microcontroller temperature datalogger built using two **Raspberry Pi Picos** (or Pico W):

- The **Slave Pico**:
  - Reads temperature from a **BMP388** sensor via I²C  
  - Detects **excursions** (out-of-range temperatures)  
  - Logs samples in flash  
  - Encodes them into a **compact bit-encoded binary stream**  
  - Sends the compact data to the Master via **UART**

- The **Master Pico**:
  - (On Pico W) connects to Wi-Fi and synchronizes time via **NTP**
  - Communicates with the Slave over **UART**
  - Stores received compact logs into its own **flash region**
  - Exports the stored compact data over **USB** for PC-side decoding

---

## 📚 Table of Contents

- [Project Overview](#project-overview)  
- [Folder & File Structure](#folder--file-structure)  
  - [bmp388/](#1-bmp388)  
  - [slave_pico/](#2-slave_pico)  
  - [master_pico/](#3-master_pico)  
  - [uart_driver/](#4-uart_driver)  
  - [activation_driver/](#5-activation_driver)  
  - [usb/](#6-usb)  
  - [ntp_driver/](#7-ntp_driver)  
  - [HMAC_SHA256/](#8-hmac_sha256)  
  - [main/](#9-main)  
  - [decode.py](#10-decodepy)  
- [Build & Compilation](#build--compilation)  
- [Flashing the Picos](#flashing-the-picos)  
- [Running the System](#running-the-system)  
- [Testing Guide](#testing-guide)  

---

## Project Overview

This project is designed as an **embedded systems coursework-level** temperature datalogger demonstrating:

- Sensor interfacing with **BMP388**
- Flash-based data logging
- **Compact binary encoding** of temperature data for storage efficiency
- **Master–Slave communication** over UART
- **Wi-Fi + NTP** time synchronization (on Master Pico W)
- USB export and PC-side decoding of logs into CSV

It is structured to be readable and extensible for embedded systems students and engineers.

---

## Folder & File Structure

Below is a GitHub Pages–friendly breakdown of the key folders and files.

---

### 1. `bmp388/`

Core sensor and storage logic for the BMP388 and log encoding.

| File | Description |
|------|-------------|
| `sensor_driver.c` | Low-level BMP388 I²C routines, sensor initialization, calibration, and **excursion detection** logic. |
| `storage_driver.c` | Implements flash-backed **ACTIVE log** (timestamp + temperature), RAM mirror, **compact bitstream encoder**, and saving of a compact blob into a dedicated flash region. |
| `bmp388_driver.h` | Public header exposing the BMP388 and storage APIs to both `slave_pico` and `master_pico`. |
| `flash_helpers.c` / `flash_helpers.h` | Safe wrappers around `flash_range_program` / sector erase with interrupt protection. |

---

### 2. `slave_pico/`

Implements the **temperature logger node**.

| File | Description |
|------|-------------|
| `slave_pico.c` | Configures I²C, initializes the BMP388, periodically samples temperature, logs to flash, triggers compact backup, and sends the compact blob to the Master via UART. Also handles button input and excursion-based sample period switching. |

**Key behaviors:**

- Baseline sampling (e.g., every 5 seconds)  
- Faster sampling during excursion (e.g., 200 ms)  
- Buttons to:
  - Start/stop sampling  
  - Request time from Master  
  - Trigger compact backup and transfer  

---

### 3. `master_pico/`

Implements the **coordinator node**.

| File | Description |
|------|-------------|
| `master_pico.c` | Runs on Pico W, manages Wi-Fi + NTP time sync, does UART handshake with Slave, requests data (`GET_DATA`), receives compact blobs, stores them into its own flash region, and dumps data to PC over USB. |

**Key behaviors:**

- Sends `"HELLO"` / `"GET_DATA"` / `"GET_TIME"` messages to Slave  
- Receives `"HI"`, `"LENGTH <N>"`, `ACK_EMPTY`, `DONE` responses  
- Saves received binary to flash with a header and length  
- Dumps stored binary to USB when commanded  

---

### 4. `uart_driver/`

UART abstraction layer.

| File | Description |
|------|-------------|
| `uart_driver.c` | Provides initialization, transmit, receive, and availability checking for RP2040 UART instances. Used by both Master and Slave. |
| `uart_driver.h` | Header for UART driver functions. |
| `uart_driver_test.c` | Simple UART test app for validating wiring and basic TX/RX behavior. |

---

### 5. `activation_driver/`

Line-based **command protocol** on top of UART.

| File | Description |
|------|-------------|
| `activation_driver.c` | Implements functions to send ASCII commands and receive lines terminated by `\n`. Used to exchange high-level text commands (e.g., `"HELLO"`, `"HI"`, `"GET_DATA"`, `"TIME 123456789"`). |
| `activation_driver.h` | Header for the activation driver APIs. |

---

### 6. `usb/`

USB CDC communication.

| File | Description |
|------|-------------|
| `usb.c` | Initializes USB stdio, provides `usb_send()` for text and `usb_send_raw()` for binary data, and exposes `usb_is_connected()` to check host connection. May include a standalone test `main()` under a compile-time flag. |
| `usb.h` | Header for USB helper functions. |

---

### 7. `ntp_driver/`

Wi-Fi and NTP for Master Pico W.

| File | Description |
|------|-------------|
| `ntp_driver.c` | Connects to Wi-Fi (SSID/password configured elsewhere), sends NTP request, and returns synchronized time as `time_t`. Includes FreeRTOS task entry for periodic NTP refresh. |
| `ntp_driver.h` | Header for NTP APIs. |

---

### 8. `HMAC_SHA256/`

Cryptographic utilities.

| File | Description |
|------|-------------|
| `hmac_sha256.c` | Implements HMAC-SHA256 using mbedTLS’s SHA-256 routines. Used currently as a demonstrative function in the main test code. |
| `hmac_sha256.h` | Header for HMAC APIs. |

---

### 9. `main/`

Standalone demonstration (not the master or slave application).

| File | Description |
|------|-------------|
| `main.c` | Example FreeRTOS-based application that demonstrates HMAC computation and NTP usage. It is not the main entrypoint for `master_pico` or `slave_pico`, but useful for reference and testing. |

---

### 10. `decode.py`

Python utility for PC-side decoding.

| File | Description |
|------|-------------|
| `decode.py` | Reads a compact binary log (exported from the Master) and converts it into CSV, reconstructing timestamps, bucketized temperatures, and excursion flags. Intended to be run on your PC after you capture the compact dump. |

---

## Build & Compilation

These instructions assume:

- You have set up the **Pico SDK** and **ARM GCC toolchain**.  
- You are using a typical CMake workflow on Linux, macOS, or Windows.

> 📌 **Tip for GitHub Pages readers**: You can document your exact Pico SDK install steps in a separate `docs/` page and link to it from here.

### 1. Configure the Pico SDK Environment

Make sure `PICO_SDK_PATH` is correctly set; for example:

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
(Replace /path/to/pico-sdk with your installation location.)

On Windows + PowerShell, you might use:

$env:PICO_SDK_PATH="C:\path\to\pico-sdk"

2. Configure the Build Directory

##From the project root (EmbeddedSystemsProg-main/):

mkdir build
cd build
cmake ..


This will generate build files for all components, including slave_pico and master_pico.

3. Compile
cmake --build .


Or with make (depending on your generator):

make


After a successful build, you should see UF2 or ELF outputs for:

slave_pico

master_pico

and any test/demo targets (bmp388, UART tests, etc.)

Flashing the Picos

You will need two Picos (or Pico Ws) — one for the Slave and one for the Master.

Flashing the Slave Pico

Hold down the BOOTSEL button on the Slave Pico.

Connect it to your computer via USB.

A RPI-RP2 drive will appear.

Copy or drag slave_pico.uf2 (from build/) onto the RPI-RP2 drive.

The Pico will reboot with the Slave firmware.

Flashing the Master Pico

Repeat the same steps with the second board:

Hold BOOTSEL.

Plug in via USB.

Copy master_pico.uf2 to RPI-RP2.

Board reboots running Master firmware.

Running the System
Physical Connections

Slave Pico:

BMP388 sensor wired to I²C pins (e.g., GP4 = SDA, GP5 = SCL).

Master ↔ Slave UART:

Master TX → Slave RX

Master RX → Slave TX

Common GND between boards

Default config: UART1, TX on GP8, RX on GP9 for both boards.

ℹ️ You can document your exact pinout wiring in a dedicated section or diagram for students.

Typical Workflow

Power both Picos (either via USB or external supply).

Master (Pico W):

Connects to your Wi-Fi (check the code for WIFI_SSID / WIFI_PASSWORD configuration).

Performs NTP sync.

Starts sending "HELLO" over UART until it receives "HI" from Slave.

Slave:

On receiving "HELLO":

Replies with "HI", completing the handshake.

Waits for button presses and/or commands.

Button Behaviors (Typical Mapping)

The actual GPIO mappings are in slave_pico.c and master_pico.c, but a common configuration is:

On Slave Pico:

GP22 – Start/stop sampling (toggle sensor on/off).

GP20 – Request time from Master (Slave asks "GET_TIME", then waits for TIME <epoch>).

GP21 – Trigger compact backup and send data to Master.

On Master Pico:

GP20 – Request compact data from Slave (GET_DATA).

GP21 – Dump stored compact data to USB (for capture on PC).

Testing Guide

This section focuses on student-friendly test scenarios.

Test 1: Basic Sensor Logging (Slave Only)

Flash only the Slave Pico with slave_pico.uf2.

Connect Slave’s USB to your PC.

Open a serial terminal (115200 baud).

Toggle the sampling button (e.g., GP22):

You should see messages like:
T=25.3 C (t=123456)

Move the sensor to a hotter/colder environment to trigger an excursion; you should observe:

A faster sampling rate.

Excursion status printed (if enabled in slave_pico.c).

Test 2: Master–Slave Compact Data Transfer

Flash both Master and Slave.

Power both and open:

A USB serial terminal for Master.

Optionally another one for Slave (if you want to see both sides).

Wait for UART handshake:

Master prints something like:
Handshake complete with Slave.

Let Slave collect some samples.

On Master, press the request-data button (e.g., GP20).

You should see logs showing:

Slave creating compact backup

Slave sending binary data

Master receiving LENGTH N and binary payload

Master saving blob to its flash region

Test 3: Export and Decode on PC

Connect the Master Pico via USB (ensuring it’s running the Master firmware).

Press Master’s dump button (e.g., GP21).

Capture the binary stream from Master’s USB into a file, e.g.:

# Example: using a serial tool on Linux
python -m serial.tools.miniterm /dev/ttyACM0 115200 --raw > dump.bin


Use decode.py to convert dump.bin to CSV:

# Adjust periods to match firmware config (example values)
python decode.py dump.bin --base 5000 --fast 200 --csv output.csv


Open output.csv in Excel, LibreOffice, or a plotting tool to visualize:

Time (ms)

Bucketized temperature

Excursion flags

Test 4: UART & USB Module Sanity Tests

To test UART in isolation, build and flash the uart_driver_test target, then follow the instructions in uart_driver_test.c comments.

To test USB with minimal code, compile the usb standalone test (if enabled) and verify that text appears over USB serial.

If you plan to host more documentation on GitHub Pages, consider adding:

A /docs directory with:

Wiring diagrams (SVG/PNG)

Architecture diagrams (data flow, state machines)

A detailed explanation of the compact encoding format and how the Python decoder reconstructs timestamps and temperatures

Then you can link those from this README for a clean, student-friendly documentation site.
