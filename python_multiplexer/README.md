# RealSense D456 Multiplexer (RSMux)

This project provides a **multiplexer for the Intel RealSense D456 camera**, designed to run on embedded platforms such as NVIDIA Jetson Orin / Orin Nano.  

The multiplexer (`RSMux`) separates **video (RGB + IR)** and **IMU (gyro + accel)** into independent ZMQ channels so that different workers/processes can consume them **without device locking**.

## Features

- **Auto MASTER/CLIENT mode**:
  - The first process takes control of the RealSense device (MASTER).
  - Other processes automatically connect as CLIENTs via TCP (ZMQ).
- **Video channel (FRAMES)**:
  - Publishes RGB + IR streams at 30 FPS.
  - Supports JPEG compression on the host side (no MJPG required from the camera).
  - Uses `PUB/SUB` with `conflate=1` (latest-wins) → subscribers always get the most recent frame.
- **IMU channel**:
  - Publishes fused **gyro + accel** pairs at 200 Hz.
  - Accel samples are linearly interpolated to match each gyro timestamp.
  - Optional one-pole low-pass filter to reduce vibration noise.
  - Uses `PUB/SUB` (conflate disabled) → high-rate stream without dropping messages.
- **Timestamps**:
  - `ts` → RealSense sensor timestamp (float seconds, e.g., `1758270534.28657`).
  - `mono_ns` → host monotonic nanoseconds (int).
- **CTRL channel**:
  - Provides device configuration/status via simple REQ/REP.

## Repository Layout

```
realsense_D456.py   # Main RSMux class (MASTER/CLIENT + ZMQ publishers)
rs_get_frame.py     # Example worker that subscribes to FRAMES
rs_get_imu.py       # Example worker that subscribes to IMU
```

## Usage

### 1. Start the multiplexer
You can either run the multiplexer standalone:

```bash
python3 realsense_D456.py
```

This process will become **MASTER** and start publishing FRAMES + IMU.

### 2. Run a worker

**Frame worker** (receives RGB/IR frames):
```bash
python3 rs_get_frame.py
```
Opens a window with the RGB feed and prints frame rate + latency.

**IMU worker** (receives fused IMU data):
```bash
python3 rs_get_imu.py
```
Prints gyro/accel packets at ~200 Hz with latency and rate information.

### 3. Single-process mode (loopback)

Workers can also be run alone (without `realsense_D456.py`).  
In that case, the worker becomes **MASTER** and both publishes + subscribes (loopback mode).

---

## Configuration

Inside `realsense_D456.py`:

- `ENABLE_JPEG = True` → enable JPEG compression (recommended).
- `JPEG_QUALITY = 90` → set JPEG quality (80–95 is typical).
- `USE_INTERP = True` → enable accel interpolation for gyro alignment.
- `USE_LPF = True` → enable low-pass filter on accel.
- `LPF_FC_HZ = 70.0` → cutoff frequency (Hz) for LPF.

---

## 🖧 Architecture

```
+-------------------+         +-------------------+
|   MASTER process  |         |   CLIENT worker   |
| (owns USB camera) |         | (e.g. rs_get_imu) |
|                   |         |                   |
|  FRAMES PUB ----->+--------> SUB FRAMES         |
|  IMU PUB -------->+--------> SUB IMU            |
|  CTRL REP <-------+<-------- REQ CTRL           |
+-------------------+         +-------------------+
```

- Multiple workers can subscribe simultaneously.
- Only one MASTER holds the camera device at a time.

---

## Requirements

- Python 3.8+
- [pyrealsense2](https://github.com/IntelRealSense/librealsense)
- OpenCV (`pip install opencv-python`)
- PyZMQ (`pip install pyzmq`)
- NumPy

---

## 📌 Notes

- D456 does **not** need MJPG support. We capture in `BGR8`/`Y8` and encode to JPEG on the host.
- For IR data, you can switch to PNG (lossless) if desired.
- Workers and MASTER can run on the same machine; only one needs USB access.

---
