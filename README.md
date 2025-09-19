# Realsense D456 Camera RGB/IMU Logger

This project provides a lightweight logger for the Intel RealSense D456 camera.  
It records **RGB frames** and **IMU (gyro + accel)** data with synchronized timestamps, designed for research and robotics applications where data completeness and timing accuracy are critical.

---

## Installation

Make sure required dependencies are installed (librealsense2, OpenCV, CMake, etc.).

```bash
git clone https://github.com/<username>/realsense-logger.git
cd realsense-logger
mkdir -p build
cd build
cmake ..
make -j$(nproc)
