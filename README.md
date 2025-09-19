# Realsense D456 Camera RGB & IMU Logger

This project provides a lightweight logger for the Intel RealSense D456
camera.\
It records **RGB frames** and **IMU (gyro + accel)** data with
synchronized timestamps, designed for research and robotics applications
where data completeness and timing accuracy are critical.

------------------------------------------------------------------------

## Installation

Make sure required dependencies are installed (librealsense2, OpenCV,
CMake, etc.).

``` bash
git clone https://github.com/sadikbarun/realsense_d456_logger.git
cd realsense_d456_logger
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

------------------------------------------------------------------------

## Usage

Run the logger with:

``` bash
./irs_logger --warmup-sec 5 --quality 100
```

### Parameters

-   `--warmup-sec <N>` : Camera warmup time before recording starts
    (default: 5 seconds).
-   `--quality <1-100>` : JPEG quality factor (100 = maximum quality,
    larger file size).

------------------------------------------------------------------------

## Output

The logger generates the following structure in the output directory:

-   `color/frame_XXXX.jpg` → RGB frames
-   `left/frameL_XXXX.jpg`, `right/frameR_XXXX.jpg` → IR frames
    (optional)
-   `imu_pairs.csv` → IMU logs (gyro + interpolated accel)
-   `events.csv` → Frame timestamps, file names, and IMU matching status

------------------------------------------------------------------------
