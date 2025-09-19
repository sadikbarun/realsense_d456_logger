# realsense_d456_logger
Realsense 456 Camera RGB/IMU logger

Installation
mkdir -p build
cd build
cmake ..
make -j{nproc}

Usage
./irs_logger --warmup-sec 5 --quality 100
