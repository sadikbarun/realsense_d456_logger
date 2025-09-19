cmake_minimum_required(VERSION 3.12)
project(irs_logger LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(realsense2 QUIET)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(irs_logger irs_logger.cpp)

if(realsense2_FOUND)
  target_link_libraries(irs_logger PRIVATE realsense2::realsense2 ${OpenCV_LIBS})
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(REALSENSE2 REQUIRED realsense2)
  target_include_directories(irs_logger PRIVATE ${REALSENSE2_INCLUDE_DIRS})
  target_link_libraries(irs_logger PRIVATE ${REALSENSE2_LIBRARIES} ${OpenCV_LIBS})
endif()

if(MSVC)
  target_compile_options(irs_logger PRIVATE /W4)
else()
  target_compile_options(irs_logger PRIVATE -Wall -Wextra -Wpedantic)
endif()

