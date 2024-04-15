cmake_minimum_required(VERSION 3.27)
project(mpu6050)
target_sources(mpu6050_lib PRIVATE src/mpu6050.cpp)
add_executable(mpu6050 src/mpu6050.cpp)