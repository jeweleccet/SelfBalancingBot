# Define library target
add_library(imu STATIC
        src/mpu6050.cpp
)

target_link_libraries(imu i2c pthread)
# Include directories
target_include_directories(imu
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include  # Include directory for public headers
)


