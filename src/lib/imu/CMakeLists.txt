# Define library target
add_library(imu STATIC
        src/mpu6050.cpp
)

# Include directories
target_include_directories(imu
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include  # Include directory for public headers
)


