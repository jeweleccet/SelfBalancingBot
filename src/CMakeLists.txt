# Main executable
add_executable(SelfBalancingBot main.cpp)

add_subdirectory(lib)
# Link libraries
target_link_libraries(SelfBalancingBot
        PRIVATE
        pid
        imu
)

# Include directories for linked libraries
target_include_directories(SelfBalancingBot
        PRIVATE  # Use PRIVATE to limit these includes to SelfBalancingBot only
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/pid/include
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/imu/include
)




