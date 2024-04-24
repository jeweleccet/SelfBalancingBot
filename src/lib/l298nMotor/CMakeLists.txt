# Define library target
add_library(l298nMotor STATIC
        src/l298nMotor.cpp
)

# Include directories
target_link_libraries(l298nMotor PRIVATE JetsonGPIO)
target_include_directories(l298nMotor
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include  # Include directory for public headers
)