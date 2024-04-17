# Define library target
add_library(pid STATIC
        src/pid.cpp
)

# Include directories
target_include_directories(pid
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include  # Include directory for public headers
)