set(LIBRARY_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

function(add_library target)
    # Forward all arguments to the orginal add_library
    _add_library(${target} ${ARGN})
    # Ensure this directory is included as an interface include
    target_include_directories(${target} INTERFACE ${LIBRARY_ROOT_DIR})
endfunction()


add_subdirectory(pid)