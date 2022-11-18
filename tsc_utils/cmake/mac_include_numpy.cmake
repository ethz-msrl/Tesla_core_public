# On Mac OSX, Homebrew puts the Numpy C headers in the python site-packages
# Directory. We need to find the include ourselves.
function(mac_include_numpy)
    # This is a bad hack with hardcoded paths but I haven't found a better way yet
    message(STATUS ${CATKIN_DEVEL_PREFIX})
    execute_process(COMMAND ${CATKIN_DEVEL_PREFIX}/../../etc/tsc_utils/find_numpy_include_dir.py
                    RESULT_VARIABLE find_numpy_result
                    OUTPUT_VARIABLE Numpy_INCLUDE_DIR)
    if(NOT (${find_numpy_result} EQUAL 0))
        message(FATAL_ERROR "Could not find Numpy include dir")
    endif()
    message(STATUS "numpy include dir: ${Numpy_INCLUDE_DIR}")
    include_directories(${Numpy_INCLUDE_DIR})
endfunction()
