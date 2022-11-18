set(LOCAL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include/mag_manip")

# using an old version of UseSWIG.cmake that outputs the wrapped cpp file to 
# build/mag_manip/include/mag_manip so we need to create that path first

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../include/mag_manip")

set(SWIG_I_INCLUDES "${LOCAL_INCLUDE_DIR}/eigen.i"
    "${LOCAL_INCLUDE_DIR}/numpy.i")

if(DISABLE_TENSORFLOW) 
    set(ADDITONAL_SWIG_FLAGS -DNO_TENSORFLOW)
endif()

create_swig_python_module(
    REAL_NAME "mag_manip"
    LIBRARY mag_manip 
    I_FILE "mag_manip.i"
    INCLUDE_DIRS "${LOCAL_INCLUDE_DIR}"
    INCLUDE_I_FILES "${SWIG_I_INCLUDES}"
    ADDITIONAL_FLAGS "${ADDITONAL_SWIG_FLAGS}"
)
