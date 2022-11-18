set(LOCAL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include/mpem")

# using an old version of UseSWIG.cmake that outputs the wrapped cpp file to 
# build/mag_manip/include/mag_manip so we need to create that path first

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../include/mpem")

set(SWIG_I_INCLUDES "${LOCAL_INCLUDE_DIR}/eigen.i"
    "${LOCAL_INCLUDE_DIR}/numpy.i")

create_swig_python_module(
    REAL_NAME "mpem"
    LIBRARY mpem 
    INCLUDE_DIRS "${LOCAL_INCLUDE_DIR}"
    I_FILE "${LOCAL_INCLUDE_DIR}/mpem.i"
    INCLUDE_I_FILES "${SWIG_I_INCLUDES}")
