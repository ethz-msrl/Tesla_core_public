
set(test_src_list 
    test_utils.cpp
    test_backward_model_saturation.cpp
    test_backward_model_linear_L2_saturation.cpp
    test_backward_model_linear_vfield_L2.cpp
    test_backward_model_linear_rbf_L2.cpp
    test_backward_model_linear_thinplatespline_L2.cpp
    test_backward_model_mpem_L2.cpp
    test_backward_model_nls.cpp
    test_backward_model_factory.cpp
    test_dipole_interactions.cpp
    test_emns.cpp
    test_emns_parameters_yaml.cpp
    test_forward_model.cpp
    test_forward_model_factory.cpp
    test_forward_model_saturation.cpp
    test_forward_model_linear_saturation.cpp
    test_forward_model_linear_vfield.cpp
    test_forward_model_linear_rbf.cpp
    test_forward_model_linear_thinplatespline.cpp
    test_forward_model_mpem.cpp
    test_interpolate_tricubic.cpp
    test_interpolate_trilinear.cpp
    test_interpolate_tricubic_scalar_field.cpp
    test_saturation_function_factory.cpp
    test_saturation_atan.cpp
    test_saturation_erf.cpp
    test_saturation_rational.cpp
    test_saturation_tanh.cpp
    test_saturation.cpp
    test_forward_model_linear_saturation_currents_jacobian_functor.cpp
    test_vfield.cpp
    test_rbf.cpp
    test_rbf_field_interpolator.cpp
    )

if(NOT DISABLE_TENSORFLOW) 
    list(APPEND test_src_list
    test_forward_model_tensorflow.cpp)
endif()

# the following tests needs the ROS parameter server so we need to wrap it into
# a ros_test and do it that way
add_rostest_gtest(test_emns_parameters_rosparam test_emns_parameters_rosparam.launch test_emns_parameters_rosparam.cpp)
target_link_libraries(test_emns_parameters_rosparam ${catkin_LIBRARIES} mag_manip)
add_rostest_gtest(test_emns_parameters_conversions test_emns_parameters_conversions.launch
    test_emns_parameters_conversions.cpp)
target_link_libraries(test_emns_parameters_conversions ${catkin_LIBRARIES} mag_manip)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    APPEND_COVERAGE_COMPILER_FLAGS()
endif()

# Add your targets here

if (CATKIN_ENABLE_TESTING)
    # Add your tests here
    set(tests "")
    foreach(test_file ${test_src_list})
        get_filename_component(test ${test_file} "NAME_WE")
        message(STATUS "Adding test: ${test}")
        catkin_add_gtest(${test} ${test_file})
        target_link_libraries(${test} ${catkin_LIBRARIES} mag_manip)
        list(APPEND tests ${test})
    endforeach()

    # Create a target ${PROJECT_NAME}_coverage_report
    if(ENABLE_COVERAGE_TESTING)
        message(STATUS "enabling coverage on ${tests}")
        #set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/other_dir_i_dont_care_about*")
        add_code_coverage(
            NAME ${PROJECT_NAME}_coverage_report
            DEPENDENCIES ${tests}
            )
    endif()
endif()
