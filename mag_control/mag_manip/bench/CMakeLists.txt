find_package(benchmark REQUIRED)

add_executable(bench_backward_model_nls bench_backward_model_nls.cpp)
target_link_libraries(bench_backward_model_nls ${catkin_LIBRARIES} mag_manip benchmark::benchmark)

add_executable(bench_backward_model_mpem_L2 bench_backward_model_mpem_L2.cpp)
target_link_libraries(bench_backward_model_mpem_L2 ${catkin_LIBRARIES} mag_manip benchmark::benchmark )

add_executable(bench_forward_model_mpem bench_forward_model_mpem.cpp)
target_link_libraries(bench_forward_model_mpem ${catkin_LIBRARIES} mag_manip benchmark::benchmark )

if (NOT DISABLE_TENSORFLOW)
add_executable(bench_forward_model_tensorflow bench_forward_model_tensorflow.cpp)
target_link_libraries(bench_forward_model_tensorflow ${catkin_LIBRARIES} mag_manip benchmark::benchmark )
endif()

add_executable(bench_rbf bench_rbf.cpp)
target_link_libraries(bench_rbf ${catkin_LIBRARIES} mag_manip benchmark::benchmark )

