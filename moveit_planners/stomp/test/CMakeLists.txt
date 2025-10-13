find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_noise_generator test_noise_generator.cpp)
target_link_libraries(test_noise_generator stomp::stomp rsl::rsl
                      tf2_eigen::tf2_eigen)

# ament_add_gtest(test_cost_functions test_cost_functions.cpp)
# target_link_libraries(test_cost_functions stomp::stomp rsl::rsl
# moveit_core::moveit_collision_detection)
