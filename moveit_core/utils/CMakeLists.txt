set(MOVEIT_LIB_NAME moveit_utils)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/lexical_casts.cpp
  src/message_checks.cpp
  src/rclcpp_utils.cpp
)

ament_target_dependencies(${MOVEIT_LIB_NAME} Boost moveit_msgs)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_resources REQUIRED)
  set(MOVEIT_TEST_LIB_NAME moveit_test_utils)
  add_library(${MOVEIT_TEST_LIB_NAME} SHARED
    src/robot_model_test_utils.cpp)
  ament_target_dependencies(${MOVEIT_TEST_LIB_NAME}
    Boost
    urdf
    srdfdom
    urdfdom
    urdfdom_headers
    moveit_resources
  )

  set_target_properties(${MOVEIT_TEST_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

  ament_export_libraries(${MOVEIT_TEST_LIB_NAME} ${MOVEIT_LIB_NAME})
  ament_export_include_directories(include)

  install(TARGETS
    ${MOVEIT_TEST_LIB_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
endif()
