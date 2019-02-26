set(MOVEIT_LIB_NAME moveit_utils)

add_library(${MOVEIT_LIB_NAME}
  src/lexical_casts.cpp
  # src/xmlrpc_casts.cpp # TODO: solve the conflicts with xmlrpc dependencies
)

include_directories(include)
ament_target_dependencies(${MOVEIT_LIB_NAME} visualization_msgs)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib")
install(DIRECTORY include/ DESTINATION "include")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_resources REQUIRED)
  set(MOVEIT_TEST_LIB_NAME moveit_test_utils)
  add_library(${MOVEIT_TEST_LIB_NAME}
    src/robot_model_test_utils.cpp)
  
  ament_target_dependencies(${MOVEIT_TEST_LIB_NAME}
    ${rclcpp_LIBRARIES}
    ${rmw_implementation_LIBRARIES}
    ${Boost_LIBRARIES}
    urdf
    urdfdom
    urdfdom_headers)

  include_directories(${moveit_resources_INCLUDE_DIRS})
  target_link_libraries(${MOVEIT_TEST_LIB_NAME} ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
  set_target_properties(${MOVEIT_TEST_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

  install(TARGETS ${MOVEIT_TEST_LIB_NAME}
          LIBRARY DESTINATION "lib"
        ARCHIVE DESTINATION "lib")
endif()
