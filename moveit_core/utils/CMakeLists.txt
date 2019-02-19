set(MOVEIT_LIB_NAME moveit_utils)

add_library(${MOVEIT_LIB_NAME}
<<<<<<< HEAD
<<<<<<< HEAD
  src/lexical_casts.cpp
  src/xmlrpc_casts.cpp
)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
=======
  src/lexical_casts.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${Boost_LIBRARIES})
>>>>>>> Replace catkin_LIBRARIES
=======
  src/lexical_casts.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${Boost_LIBRARIES})
>>>>>>> 5ff4d9b134d98b8f2e1ee120b66ce8bbd332de1e

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  set(MOVEIT_TEST_LIB_NAME moveit_test_utils)
  add_library(${MOVEIT_TEST_LIB_NAME}
    src/robot_model_test_utils.cpp)

  add_dependencies(${MOVEIT_TEST_LIB_NAME} ${catkin_EXPORTED_TARGETS})

  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  target_link_libraries(${MOVEIT_TEST_LIB_NAME} ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

  set_target_properties(${MOVEIT_TEST_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

  install(TARGETS ${MOVEIT_TEST_LIB_NAME}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()


install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
