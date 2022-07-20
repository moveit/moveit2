set(MOVEIT_PYTHON_LIB_NAME rclcpp_interface)
pybind11_add_module(${MOVEIT_PYTHON_LIB_NAME} SHARED src/python_rclcpp_interface.cpp)
set_target_properties(${MOVEIT_PYTHON_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_PYTHON_LIB_NAME} PUBLIC ${PYTHON_LIBRARIES})
ament_target_dependencies(${MOVEIT_PYTHON_LIB_NAME} PUBLIC
  rclcpp
)

if(WIN32)
  set_target_properties(${MOVEIT_PYTHON_LIB_NAME} PROPERTIES SUFFIX .pyd)
endif()

configure_build_install_location(${MOVEIT_PYTHON_LIB_NAME})
