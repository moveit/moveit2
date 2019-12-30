set(MOVEIT_LIB_NAME moveit_robot_state)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # clang is picky about typeid(*solver)
  add_compile_options(-Wno-potentially-evaluated-expression)
endif()

add_library(${MOVEIT_LIB_NAME} SHARED
  src/attached_body.cpp
  src/conversions.cpp
  src/robot_state.cpp
  src/cartesian_interpolator.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  tf2_geometry_msgs
  geometric_shapes
  urdfdom_headers
  Boost
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_kinematics_base
  moveit_transforms
)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_resources REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(resource_retriever REQUIRED)

  include_directories(${moveit_resources_INCLUDE_DIRS})

  if(WIN32)
    # TODO add windows paths
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_robot_state test/robot_state_test.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")

  target_include_directories(test_robot_state PUBLIC
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometric_shapes_INCLUDE_DIRS}
    ${srdfdom_INCLUDE_DIRS}
  )

  target_link_libraries(test_robot_state
    moveit_test_utils
    moveit_utils
    moveit_exceptions
    ${tf2_geometry_msgs_LIBRARIES}
    ${srdfdom_LIBRARIES}
    ${MOVEIT_LIB_NAME}
    ${moveit_profiler_LIBRARIES}
    ${urdf_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    resource_retriever::resource_retriever
  )

  # As an executable, this benchmark is not run as a test by default
  ament_add_gtest(test_robot_state_benchmark test/robot_state_benchmark.cpp)
  target_include_directories(test_robot_state_benchmark PUBLIC
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometric_shapes_INCLUDE_DIRS}
    ${srdfdom_INCLUDE_DIRS}
  )

  target_link_libraries(test_robot_state_benchmark
    moveit_test_utils
    moveit_utils
    moveit_exceptions
    ${tf2_geometry_msgs_LIBRARIES}
    ${srdfdom_LIBRARIES}
    ${MOVEIT_LIB_NAME}
    ${moveit_profiler_LIBRARIES}
    ${urdf_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    resource_retriever::resource_retriever
  )

  ament_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
  target_include_directories(test_robot_state_complex PUBLIC
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometric_shapes_INCLUDE_DIRS}
    ${srdfdom_INCLUDE_DIRS}
  )

  target_link_libraries(test_robot_state_complex
    moveit_test_utils
    moveit_utils
    moveit_exceptions
    ${tf2_geometry_msgs_LIBRARIES}
    ${srdfdom_LIBRARIES}
    ${MOVEIT_LIB_NAME}
    ${moveit_profiler_LIBRARIES}
    ${urdf_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    resource_retriever::resource_retriever
  )

  ament_add_gtest(test_aabb test/test_aabb.cpp)
  target_include_directories(test_aabb PUBLIC
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${geometric_shapes_INCLUDE_DIRS}
    ${srdfdom_INCLUDE_DIRS}
  )

  target_link_libraries(test_aabb
    moveit_test_utils
    moveit_utils
    moveit_exceptions
    ${tf2_geometry_msgs_LIBRARIES}
    ${srdfdom_LIBRARIES}
    ${MOVEIT_LIB_NAME}
    ${moveit_profiler_LIBRARIES}
    ${urdf_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    resource_retriever::resource_retriever
  )

  ament_add_gtest(test_cartesian_interpolator test/test_cartesian_interpolator.cpp)
  target_link_libraries(test_cartesian_interpolator
    moveit_test_utils
    ${urdfdom_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
    ${MOVEIT_LIB_NAME}
  )
endif()
