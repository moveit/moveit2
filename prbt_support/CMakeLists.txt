cmake_minimum_required(VERSION 2.8.3)
project(moveit_resources_prbt_support)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  sensor_msgs
)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-std=c++11)
add_definitions(-Wconversion) # At least this line needs to be below find_package to keep
                              #  the flag away from the gmock build
add_definitions(-Wpedantic)

catkin_package(
  CATKIN_DEPENDS roscpp
)

################
## Clang tidy ##
################
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()


#############
## Install ##
#############

## Install URDF and meshes
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(moveit_core REQUIRED)
  find_package(moveit_ros_planning REQUIRED)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen3 REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  #catkin_lint: ignore_once missing_directory <-- needed for successful catkin_linting
  #Declare system to supress strict compiler errors in external library headers
  include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${gmock_SOURCE_DIR}/include/
  )

  #----------------------------------------

#  add_rostest_gtest(urdf_tests
#  test/urdf_tests.test
#  test/urdf_tests.cpp
#  )
#
#  target_link_libraries(urdf_tests
#    ${catkin_LIBRARIES}
#    ${roscpp_LIBRARIES}
#    ${moveit_core_LIBRARIES}
#    ${moveit_ros_planning_LIBRARIES}
#    ${Eigen3_LIBRARIES})

  roslint_python(
    test/acceptance_tests/acceptance_test_joint_limits.py
  )
  roslint_add_test()

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage (adding -j1 recommended)
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()

endif()
