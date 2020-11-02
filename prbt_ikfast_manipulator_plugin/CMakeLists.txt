cmake_minimum_required(VERSION 3.1.3)
project(moveit_resources_prbt_ikfast_manipulator_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-unused-variable)

# enable aligned new in gcc7+
if(CMAKE_COMPILER_IS_GNUCXX)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -faligned-new")
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf2_kdl
  tf2_eigen
  eigen_conversions
)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package()

set(IKFAST_LIBRARY_NAME prbt_manipulator_moveit_ikfast_plugin)
add_library(${IKFAST_LIBRARY_NAME} src/prbt_manipulator_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable)

install(TARGETS
  ${IKFAST_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  FILES
  prbt_manipulator_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
