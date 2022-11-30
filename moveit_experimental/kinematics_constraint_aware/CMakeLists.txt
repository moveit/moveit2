add_library(moveit_kinematics_constraint_aware src/kinematics_constraint_aware.cpp)
set_target_properties(moveit_kinematics_constraint_aware PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(moveit_kinematics_constraint_aware ${catkin_EXPORTED_TARGETS})

target_link_libraries(moveit_kinematics_constraint_aware ${catkin_LIBRARIES})

install(TARGETS moveit_kinematics_constraint_aware
        EXPORT moveit_kinematics_constraint_aware
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
