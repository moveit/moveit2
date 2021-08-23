set(MOVEIT_LIB_NAME moveit_ikfast_kinematics_plugin)

# ##############################################################################
# Install ##
# ##############################################################################

install(
  PROGRAMS scripts/auto_create_ikfast_moveit_plugin.sh
           scripts/create_ikfast_moveit_plugin.py
           scripts/round_collada_numbers.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY templates
        DESTINATION share/${PROJECT_NAME}/ikfast_kinematics_plugin)
