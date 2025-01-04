cmake_minimum_required(VERSION 3.22)
project([GENERATED_PACKAGE_NAME])

find_package(ament_cmake REQUIRED)

ament_package()

<<<<<<< HEAD
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
=======
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

>>>>>>> a61d973f0 (Fix: Conditionally install launch directory if it exists (#3191))
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
