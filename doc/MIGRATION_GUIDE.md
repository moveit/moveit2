# MoveIt2 style guide

## Logging

Each source file should have a global static constant logger, we use <b>LOGGER</b> as the logger name with upper-case capitals, the logger should be inside the namespace by default. 

The logger name consists of two parts <b>LIBRARY_NAME.SOURCE_NAME</b> e.g.

`static const rclcpp::Logger LOGGER = rclcpp::get_logger("LIBRARY_NAME.SOURCE_NAME");`

You could find the `LIBRARY_NAME` inside the `CMakeLists.txt` file of a package `set(MOVEIT_LIB_NAME LIBRARY_NAME)`
