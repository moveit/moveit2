# MoveIt 2 Migration Guide

## Logging

In MoveIt 1 we commonly use named ROS logging macros (i.e. `ROS_INFO_NAMED`) with file-specific logger names defined as `constexpr char LOGNAME[]="logger_name"`.
ROS 2 provides similar macros (i.e. `RCLCPP_INFO`) that instead of an optional name always require a `rclcpp::Logger` instance for scoping the namespace.
All source files that use ROS logging should now define a file-specific `static const rclcpp::Logger` named `LOGGER`, located at the top of the file and inside the namespace with the narrowest scope (if there is one).
This leads to the following basic migration steps:

1. Replace `LOGNAME` with `rclcpp::Logger` instance:

    <b>Old:</b>

        constexpr char LOGNAME[] = "logger_name";

    <b>New:</b>

        static const rclcpp::Logger LOGGER = rclcpp::get_logger("logger_name");

2. Replace logging macros:

    <b>Old:</b>

        ROS_INFO_NAMED(LOGNAME, "Very important info message");

   <b>New:</b>

       RCLCPP_INFO(LOGGER, "Very important info message");

### Logger naming convention

Migrating the loggers is a good opportunity to make logger names more consistent.
In order to create unique and descriptive logger names we encourage the following naming pattern: general `LIBRARY_NAME.SOURCE_FILE_NAME`.
For instance, the file `joint_model_group.cpp` inside the library `moveit_robot_model` contains the following logger:

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_robot_model.joint_model_group");

For finding the `LIBRARY_NAME` refer to the line `set(MOVEIT_LIB_NAME LIBRARY_NAME)` at the top of the library's `CMakeLists.txt`.
If the source file name is the same or very similar to the library name it is sufficient to only use the source file name.

### Logging in header files

Some classes declared in header files may contain log messages, for instance to warn about not-implemented virtual functions in abstract classes.
A logger defined in the header file would not tell us what derived class is missing the implementation, since the source name would be resolved from the header file.
For this case, the base class should declare a private member variable `static const rclcpp::Logger LOGGER` which is to be defined in the implementing class using the corresponding source file name.
