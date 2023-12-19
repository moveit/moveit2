# MoveIt 2 Migration Guide

## Logging

In MoveIt 1 we commonly use named ROS logging macros (i.e. `ROS_INFO_NAMED`) with file-specific logger names defined as `constexpr char LOGNAME[]="logger_name"`.
ROS 2 provides similar macros (i.e. `RCLCPP_INFO`) that instead of an optional name always require a `rclcpp::Logger` instance for scoping the namespace.

In ROS 2 logging is tied to the Node object for publishing to /rosout.
For namespaces there is the option of creating a child logger.
However because of initaliziation order (child logger has to be created after rclcpp::init is called and from a method on the logger from the node) you can no longer define the logger object as a file level static.
`moveit_core` provides a `util` library which contains some functions to make this situation more ergonomic.
```C++
#include <moveit/utils/logger.hpp>
```

Wherever the node is created there is the option to set the root logging namespace to be from the node for moveit using this syntax:

```C++
moveit::setNodeLoggerName(node->get_name());
```

Then wherever you call a logging macro you can use the `moveit::getLogger()` function:
```C++
RCLCPP_INFO(moveit::getLogger("my_namespace"), "Very important info message");
```

### Logger naming convention

Migrating the loggers is a good opportunity to make logger names more consistent.
In order to create unique and descriptive logger names we encourage the following naming pattern: general `LIBRARY_NAME.SOURCE_FILE_NAME`.

For finding the `LIBRARY_NAME` refer to the line `add_library(LIBRARY_NAME ...)` in the library's `CMakeLists.txt`.
If the source file name is the same or very similar to the library name it is sufficient to only use the source file name.

### Logging in header files

Some classes declared in header files may contain log messages, for instance to warn about not-implemented virtual functions in abstract classes.
A logger defined in the header file would not tell us what derived class is missing the implementation, since the source name would be resolved from the header file.
For this case, the base class should declare a private member variable `rclcpp::Logger logger_;` which is to be defined in the implementing class using the corresponding source file name.
