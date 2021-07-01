#ifndef MOVEIT_ROS_WAREHOUSE__VISIBILITY_CONTROL_H_
#define MOVEIT_ROS_WAREHOUSE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MOVEIT_ROS_WAREHOUSE_EXPORT __attribute__((dllexport))
#define MOVEIT_ROS_WAREHOUSE_IMPORT __attribute__((dllimport))
#else
#define MOVEIT_ROS_WAREHOUSE_EXPORT __declspec(dllexport)
#define MOVEIT_ROS_WAREHOUSE_IMPORT __declspec(dllimport)
#endif
#ifdef MOVEIT_ROS_WAREHOUSE_BUILDING_DLL
#define MOVEIT_ROS_WAREHOUSE_PUBLIC MOVEIT_ROS_WAREHOUSE_EXPORT
#else
#define MOVEIT_ROS_WAREHOUSE_PUBLIC MOVEIT_ROS_WAREHOUSE_IMPORT
#endif
#define MOVEIT_ROS_WAREHOUSE_PUBLIC_TYPE MOVEIT_ROS_WAREHOUSE_PUBLIC
#define MOVEIT_ROS_WAREHOUSE_LOCAL
#else
#define MOVEIT_ROS_WAREHOUSE_EXPORT __attribute__((visibility("default")))
#define MOVEIT_ROS_WAREHOUSE_IMPORT
#if __GNUC__ >= 4
#define MOVEIT_ROS_WAREHOUSE_PUBLIC __attribute__((visibility("default")))
#define MOVEIT_ROS_WAREHOUSE_LOCAL __attribute__((visibility("hidden")))
#else
#define MOVEIT_ROS_WAREHOUSE_PUBLIC
#define MOVEIT_ROS_WAREHOUSE_LOCAL
#endif
#define MOVEIT_ROS_WAREHOUSE_PUBLIC_TYPE
#endif

#endif  // MOVEIT_ROS_WAREHOUSE__VISIBILITY_CONTROL_H_
