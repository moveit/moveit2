#ifndef MOVEIT_MOVE_GROUP_INTERFACE__VISIBILITY_CONTROL_H_
#define MOVEIT_MOVE_GROUP_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVEIT_MOVE_GROUP_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define MOVEIT_MOVE_GROUP_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVEIT_MOVE_GROUP_INTERFACE_EXPORT __declspec(dllexport)
    #define MOVEIT_MOVE_GROUP_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVEIT_MOVE_GROUP_INTERFACE_BUILDING_DLL
    #define MOVEIT_MOVE_GROUP_INTERFACE_PUBLIC MOVEIT_MOVE_GROUP_INTERFACE_EXPORT
  #else
    #define MOVEIT_MOVE_GROUP_INTERFACE_PUBLIC MOVEIT_MOVE_GROUP_INTERFACE_IMPORT
  #endif
  #define MOVEIT_MOVE_GROUP_INTERFACE_PUBLIC_TYPE MOVEIT_MOVE_GROUP_INTERFACE_PUBLIC
  #define MOVEIT_MOVE_GROUP_INTERFACE_LOCAL
#else
  #define MOVEIT_MOVE_GROUP_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define MOVEIT_MOVE_GROUP_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define MOVEIT_MOVE_GROUP_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define MOVEIT_MOVE_GROUP_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVEIT_MOVE_GROUP_INTERFACE_PUBLIC
    #define MOVEIT_MOVE_GROUP_INTERFACE_LOCAL
  #endif
  #define MOVEIT_MOVE_GROUP_INTERFACE_PUBLIC_TYPE
#endif

#endif  // MOVEIT_MOVE_GROUP_INTERFACE__VISIBILITY_CONTROL_H_
