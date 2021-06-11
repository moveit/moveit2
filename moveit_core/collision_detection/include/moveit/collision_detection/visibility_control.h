#ifndef MOVEIT_COLLISION_DETECTION__VISIBILITY_CONTROL_H_
#define MOVEIT_COLLISION_DETECTION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVEIT_COLLISION_DETECTION_EXPORT __attribute__ ((dllexport))
    #define MOVEIT_COLLISION_DETECTION_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVEIT_COLLISION_DETECTION_EXPORT __declspec(dllexport)
    #define MOVEIT_COLLISION_DETECTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVEIT_COLLISION_DETECTION_BUILDING_DLL
    #define MOVEIT_COLLISION_DETECTION_PUBLIC MOVEIT_COLLISION_DETECTION_EXPORT
  #else
    #define MOVEIT_COLLISION_DETECTION_PUBLIC MOVEIT_COLLISION_DETECTION_IMPORT
  #endif
  #define MOVEIT_COLLISION_DETECTION_PUBLIC_TYPE MOVEIT_COLLISION_DETECTION_PUBLIC
  #define MOVEIT_COLLISION_DETECTION_LOCAL
#else
  #define MOVEIT_COLLISION_DETECTION_EXPORT __attribute__ ((visibility("default")))
  #define MOVEIT_COLLISION_DETECTION_IMPORT
  #if __GNUC__ >= 4
    #define MOVEIT_COLLISION_DETECTION_PUBLIC __attribute__ ((visibility("default")))
    #define MOVEIT_COLLISION_DETECTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVEIT_COLLISION_DETECTION_PUBLIC
    #define MOVEIT_COLLISION_DETECTION_LOCAL
  #endif
  #define MOVEIT_COLLISION_DETECTION_PUBLIC_TYPE
#endif

#endif  // MOVEIT_COLLISION_DETECTION__VISIBILITY_CONTROL_H_
