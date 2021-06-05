#ifndef TRAJECTORY_EXECUTION_MANAGER__VISIBILITY_CONTROL_H_
#define TRAJECTORY_EXECUTION_MANAGER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAJECTORY_EXECUTION_MANAGER_EXPORT __attribute__ ((dllexport))
    #define TRAJECTORY_EXECUTION_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define TRAJECTORY_EXECUTION_MANAGER_EXPORT __declspec(dllexport)
    #define TRAJECTORY_EXECUTION_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAJECTORY_EXECUTION_MANAGER_BUILDING_DLL
    #define TRAJECTORY_EXECUTION_MANAGER_PUBLIC TRAJECTORY_EXECUTION_MANAGER_EXPORT
  #else
    #define TRAJECTORY_EXECUTION_MANAGER_PUBLIC TRAJECTORY_EXECUTION_MANAGER_IMPORT
  #endif
  #define TRAJECTORY_EXECUTION_MANAGER_PUBLIC_TYPE TRAJECTORY_EXECUTION_MANAGER_PUBLIC
  #define TRAJECTORY_EXECUTION_MANAGER_LOCAL
#else
  #define TRAJECTORY_EXECUTION_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define TRAJECTORY_EXECUTION_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define TRAJECTORY_EXECUTION_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define TRAJECTORY_EXECUTION_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRAJECTORY_EXECUTION_MANAGER_PUBLIC
    #define TRAJECTORY_EXECUTION_MANAGER_LOCAL
  #endif
  #define TRAJECTORY_EXECUTION_MANAGER_PUBLIC_TYPE
#endif

#endif  // TRAJECTORY_EXECUTION_MANAGER__VISIBILITY_CONTROL_H_
