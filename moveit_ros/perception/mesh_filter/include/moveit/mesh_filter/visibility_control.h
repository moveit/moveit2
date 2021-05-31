#ifndef MOVEIT_MESH_FILTER__VISIBILITY_CONTROL_H_
#define MOVEIT_MESH_FILTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVEIT_MESH_FILTER_EXPORT __attribute__ ((dllexport))
    #define MOVEIT_MESH_FILTER_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVEIT_MESH_FILTER_EXPORT __declspec(dllexport)
    #define MOVEIT_MESH_FILTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVEIT_MESH_FILTER_BUILDING_DLL
    #define MOVEIT_MESH_FILTER_PUBLIC MOVEIT_MESH_FILTER_EXPORT
  #else
    #define MOVEIT_MESH_FILTER_PUBLIC MOVEIT_MESH_FILTER_IMPORT
  #endif
  #define MOVEIT_MESH_FILTER_PUBLIC_TYPE MOVEIT_MESH_FILTER_PUBLIC
  #define MOVEIT_MESH_FILTER_LOCAL
#else
  #define MOVEIT_MESH_FILTER_EXPORT __attribute__ ((visibility("default")))
  #define MOVEIT_MESH_FILTER_IMPORT
  #if __GNUC__ >= 4
    #define MOVEIT_MESH_FILTER_PUBLIC __attribute__ ((visibility("default")))
    #define MOVEIT_MESH_FILTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVEIT_MESH_FILTER_PUBLIC
    #define MOVEIT_MESH_FILTER_LOCAL
  #endif
  #define MOVEIT_MESH_FILTER_PUBLIC_TYPE
#endif

#endif  // MOVEIT_MESH_FILTER__VISIBILITY_CONTROL_H_
