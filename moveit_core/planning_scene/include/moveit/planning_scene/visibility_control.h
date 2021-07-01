#ifndef MOVEIT_PLANNING_SCENE__VISIBILITY_CONTROL_H_
#define MOVEIT_PLANNING_SCENE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MOVEIT_PLANNING_SCENE_EXPORT __attribute__((dllexport))
#define MOVEIT_PLANNING_SCENE_IMPORT __attribute__((dllimport))
#else
#define MOVEIT_PLANNING_SCENE_EXPORT __declspec(dllexport)
#define MOVEIT_PLANNING_SCENE_IMPORT __declspec(dllimport)
#endif
#ifdef MOVEIT_PLANNING_SCENE_BUILDING_DLL
#define MOVEIT_PLANNING_SCENE_PUBLIC MOVEIT_PLANNING_SCENE_EXPORT
#else
#define MOVEIT_PLANNING_SCENE_PUBLIC MOVEIT_PLANNING_SCENE_IMPORT
#endif
#define MOVEIT_PLANNING_SCENE_PUBLIC_TYPE MOVEIT_PLANNING_SCENE_PUBLIC
#define MOVEIT_PLANNING_SCENE_LOCAL
#else
#define MOVEIT_PLANNING_SCENE_EXPORT __attribute__((visibility("default")))
#define MOVEIT_PLANNING_SCENE_IMPORT
#if __GNUC__ >= 4
#define MOVEIT_PLANNING_SCENE_PUBLIC __attribute__((visibility("default")))
#define MOVEIT_PLANNING_SCENE_LOCAL __attribute__((visibility("hidden")))
#else
#define MOVEIT_PLANNING_SCENE_PUBLIC
#define MOVEIT_PLANNING_SCENE_LOCAL
#endif
#define MOVEIT_PLANNING_SCENE_PUBLIC_TYPE
#endif

#endif  // MOVEIT_PLANNING_SCENE__VISIBILITY_CONTROL_H_
