#ifndef MOVEIT_KINEMATICS_BASE__VISIBILITY_CONTROL_H_
#define MOVEIT_KINEMATICS_BASE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MOVEIT_KINEMATICS_BASE_EXPORT __attribute__((dllexport))
#define MOVEIT_KINEMATICS_BASE_IMPORT __attribute__((dllimport))
#else
#define MOVEIT_KINEMATICS_BASE_EXPORT __declspec(dllexport)
#define MOVEIT_KINEMATICS_BASE_IMPORT __declspec(dllimport)
#endif
#ifdef MOVEIT_KINEMATICS_BASE_BUILDING_DLL
#define MOVEIT_KINEMATICS_BASE_PUBLIC MOVEIT_KINEMATICS_BASE_EXPORT
#else
#define MOVEIT_KINEMATICS_BASE_PUBLIC MOVEIT_KINEMATICS_BASE_IMPORT
#endif
#define MOVEIT_KINEMATICS_BASE_PUBLIC_TYPE MOVEIT_KINEMATICS_BASE_PUBLIC
#define MOVEIT_KINEMATICS_BASE_LOCAL
#else
#define MOVEIT_KINEMATICS_BASE_EXPORT __attribute__((visibility("default")))
#define MOVEIT_KINEMATICS_BASE_IMPORT
#if __GNUC__ >= 4
#define MOVEIT_KINEMATICS_BASE_PUBLIC __attribute__((visibility("default")))
#define MOVEIT_KINEMATICS_BASE_LOCAL __attribute__((visibility("hidden")))
#else
#define MOVEIT_KINEMATICS_BASE_PUBLIC
#define MOVEIT_KINEMATICS_BASE_LOCAL
#endif
#define MOVEIT_KINEMATICS_BASE_PUBLIC_TYPE
#endif

#endif  // MOVEIT_KINEMATICS_BASE__VISIBILITY_CONTROL_H_
