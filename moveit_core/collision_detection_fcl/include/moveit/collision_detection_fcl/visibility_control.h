#ifndef COLLISION_DETECTION_FCL__VISIBILITY_CONTROL_H_
#define COLLISION_DETECTION_FCL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COLLISION_DETECTION_FCL_EXPORT __attribute__((dllexport))
#define COLLISION_DETECTION_FCL_IMPORT __attribute__((dllimport))
#else
#define COLLISION_DETECTION_FCL_EXPORT __declspec(dllexport)
#define COLLISION_DETECTION_FCL_IMPORT __declspec(dllimport)
#endif
#ifdef COLLISION_DETECTION_FCL_BUILDING_DLL
#define COLLISION_DETECTION_FCL_PUBLIC COLLISION_DETECTION_FCL_EXPORT
#else
#define COLLISION_DETECTION_FCL_PUBLIC COLLISION_DETECTION_FCL_IMPORT
#endif
#define COLLISION_DETECTION_FCL_PUBLIC_TYPE COLLISION_DETECTION_FCL_PUBLIC
#define COLLISION_DETECTION_FCL_LOCAL
#else
#define COLLISION_DETECTION_FCL_EXPORT __attribute__((visibility("default")))
#define COLLISION_DETECTION_FCL_IMPORT
#if __GNUC__ >= 4
#define COLLISION_DETECTION_FCL_PUBLIC __attribute__((visibility("default")))
#define COLLISION_DETECTION_FCL_LOCAL __attribute__((visibility("hidden")))
#else
#define COLLISION_DETECTION_FCL_PUBLIC
#define COLLISION_DETECTION_FCL_LOCAL
#endif
#define COLLISION_DETECTION_FCL_PUBLIC_TYPE
#endif

#endif  // COLLISION_DETECTION_FCL__VISIBILITY_CONTROL_H_
