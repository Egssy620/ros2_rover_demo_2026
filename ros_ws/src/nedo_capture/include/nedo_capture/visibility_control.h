#ifndef NEDO_CAPTURE__VISIBILITY_CONTROL_H_
#define NEDO_CAPTURE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NEDO_CAPTURE_EXPORT __attribute__ ((dllexport))
    #define NEDO_CAPTURE_IMPORT __attribute__ ((dllimport))
  #else
    #define NEDO_CAPTURE_EXPORT __declspec(dllexport)
    #define NEDO_CAPTURE_IMPORT __declspec(dllimport)
  #endif
  #ifdef NEDO_CAPTURE_BUILDING_LIBRARY
    #define NEDO_CAPTURE_PUBLIC NEDO_CAPTURE_EXPORT
  #else
    #define NEDO_CAPTURE_PUBLIC NEDO_CAPTURE_IMPORT
  #endif
  #define NEDO_CAPTURE_PUBLIC_TYPE NEDO_CAPTURE_PUBLIC
  #define NEDO_CAPTURE_LOCAL
#else
  #define NEDO_CAPTURE_EXPORT __attribute__ ((visibility("default")))
  #define NEDO_CAPTURE_IMPORT
  #if __GNUC__ >= 4
    #define NEDO_CAPTURE_PUBLIC __attribute__ ((visibility("default")))
    #define NEDO_CAPTURE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NEDO_CAPTURE_PUBLIC
    #define NEDO_CAPTURE_LOCAL
  #endif
  #define NEDO_CAPTURE_PUBLIC_TYPE
#endif

#endif  // NEDO_CAPTURE__VISIBILITY_CONTROL_H_
