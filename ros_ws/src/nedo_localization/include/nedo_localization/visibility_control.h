#ifndef NEDO_LOCALIZATION__VISIBILITY_CONTROL_H_
#define NEDO_LOCALIZATION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NEDO_LOCALIZATION_EXPORT __attribute__ ((dllexport))
    #define NEDO_LOCALIZATION_IMPORT __attribute__ ((dllimport))
  #else
    #define NEDO_LOCALIZATION_EXPORT __declspec(dllexport)
    #define NEDO_LOCALIZATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef NEDO_LOCALIZATION_BUILDING_LIBRARY
    #define NEDO_LOCALIZATION_PUBLIC NEDO_LOCALIZATION_EXPORT
  #else
    #define NEDO_LOCALIZATION_PUBLIC NEDO_LOCALIZATION_IMPORT
  #endif
  #define NEDO_LOCALIZATION_PUBLIC_TYPE NEDO_LOCALIZATION_PUBLIC
  #define NEDO_LOCALIZATION_LOCAL
#else
  #define NEDO_LOCALIZATION_EXPORT __attribute__ ((visibility("default")))
  #define NEDO_LOCALIZATION_IMPORT
  #if __GNUC__ >= 4
    #define NEDO_LOCALIZATION_PUBLIC __attribute__ ((visibility("default")))
    #define NEDO_LOCALIZATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NEDO_LOCALIZATION_PUBLIC
    #define NEDO_LOCALIZATION_LOCAL
  #endif
  #define NEDO_LOCALIZATION_PUBLIC_TYPE
#endif

#endif  // NEDO_LOCALIZATION__VISIBILITY_CONTROL_H_
