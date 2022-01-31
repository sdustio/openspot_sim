#ifndef SDNOVA_SIMULATION__VISIBILITY_CONTROL_H_
#define SDNOVA_SIMULATION__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SDNOVA_SIMULATION_EXPORT __attribute__ ((dllexport))
    #define SDNOVA_SIMULATION_IMPORT __attribute__ ((dllimport))
  #else
    #define SDNOVA_SIMULATION_EXPORT __declspec(dllexport)
    #define SDNOVA_SIMULATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef SDNOVA_SIMULATION_BUILDING_LIBRARY
    #define SDNOVA_SIMULATION_PUBLIC SDNOVA_SIMULATION_EXPORT
  #else
    #define SDNOVA_SIMULATION_PUBLIC SDNOVA_SIMULATION_IMPORT
  #endif
  #define SDNOVA_SIMULATION_PUBLIC_TYPE SDNOVA_SIMULATION_PUBLIC
  #define SDNOVA_SIMULATION_LOCAL
#else
  #define SDNOVA_SIMULATION_EXPORT __attribute__ ((visibility("default")))
  #define SDNOVA_SIMULATION_IMPORT
  #if __GNUC__ >= 4
    #define SDNOVA_SIMULATION_PUBLIC __attribute__ ((visibility("default")))
    #define SDNOVA_SIMULATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SDNOVA_SIMULATION_PUBLIC
    #define SDNOVA_SIMULATION_LOCAL
  #endif
  #define SDNOVA_SIMULATION_PUBLIC_TYPE
#endif

#endif  // SDNOVA_SIMULATION__VISIBILITY_CONTROL_H_
