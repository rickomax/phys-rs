#pragma once

#if defined(_WIN32) || defined(_WIN64)
  #ifdef MAGICPHYSX_BUILDING_DLL
    #define MAGICPHYSX_EXPORT __declspec(dllexport)
  #else
    #define MAGICPHYSX_EXPORT __declspec(dllimport)
  #endif
#elif defined(__GNUC__) || defined(__clang__)
  #define MAGICPHYSX_EXPORT __attribute__((visibility("default")))
#else
  #define MAGICPHYSX_EXPORT
#endif
