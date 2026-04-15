cmake_minimum_required(VERSION 3.21)

cmake_policy(SET CMP0091 NEW)

project(magicphysx VERSION 1.0.0 LANGUAGES CXX)

set(PHYSX_ROOT_DIR "" CACHE PATH "Root directory of the PhysX 5.1.3 SDK")
set(PHYSX_LIB_SUFFIX "_static_64" CACHE STRING "Suffix for PhysX library names")
set(PHYSX_LIB_DIR "" CACHE PATH "Directory containing the PhysX libraries for the active configuration")

if(NOT PHYSX_ROOT_DIR)
    message(FATAL_ERROR "PHYSX_ROOT_DIR is not set. Run CMake with -DPHYSX_ROOT_DIR=/path/to/PhysX-5.1.3")
endif()

find_path(PHYSX_INCLUDE_DIR
    NAMES PxPhysicsAPI.h
    HINTS
        "${PHYSX_ROOT_DIR}/include"
        "${PHYSX_ROOT_DIR}/physx/include"
        "${PHYSX_ROOT_DIR}/src/structgen/structgen/physx/include"
    NO_DEFAULT_PATH
    REQUIRED
)

if(NOT PHYSX_LIB_DIR)
    if(CMAKE_CONFIGURATION_TYPES)
        message(FATAL_ERROR
            "PHYSX_LIB_DIR is not set.\n"
            "With multi-config generators like Visual Studio, pass it explicitly.\n"
            "Example:\n"
            "  -DPHYSX_LIB_DIR=C:/path/to/PhysX/bin/win.x86_64.vc143.md/debug"
        )
    else()
        set(_PHYSX_PLATFORM "win.x86_64.vc143.md")
        string(TOLOWER "${CMAKE_BUILD_TYPE}" _PHYSX_BUILD_TYPE_LOWER)

        foreach(_candidate
            "${PHYSX_ROOT_DIR}/bin/${_PHYSX_PLATFORM}/${_PHYSX_BUILD_TYPE_LOWER}"
            "${PHYSX_ROOT_DIR}/lib/${_PHYSX_PLATFORM}/${_PHYSX_BUILD_TYPE_LOWER}"
            "${PHYSX_ROOT_DIR}/physx/bin/${_PHYSX_PLATFORM}/${_PHYSX_BUILD_TYPE_LOWER}"
            "${PHYSX_ROOT_DIR}/physx/lib/${_PHYSX_BUILD_TYPE_LOWER}"
            "${PHYSX_ROOT_DIR}/physx/lib")
            if(EXISTS "${_candidate}")
                set(PHYSX_LIB_DIR "${_candidate}")
                break()
            endif()
        endforeach()
    endif()
endif()

if(NOT PHYSX_LIB_DIR)
    message(FATAL_ERROR "Could not find the PhysX library directory.")
endif()

function(find_physx_lib OUT_VAR LIB_NAME)
    find_library(${OUT_VAR}
        NAMES
            "${LIB_NAME}${PHYSX_LIB_SUFFIX}"
            "${LIB_NAME}"
        PATHS "${PHYSX_LIB_DIR}"
        NO_DEFAULT_PATH
        REQUIRED
    )
endfunction()

find_physx_lib(LIB_PhysXFoundation "PhysXFoundation")
find_physx_lib(LIB_PhysXCommon "PhysXCommon")
find_physx_lib(LIB_PhysX "PhysX")
find_physx_lib(LIB_PhysXExtensions "PhysXExtensions")
find_physx_lib(LIB_PhysXPvdSDK "PhysXPvdSDK")
find_physx_lib(LIB_PhysXCharacterKinematic "PhysXCharacterKinematic")
find_physx_lib(LIB_PhysXCooking "PhysXCooking")

find_library(LIB_PhysXVehicle
    NAMES "PhysXVehicle${PHYSX_LIB_SUFFIX}" "PhysXVehicle"
    PATHS "${PHYSX_LIB_DIR}"
    NO_DEFAULT_PATH
)

find_library(LIB_PhysXVehicle2
    NAMES "PhysXVehicle2${PHYSX_LIB_SUFFIX}" "PhysXVehicle2"
    PATHS "${PHYSX_LIB_DIR}"
    NO_DEFAULT_PATH
)

set(PHYSX_LIBS
    "${LIB_PhysX}"
    "${LIB_PhysXExtensions}"
    "${LIB_PhysXCharacterKinematic}"
    "${LIB_PhysXCooking}"
    "${LIB_PhysXPvdSDK}"
    "${LIB_PhysXCommon}"
    "${LIB_PhysXFoundation}"
)

if(LIB_PhysXVehicle)
    list(APPEND PHYSX_LIBS "${LIB_PhysXVehicle}")
endif()

if(LIB_PhysXVehicle2)
    list(APPEND PHYSX_LIBS "${LIB_PhysXVehicle2}")
endif()

add_library(magicphysx SHARED
    "${CMAKE_CURRENT_SOURCE_DIR}/physx_api.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/physx_generated.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/structgen_out.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/magicphysx_export.h"
)

target_compile_features(magicphysx PRIVATE cxx_std_17)

set_target_properties(magicphysx PROPERTIES
    CXX_EXTENSIONS OFF
    OUTPUT_NAME "libmagicphysx"
    MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
)

target_compile_definitions(magicphysx PRIVATE
    MAGICPHYSX_BUILDING_DLL
    PX_PHYSX_STATIC_LIB
    $<$<CONFIG:Debug>:NDEBUG _ITERATOR_DEBUG_LEVEL=0>
)

target_include_directories(magicphysx PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${PHYSX_INCLUDE_DIR}"
)

target_compile_options(magicphysx PRIVATE
    /MP
    /EHsc
    /wd4100
)

target_link_libraries(magicphysx PRIVATE ${PHYSX_LIBS})

install(TARGETS magicphysx
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/magicphysx_export.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/structgen_out.hpp"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/magicphysx"
)