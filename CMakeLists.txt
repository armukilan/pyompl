cmake_minimum_required(VERSION 3.15)
project(pyompl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

# ============================================================
# PLATFORM DETECTION - collect all variables first
# then create target once
# ============================================================

if(WIN32)
    find_package(Boost REQUIRED COMPONENTS
        serialization thread filesystem date_time chrono atomic program_options
        HINTS "C:/vcpkg/installed/x64-windows"
    )
    find_package(Eigen3 REQUIRED HINTS "C:/vcpkg/installed/x64-windows")

    set(OMPL_LIB "$ENV{GITHUB_WORKSPACE}/ompl/build/src/ompl/Release/ompl.lib")
    if(NOT EXISTS "${OMPL_LIB}")
        set(OMPL_LIB "${CMAKE_SOURCE_DIR}/ompl/build/src/ompl/Release/ompl.lib")
    endif()

    set(OMPL_EXTRA_INCLUDES
        "C:/vcpkg/installed/x64-windows/include"
        "C:/vcpkg/installed/x64-windows/include/eigen3"
    )
    set(OMPL_EXTRA_LIBS "")
    set(OMPL_EXTRA_DEFINES BOOST_ALL_NO_LIB)

elseif(APPLE)
    find_package(Boost REQUIRED COMPONENTS
        serialization thread filesystem date_time chrono atomic program_options
    )
    find_package(Eigen3 REQUIRED)

    # Try homebrew OMPL first, then fall back to built from source
    find_library(OMPL_LIB ompl
        HINTS
            "/opt/homebrew/lib"
            "/usr/local/lib"
    )

    if(NOT OMPL_LIB)
        message(FATAL_ERROR "Could not find libompl. Install via: brew install ompl")
    endif()

    message(STATUS "Found OMPL: ${OMPL_LIB}")

    # Find OMPL headers from homebrew
    find_path(OMPL_INCLUDE_DIRS ompl/base/SpaceInformation.h
        HINTS
            "/opt/homebrew/include"
            "/usr/local/include"
    )

    set(OMPL_EXTRA_INCLUDES "${OMPL_INCLUDE_DIRS}")
    set(OMPL_EXTRA_LIBS "")
    set(OMPL_EXTRA_DEFINES "")

else()
    # Linux - OMPL builds as shared lib, boost from /usr/local
    find_package(Boost REQUIRED COMPONENTS
        serialization thread filesystem date_time chrono atomic program_options
        HINTS "/usr/local"
    )
    find_package(Eigen3 REQUIRED)

    set(OMPL_LIB "/project/ompl/build/src/ompl/libompl.so")
    set(OMPL_EXTRA_INCLUDES "")
    set(OMPL_EXTRA_LIBS "")
    set(OMPL_EXTRA_DEFINES "")

endif()

# ============================================================
# CREATE TARGET
# ============================================================

pybind11_add_module(_core src/pyompl/_core.cpp)

# ============================================================
# CONFIGURE TARGET - all set_target_properties AFTER creation
# ============================================================

target_include_directories(_core PRIVATE
    "${CMAKE_SOURCE_DIR}/ompl/src"
    "${CMAKE_SOURCE_DIR}/ompl/build/src"
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OMPL_EXTRA_INCLUDES}
)

target_compile_definitions(_core PRIVATE
    _USE_MATH_DEFINES
    ${OMPL_EXTRA_DEFINES}
)

target_link_libraries(_core PRIVATE
    "${OMPL_LIB}"
    ${Boost_LIBRARIES}
    ${OMPL_EXTRA_LIBS}
)

if(APPLE)
    target_link_options(_core PRIVATE -mmacosx-version-min=11.0)
    set_target_properties(_core PROPERTIES
        BUILD_RPATH "$ENV{GITHUB_WORKSPACE}/ompl/build/src/ompl"
        INSTALL_RPATH "@loader_path"
    )
endif()

if(UNIX AND NOT APPLE)
    set_target_properties(_core PROPERTIES
        BUILD_RPATH "/project/ompl/build/src/ompl;/usr/local/lib"
        INSTALL_RPATH "/project/ompl/build/src/ompl;/usr/local/lib"
    )
endif()

install(TARGETS _core DESTINATION pyompl)