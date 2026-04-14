cmake_minimum_required(VERSION 3.15)
project(pyompl)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Tell CMake to find Python directly, bypassing vcpkg wrapper
set(Python_FIND_REGISTRY LAST)
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

if(WIN32)
    find_package(Boost REQUIRED COMPONENTS
        serialization thread filesystem date_time chrono atomic program_options
        HINTS "C:/vcpkg/installed/x64-windows"
    )
    find_package(Eigen3 REQUIRED HINTS "C:/vcpkg/installed/x64-windows")

    # FLANN (required for OMPL_BUILD_VAMP=ON)
    find_package(flann CONFIG REQUIRED
        HINTS "C:/vcpkg/installed/x64-windows"
    )

    set(OMPL_LIB "$ENV{GITHUB_WORKSPACE}/ompl/build/src/ompl/Release/ompl.lib")
    if(NOT EXISTS "${OMPL_LIB}")
        set(OMPL_LIB "${CMAKE_SOURCE_DIR}/ompl/build/src/ompl/Release/ompl.lib")
    endif()
    set(OMPL_EXTRA_INCLUDES
        "C:/vcpkg/installed/x64-windows/include"
        "C:/vcpkg/installed/x64-windows/include/eigen3"
    )
    set(OMPL_EXTRA_LIBS flann::flann)
    set(OMPL_EXTRA_DEFINES BOOST_ALL_NO_LIB)

elseif(APPLE)
    find_package(Boost REQUIRED COMPONENTS
        serialization thread filesystem date_time chrono atomic program_options
    )
    find_package(Eigen3 REQUIRED)

    # FLANN (required for OMPL_BUILD_VAMP=ON)
    find_package(flann CONFIG REQUIRED
        HINTS
            "/opt/homebrew"
            "/opt/homebrew/lib/cmake"
            "/usr/local"
            "/usr/local/lib/cmake"
    )

    # OMPL 2.0.0 is built from source and installed to /usr/local
    find_library(OMPL_LIB ompl
        HINTS
            "/usr/local/lib"
            "/opt/homebrew/lib"
        REQUIRED
    )
    message(STATUS "Found OMPL lib: ${OMPL_LIB}")

    # OMPL 2.0.0 installs headers in ompl-2.0 subdirectory
    find_path(OMPL_HEADERS ompl/base/SpaceInformation.h
        HINTS
            "/usr/local/include/ompl-2.0"
            "/opt/homebrew/include/ompl-2.0"
            "/usr/local/include"
            "/opt/homebrew/include"
        REQUIRED
    )
    message(STATUS "Found OMPL headers: ${OMPL_HEADERS}")

    set(OMPL_EXTRA_INCLUDES
        "${OMPL_HEADERS}"
        "/opt/homebrew/include/eigen3"
        "/usr/local/include/eigen3"
        ${EIGEN3_INCLUDE_DIR}
    )
    set(OMPL_EXTRA_LIBS flann::flann)
    set(OMPL_EXTRA_DEFINES "")

else()
    # Linux
    find_package(Boost REQUIRED COMPONENTS
        serialization thread filesystem date_time chrono atomic program_options
        HINTS "/usr/local"
    )
    find_package(Eigen3 REQUIRED)

    # FLANN (required for OMPL_BUILD_VAMP=ON)
    # Installed via: dnf install flann-devel
    find_package(flann REQUIRED)

    set(OMPL_LIB "/project/ompl/build/src/ompl/libompl.so")
    set(OMPL_EXTRA_INCLUDES "")
    set(OMPL_EXTRA_LIBS flann::flann)
    set(OMPL_EXTRA_DEFINES "")
endif()

pybind11_add_module(_core src/pyompl/_core.cpp)

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
endif()

if(UNIX AND NOT APPLE)
    set_target_properties(_core PROPERTIES
        BUILD_RPATH "/project/ompl/build/src/ompl;/usr/local/lib"
        INSTALL_RPATH "/project/ompl/build/src/ompl;/usr/local/lib"
    )
endif()

install(TARGETS _core DESTINATION pyompl)