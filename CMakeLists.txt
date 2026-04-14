cmake_minimum_required(VERSION 3.15)
project(pyompl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

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

elseif(APPLE)
    find_package(Boost REQUIRED COMPONENTS
        serialization thread filesystem date_time chrono atomic program_options
    )
    find_package(Eigen3 REQUIRED)

    set(OMPL_LIB "$ENV{GITHUB_WORKSPACE}/ompl/build/src/ompl/libompl.a")

    if(NOT EXISTS "${OMPL_LIB}")
        set(OMPL_LIB "${CMAKE_SOURCE_DIR}/ompl/build/src/ompl/libompl.a")
    endif()

else()
    # Linux - boost and ompl built into /project/ompl/build and /usr/local
    find_package(Boost REQUIRED COMPONENTS
        serialization thread filesystem date_time chrono atomic program_options
        HINTS "/usr/local"
    )
    find_package(Eigen3 REQUIRED)

    set(OMPL_LIB "/project/ompl/build/src/ompl/libompl.so")
    set(OMPL_RPATH "/project/ompl/build/src/ompl")
    set_target_properties(_core PROPERTIES
        BUILD_RPATH "${OMPL_RPATH}"
        INSTALL_RPATH "${OMPL_RPATH}"
    )
endif()

pybind11_add_module(_core src/pyompl/_core.cpp)

target_include_directories(_core PRIVATE
    "${CMAKE_SOURCE_DIR}/ompl/src"
    "${CMAKE_SOURCE_DIR}/ompl/build/src"
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(_core PRIVATE
    _USE_MATH_DEFINES
    BOOST_ALL_NO_LIB
)

target_link_libraries(_core PRIVATE
    "${OMPL_LIB}"
    ${Boost_LIBRARIES}
)

if(APPLE)
    target_link_options(_core PRIVATE -mmacosx-version-min=11.0)
endif()

install(TARGETS _core DESTINATION pyompl)