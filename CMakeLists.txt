cmake_minimum_required(VERSION 3.15)
project(pyompl)

include(ExternalProject)

find_package(pybind11 CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread filesystem date_time chrono atomic program_options)
find_package(Eigen3 REQUIRED)

set(OMPL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/ompl/src")
set(OMPL_BUILD_DIR "${CMAKE_BINARY_DIR}/ompl_build")
set(OMPL_LIB "${OMPL_BUILD_DIR}/src/ompl/libompl.a")

# Build OMPL as part of our build
ExternalProject_Add(ompl_external
    SOURCE_DIR "${CMAKE_SOURCE_DIR}/ompl"
    BINARY_DIR "${OMPL_BUILD_DIR}"
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DOMPL_BUILD_DEMOS=OFF
        -DOMPL_BUILD_TESTS=OFF
        -DOMPL_BUILD_PYTESTS=OFF
        -DOMPL_BUILD_PYBINDINGS=OFF
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
        -DBoost_ROOT=${Boost_DIR}/../../..
        -DBOOST_ROOT=${Boost_DIR}/../../..
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    BUILD_COMMAND ${CMAKE_COMMAND} --build . --config Release --target ompl -j4
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS "${OMPL_LIB}"
)

pybind11_add_module(_core src/pyompl/_core.cpp)

add_dependencies(_core ompl_external)

target_include_directories(_core PRIVATE
    ${OMPL_INCLUDE_DIR}
    ${OMPL_BUILD_DIR}/src
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(_core PRIVATE _USE_MATH_DEFINES)

target_link_libraries(_core PRIVATE
    "${OMPL_LIB}"
    ${Boost_LIBRARIES}
)

if(WIN32)
    set(VCPKG_INCLUDE "C:/vcpkg/installed/x64-windows/include")
    set(VCPKG_LIB "C:/vcpkg/installed/x64-windows/lib")

    target_include_directories(_core PRIVATE
        ${VCPKG_INCLUDE}
        ${VCPKG_INCLUDE}/eigen3
    )

    file(GLOB BOOST_LIBS
        "${VCPKG_LIB}/boost_serialization-*.lib"
        "${VCPKG_LIB}/boost_thread-*.lib"
        "${VCPKG_LIB}/boost_filesystem-*.lib"
        "${VCPKG_LIB}/boost_date_time-*.lib"
        "${VCPKG_LIB}/boost_chrono-*.lib"
        "${VCPKG_LIB}/boost_atomic-*.lib"
        "${VCPKG_LIB}/boost_program_options-*.lib")

    # Override OMPL lib path for Windows
    set(OMPL_LIB "${OMPL_BUILD_DIR}/src/ompl/Release/ompl.lib")
    target_link_libraries(_core PRIVATE ${BOOST_LIBS})

elseif(APPLE)
    target_link_options(_core PRIVATE -mmacosx-version-min=10.15)
endif()

install(TARGETS _core DESTINATION pyompl)