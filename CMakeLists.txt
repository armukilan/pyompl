cmake_minimum_required(VERSION 3.15)
project(pyompl)


find_package(pybind11 CONFIG REQUIRED)

set(OMPL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/ompl/src")
set(OMPL_BUILD_INCLUDE "${CMAKE_SOURCE_DIR}/ompl/build/src")

pybind11_add_module(_core src/pyompl/_core.cpp)

target_include_directories(_core PRIVATE
    ${OMPL_INCLUDE_DIR}
    ${OMPL_BUILD_INCLUDE}
)

target_compile_definitions(_core PRIVATE _USE_MATH_DEFINES)

if(WIN32)
    # Use GITHUB_WORKSPACE if set (CI), otherwise use local path
    if(DEFINED ENV{GITHUB_WORKSPACE})
        set(OMPL_BASE "$ENV{GITHUB_WORKSPACE}")
    else()
        set(OMPL_BASE "${CMAKE_SOURCE_DIR}")
    endif()

    set(VCPKG_INCLUDE "C:/vcpkg/installed/x64-windows/include")
    set(VCPKG_LIB "C:/vcpkg/installed/x64-windows/lib")
    set(OMPL_LIB_FILE "${OMPL_BASE}/ompl/build/src/ompl/Release/ompl.lib")

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
        "${VCPKG_LIB}/boost_atomic-*.lib")

    target_link_libraries(_core PRIVATE ${OMPL_LIB_FILE} ${BOOST_LIBS})

else()
    # Use GITHUB_WORKSPACE if set (CI), otherwise use local path
    if(DEFINED ENV{GITHUB_WORKSPACE})
        set(OMPL_BASE "$ENV{GITHUB_WORKSPACE}")
    elseif(EXISTS "/project/ompl")
        set(OMPL_BASE "/project")
    else()
        set(OMPL_BASE "${CMAKE_SOURCE_DIR}")
    endif()

    set(OMPL_LIB_FILE "${OMPL_BASE}/ompl/build/src/ompl/libompl.a")

    find_package(Boost REQUIRED COMPONENTS serialization thread filesystem date_time chrono atomic)
    find_package(Eigen3 REQUIRED)

    target_include_directories(_core PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
    )

    target_link_libraries(_core PRIVATE
        ${OMPL_LIB_FILE}
        ${Boost_LIBRARIES}
    )

    if(APPLE)
        target_link_options(_core PRIVATE -mmacosx-version-min=10.15)
    endif()

endif()

install(TARGETS _core DESTINATION pyompl)