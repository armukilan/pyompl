cmake_minimum_required(VERSION 3.15)
project(pyompl)

find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

set(OMPL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/ompl/src")
set(OMPL_BUILD_INCLUDE "${CMAKE_SOURCE_DIR}/ompl/build/src")
set(VCPKG_INCLUDE_DIR "C:/Users/mukil/dev/vcpkg/installed/x64-windows/include")
set(OMPL_LIB "${CMAKE_SOURCE_DIR}/ompl/build/src/ompl/Release/ompl.lib")
set(VCPKG_LIB_DIR "C:/Users/mukil/dev/vcpkg/installed/x64-windows/lib")

pybind11_add_module(_core src/pyompl/_core.cpp)

target_include_directories(_core PRIVATE
    ${OMPL_INCLUDE_DIR}
    ${OMPL_BUILD_INCLUDE}
    ${VCPKG_INCLUDE_DIR}
    ${VCPKG_INCLUDE_DIR}/eigen3
)

target_compile_definitions(_core PRIVATE
    _USE_MATH_DEFINES
    BOOST_ALL_NO_LIB
    BOOST_SERIALIZATION_DYN_LINK
    BOOST_THREAD_DYN_LINK
    BOOST_ATOMIC_DYN_LINK
    BOOST_CHRONO_DYN_LINK
    BOOST_DATE_TIME_DYN_LINK
)

target_link_libraries(_core PRIVATE
    ${OMPL_LIB}
    ${VCPKG_LIB_DIR}/boost_serialization-vc145-mt-x64-1_90.lib
    ${VCPKG_LIB_DIR}/boost_thread-vc145-mt-x64-1_90.lib
    ${VCPKG_LIB_DIR}/boost_filesystem-vc145-mt-x64-1_90.lib
    ${VCPKG_LIB_DIR}/boost_date_time-vc145-mt-x64-1_90.lib
    ${VCPKG_LIB_DIR}/boost_chrono-vc145-mt-x64-1_90.lib
    ${VCPKG_LIB_DIR}/boost_atomic-vc145-mt-x64-1_90.lib
)

install(TARGETS _core DESTINATION pyompl)