cmake_minimum_required(VERSION 3.16)

project(acados_cpp CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Acados C library (will be built as part of main Acados build)
# The C library should be available from the parent build

# Create C++ wrapper library
add_library(acados_cpp
    src/ocp_nlp.cpp
)

target_include_directories(acados_cpp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../acados_c  # For C interface headers
        ${CMAKE_SOURCE_DIR}/acados  # For core acados headers
)

# Link against Acados C library
# This will be provided by the main Acados build
target_link_libraries(acados_cpp
    PUBLIC
        acados  # Main Acados C library
)

# Installation
install(TARGETS acados_cpp
    EXPORT acados_cpp-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

install(EXPORT acados_cpp-targets
    FILE acados_cpp-targets.cmake
    NAMESPACE acados::
    DESTINATION lib/cmake/acados_cpp
)

# Create package config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/acados_cpp-config-version.cmake"
    VERSION 0.1.0
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/acados_cpp-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/acados_cpp-config.cmake"
    INSTALL_DESTINATION lib/cmake/acados_cpp
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/acados_cpp-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/acados_cpp-config-version.cmake"
    DESTINATION lib/cmake/acados_cpp
)
