# Minimum version required
cmake_minimum_required (VERSION 3.2)

# Project name
project (geometry_utils
         VERSION 1.0.0)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE LIB_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/geometry_utils/src/*.cpp
)

add_library(geometry_utils SHARED
    ${LIB_SOURCES}
)

find_package(Eigen3 REQUIRED)

target_include_directories(geometry_utils
    PUBLIC  # 公开头文件，让使用库的程序能#include
    ${CMAKE_CURRENT_SOURCE_DIR}/geometry_utils/inc
    ${EIGEN3_INCLUDE_DIRS}  # 依赖Eigen
)

set_target_properties(geometry_utils PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib  # .so输出到build/lib
    VERSION ${PROJECT_VERSION}                                # 库版本（如libgeometry_utils.so.1.0）
    SOVERSION 1                                               # API版本（如libgeometry_utils.so.1）
)

