cmake_minimum_required(VERSION 3.1)

project(ai_openvdb)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib )

find_package(Arnold REQUIRED)
find_package(OpenVDB REQUIRED)

# Dependencies for OpenVDB
find_package(Boost REQUIRED)
find_package(IlmBase REQUIRED)
find_package(TBB REQUIRED)

add_subdirectory(src)
