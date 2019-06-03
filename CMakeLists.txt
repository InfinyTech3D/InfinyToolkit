cmake_minimum_required(VERSION 3.1)
project(Sleeve)

set(SLEEVE_VERSION 0.1)
set(USE_SLEEVE_PLUGIN true CACHE BOOL "Use Sleeve plugin")

set(HEADER_FILES
    SleevePinceManager.h
    SleevePositionsMapper.h
)

set(SOURCE_FILES
    SleevePinceManager.cpp
    SleevePositionsMapper.cpp
    initSleevePlugin.cpp
)

set(README_FILES PluginSleeve.txt)

find_package(SofaGeneral REQUIRED)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${SCENES_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SLEEVEPLUGIN")
target_link_libraries(${PROJECT_NAME} PUBLIC SofaComponentGeneral)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

## Install rules for the library; CMake package configurations files
sofa_create_package(${PROJECT_NAME} ${SLEEVE_VERSION} ${PROJECT_NAME} ${PROJECT_NAME})

