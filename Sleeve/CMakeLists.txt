cmake_minimum_required(VERSION 3.1)
project(Sleeve)

set(SLEEVE_VERSION 0.1)

set(HEADER_FILES

)

set(SOURCE_FILES
    initSleevePlugin.cpp
)

set(README_FILES PluginSleeve.txt)

find_package(SofaGeneral REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SLEEVEPLUGIN")
target_link_libraries(${PROJECT_NAME} PUBLIC SofaComponentGeneral)


target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX _d)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${SLEEVE_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SLEEVE")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

sofa_create_package(Sleeve ${SLEEVE_VERSION} ${PROJECT_NAME} ${PROJECT_NAME})

