cmake_minimum_required(VERSION 3.1)
project(InteractionTools VERSION 0.1)

find_package(SofaGeneral REQUIRED)

set(USE_INTERACTIONTOOLS_PLUGIN true CACHE BOOL "Use Interaction Tools plugin")

set(PLIERS_TOOLS_SRC_DIR src/InteractionTools)

set(HEADER_FILES
    ${PLIERS_TOOLS_SRC_DIR}/PliersPinceManager.h
    ${PLIERS_TOOLS_SRC_DIR}/PliersPositionsMapper.h
)

set(SOURCE_FILES
    ${PLIERS_TOOLS_SRC_DIR}/PliersPinceManager.cpp
    ${PLIERS_TOOLS_SRC_DIR}/PliersPositionsMapper.cpp
    ${PLIERS_TOOLS_SRC_DIR}/initInteractionToolsPlugin.cpp
)

set(README_FILES PluginInteractionTools.txt)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_INTERACTIONTOOLS")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} SofaComponentGeneral)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

## Install rules for the library; CMake package configurations files
sofa_generate_package(
    NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME}
    VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR ${PLIERS_TOOLS_SRC_DIR}
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
