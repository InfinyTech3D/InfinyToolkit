cmake_minimum_required(VERSION 3.1)
project(InteractionTools VERSION 0.1)

find_package(SofaGeneral REQUIRED)
find_package(SofaHaptics REQUIRED)
find_package(SofaCarving REQUIRED)
#find_package(SofaSphFluid REQUIRED)
find_package(MeshRefinement REQUIRED)

set(USE_INTERACTIONTOOLS_PLUGIN true CACHE BOOL "Use Interaction Tools plugin")

set(INTERACTIONTOOLS_SRC_DIR src/InteractionTools)

set(HEADER_FILES
    ${INTERACTIONTOOLS_SRC_DIR}/config.h.in
    ${INTERACTIONTOOLS_SRC_DIR}/RotationEngine.h
    ${INTERACTIONTOOLS_SRC_DIR}/PliersToolManager.h
    ${INTERACTIONTOOLS_SRC_DIR}/PliersPositionsMapper.h
    ${INTERACTIONTOOLS_SRC_DIR}/HapticEmulator.h    
    ${INTERACTIONTOOLS_SRC_DIR}/HapticCarvingManager.h
    ${INTERACTIONTOOLS_SRC_DIR}/SurfaceCarvingManager.h
    ${INTERACTIONTOOLS_SRC_DIR}/AdvanceCarvingManager.h
    ${INTERACTIONTOOLS_SRC_DIR}/BruteForceFeedback.h
    ${INTERACTIONTOOLS_SRC_DIR}/NeedleTracker.h
	${INTERACTIONTOOLS_SRC_DIR}/MiddleForceField.h
	${INTERACTIONTOOLS_SRC_DIR}/MiddleForceField.inl
)

set(SOURCE_FILES
    ${INTERACTIONTOOLS_SRC_DIR}/RotationEngine.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/PliersToolManager.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/PliersPositionsMapper.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/HapticEmulator.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/HapticCarvingManager.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/SurfaceCarvingManager.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/AdvanceCarvingManager.cpp    
    ${INTERACTIONTOOLS_SRC_DIR}/BruteForceFeedback.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/NeedleTracker.cpp
	${INTERACTIONTOOLS_SRC_DIR}/MiddleForceField.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/initInteractionToolsPlugin.cpp
)

set(README_FILES PluginInteractionTools.txt)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_INTERACTIONTOOLS")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} SofaCore SofaHaptics SofaCarving)

if(MeshRefinement_FOUND)
    target_link_libraries(${PROJECT_NAME} MeshRefinement)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

## Install rules for the library; CMake package configurations files
sofa_generate_package(
    NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME}
    VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR ${INTERACTIONTOOLS_SRC_DIR}
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )
