##############################################################################
#             Copyright (C) - InfinyTech3D - All Rights Reserved             #
#                                                                            #
#  Unauthorized copying of this file, via any medium is strictly prohibited  #
#  Proprietary and confidential.                                             #
#                                                                            #
#  Written by Erik Pernod <erik.pernod@infinytech3d.com>, September 2019     #
##############################################################################

cmake_minimum_required(VERSION 3.1)
project(InteractionTools VERSION 0.1)

find_package(SofaHaptics REQUIRED)
find_package(SofaCarving REQUIRED)
find_package(SofaGeneralObjectInteraction REQUIRED)

find_package(SofaEngine REQUIRED)
find_package(SofaLoader REQUIRED)

set(USE_INTERACTIONTOOLS_PLUGIN true CACHE BOOL "Use Interaction Tools plugin")

set(INTERACTIONTOOLS_SRC_DIR src/InteractionTools)

set(HEADER_FILES
    ${INTERACTIONTOOLS_SRC_DIR}/config.h.in
    ${INTERACTIONTOOLS_SRC_DIR}/RotationEngine.h
    ${INTERACTIONTOOLS_SRC_DIR}/PliersToolManager.h
    ${INTERACTIONTOOLS_SRC_DIR}/PliersPositionsMapper.h
    ${INTERACTIONTOOLS_SRC_DIR}/HapticEmulator.h    
    ${INTERACTIONTOOLS_SRC_DIR}/NeedleTracker.h
	${INTERACTIONTOOLS_SRC_DIR}/MiddleForceField.h
	${INTERACTIONTOOLS_SRC_DIR}/MiddleForceField.inl
    
    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/BaseCarvingPerformer.h
    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/RefineCarvingPerformer.h
    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/SurfaceCarvingPerformer.h

    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/HapticCarvingManager.h
    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/SurfaceCarvingManager.h
)

set(SOURCE_FILES
    ${INTERACTIONTOOLS_SRC_DIR}/RotationEngine.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/PliersToolManager.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/PliersPositionsMapper.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/HapticEmulator.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/NeedleTracker.cpp
	${INTERACTIONTOOLS_SRC_DIR}/MiddleForceField.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/initInteractionToolsPlugin.cpp
    
    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/BaseCarvingPerformer.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/RefineCarvingPerformer.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/SurfaceCarvingPerformer.cpp

    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/HapticCarvingManager.cpp
    ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/SurfaceCarvingManager.cpp
)

find_package(MeshRefinement QUIET)
if (MeshRefinement_FOUND)
    message("MeshRefinement plugin found - Adding it to ${PROJECT_NAME}")
    list(APPEND HEADER_FILES ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/AdvancedCarvingManager.h)
	list(APPEND HEADER_FILES ${INTERACTIONTOOLS_SRC_DIR}/BruteForceFeedback.h)
	
    list(APPEND SOURCE_FILES ${INTERACTIONTOOLS_SRC_DIR}/CarvingTools/AdvancedCarvingManager.cpp)
	list(APPEND SOURCE_FILES ${INTERACTIONTOOLS_SRC_DIR}/BruteForceFeedback.cpp)
    add_definitions(-DHAS_MESHREFINEMENT_PLUGIN)
endif()

set(README_FILES PluginInteractionTools.txt)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_INTERACTIONTOOLS")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} SofaEngine SofaLoader SofaHaptics SofaCarving SofaGeneralObjectInteraction)

if(MeshRefinement_FOUND)
    target_link_libraries(${PROJECT_NAME} MeshRefinement)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

## Install rules for the library; CMake package configurations files
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )