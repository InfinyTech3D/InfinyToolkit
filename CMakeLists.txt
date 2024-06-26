##############################################################################
#             Copyright (C) - InfinyTech3D - All Rights Reserved             #
#                                                                            #
#  Unauthorized copying of this file, via any medium is strictly prohibited  #
#  Proprietary and confidential.                                             #
#                                                                            #
#  Written by Erik Pernod <erik.pernod@infinytech3d.com>, September 2019     #
##############################################################################

cmake_minimum_required(VERSION 3.12)
project(InfinyToolkit VERSION 0.1)

find_package(Sofa.Component.StateContainer REQUIRED)
find_package(Sofa.Component.MechanicalLoad REQUIRED)
find_package(Sofa.Component.Topology.Container.Dynamic REQUIRED)
find_package(Sofa.Component.SolidMechanics.Spring REQUIRED)
find_package(Sofa.Component.Constraint.Projective REQUIRED)
find_package(Sofa.Component.Collision.Geometry REQUIRED)
find_package(Sofa.Component.Controller REQUIRED)
find_package(Sofa.Component.Haptics REQUIRED)
find_package(Sofa.GUI.Component REQUIRED)

set(USE_INFINYTOOLKIT_PLUGIN true CACHE BOOL "Use Interaction Tools plugin")

set(INFINYTOOLKIT_SRC_DIR src/InfinyToolkit)

set(HEADER_FILES
    ${INFINYTOOLKIT_SRC_DIR}/config.h.in    
    ${INFINYTOOLKIT_SRC_DIR}/CollisionDetectionDisplay.h
    ${INFINYTOOLKIT_SRC_DIR}/RotationEngine.h
    ${INFINYTOOLKIT_SRC_DIR}/PliersToolManager.h
    ${INFINYTOOLKIT_SRC_DIR}/PliersPositionsMapper.h
    ${INFINYTOOLKIT_SRC_DIR}/HapticEmulator.h    
    ${INFINYTOOLKIT_SRC_DIR}/NeedleTracker.h
    ${INFINYTOOLKIT_SRC_DIR}/MiddleForceField.h
    ${INFINYTOOLKIT_SRC_DIR}/MiddleForceField.inl
    ${INFINYTOOLKIT_SRC_DIR}/Triangle2RefinedTriangleTopologicalMapping.h

    ## Carving tools sections
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/AdvancedCarvingManager.h
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/BaseCarvingPerformer.h
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/SimpleCarvingPerformer.h
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/BurningPerformer.h
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/SurfaceCarvingPerformer.h
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/HapticCarvingManager.h
    
    ## Mesh tools sections
    ${INFINYTOOLKIT_SRC_DIR}/MeshTools/NearestTexcoordsMap.h

    ${INFINYTOOLKIT_SRC_DIR}/BruteForceFeedback.h
)

set(SOURCE_FILES
    ${INFINYTOOLKIT_SRC_DIR}/CollisionDetectionDisplay.cpp
    ${INFINYTOOLKIT_SRC_DIR}/RotationEngine.cpp
    ${INFINYTOOLKIT_SRC_DIR}/PliersToolManager.cpp
    ${INFINYTOOLKIT_SRC_DIR}/PliersPositionsMapper.cpp
    ${INFINYTOOLKIT_SRC_DIR}/HapticEmulator.cpp
    ${INFINYTOOLKIT_SRC_DIR}/NeedleTracker.cpp
    ${INFINYTOOLKIT_SRC_DIR}/MiddleForceField.cpp
    ${INFINYTOOLKIT_SRC_DIR}/Triangle2RefinedTriangleTopologicalMapping.cpp
    ${INFINYTOOLKIT_SRC_DIR}/initInfinyToolkitPlugin.cpp
    
    ## Carving tools sections
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/AdvancedCarvingManager.cpp
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/BaseCarvingPerformer.cpp
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/SimpleCarvingPerformer.cpp
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/BurningPerformer.cpp
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/SurfaceCarvingPerformer.cpp
    ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/HapticCarvingManager.cpp
    
    ## Mesh tools sections
    ${INFINYTOOLKIT_SRC_DIR}/MeshTools/NearestTexcoordsMap.cpp

    ${INFINYTOOLKIT_SRC_DIR}/BruteForceFeedback.cpp
)

# Add component for carving using refinement mesh
find_package(MeshRefinement QUIET)
if (MeshRefinement_FOUND)
    message("MeshRefinement plugin found - Adding it to ${PROJECT_NAME}")
    list(APPEND HEADER_FILES 
        ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/RefineCarvingPerformer.h
        ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/CuttingPerformer.h
    )
    
    list(APPEND SOURCE_FILES 
        ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/RefineCarvingPerformer.cpp
        ${INFINYTOOLKIT_SRC_DIR}/CarvingTools/CuttingPerformer.cpp
    )
    
    add_definitions(-DHAS_MESHREFINEMENT_PLUGIN)
endif()

set(README_FILES README.md)

# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Set define dllimport/dllexport mechanism on Windows.
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_INFINYTOOLKIT")

# Link the plugin library to its dependencies (other libraries).
target_link_libraries(${PROJECT_NAME} 
    Sofa.Component.StateContainer 
    Sofa.Component.MechanicalLoad 
    Sofa.Component.Topology.Container.Dynamic
    Sofa.Component.SolidMechanics.Spring
    Sofa.Component.Constraint.Projective
    Sofa.Component.Collision.Geometry
    Sofa.Component.Controller 
    Sofa.Component.Haptics
    Sofa.GUI.Component
)


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
    
# Organize projects into folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
SET_PROPERTY(TARGET ${PROJECT_NAME} PROPERTY FOLDER "plugins")