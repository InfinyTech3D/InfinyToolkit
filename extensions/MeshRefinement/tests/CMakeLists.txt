cmake_minimum_required(VERSION 3.12)
project(InfinyToolkit.MeshRefinement_test)

set(SOURCE_FILES
    TetrahedronSubdivisionController_test.cpp
)

# The scenes still live in the MeshRefinement plugin, next to the meshes they load.
# They exercise this plugin's controller, so ownership of them ought to follow it
# here eventually; pointing at them is the smaller change for now.
add_definitions("-DINFINYTOOLKIT_MESHREFINEMENT_TEST_SCENES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/../../../../MeshRefinement/examples/\"")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} InfinyToolkit.MeshRefinement Sofa.Testing Sofa.Component.Topology.Container.Dynamic)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
