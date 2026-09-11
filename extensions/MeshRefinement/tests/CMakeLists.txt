cmake_minimum_required(VERSION 3.12)
project(InfinyToolkit.MeshRefinement_test)

set(SOURCE_FILES
    TetrahedronSubdivisionController_test.cpp
)

add_definitions("-DINFINYTOOLKIT_MESHREFINEMENT_TEST_SCENES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/scenes/\"")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} InfinyToolkit.MeshRefinement Sofa.Testing Sofa.Component.Topology.Container.Dynamic)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
