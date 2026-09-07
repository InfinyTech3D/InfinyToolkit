cmake_minimum_required(VERSION 3.12)
project(InfinyToolkit.MeshRefinement VERSION 0.1 LANGUAGES CXX)

# The public face of the MeshRefinement plugin. The controller a scene instantiates
# lives here; the subdivision engine stays private to MeshRefinement, reached only
# through TetrahedronSubdivisionManager, its pimpl facade.
set(INFINYTOOLKIT_MESHREFINEMENT_SRC_DIR src/InfinyToolkit/MeshRefinement)

set(HEADER_FILES
    ${INFINYTOOLKIT_MESHREFINEMENT_SRC_DIR}/config.h.in
    ${INFINYTOOLKIT_MESHREFINEMENT_SRC_DIR}/init.h
    ${INFINYTOOLKIT_MESHREFINEMENT_SRC_DIR}/TetrahedronSubdivisionController.h
    ${INFINYTOOLKIT_MESHREFINEMENT_SRC_DIR}/TetrahedronSubdivisionController.inl
)

set(SOURCE_FILES
    ${INFINYTOOLKIT_MESHREFINEMENT_SRC_DIR}/init.cpp
    ${INFINYTOOLKIT_MESHREFINEMENT_SRC_DIR}/TetrahedronSubdivisionController.cpp
)

sofa_find_package(MeshRefinement REQUIRED)
sofa_find_package(Sofa.Component.Topology.Container.Dynamic REQUIRED)

# Optional: the Cuda template instantiations of the controller. They used to sit in
# MeshRefinement.CUDA, which cannot host them now that the controller lives here -
# MeshRefinement must not depend on InfinyToolkit. MeshRefinement.CUDA still
# instantiates the engine facade for the same types, being the only place that can
# see its private .inl, and this target links those symbols.
sofa_find_package(SofaCUDA QUIET)
sofa_find_package(MeshRefinement.CUDA QUIET)
if(SofaCUDA_FOUND AND MeshRefinement.CUDA_FOUND)
    message("${PROJECT_NAME}: SofaCUDA found - adding the Cuda subdivision controller")
    list(APPEND SOURCE_FILES ${INFINYTOOLKIT_MESHREFINEMENT_SRC_DIR}/CudaSubdivisionController.cpp)
    add_definitions(-DINFINYTOOLKIT_MESHREFINEMENT_USES_SOFACUDA)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_INFINYTOOLKIT_MESHREFINEMENT")
target_link_libraries(${PROJECT_NAME} MeshRefinement Sofa.Component.Topology.Container.Dynamic)

if(SofaCUDA_FOUND AND MeshRefinement.CUDA_FOUND)
    target_link_libraries(${PROJECT_NAME} SofaCUDA MeshRefinement.CUDA)
endif()

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)

if(SOFA_BUILD_TESTS)
    add_subdirectory(tests)
endif()
