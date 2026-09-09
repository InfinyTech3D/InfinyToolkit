/*******************************************************************************
 *                 - Copyright (C) 2026-Present InfinyTech3D -                 *
 *                             All Rights Reserved                             *
 *                                                                             *
 * The contents of this file are confidential and proprietary to InfinyTech3D. *
 * Unauthorized copying, modification, distribution or disclosure of this      *
 * file, via any medium, is strictly prohibited.                               *
 *                                                                             *
 * This file may only be used under the terms and conditions of a valid        *
 * commercial license agreement with InfinyTech3D.                             *
 *                                                                             *
 * For licensing inquiries: contact@infinytech3d.com                           *
 * Further information: https://infinytech3d.com                               *
 ******************************************************************************/

/// The Cuda instantiations of the controller. These used to live in
/// MeshRefinement.CUDA, which cannot host them any more: the controller is here now,
/// and MeshRefinement must not depend on InfinyToolkit. The engine side of the pair -
/// MeshRefinementAPI for the same Cuda types - is still instantiated by
/// MeshRefinement.CUDA, which is the only translation unit that can see the private
/// .inl; this file links those symbols through the facade's public header.
///
/// Only compiled when SofaCUDA is found.

#include <InfinyToolkit/MeshRefinement/TetrahedronSubdivisionController.inl>
#include <sofa/gpu/cuda/CudaTypes.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::infinytoolkit::meshrefinement
{

void registerCudaTetrahedronSubdivisionController(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Controller handling subdivision operations on a tetrahedral mesh, on Cuda vector types.")
        .add< sofa::infinytoolkit::TetrahedronSubdivisionController<sofa::gpu::cuda::CudaVec3fTypes> >()
#ifdef SOFA_GPU_CUDA_DOUBLE
        .add< sofa::infinytoolkit::TetrahedronSubdivisionController<sofa::gpu::cuda::CudaVec3dTypes> >()
#endif
    );
}

} // namespace sofa::infinytoolkit::meshrefinement

namespace sofa::infinytoolkit
{

template class SOFA_INFINYTOOLKIT_MESHREFINEMENT_API TetrahedronSubdivisionController<sofa::gpu::cuda::CudaVec3fTypes>;

#ifdef SOFA_GPU_CUDA_DOUBLE
template class SOFA_INFINYTOOLKIT_MESHREFINEMENT_API TetrahedronSubdivisionController<sofa::gpu::cuda::CudaVec3dTypes>;
#endif

} // namespace sofa::infinytoolkit
