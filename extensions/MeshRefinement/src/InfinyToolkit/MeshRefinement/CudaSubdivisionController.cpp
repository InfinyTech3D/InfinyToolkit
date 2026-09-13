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

/// Instantiates and registers the controller for the Cuda vector types.
/// The matching MeshRefinementAPI instantiations are provided by MeshRefinement.CUDA,
/// Only compiled when both SofaCUDA and MeshRefinement.CUDA are found.

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
