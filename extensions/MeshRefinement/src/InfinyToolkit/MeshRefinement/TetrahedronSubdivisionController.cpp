/*******************************************************************************
 *                 - Copyright (C) 2019-Present InfinyTech3D -                 *
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
#define SOFA_INFINYTOOLKIT_TETRAHEDRON_SUBDIVISION_CONTROLLER_CPP

#include <InfinyToolkit/MeshRefinement/TetrahedronSubdivisionController.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa::infinytoolkit::meshrefinement
{

using namespace sofa::core::topology;
using namespace sofa::component::topology;

void registerTetrahedronSubdivisionController(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Controller handling subdivision operations on a tetrahedral mesh: cutting along a plane or between two surface triangles, and refining the whole mesh or a given set of tetrahedra.")
        .add< sofa::infinytoolkit::TetrahedronSubdivisionController<sofa::defaulttype::Vec3Types> >());
}

} // namespace sofa::infinytoolkit::meshrefinement

namespace sofa::infinytoolkit
{
template class SOFA_INFINYTOOLKIT_MESHREFINEMENT_API TetrahedronSubdivisionController<sofa::defaulttype::Vec3Types>;

} // namespace sofa::infinytoolkit
