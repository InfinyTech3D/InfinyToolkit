/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#define SOFA_COMPONENT_FORCEFIELD_MIDDLEFORCEFIELD_CPP

#include <sofa/core/ObjectFactory.h>
#include <InteractionTools/MiddleForceField.inl>

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

int MiddleForceFieldClass = core::RegisterObject("Middle interpolated force applied to given degrees of freedom")
        .add< MiddleForceField<Vec3Types> >()

        ;
template class SOFA_INTERACTIONTOOLS_API MiddleForceField<Vec3Types>;

} // namespace forcefield

} // namespace component

} // namespace sofa
