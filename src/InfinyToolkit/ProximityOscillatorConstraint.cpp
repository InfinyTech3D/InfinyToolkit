/*****************************************************************************
 *                - Copyright (C) 2020-Present InfinyTech3D -                *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework.     *
 *                                                                           *
 * This file is dual-licensed:                                               *
 *                                                                           *
 * 1) Commercial License:                                                    *
 *      This file may be used under the terms of a valid commercial license  *
 *      agreement provided wih the software by InfinyTech3D.                 *
 *                                                                           *
 * 2) GNU General Public License (GPLv3) Usage                               *
 *      Alternatively, this file may be used under the terms of the          *
 *      GNU General Public License version 3 as published by the             *
 *      Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  *
 *                                                                           *
 * Contact: contact@infinytech3d.com                                         *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
#define SOFA_COMPONENT_FORCEFIELD_ProximityOscillatorConstraint_CPP

#include <sofa/core/ObjectFactory.h>
#include <InfinyToolkit/ProximityOscillatorConstraint.inl>

namespace sofa::infinytoolkit
{

using namespace sofa::defaulttype;

void registerProximityOscillatorConstraint(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Middle interpolated force applied to given degrees of freedom.")
        .add< ProximityOscillatorConstraint<Vec3Types> >());
}

template class SOFA_INFINYTOOLKIT_API ProximityOscillatorConstraint<Vec3Types>;

} // namespace sofa::infinytoolkit
