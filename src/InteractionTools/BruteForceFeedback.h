/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_INTERACTIONTOOLS_BRUTEFORCEFEEDBACK_H
#define SOFA_INTERACTIONTOOLS_BRUTEFORCEFEEDBACK_H

#include <InteractionTools/config.h>
#include <SofaHaptics/ForceFeedback.h>
#include <sofa/defaulttype/VecTypes.h>
#include <InteractionTools/AdvanceCarvingManager.h>

namespace sofa
{

namespace component
{

namespace controller
{


/// @brief Null force feedback for haptic feedback device
class SOFA_INTERACTIONTOOLS_API BruteForceFeedback : public sofa::component::controller::ForceFeedback
{
public:
    SOFA_CLASS(BruteForceFeedback,sofa::component::controller::ForceFeedback);
    void init() override;

    void computeForce(SReal x, SReal y, SReal z, SReal u, SReal v, SReal w, SReal q, SReal& fx, SReal& fy, SReal& fz) override;
    void computeWrench(const sofa::defaulttype::SolidTypes<SReal>::Transform &world_H_tool, const sofa::defaulttype::SolidTypes<SReal>::SpatialVector &V_tool_world, sofa::defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world ) override;

    void setCurrentForce(sofa::defaulttype::Vector3 _force) { currentForce = _force; }

    Data< double > forceCoef; ///< multiply haptic force by this coef.

protected:
    BruteForceFeedback();
protected:
    sofa::component::collision::AdvanceCarvingManager::SPtr m_ACarving;

    sofa::defaulttype::Vector3 currentForce;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_INTERACTIONTOOLS_BRUTEFORCEFEEDBACK_H
