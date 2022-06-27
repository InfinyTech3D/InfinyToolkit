/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InteractionTools/config.h>
#include <sofa/component/haptics/ForceFeedback.h>
#include <sofa/defaulttype/VecTypes.h>
#include <InteractionTools/CarvingTools/AdvancedCarvingManager.h>

namespace sofa
{

namespace component
{

namespace haptics
{


/// @brief Null force feedback for haptic feedback device
class SOFA_INTERACTIONTOOLS_API BruteForceFeedback : public sofa::component::haptics::ForceFeedback
{
public:
    SOFA_CLASS(BruteForceFeedback,sofa::component::haptics::ForceFeedback);
    void init() override;

    void computeForce(SReal x, SReal y, SReal z, SReal u, SReal v, SReal w, SReal q, SReal& fx, SReal& fy, SReal& fz) override;
    void computeWrench(const sofa::defaulttype::SolidTypes<SReal>::Transform &world_H_tool, const sofa::defaulttype::SolidTypes<SReal>::SpatialVector &V_tool_world, sofa::defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world ) override;

    void setCurrentForce(sofa::type::Vector3 _force) { currentForce = _force; }

    Data< double > forceCoef; ///< multiply haptic force by this coef.

protected:
    BruteForceFeedback();
protected:
    sofa::component::collision::AdvancedCarvingManager::SPtr m_ACarving;

    sofa::type::Vector3 currentForce;
};

} // namespace haptics

} // namespace component

} // namespace sofa
