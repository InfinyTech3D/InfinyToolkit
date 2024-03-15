/*****************************************************************************
 *                 - Copyright (C) - 2020 - InfinyTech3D -                   *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework      *
 *                                                                           *
 * Commercial License Usage:                                                 *
 * Licensees holding valid commercial license from InfinyTech3D may use this *
 * file in accordance with the commercial license agreement provided with    *
 * the Software or, alternatively, in accordance with the terms contained in *
 * a written agreement between you and InfinyTech3D. For further information *
 * on the licensing terms and conditions, contact: contact@infinytech3d.com  *
 *                                                                           *
 * GNU General Public License Usage:                                         *
 * Alternatively, this file may be used under the terms of the GNU General   *
 * Public License version 3. The licenses are as published by the Free       *
 * Software Foundation and appearing in the file LICENSE.GPL3 included in    *
 * the packaging of this file. Please review the following information to    *
 * ensure the GNU General Public License requirements will be met:           *
 * https://www.gnu.org/licenses/gpl-3.0.html.                                *
 *                                                                           *
 * Authors: see Authors.txt                                                  *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
#pragma once

#include <InfinyToolkit/config.h>
#include <sofa/component/haptics/ForceFeedback.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <InfinyToolkit/CarvingTools/AdvancedCarvingManager.h>

namespace sofa::infinytoolkit
{

/// @brief Null force feedback for haptic feedback device
class SOFA_INFINYTOOLKIT_API BruteForceFeedback : public sofa::component::haptics::ForceFeedback
{
public:
    SOFA_CLASS(BruteForceFeedback,sofa::component::haptics::ForceFeedback);
    void init() override;

    void computeForce(SReal x, SReal y, SReal z, SReal u, SReal v, SReal w, SReal q, SReal& fx, SReal& fy, SReal& fz) override;
    void computeWrench(const sofa::defaulttype::SolidTypes<SReal>::Transform &world_H_tool, const sofa::defaulttype::SolidTypes<SReal>::SpatialVector &V_tool_world, sofa::defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world ) override;

    void setCurrentForce(sofa::type::Vec3 _force) { currentForce = _force; }

    Data< double > forceCoef; ///< multiply haptic force by this coef.

protected:
    BruteForceFeedback();
protected:
    sofa::infinytoolkit::AdvancedCarvingManager::SPtr m_ACarving;

    sofa::type::Vec3 currentForce;
};

} // namespace sofa::infinytoolkit
