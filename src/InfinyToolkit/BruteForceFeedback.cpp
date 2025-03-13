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

#include <InfinyToolkit/BruteForceFeedback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/Node.h>

using namespace std;

namespace sofa::infinytoolkit
{

void registerBruteForceFeedback(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Null force feedback for haptic feedback device")
        .add< BruteForceFeedback >());
}

using namespace sofa::type;

BruteForceFeedback::BruteForceFeedback()
    : forceCoef(initData(&forceCoef, 0.03, "forceCoef", "multiply haptic force by this coef."))
    , m_ACarving(NULL)
{

}

void BruteForceFeedback::init()
{
    this->ForceFeedback::init();
    currentForce = sofa::type::Vec3(0, 0, 0);

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_ACarving = context->get<sofa::infinytoolkit::AdvancedCarvingManager>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_ACarving == NULL)
    {
        msg_error() << "AdvanceCarvingManager not found";
    }
};

void BruteForceFeedback::computeForce(SReal x, SReal y, SReal z, SReal /*u*/, SReal /*v*/, SReal /*w*/, SReal /*q*/, SReal& fx, SReal& fy, SReal& fz)
{
    SOFA_UNUSED(x);
    SOFA_UNUSED(y);
    SOFA_UNUSED(z);

    const SReal& fCoef = forceCoef.getValue();
    //Vec3 position = Vec3(x, y, z);
    currentForce = Vec3(0, 0, 0);
    if (m_ACarving)
    {
        //currentForce = m_ACarving->computeForceFeedBack(position);
    }

    fx = currentForce[0] * fCoef;
    fy = currentForce[1] * fCoef;
    fz = currentForce[2] * fCoef;
};

void BruteForceFeedback::computeWrench(const sofa::defaulttype::SolidTypes<SReal>::Transform &/*world_H_tool*/, const sofa::defaulttype::SolidTypes<SReal>::SpatialVector &/*V_tool_world*/, sofa::defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world )
{
    W_tool_world.clear();
};


} // namespace sofa::infinytoolkit
