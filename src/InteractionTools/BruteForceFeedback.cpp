/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/BruteForceFeedback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/Node.h>

using namespace std;

namespace sofa
{
namespace component
{
namespace controller
{

BruteForceFeedback::BruteForceFeedback()
    : forceCoef(initData(&forceCoef, 0.03, "forceCoef", "multiply haptic force by this coef."))
    , m_ACarving(NULL)
{

}

void BruteForceFeedback::init()
{
    this->ForceFeedback::init();
    currentForce = sofa::defaulttype::Vector3(0, 0, 0);

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_ACarving = context->get<sofa::component::collision::AdvanceCarvingManager>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_ACarving == NULL)
    {
        msg_error() << "AdvanceCarvingManager not found";
    }
};

void BruteForceFeedback::computeForce(SReal x, SReal y, SReal z, SReal /*u*/, SReal /*v*/, SReal /*w*/, SReal /*q*/, SReal& fx, SReal& fy, SReal& fz)
{
    const SReal& fCoef = forceCoef.getValue();
    defaulttype::Vec3 position = defaulttype::Vec3(x, y, z);
    currentForce = defaulttype::Vec3(0, 0, 0);
    if (m_ACarving)
    {
        currentForce = m_ACarving->computeForceFeedBack(position);
    }

    fx = currentForce[0] * fCoef;
    fy = currentForce[1] * fCoef;
    fz = currentForce[2] * fCoef;
};

void BruteForceFeedback::computeWrench(const sofa::defaulttype::SolidTypes<SReal>::Transform &/*world_H_tool*/, const sofa::defaulttype::SolidTypes<SReal>::SpatialVector &/*V_tool_world*/, sofa::defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world )
{
    W_tool_world.clear();
};

static int nullForceFeedbackClass = sofa::core::RegisterObject("Null force feedback for haptic feedback device")
        .add< BruteForceFeedback >();

} // namespace controller
} // namespace component
} // namespace sofa
