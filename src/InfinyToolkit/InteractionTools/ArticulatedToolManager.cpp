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

#include <InfinyToolkit/InteractionTools/ArticulatedToolManager.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/accessor.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyModifier.h>

#include <sofa/component/collision/geometry/SphereModel.h>


namespace sofa::infinytoolkit
{

SOFA_DECL_CLASS(ArticulatedToolManager)

using namespace defaulttype;
using namespace sofa::core::topology;
using namespace sofa::component::topology::container::dynamic;

typedef sofa::core::behavior::MechanicalState< sofa::defaulttype::Vec3Types > mechaState;
using SphereModel = sofa::component::collision::geometry::SphereCollisionModel< sofa::defaulttype::Vec3Types >;

using sofa::core::objectmodel::KeypressedEvent;
using sofa::core::objectmodel::KeyreleasedEvent;
using TetraID = sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer::TetraID;


int ArticulatedToolManagerClass = core::RegisterObject("Handle sleeve Pince.")
        .add< ArticulatedToolManager >();


ArticulatedToolManager::ArticulatedToolManager()
    : l_jawModel1(initLink("jawModel1", "link to the first jaw model component, if not set will search through graph and take first one encountered."))
    , l_jawModel2(initLink("jawModel2", "link to the second jaw model component, if not set will search through graph and take second one encountered."))
    , d_handleFactor(initData(&d_handleFactor, SReal(1.0), "handleFactor", "jaw speed factor."))
    , d_outputPositions(initData(&d_outputPositions, "outputPositions", "jaw positions."))
    , m_stiffness(500)
{
    this->f_listening.setValue(true);
    m_idgrabed.clear();
}


void ArticulatedToolManager::init()
{
    if (l_jawModel1.get() == nullptr || l_jawModel2.get() == nullptr)
    {
        std::vector<BaseJawModel*> jawModels;
        this->getContext()->get<BaseJawModel>(&jawModels, core::objectmodel::BaseContext::SearchRoot);
    
        if (jawModels.size() != 2)
        {
            msg_error() << "No jaw models given as inputs nor found inside the scene. Please provide them using the links jawModel1 and jawModel2.";
            sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
            return;
        }

        m_jawModel1 = jawModels[0];
        m_jawModel2 = jawModels[1];
    }
    else
    {
        m_jawModel1 = l_jawModel1.get();
        m_jawModel2 = l_jawModel2.get();
    }
    

    if (m_jawModel1 == nullptr || m_jawModel2 == nullptr)
    {
        msg_error() << "error mechanical state not found";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    const RigidCoord& position = d_inputPosition.getValue();

    sofa::helper::WriteAccessor <Data< type::vector<RigidCoord> > > my_positions = d_outputPositions;
    my_positions.resize(3);
    my_positions[0] = position;
    my_positions[1] = position;
    my_positions[2] = position;

    computeBoundingBox();

    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}


int ArticulatedToolManager::testModels()
{
    if (m_jawModel1 == nullptr)
        return -20;

    if (m_jawModel2 == nullptr)
        return -21;

    return 52;
}


void ArticulatedToolManager::computeBoundingBox()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    m_jawModel1->computeBoundingBox();
    m_jawModel2->computeBoundingBox();
}


void ArticulatedToolManager::computeVertexIdsInBroadPhase(float margin)
{
    // First compute boundingbox
    computeBoundingBox();    

    //if (m_model == nullptr)
    //    return;

    //// Add to m_idBroadPhase all model vertices inside the BB
    //m_idBroadPhase.clear();
    //for (Index i = 0; i < m_model->getSize(); i++)
    //{
    //    SReal x = m_model->getPX(i);
    //    SReal y = m_model->getPY(i);
    //    SReal z = m_model->getPZ(i);
    //    //if (x > m_min[0] - margin && x < m_max[0] + margin
    //    //    && y > m_min[1] - margin && y < m_max[1] + margin
    //    //    && z > m_min[2] - margin && z < m_max[2] + margin)
    //    //{
    //    //    m_idBroadPhase.push_back(i);
    //    //}
    //}
}


void ArticulatedToolManager::unactiveTool()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    m_jawModel1->activeTool(false);
    m_jawModel2->activeTool(false);
}


void ArticulatedToolManager::reactiveTool()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    m_jawModel1->activeTool(true);
    m_jawModel2->activeTool(true);
}

void ArticulatedToolManager::performAction()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    m_jawModel1->performAction();
    m_jawModel2->performAction();
}


void ArticulatedToolManager::stopAction()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

}



void ArticulatedToolManager::openTool()
{
    helper::WriteAccessor<Data<SReal>> jawAngle1 = d_angleJaw1;
    helper::WriteAccessor<Data<SReal>> jawAngle2 = d_angleJaw2;
    const SReal& factor = d_handleFactor.getValue();

    jawAngle1 += factor;
    jawAngle2 -= factor;

    if (jawAngle1 < 0.1)
        stopAction();
}


void ArticulatedToolManager::closeTool()
{
    helper::WriteAccessor<Data<SReal>> jawAngle1 = d_angleJaw1;
    helper::WriteAccessor<Data<SReal>> jawAngle2 = d_angleJaw2;
    const SReal& factor = d_handleFactor.getValue();

    jawAngle1 -= factor;
    jawAngle2 += factor;

    if (jawAngle1 < 0.1)
        performAction();
}


void ArticulatedToolManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (KeypressedEvent::checkEventType(event))
    {
        KeypressedEvent *ev = static_cast<KeypressedEvent *>(event);

        switch (ev->getKey())
        {

        case 'T':
        case 't':
        {
            //releaseGrab();

            computeVertexIdsInBroadPhase();
            closeTool();
           
            break;
        }
        case 'G':
        case 'g':
        {
            openTool();
            break;
        }
        case 'Y':
        case 'y':
        {
            //m_mord1->applyTranslation(0, -0.1, 0);
            //m_mord2->applyTranslation(0, 0.1, 0);
            break;
        }
        case 'H':
        case 'h':
        {
            //m_mord1->applyTranslation(0, 0.1, 0);
            //m_mord2->applyTranslation(0, -0.1, 0);
            break;
        }
        case 'J':
        case 'j':
        {
            //m_mord1->applyTranslation(0, 0.1, 0);
            //m_mord2->applyTranslation(0, 0.1, 0);
            break;
        }
        }
    }
}

void ArticulatedToolManager::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    //sofa::type::RGBAColor color(0.2f, 1.0f, 1.0f, 1.0f);
    //vparams->drawTool()->drawLine(m_min, m_max, sofa::type::RGBAColor(1.0, 0.0, 1.0, 1.0));
    //
    //vparams->drawTool()->drawLine(zero, xAxis, sofa::type::RGBAColor(1.0, 0.0, 0.0, 0.0));
    //vparams->drawTool()->drawLine(zero, yAxis, sofa::type::RGBAColor(0.0, 1.0, 0.0, 0.0));
    //vparams->drawTool()->drawLine(zero, zAxis, sofa::type::RGBAColor(0.0, 0.0, 1.0, 0.0));

    //if (m_model == nullptr)
    //    return;

    //for (unsigned int i = 0; i < m_idgrabed.size(); i++)
    //{
    //    SReal x = m_model->getPX(m_idgrabed[i]);
    //    SReal y = m_model->getPY(m_idgrabed[i]);
    //    SReal z = m_model->getPZ(m_idgrabed[i]);
    //    vparams->drawTool()->drawPoint(Vec3(x, y, z), sofa::type::RGBAColor(255.0, 0.0, 0.0, 1.0));
    //}


    //TetrahedronSetTopologyContainer* tetraCon;
    //m_model->getContext()->get(tetraCon);
    //if (tetraCon == nullptr) {
    //    msg_info() << "Error: NO tetraCon";
    //    return;
    //}

    //for (unsigned int i = 0; i < tetraIdsOnCut.size(); i++)
    //{
    //    const BaseMeshTopology::Tetra& tetra = tetraCon->getTetra(tetraIdsOnCut[i]);
    //    
    //    Vec3 p0 = Vec3(m_model->getPX(tetra[0]), m_model->getPY(tetra[0]), m_model->getPZ(tetra[0]));
    //    Vec3 p1 = Vec3(m_model->getPX(tetra[1]), m_model->getPY(tetra[1]), m_model->getPZ(tetra[1]));
    //    Vec3 p2 = Vec3(m_model->getPX(tetra[2]), m_model->getPY(tetra[2]), m_model->getPZ(tetra[2]));
    //    Vec3 p3 = Vec3(m_model->getPX(tetra[3]), m_model->getPY(tetra[3]), m_model->getPZ(tetra[3]));

    //    vparams->drawTool()->drawTetrahedron(p0, p1, p2, p3, color);
    //}
}

} // namespace sofa::infinytoolkit
