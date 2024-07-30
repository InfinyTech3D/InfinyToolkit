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
    , l_detectionNP(initLink("detectionNP", "link to the second jaw model component, if not set will search through graph and take second one encountered."))
    , l_targetModel(initLink("targetModel", "link to the second jaw model component, if not set will search through graph and take second one encountered."))
    , d_handleFactor(initData(&d_handleFactor, SReal(1.0), "handleFactor", "jaw speed factor."))
    , d_outputPositions(initData(&d_outputPositions, "outputPositions", "jaw positions."))
    , m_stiffness(500)
    , d_drawContacts(initData(&d_drawContacts, false, "drawContacts", "if true, will draw slices BB, ray and intersected triangles"))
{
    this->f_listening.setValue(true);
    m_idgrabed.clear();
}


void ArticulatedToolManager::init()
{
    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Loading);

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
        msg_error() << "Error mechanical state not found";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    if (l_targetModel.get() == nullptr)
    {
        msg_error() << "Error no target mechanical state found";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }


    // If no NarrowPhaseDetection is set using the link try to find the component
    if (l_detectionNP.get() == nullptr)
    {
        l_detectionNP.set(getContext()->get<core::collision::NarrowPhaseDetection>());
    }

    if (l_detectionNP.get() == nullptr) {
        msg_error() << "NarrowPhaseDetection not found";
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


void ArticulatedToolManager::filterCollision()
{
    if (!this->isComponentStateValid())
        return;

    clearContacts();

    // get the collision output
    const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = l_detectionNP.get()->getDetectionOutputs();
    if (detectionOutputs.size() == 0) // exit if no collision
    {
        return;
    }

    // loop on the contact to get the one between the CarvingSurface and the CarvingTool collision model
    const ContactVector* contacts = nullptr;
    for (core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); it != detectionOutputs.end(); ++it)
    {
        sofa::core::CollisionModel* collMod1 = it->first.first;
        sofa::core::CollisionModel* collMod2 = it->first.second;

        dmsg_info() << "collMod1: " << collMod1->getTypeName() << " -> " << collMod1->getContext()->getName();
        dmsg_info() << "collMod1: " << collMod2->getTypeName() << " -> " << collMod2->getContext()->getName();


        // Get the number of contacts        
        contacts = dynamic_cast<const ContactVector*>(it->second);
        if (contacts == nullptr)
            continue;

        size_t ncontacts = contacts->size();
        if (contacts->size() == 0)
            continue;

        for (size_t j = 0; j < ncontacts; ++j)
        {
            // update the triangle id if a mapping is present
            GrabContactInfo* info = new GrabContactInfo();

            const ContactVector::value_type& c = (*contacts)[j];

            if (c.elem.first.getCollisionModel()->getEnumType() == sofa::core::CollisionModel::TRIANGLE_TYPE) // first model is target model
            {
                info->idTool = c.elem.second.getIndex();
                sofa::Index idTri = c.elem.first.getIndex();
                info->idsModel = c.elem.first.getCollisionModel()->getCollisionTopology()->getTriangle(idTri);

                if (c.elem.second.getCollisionModel() == l_jawModel1.get()->l_jawCollision.get())
                {
                    std::cout << "elem.second: Jaw1" << std::endl;
                    info->toolId = 0;
                }
                else
                {
                    std::cout << "elem.second: Jaw2" << std::endl;
                    info->toolId = 1;
                }
            }
            else
            {

                info->idTool = c.elem.first.getIndex();
                sofa::Index idTri = c.elem.second.getIndex();
                info->idsModel = c.elem.second.getCollisionModel()->getCollisionTopology()->getTriangle(idTri);

                if (c.elem.first.getCollisionModel() == l_jawModel1.get()->l_jawCollision.get())
                {
                    std::cout << "elem.first: Jaw1" << std::endl;
                    info->toolId = 0;
                }
                else
                {
                    std::cout << "elem.first: Jaw2" << std::endl;
                    info->toolId = 1;
                }
            }

            info->normal = c.normal;
            info->dist = c.value;
            std::cout << "Type first: " << c.elem.first.getCollisionModel()->getEnumType() << std::endl;
            std::cout << "Type second: " << c.elem.second.getCollisionModel()->getEnumType() << std::endl;


            dmsg_info() << j << " contact: " << c.elem.first.getIndex() << " | " << c.elem.second.getIndex()
                << " -> " << " pA: " << c.point[0] << " pB: " << c.point[1]
                << " | normal: " << c.normal << " d: " << c.value
                << " | cDir: " << (c.point[1] - c.point[0]).normalized() << " d: " << (c.point[1] - c.point[0]).norm();


            m_contactInfos.push_back(info);
        }
    }
}


void ArticulatedToolManager::clearContacts()
{
    for (unsigned int i = 0; i < m_contactInfos.size(); i++)
    {
        delete m_contactInfos[i];
        m_contactInfos[i] = nullptr;
    }
    m_contactInfos.clear();
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

            //computeVertexIdsInBroadPhase();
            //closeTool();

            filterCollision();
           
            break;
        }
        case 'G':
        case 'g':
        {
            openTool();
            break;
        }
        case '0':
        {
            //releaseGrab();
            computeBoundingBox();
            break;
        }
        case '8': // Up
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(0, 0.1, 0);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(0, 0.1, 0);
            break;
        }
        case '5': // Down
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(0, -0.1, 0);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(0, -0.1, 0);
            break;
        }
        case '4': // Left
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(-0.1, 0, 0);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(-0.1, 0, 0);
            break;
        }
        case '6': // Right
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(0.1, 0, 0);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(0.1, 0, 0);
            break;
        }
        case '+': // forward
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(0.0, 0, -0.1);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(0.0, 0, -0.1);
            break;
        }
        case '-': // backward
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(0.0, 0, 0.1);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(0.0, 0, 0.1);
            break;
        }
        case '9': // turn right
        {
            l_jawModel1.get()->l_jawController.get()->applyRotation(0.0, 1.0, 0.0);
            l_jawModel2.get()->l_jawController.get()->applyRotation(0.0, 1.0, 0.0);
            break;
        }
        case '7': // turn left
        {
            l_jawModel1.get()->l_jawController.get()->applyRotation(0.0, -1.0, 0.0);
            l_jawModel2.get()->l_jawController.get()->applyRotation(0.0, -1.0, 0.0);
            break;
        }
        case '1': // turn right
        {
            l_jawModel1.get()->l_jawController.get()->applyRotation(1.0, 0.0, 0.0);
            l_jawModel2.get()->l_jawController.get()->applyRotation(1.0, 0.0, 0.0);
            break;
        }
        case '3': // turn left
        {
            l_jawModel1.get()->l_jawController.get()->applyRotation(-1.0, 0.0, 0.0);
            l_jawModel2.get()->l_jawController.get()->applyRotation(-1.0, 0.0, 0.0);
            break;
        }
        case 'Y':
        case 'y':
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(0, -0.1, 0);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(0, 0.1, 0);
            break;
        }
        case 'H':
        case 'h':
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(0, 0.1, 0);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(0, -0.1, 0);
            break;
        }
        case 'J':
        case 'j':
        {
            l_jawModel1.get()->l_jawController.get()->applyTranslation(0, 0.1, 0);
            l_jawModel2.get()->l_jawController.get()->applyTranslation(0, 0.1, 0);
            break;
        }
        }
    }
}

void ArticulatedToolManager::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    if (!this->isComponentStateValid())
        return;


    auto m_model = l_targetModel.get();
    auto m_jaw1 = l_jawModel1.get()->l_jawDofs;
    auto m_jaw2 = l_jawModel2.get()->l_jawDofs;

    if (d_drawContacts.getValue())
    {
        for (GrabContactInfo* cInfo : m_contactInfos)
        {
            std::vector<Vec3> vertices;

            for (int i = 0; i < 3; ++i)
            {
                if (cInfo->toolId == 0)
                    vertices.push_back(Vec3(m_jaw1->getPX(cInfo->idTool), m_jaw1->getPY(cInfo->idTool), m_jaw1->getPZ(cInfo->idTool)));
                else
                    vertices.push_back(Vec3(m_jaw2->getPX(cInfo->idTool), m_jaw2->getPY(cInfo->idTool), m_jaw2->getPZ(cInfo->idTool)));

                vertices.push_back(Vec3(m_model->getPX(cInfo->idsModel[i]), m_model->getPY(cInfo->idsModel[i]), m_model->getPZ(cInfo->idsModel[i])));
            }

            sofa::type::RGBAColor color4(1.0f, 1.0, 0.0f, 1.0);

            vparams->drawTool()->drawLines(vertices, 10, color4);
        }
    }

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
