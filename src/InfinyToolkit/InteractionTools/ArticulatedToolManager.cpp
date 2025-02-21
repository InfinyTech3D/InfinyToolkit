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
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyModifier.h>

#include <sofa/component/collision/geometry/SphereModel.h>


namespace sofa::infinytoolkit
{

using namespace defaulttype;
using namespace sofa::core::topology;
using namespace sofa::component::topology::container::dynamic;

typedef sofa::core::behavior::MechanicalState< sofa::defaulttype::Vec3Types > mechaState;
using SphereModel = sofa::component::collision::geometry::SphereCollisionModel< sofa::defaulttype::Vec3Types >;

using sofa::core::objectmodel::KeypressedEvent;
using sofa::core::objectmodel::KeyreleasedEvent;
using TetraID = sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer::TetraID;


void registerArticulatedToolManager(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Tool manager to control several jaw models")
        .add< ArticulatedToolManager >());
}


ArticulatedToolManager::ArticulatedToolManager()
    : l_jawModel1(initLink("jawModel1", "link to the first jaw model component, if not set will search through graph and take first one encountered."))
    , l_jawModel2(initLink("jawModel2", "link to the second jaw model component, if not set will search through graph and take second one encountered."))
    , l_detectionNP(initLink("detectionNP", "link to the second jaw model component, if not set will search through graph and take second one encountered."))
    , l_targetModel(initLink("targetModel", "link to the second jaw model component, if not set will search through graph and take second one encountered."))
    , d_handleFactor(initData(&d_handleFactor, SReal(1.0), "handleFactor", "jaw speed factor."))
    , d_outputPositions(initData(&d_outputPositions, "outputPositions", "jaw positions."))
    , d_isCutter(initData(&d_isCutter, false, "isCutter", "if true, will draw slices BB, ray and intersected triangles"))
    , d_cutMaxStep(initData(&d_cutMaxStep, int(10), "cutMaxStep", "number of step before really cutting"))
    , d_cutMode(initData(&d_cutMode, int(0), "cutMode", "mode of cut (debug)"))
    , d_isControlled(initData(&d_isControlled, false, "isControlled", "if true, will draw slices BB, ray and intersected triangles"))
    , d_drawContacts(initData(&d_drawContacts, false, "drawContacts", "if true, will draw slices BB, ray and intersected triangles"))
    , d_manageBurning(initData(&d_manageBurning, false, "manageBurning", "if true, will draw slices BB, ray and intersected triangles"))
    , m_vtexcoords(initData(&m_vtexcoords, "texcoords", "coordinates of the texture"))
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

    l_jawModel1->setTargetModel(l_targetModel.get());
    l_jawModel2->setTargetModel(l_targetModel.get());


    m_cutCount = 0;

    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}


void ArticulatedToolManager::bwdInit()
{
    if (d_manageBurning.getValue())
    {
        TetrahedronSetTopologyContainer* tetraCon;
        l_targetModel->getContext()->get(tetraCon);

        m_vtexcoords.createTopologyHandler(tetraCon);
    
        helper::WriteAccessor< Data<VecTexCoord> > texcoords = m_vtexcoords;
        texcoords.resize(tetraCon->getNbPoints());
    }
}


int ArticulatedToolManager::testModels()
{
    if (m_jawModel1 == nullptr)
        return -20;

    if (m_jawModel2 == nullptr)
        return -21;

    return 52;
}


bool ArticulatedToolManager::computeBoundingBox()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return false;

    m_jawModel1->computeBoundingBox();
    m_jawModel2->computeBoundingBox();

    return true;
}


bool ArticulatedToolManager::deActivateTool()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return false;

    m_jawModel1->activateTool(false);
    m_jawModel2->activateTool(false);
    
    return true;
}


bool ArticulatedToolManager::activateTool()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return false;

    m_jawModel1->activateTool(true);
    m_jawModel2->activateTool(true);

    return true;
}


int ArticulatedToolManager::performAction()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return -1;

    filterCollision();

    int nbrContacts = (int)(m_jawModel1->getContacts().size() + m_jawModel2->getContacts().size());

    m_jawModel1->performAction();
    m_jawModel2->performAction();

    return nbrContacts;
}


bool ArticulatedToolManager::stopAction()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return false;

    m_jawModel1->stopAction();
    m_jawModel2->stopAction();

    return true;
}


bool ArticulatedToolManager::performSecondaryAction()
{
    m_performCut = true;
    return true;
}


 void ArticulatedToolManager::performCut()
{
    //msg_warning() << "performSecondaryAction()";
    if (!d_isCutter.getValue())
        return;

    sofa::type::vector<int> idVGrab1 = l_jawModel1.get()->getRawContactModelIds();
    sofa::type::vector<int> idVGrab2 = l_jawModel2.get()->getRawContactModelIds();

    // Detect all tetra on the cut path
    TetrahedronSetTopologyContainer* tetraCon;
    l_targetModel->getContext()->get(tetraCon);
    if (tetraCon == nullptr) {
        msg_info() << "Error: NO tetraCon";
        return;
    }

    std::set<int> idVGrab;
    for (auto id : idVGrab1)
    {
        idVGrab.insert(id);
    }

    for (auto id : idVGrab2)
    {
        idVGrab.insert(id);
    }

    const int cutMax = d_cutMaxStep.getValue();
    float invC = 1.0 / float(cutMax);
    if (m_cutCount < cutMax)
    {

        helper::WriteAccessor< Data<VecTexCoord> > texcoords = m_vtexcoords;
        float coef = float(m_cutCount) * invC;
        for (auto id : idVGrab)
        {
            texcoords[id][0] = coef;
            texcoords[id][1] = coef;
        }
    }

    if (m_cutCount < cutMax * 2) {
        m_cutCount++;
        return;
    }

    m_cutCount = 0;
    
    

    TetrahedronSetTopologyModifier* tetraModif;
    tetraCon->getContext()->get(tetraModif);

    if (tetraModif == nullptr) {
        msg_info() << "Error: NO tetraModif";
        return;
    }

    // First get all tetra that are on the first side
    sofa::type::vector<sofa::core::topology::Topology::TetrahedronID> tetraIds;
    std::map< sofa::core::topology::Topology::TetrahedronID, int> tetraCounter;
    for (auto id : idVGrab)
    {
        const BaseMeshTopology::TetrahedraAroundVertex& tetraAV = tetraCon->getTetrahedraAroundVertex(id);
        for (int j = 0; j < tetraAV.size(); ++j)
        {
            int tetraId = tetraAV[j];

            auto elem = tetraCounter.find(tetraId);
            if (elem == tetraCounter.end()) // first time
            {
                tetraCounter[tetraId] = 1;
            }
            else
            {
                tetraCounter[tetraId] = elem->second + 1;
            }
        }
    }

    if (d_cutMode.getValue() == 0)
    {
        for (auto elem : tetraCounter)
        {
            if (elem.second > 0)
            {
                tetraIds.push_back(elem.first);
            }
        }
    }
    else
    {
        for (auto elem : tetraCounter)
        {
            if (elem.second > 1)
            {
                tetraIds.push_back(elem.first);
            }
        }
    }
    
    std::cout << "tetra2Rmove: " << tetraIds << std::endl;

    // remove springs first
    stopAction();
    tetraModif->removeTetrahedra(tetraIds);
   

    return;
}

bool ArticulatedToolManager::stopSecondaryAction()
{
    //msg_warning() << "stopSecondaryAction()";
    return true;
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


void ArticulatedToolManager::clearContacts()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    m_jawModel1->clearContacts();
    m_jawModel2->clearContacts();
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

        //dmsg_warning() << "collMod1: " << collMod1->getTypeName() << " -> " << collMod1->getContext()->getName();
        //dmsg_warning() << "collMod2: " << collMod2->getTypeName() << " -> " << collMod2->getContext()->getName();


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
            bool firstJaw = false;

            if (c.elem.first.getCollisionModel() == l_jawModel1.get()->l_jawCollision.get() || c.elem.first.getCollisionModel() == l_jawModel2.get()->l_jawCollision.get()) // first model is a jaw
            {
                info->idTool = c.elem.first.getIndex(); // id of the tool collision model
                if (c.elem.second.getCollisionModel()->getEnumType() == sofa::core::CollisionModel::TRIANGLE_TYPE) // first model is triangle model
                {
                    sofa::Index idTri = c.elem.second.getIndex();
                    info->idsModel = c.elem.second.getCollisionModel()->getCollisionTopology()->getTriangle(idTri);
                }
                else
                {
                    info->idvModel = c.elem.second.getIndex();
                }
                
                if (c.elem.first.getCollisionModel() == l_jawModel1.get()->l_jawCollision.get())
                    firstJaw = true;
            }
            else if (c.elem.second.getCollisionModel() == l_jawModel1.get()->l_jawCollision.get() || c.elem.second.getCollisionModel() == l_jawModel2.get()->l_jawCollision.get()) // second model is a jaw
            {
                info->idTool = c.elem.second.getIndex();
                if (c.elem.first.getCollisionModel()->getEnumType() == sofa::core::CollisionModel::TRIANGLE_TYPE) // first model is triangle model
                {
                    sofa::Index idTri = c.elem.first.getIndex();
                    info->idsModel = c.elem.first.getCollisionModel()->getCollisionTopology()->getTriangle(idTri);
                }
                else
                {
                    info->idvModel = c.elem.first.getIndex();
                }

                if (c.elem.second.getCollisionModel() == l_jawModel1.get()->l_jawCollision.get())
                    firstJaw = true;
            }
            else // not related to this tool
            {
                continue;
            }

            info->normal = c.normal;
            info->dist = c.value;

            //dmsg_info() << j << " contact: " << c.elem.first.getIndex() << " | " << c.elem.second.getIndex()
            //    << " -> " << " pA: " << c.point[0] << " pB: " << c.point[1]
            //    << " | normal: " << c.normal << " d: " << c.value
            //    << " | cDir: " << (c.point[1] - c.point[0]).normalized() << " d: " << (c.point[1] - c.point[0]).norm();

            if (firstJaw)
                l_jawModel1->addContact(info);
            else
                l_jawModel2->addContact(info);
        }
    }
}


void ArticulatedToolManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (m_performCut && sofa::simulation::AnimateEndEvent::checkEventType(event))
    {
        performCut();
        m_performCut = false;
    }


    if (KeypressedEvent::checkEventType(event))
    {
        if (!d_isControlled.getValue())
            return;

        KeypressedEvent *ev = static_cast<KeypressedEvent *>(event);

        switch (ev->getKey())
        {

        case 'J':
        case 'j':
        {
            stopAction();

            //closeTool();

            //filterCollision();

            performAction();
            deActivateTool();
           
            break;
        }
        case 'G':
        case 'g':
        {
            stopAction();
            activateTool();

            break;
        }
        case 'U':
        case 'u':
        {
            performSecondaryAction();
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
        // first jaw
        l_jawModel1->drawImpl(vparams);
        l_jawModel2->drawImpl(vparams);
    }
}

} // namespace sofa::infinytoolkit
