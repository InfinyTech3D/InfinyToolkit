/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2015 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <InteractionTools/RotationEngine.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/Node.h>

namespace sofa
{

namespace component
{

namespace engine
{

    

SOFA_DECL_CLASS(SurfaceCarvingManager)

int RotationEngineClass = core::RegisterObject("RotationEngine test.")
.add< RotationEngine >()
;

RotationEngine::RotationEngine()
{
    this->f_listening.setValue(true);

}

RotationEngine::~RotationEngine()
{
}


void RotationEngine::bwdInit()
{  
    m_toolNode = nullptr;

    getContext()->get<MechanicalObject3>(&m_mstates, core::objectmodel::Tag("head"), core::objectmodel::BaseContext::SearchRoot);
    std::cout << "mstates: " << m_mstates.size() << std::endl;
    
    for (int i = 0; i < m_mstates.size(); i++)
    {
        std::cout << i << " -> " << m_mstates[i]->name << std::endl;
    }
    
    sofa::simulation::Node::SPtr rootNode = static_cast<simulation::Node*>(this->getContext()->getRootContext());
    sofa::helper::vector<sofa::core::objectmodel::BaseNode* > childNodes = rootNode->getChildren();

    for (int i = 0; i < childNodes.size(); ++i)
    {        
        sofa::simulation::Node* node = dynamic_cast<sofa::simulation::Node*>(childNodes[i]);        

        bool res = findNode(node);
        if (res == true)
            break;
    }
}

bool RotationEngine::findNode(sofa::simulation::Node::SPtr node)
{
    // check this node
    if (node->hasTag(sofa::core::objectmodel::Tag("toolCollision")))
    {
        m_toolNode = node;
        return true;
    }

    // check its children
    sofa::helper::vector<sofa::core::objectmodel::BaseNode* > childNodes = node->getChildren();
    for (int i = 0; i < childNodes.size(); ++i)
    {
        sofa::simulation::Node* node = dynamic_cast<sofa::simulation::Node*>(childNodes[i]);
        if (node->hasTag(sofa::core::objectmodel::Tag("toolCollision")))
        {
            m_toolNode = node;
            return true;
        }
    }

    return false;
}


void RotationEngine::init()
{
    addInput(&m_rotation);
}


void RotationEngine::reinit()
{

}


void RotationEngine::doUpdate()
{
    std::cout << "RotationEngine::doUpdate()" << std::endl;
    for (int i = 0; i < m_mstates.size(); i++)
    {
        MechanicalObject3* state = m_mstates[i];
        sofa::defaulttype::Vector3 rot = state->getRotation();
        std::cout << "rot: " << rot << std::endl;

        state->setRotation(rot[0] + 1, rot[1], rot[2]);
        state->reinit();
        rot = state->getRotation();
        std::cout << "rot apres: " << rot << std::endl;
    }
}


void RotationEngine::process()
{
    std::cout << "RotationEngine::process()" << std::endl;
    for (int i = 0; i < m_mstates.size(); i++)
    {
        MechanicalObject3* state = m_mstates[i];
        //sofa::defaulttype::Vector3 rot = state->getRotation();
        //std::cout << "rot: " << rot << std::endl;
        
        state->setRotation(0, 0, 0.5);
        state->reinit();
    }
}

void RotationEngine::handleEvent(sofa::core::objectmodel::Event* event)
{   
    /*if (dynamic_cast<simulation::AnimateEndEvent*>(event))
    {
        if (d_active.getValue()) {
            CheckCollisionDetection();
        }
    }*/

    if (sofa::core::objectmodel::KeypressedEvent::checkEventType(event))
    {
        sofa::core::objectmodel::KeypressedEvent* ke = static_cast<sofa::core::objectmodel::KeypressedEvent*>(event);
        msg_info() << " handleEvent gets character '" << ke->getKey() << "'. ";
        if (ke->getKey() == 'K') 
        {
            for (int i = 0; i < m_mstates.size(); i++)
            {
                MechanicalObject3* state = m_mstates[i];
                //sofa::defaulttype::Vector3 rot = state->getRotation();

                state->setRotation(0, 0, 0.5);
                state->reinit();
            }
        }
        else if (ke->getKey() == 'L') {
            for (int i = 0; i < m_mstates.size(); i++)
            {
                MechanicalObject3* state = m_mstates[i];
                //sofa::defaulttype::Vector3 rot = state->getRotation();

                state->setRotation(0, 0, -0.5);
                state->reinit();
            }
        }
        else if (ke->getKey() == 'G')
        {
            if (m_toolNode != nullptr) {
                m_toolNode->setActive(true);
            }
        }
        else if (ke->getKey() == 'H')
        {
            if (m_toolNode != nullptr) {
                m_toolNode->setActive(false);
            }
        }
    }
}

} // namespace engine

} // namespace component

} // namespace sofa
