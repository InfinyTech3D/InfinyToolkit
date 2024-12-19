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

#include <InfinyToolkit/RotationEngine.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/Node.h>

namespace sofa::infinytoolkit
{

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
    msg_info() << "mstates: " << m_mstates.size();
    
    sofa::simulation::Node::SPtr rootNode = static_cast<simulation::Node*>(this->getContext()->getRootContext());
    sofa::type::vector<sofa::core::objectmodel::BaseNode* > childNodes = rootNode->getChildren();

    for (unsigned int i = 0; i < childNodes.size(); ++i)
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
    sofa::type::vector<sofa::core::objectmodel::BaseNode* > childNodes = node->getChildren();
    for (unsigned int i = 0; i < childNodes.size(); ++i)
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
    for (unsigned int i = 0; i < m_mstates.size(); i++)
    {
        MechanicalObject3* state = m_mstates[i];
        sofa::type::Vec3 rot = state->getRotation();

        state->setRotation(rot[0] + 1, rot[1], rot[2]);
        state->reinit();
        rot = state->getRotation();
    }
}


void RotationEngine::process()
{
    for (unsigned int i = 0; i < m_mstates.size(); i++)
    {
        MechanicalObject3* state = m_mstates[i];       
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
            for (unsigned int i = 0; i < m_mstates.size(); i++)
            {
                MechanicalObject3* state = m_mstates[i];
                //sofa::type::Vec3 rot = state->getRotation();

                state->setRotation(0, 0, 0.5);
                state->reinit();
            }
        }
        else if (ke->getKey() == 'L') {
            for (unsigned int i = 0; i < m_mstates.size(); i++)
            {
                MechanicalObject3* state = m_mstates[i];
                //sofa::type::Vec3 rot = state->getRotation();

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

} // namespace sofa::infinytoolkit
