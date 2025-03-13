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

#include <InfinyToolkit/CollisionDetectionDisplay.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <limits>

namespace sofa::infinytoolkit
{

void registerCollisionDetectionDisplay(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Controller to store current collision contacts and draw them.")
        .add< CollisionDetectionDisplay >());
}

CollisionDetectionDisplay::CollisionDetectionDisplay()
    : d_drawContacts(initData(&d_drawContacts, false, "drawContacts", "if true, will draw slices BB, ray and intersected triangles"))
    , d_collisionPoints(initData(&d_collisionPoints, "collisionPoints", "vector of pair of collision points"))
{
    f_listening.setValue(true);
}


CollisionDetectionDisplay::~CollisionDetectionDisplay()
{
    clearContacts();
}

void CollisionDetectionDisplay::init()
{
    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Loading);

    // If no NarrowPhaseDetection is set using the link try to find the component
    if (l_detectionNP.get() == nullptr)
    {
        l_detectionNP.set(getContext()->get<core::collision::NarrowPhaseDetection>());
    }

    if (l_detectionNP.get() == nullptr) {
        msg_error() << "NarrowPhaseDetection not found";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
    }

    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}


void CollisionDetectionDisplay::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    if (simulation::AnimateEndEvent::checkEventType(event))
    {
        filterCollision();
    }
}


void CollisionDetectionDisplay::filterCollision()
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
    auto colPoints = sofa::helper::getWriteAccessor(d_collisionPoints);
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
            const ContactVector::value_type& c = (*contacts)[j];
            // update the triangle id if a mapping is present
            contactInfo* info = new contactInfo();
            
            dmsg_info() << j << " contact: " << c.elem.first.getIndex() << " | " << c.elem.second.getIndex()
                << " -> " << " pA: " << c.point[0] << " pB: " << c.point[1]
                << " | normal: " << c.normal << " d: " << c.value 
                << " | cDir: " << (c.point[1] - c.point[0]).normalized() << " d: " << (c.point[1] - c.point[0]).norm();

            info->idA = c.elem.first.getIndex();
            info->idB = c.elem.second.getIndex();
            info->normal = c.normal;
            info->pointA = c.point[0];
            info->pointB = c.point[1];
            info->dist = c.value;
        
            m_contactInfos.push_back(info);
            colPoints.push_back(info->pointA);
            colPoints.push_back(info->pointB);
        }
    }
}

void CollisionDetectionDisplay::clearContacts()
{
    for (unsigned int i = 0; i < m_contactInfos.size(); i++)
    {
        delete m_contactInfos[i];
        m_contactInfos[i] = nullptr;
    }
    m_contactInfos.clear();

    auto points = sofa::helper::getWriteAccessor(d_collisionPoints);
    points.clear();
}


void CollisionDetectionDisplay::draw(const core::visual::VisualParams* vparams)
{
    if (!this->isComponentStateValid() || !d_drawContacts.getValue())
        return;

    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();
    for (contactInfo* cInfo : m_contactInfos)
    {
        std::vector<Vec3> vertices;
        vertices.push_back(cInfo->pointA);
        vertices.push_back(cInfo->pointB);

        sofa::type::RGBAColor color4(1.0f, 1.0, 0.0f, 1.0);

        vparams->drawTool()->drawLines(vertices, 1, color4);
    }
}


} // namespace sofa::infinytoolkit
