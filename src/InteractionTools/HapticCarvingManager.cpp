/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <InteractionTools/HapticCarvingManager.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/objectmodel/ScriptEvent.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>

#include <sofa/core/topology/TopologicalMapping.h>
#include <SofaUserInteraction/TopologicalChangeManager.h>
#include <sofa/helper/AdvancedTimer.h>

namespace sofa
{

namespace component
{

namespace collision
{

int HapticCarvingManagerClass = core::RegisterObject("Manager handling carving operations between an haptic tool and an object.")
.add< HapticCarvingManager >()
;

HapticCarvingManager::HapticCarvingManager()
    : CarvingManager()
    , m_forceFeedback(nullptr)
{
    this->f_listening.setValue(true);
}


HapticCarvingManager::~HapticCarvingManager()
{

}


void HapticCarvingManager::init()
{
    CarvingManager::init();

    // check if forcefeedback
    m_forceFeedback = getContext()->get<sofa::component::controller::ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
    if (m_forceFeedback)
        msg_info() << "Forcefeedback found: " << m_forceFeedback->getName();
    else
        msg_info() << "NO Forcefeedback found: ";
}


void HapticCarvingManager::doCarve()
{
    if (!m_carvingReady)
        return;

    // get the collision output
    const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = m_detectionNP->getDetectionOutputs();
    if (detectionOutputs.size() == 0)
        return;

    sofa::helper::ScopedAdvancedTimer("HapticCarvingElems");

    if (m_forceFeedback)
        m_forceFeedback->setLock(true);

    // loop on the contact to get the one between the CarvingSurface and the CarvingTool collision model
    const ContactVector* contacts = NULL;
    for (core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); it != detectionOutputs.end(); ++it)
    {
        sofa::core::CollisionModel* collMod1 = it->first.first;
        sofa::core::CollisionModel* collMod2 = it->first.second;
        sofa::core::CollisionModel* targetModel = nullptr;

        if (collMod1 == m_toolCollisionModel && collMod2->hasTag(sofa::core::objectmodel::Tag("CarvingSurface"))) {
            targetModel = collMod2;
        }
        else if (collMod2 == m_toolCollisionModel && collMod1->hasTag(sofa::core::objectmodel::Tag("CarvingSurface"))) {
            targetModel = collMod1;
        }
        else {
            continue;
        }

        contacts = dynamic_cast<const ContactVector*>(it->second);
        if (contacts == nullptr || contacts->size() == 0) { 
            continue; 
        }

        size_t ncontacts = 0;
        if (contacts != NULL) {
            ncontacts = contacts->size();
        }

        if (ncontacts == 0) {
            continue;
        }

        int nbelems = 0;
        helper::vector<int> elemsToRemove;

        for (size_t j = 0; j < ncontacts; ++j)
        {
            const ContactVector::value_type& c = (*contacts)[j];

            if (c.value < d_carvingDistance.getValue())
            {
                int triangleIdx = (c.elem.first.getCollisionModel() == m_toolCollisionModel ? c.elem.second.getIndex() : c.elem.first.getIndex());
                elemsToRemove.push_back(triangleIdx);
            }
        }

        if (!elemsToRemove.empty())
        {
            static TopologicalChangeManager manager;
            nbelems += manager.removeItemsFromCollisionModel(targetModel, elemsToRemove);
        }
    }

    if (m_forceFeedback)
        m_forceFeedback->setLock(false);
}

} // namespace collision

} // namespace component

} // namespace sofa
