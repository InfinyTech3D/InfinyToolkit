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

#include <InfinyToolkit/CarvingTools/HapticCarvingManager.h>
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
#include <sofa/gui/component/performer/TopologicalChangeManager.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

namespace sofa::infinytoolkit
{

void registerHapticCarvingManager(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Manager handling carving operations between an haptic tool and an object.")
        .add< HapticCarvingManager >());
}


HapticCarvingManager::HapticCarvingManager()
    : AdvancedCarvingManager()
    , m_forceFeedback(nullptr)
{
    this->f_listening.setValue(true);
}


HapticCarvingManager::~HapticCarvingManager()
{

}


void HapticCarvingManager::init()
{
    AdvancedCarvingManager::init();

    // check if forcefeedback
    m_forceFeedback = getContext()->get<sofa::component::haptics::ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
    if (m_forceFeedback) {
        msg_info() << "Forcefeedback found: " << m_forceFeedback->getName();
    }
    else {
        msg_info() << "NO Forcefeedback found: ";
    }
}


void HapticCarvingManager::doCarve()
{
    if (d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    // get the collision output
    const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = l_detectionNP.get()->getDetectionOutputs();
    if (detectionOutputs.size() == 0)
        return;

    sofa::helper::ScopedAdvancedTimer("HapticCarvingElems");

    if (m_forceFeedback)
        m_forceFeedback->setLock(true);

    // loop on the contact to get the one between the CarvingSurface and the CarvingTool collision model
    const ContactVector* contacts = nullptr;
    sofa::core::CollisionModel* toolCollisionModel = l_toolModel.get();
    for (core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); it != detectionOutputs.end(); ++it)
    {
        sofa::core::CollisionModel* collMod1 = it->first.first;
        sofa::core::CollisionModel* collMod2 = it->first.second;
        sofa::core::CollisionModel* targetModel = nullptr;

        if (collMod1 == toolCollisionModel && collMod2->hasTag(sofa::core::objectmodel::Tag("CarvingSurface"))) {
            targetModel = collMod2;
        }
        else if (collMod2 == toolCollisionModel && collMod1->hasTag(sofa::core::objectmodel::Tag("CarvingSurface"))) {
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
        type::vector<sofa::Index> elemsToRemove;

        for (size_t j = 0; j < ncontacts; ++j)
        {
            const ContactVector::value_type& c = (*contacts)[j];

            if (c.value < d_carvingDistance.getValue())
            {
                int triangleIdx = (c.elem.first.getCollisionModel() == toolCollisionModel ? c.elem.second.getIndex() : c.elem.first.getIndex());
                elemsToRemove.push_back(triangleIdx);
            }
        }

        if (!elemsToRemove.empty())
        {
            static sofa::gui::component::performer::TopologicalChangeManager manager;
            nbelems += manager.removeItemsFromCollisionModel(targetModel, elemsToRemove);
        }
    }

    if (m_forceFeedback)
        m_forceFeedback->setLock(false);
}

} // namespace sofa::infinytoolkit
