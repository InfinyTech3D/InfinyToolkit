/*****************************************************************************
 *                - Copyright (C) 2020-Present InfinyTech3D -                *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework.     *
 *                                                                           *
 * This file is dual-licensed:                                               *
 *                                                                           *
 * 1) Commercial License:                                                    *
 *      This file may be used under the terms of a valid commercial license  *
 *      agreement provided wih the software by InfinyTech3D.                 *
 *                                                                           *
 * 2) GNU General Public License (GPLv3) Usage                               *
 *      Alternatively, this file may be used under the terms of the          *
 *      GNU General Public License version 3 as published by the             *
 *      Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  *
 *                                                                           *
 * Contact: contact@infinytech3d.com                                         *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
#include <InfinyToolkit/CarvingTools/SimpleCarvingPerformer.h>
#include <InfinyToolkit/CarvingTools/AdvancedCarvingManager.h>

#include <sofa/component/statecontainer/MechanicalObject.h>

namespace sofa::infinytoolkit
{

SimpleCarvingPerformer::SimpleCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr)
    : BaseCarvingPerformer(topo, _carvingMgr)
{
    m_performerType = "SimpleCarvingPerformer";
}


bool SimpleCarvingPerformer::initPerformer()
{
    m_topoModif = m_topologyCon->getContext()->get<TetrahedronSetTopologyModifier>();

    if (m_topoModif == nullptr) {
        msg_error("SimpleCarvingPerformer") << "InitPerformer failed, no TetrahedronSetTopologyModifier found in Node: " << m_topologyCon->getContext()->getName();
        return false;
    }
    
    return true;
}


bool SimpleCarvingPerformer::runPerformer()
{
    if (m_pointContacts.empty() && m_triangleContacts.empty())
        return true;

    m_tetra2Remove.clear();
    const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();

    std::set<Index> tetraIds;
    // check points first
    for (const contactInfo* cInfo : m_pointContacts)
    {
        if (cInfo->dist<= _carvingDistance)
        {
            const TetrahedronSetTopologyContainer::TetrahedraAroundVertex& tetraAV = m_topologyCon->getTetrahedraAroundVertex(cInfo->elemId);
            for (Index tetraId : tetraAV)
            {
                tetraIds.insert(tetraId);
            }
        }
    }

    // check triangles
    for (const contactInfo* cInfo : m_triangleContacts)
    {
        if (cInfo->dist <= _carvingDistance)
        {
            const TetrahedronSetTopologyContainer::TetrahedraAroundTriangle& tetraAT = m_topologyCon->getTetrahedraAroundTriangle(cInfo->elemId);
            for (Index tetraId : tetraAT)
            {
                tetraIds.insert(tetraId);
            }
        }
    }

    // convert to vector
    for (auto tetraId : tetraIds) {
        m_tetra2Remove.push_back(tetraId);
    }

    if (!m_tetra2Remove.empty())
        m_topoModif->removeTetrahedra(m_tetra2Remove);
 
    return true;
}

} // namespace sofa::infinytoolkit
