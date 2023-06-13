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
