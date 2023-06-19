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

#include <InfinyToolkit/CarvingTools/BurningPerformer.h>
#include <InfinyToolkit/CarvingTools/AdvancedCarvingManager.h>

#include <sofa/component/statecontainer/MechanicalObject.h>

namespace sofa::infinytoolkit
{

BurningPerformer::BurningPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr)
    : BaseCarvingPerformer(topo, _carvingMgr)
{
    m_performerType = "BurningPerformer";
}


bool BurningPerformer::initPerformer()
{
    m_carvingMgr->m_vtexcoords.createTopologyHandler(m_topologyCon.get());
    
    helper::WriteAccessor< Data<VecTexCoord> > texcoords = m_carvingMgr->m_vtexcoords;
    texcoords.resize(m_topologyCon->getNbPoints());

    return true;
}


bool BurningPerformer::runPerformer()
{
    if (m_pointContacts.empty())
        return true;

    helper::WriteAccessor< Data<VecTexCoord> > texcoords = m_carvingMgr->m_vtexcoords;

    const SReal& _refineDistance = m_carvingMgr->d_refineDistance.getValue();
    const SReal invRefDistance = 1 / _refineDistance;

    for (contactInfo * cInfo : m_pointContacts)
    {
        SReal dist = (cInfo->pointB - cInfo->pointA).norm();
        if (dist > _refineDistance)
            continue;

        float coef = float((_refineDistance - dist) * invRefDistance);
        float& val = texcoords[cInfo->elemId][0];
        if (coef > val)
        {
            val = coef;
            texcoords[cInfo->elemId][1] = coef;
        }
    }

    return true;
}

void BurningPerformer::draw(const core::visual::VisualParams* vparams)
{
    BaseCarvingPerformer::draw(vparams);
}

} // namespace sofa::infinytoolkit
