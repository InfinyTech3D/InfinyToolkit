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

    const SReal& _refineDistance = m_carvingMgr->d_refineDistance.getValue();
    const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();

    if (_refineDistance < _carvingDistance)
    {
        msg_warning("BurningPerformer") << "d_refineDistance used for burning should be higher than the d_carvingDistance";
    }

    return true;
}


bool BurningPerformer::runPerformer()
{
    if (m_pointContacts.empty())
        return true;

    helper::WriteAccessor< Data<VecTexCoord> > texcoords = m_carvingMgr->m_vtexcoords;

    const SReal& _refineDistance = m_carvingMgr->d_refineDistance.getValue();
    const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
    const SReal invRefDistance = 1 / (_refineDistance - _carvingDistance);

    for (contactInfo * cInfo : m_pointContacts)
    {
        const SReal& dist = cInfo->dist;// (cInfo->pointB - cInfo->pointA).norm();
        if (dist > _refineDistance)
            continue;

        float coef = 1.f - float((dist - _carvingDistance) * invRefDistance);
        if (coef < 0.f)
            coef = 0.f;
        else if (coef > 1.f)
            coef = 1.f;

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
