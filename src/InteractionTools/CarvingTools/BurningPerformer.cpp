/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/CarvingTools/BurningPerformer.h>
#include <InteractionTools/CarvingTools/AdvancedCarvingManager.h>

#include <SofaBaseMechanics/MechanicalObject.h>

namespace sofa::component::controller
{

BurningPerformer::BurningPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr)
    : BaseCarvingPerformer(topo, _carvingMgr)
{

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
    helper::WriteAccessor< Data<VecTexCoord> > texcoords = m_carvingMgr->m_vtexcoords;

    const SReal& _refineDistance = m_carvingMgr->d_refineDistance.getValue();
    const SReal invRefDistance = 1 / _refineDistance;

    for each (contactInfo * cInfo in m_pointContacts)
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
 
}

} // namespace sofa::component::controller
