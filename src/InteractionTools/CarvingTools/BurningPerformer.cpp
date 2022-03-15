/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/CarvingTools/BurningPerformer.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <SofaBaseMechanics/MechanicalObject.h>

namespace sofa::component::controller
{

BurningPerformer::BurningPerformer(TetrahedronSetTopologyContainer::SPtr topo, const SReal& burningDistance)
    : BaseCarvingPerformer(topo, burningDistance, burningDistance)
    , m_vtexcoords(initData(&m_vtexcoords, "texcoords", "coordinates of the texture"))
{

}


bool BurningPerformer::initPerformer()
{
    std::cout << "BurningPerformer::initPerformer: " << m_topologyCon->getName() << std::endl;

    //if (m_surfaceCollisionModels.size() != 1)
    //{
    //    msg_error() << "textcoord handling is only possible with only one target model for the moment. Found: " << m_surfaceCollisionModels.size();
    //    m_texCoordsHandling = false;
    //    return;
    //}
    
    m_vtexcoords.createTopologyHandler(m_topologyCon.get());
    helper::WriteAccessor< Data<VecTexCoord> > texcoords = m_vtexcoords;
    texcoords.resize(m_topologyCon->getNbPoints());

    return true;
}


bool BurningPerformer::runPerformer()
{
    std::cout << "BurningPerformer::runPerformer" << std::endl;
    helper::WriteAccessor< Data<VecTexCoord> > texcoords = m_vtexcoords;
    SReal invRefDistance = 1 / m_carvingDistance;

    for each (contactInfo * cInfo in m_pointContacts)
    {
        SReal dist = (cInfo->pointB - cInfo->pointA).norm();
        if (dist > m_carvingDistance)
            continue;

        float coef = float((m_carvingDistance - dist) * invRefDistance);
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
