/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InfinyToolkit/CarvingTools/SurfaceCarvingPerformer.h>
#include <InfinyToolkit/CarvingTools/AdvancedCarvingManager.h>

#include <sofa/component/statecontainer/MechanicalObject.h>

namespace sofa::infinytoolkit
{

SurfaceCarvingPerformer::SurfaceCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr)
    : BaseCarvingPerformer(topo, _carvingMgr)
{

}


bool SurfaceCarvingPerformer::initPerformer()
{
    return true;
}


void SurfaceCarvingPerformer::filterContacts()
{
    m_tetraId2remove.clear();
    m_tetraId2refine.clear();

    const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
    sofa::core::behavior::BaseMechanicalState* mstate = m_topologyCon->getContext()->getMechanicalState();

    // Filter tetra
    for (contactInfo * cInfo : m_pointContacts)
    {
        if (cInfo->dist > _carvingDistance) // only carving points
            continue;

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = m_topologyCon->getTetrahedraAroundVertex(cInfo->elemId);
        for (auto tetraID : tetraAV)
        {
            m_tetraId2remove.insert(tetraID);

            if (m_baryMap.find(tetraID) != m_baryMap.end()) // already in
                continue;

            const BaseMeshTopology::Tetrahedron& tetra = m_topologyCon->getTetrahedron(tetraID);

            Vec3 bary = Vec3(0, 0, 0);
            for (unsigned int k = 0; k < 4; ++k)
            {
                bary[0] += mstate->getPX(tetra[k]);
                bary[1] += mstate->getPY(tetra[k]);
                bary[2] += mstate->getPZ(tetra[k]);
            }
            bary *= 0.25;

            m_baryMap[tetraID] = bary;
        }
    }

    

}


bool SurfaceCarvingPerformer::runPerformer()
{
    //doMoveCarve1();

    doMoveCarve2();

    return true;
}

void SurfaceCarvingPerformer::draw(const core::visual::VisualParams* vparams)
{
    BaseCarvingPerformer::draw(vparams);

    if (!m_tetraId2remove.empty())
    {
        std::vector<Vector3> pos;
        sofa::core::behavior::BaseMechanicalState* mstate = m_topologyCon->getContext()->getMechanicalState();

        for (auto tetraId : m_tetraId2remove)
        {
            const sofa::core::topology::Topology::Tetrahedron& tetra = m_topologyCon->getTetrahedron(tetraId);
            for (unsigned int k = 0; k < 4; ++k)
                pos.push_back(Vector3(mstate->getPX(tetra[k]), mstate->getPY(tetra[k]), mstate->getPZ(tetra[k])));

            vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 1.0), 0.7f);
        }
    }

    if (!m_tetraId2refine.empty())
    {
        std::vector<Vector3> pos;
        sofa::core::behavior::BaseMechanicalState* mstate = m_topologyCon->getContext()->getMechanicalState();

        for (auto tetraId : m_tetraId2refine)
        {
            const sofa::core::topology::Topology::Tetrahedron& tetra = m_topologyCon->getTetrahedron(tetraId);
            for (unsigned int k = 0; k < 4; ++k)
                pos.push_back(Vector3(mstate->getPX(tetra[k]), mstate->getPY(tetra[k]), mstate->getPZ(tetra[k])));

            vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0), 0.7f);
        }
    }
}


void SurfaceCarvingPerformer::doMoveCarve1()
{
    SReal carveFactor = 0.1f; // d_carvingSpeed.getValue();
    component::statecontainer::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    m_topologyCon->getContext()->get(meca);
    helper::WriteAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > vertices = meca->write(core::VecCoordId::position());

    std::map <BaseMeshTopology::PointID, Vec3> tmpVertices;
    std::map <BaseMeshTopology::PointID, unsigned int> coefVertices;

    const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
    const SReal carv2x3 = _carvingDistance * _carvingDistance * 4;
    const SReal invCarv2x3 = 1 / carv2x3;

    for (contactInfo * cInfo : m_pointContacts)
    {
        if (cInfo->dist > _carvingDistance) // only carving points
            continue;

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = m_topologyCon->getTetrahedraAroundVertex(cInfo->elemId);

        Vec3 toolPos = cInfo->pointA;
        for (auto tetraID : tetraAV)
        {
            const BaseMeshTopology::Tetrahedron& tetra = m_topologyCon->getTetrahedron(tetraID);

            Vec3 bary = m_baryMap[tetraID];
            for (unsigned int k = 0; k < 4; ++k)
            {
                BaseMeshTopology::PointID vId = tetra[k];
                Vec3 currentPos = vertices[vId];
                SReal dist2 = (currentPos - toolPos).norm2();

                SReal coef = 0.0f;
                if (dist2 < carv2x3)
                    coef = (1 - dist2 * invCarv2x3) * 0.5;
                else
                    continue;

                Vec3 newPos = currentPos + (bary - currentPos) * coef;

                auto itM = tmpVertices.find(vId);
                if (itM == tmpVertices.end())
                {
                    tmpVertices[vId] = newPos;
                    coefVertices[vId] = 1;
                }
                else
                {
                    tmpVertices[vId] += newPos;
                    coefVertices[vId] += 1;
                }
            }
        }
    }

    for (auto itM : tmpVertices)
    {
        BaseMeshTopology::PointID vId = itM.first;
        unsigned int nbr = coefVertices[vId];
        Vec3 value = itM.second / nbr;
        vertices[vId] = value;
    }
}

void SurfaceCarvingPerformer::doMoveCarve2()
{
    SReal carveFactor = 1.0f; // d_carvingSpeed.getValue();
    component::statecontainer::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    m_topologyCon->getContext()->get(meca);
    helper::WriteAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > vertices = meca->write(core::VecCoordId::position());
    helper::WriteAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > restVertices = meca->write(core::VecCoordId::resetPosition());

    std::map <BaseMeshTopology::PointID, Vec3> tmpVertices;
    std::map <BaseMeshTopology::PointID, unsigned int> coefVertices;

    const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
    const SReal carv2x3 = _carvingDistance * _carvingDistance * 4;
    const SReal invCarv2x3 = 1 / carv2x3;

    for (contactInfo * cInfo : m_pointContacts)
    {
        if (cInfo->dist > _carvingDistance) // only carving points
            continue;

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = m_topologyCon->getTetrahedraAroundVertex(cInfo->elemId);

        Vec3 toolPos = cInfo->pointA;
        Vec3 dir = cInfo->normal;
        std::set <BaseMeshTopology::PointID> indices;

        for (auto tetraID : tetraAV)
        {
            const BaseMeshTopology::Tetrahedron& tetra = m_topologyCon->getTetrahedron(tetraID);
            for (unsigned int k = 0; k < 4; ++k)
            {
                BaseMeshTopology::PointID vId = tetra[k];
                indices.insert(vId);
            }
        }

        for (auto vId : indices)
        {
            Vec3 currentPos = vertices[vId];
            SReal dist2 = (currentPos - toolPos).norm2();

            SReal coef = 0.0f;
            if (dist2 < carv2x3)
                coef = (1 - dist2 * invCarv2x3);
            else
                continue;

            Vec3 newPos = currentPos + dir * coef * carveFactor;

            auto itM = tmpVertices.find(vId);
            if (itM == tmpVertices.end())
            {
                tmpVertices[vId] = newPos;
                coefVertices[vId] = 1;
            }
            else
            {
                tmpVertices[vId] += newPos;
                coefVertices[vId] += 1;
            }
        }
    }

    for (auto itM : tmpVertices)
    {
        BaseMeshTopology::PointID vId = itM.first;
        unsigned int nbr = coefVertices[vId];
        Vec3 value = itM.second / nbr;
        restVertices[vId] = value;
    }
}


} // namespace sofa::infinytoolkit
