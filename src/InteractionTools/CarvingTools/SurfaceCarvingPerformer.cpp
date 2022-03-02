/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/CarvingTools/SurfaceCarvingPerformer.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <SofaBaseMechanics/MechanicalObject.h>

namespace sofa::component::controller
{

SurfaceCarvingPerformer::SurfaceCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, const SReal& carvingDistance, const SReal& refineDistance)
    : BaseCarvingPerformer(topo, carvingDistance, refineDistance)
{

}


bool SurfaceCarvingPerformer::initPerformer()
{
    std::cout << "SurfaceCarvingPerformer::initPerformer: " << m_topologyCon->getName() << std::endl;
    return true;
}


void SurfaceCarvingPerformer::filterContacts()
{
    m_tetraId2remove.clear();
    m_tetraId2refine.clear();

    sofa::core::behavior::BaseMechanicalState* mstate = m_topologyCon->getContext()->getMechanicalState();

    // Filter tetra
    for each (contactInfo * cInfo in m_pointContacts)
    {
        if (cInfo->dist > m_carvingDistance) // only carving points
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
    std::cout << "SurfaceCarvingPerformer::runPerformer" << std::endl;
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
    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    m_topologyCon->getContext()->get(meca);
    helper::WriteAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > vertices = meca->write(core::VecCoordId::position());

    std::map <BaseMeshTopology::PointID, Vec3> tmpVertices;
    std::map <BaseMeshTopology::PointID, unsigned int> coefVertices;

    SReal carv2x3 = m_carvingDistance * m_carvingDistance * 4;
    SReal invCarv2x3 = 1 / carv2x3;

    for each (contactInfo * cInfo in m_pointContacts)
    {
        if (cInfo->dist > m_carvingDistance) // only carving points
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

                std::cout << "dist2: " << dist2 << " | coef: " << coef << std::endl;

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
    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    m_topologyCon->getContext()->get(meca);
    helper::WriteAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > vertices = meca->write(core::VecCoordId::position());
    helper::WriteAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > restVertices = meca->write(core::VecCoordId::resetPosition());

    std::map <BaseMeshTopology::PointID, Vec3> tmpVertices;
    std::map <BaseMeshTopology::PointID, unsigned int> coefVertices;

    SReal carv2x3 = m_carvingDistance * m_carvingDistance * 4;
    SReal invCarv2x3 = 1 / carv2x3;

    for each (contactInfo * cInfo in m_pointContacts)
    {
        if (cInfo->dist > m_carvingDistance) // only carving points
            continue;

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = m_topologyCon->getTetrahedraAroundVertex(cInfo->elemId);

        Vec3 toolPos = cInfo->pointA;
        Vec3 dir = cInfo->normal;
        std::cout << "toolPos: " << toolPos << " | dir: " << dir << " | dist: " << cInfo->dist << std::endl;
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

            std::cout << vId << " | dist2: " << dist2 << " | coef: " << coef << std::endl;

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

//bool AdvancedCarvingManager::doMoveCarvePoint()
//{
//
//
//
//
//
//    for (auto it : layer1_tetraAV)
//    {
//        DataTypes::Coord bbary;
//        for (auto tetraID : it.second)
//        {
//            auto itB = barycenter.find(tetraID);
//            if (itB == barycenter.end())
//            {
//                msg_error() << "barycenter not found for tetra: " << tetraID;
//                continue;
//            }
//            bbary += itB->second;
//        }
//
//        bbary /= it.second.size();
//        double dist = (bbary - pos[it.first]).norm2();
//        auto itM = contactInfoMap.find(it.first);
//        if (itM == contactInfoMap.end())
//        {
//            msg_error() << "point not found in contact map: " << it.first;
//            continue;
//        }
//
//        if (dist < 0.01)
//        {
//            std::cout << "dist: " << dist << std::endl;
//
//            // compute direction from tool to point to move
//
//            DataTypes::Coord pointProjection = DataTypes::Coord(0.0, 0.0, 0.0);
//            pointProjection = pos[it.first] + itM->second->normal/* * itM->second->dist*/;
//            pos[it.first] = pos[it.first] * (1 - carveFactor) + (pointProjection)*carveFactor * 0.5;
//        }
//        else
//        {
//            pos[it.first] = pos[it.first] * (1 - carveFactor) + (bbary)*carveFactor + (itM->second->normal) * carveFactor * 0.5;
//        }
//
//
//        //int fac = int(std::round(it.second.size() / 3));
//
//        //bbary += pointProjection * fac;
//        //bbary /= it.second.size();
//
//        //pointProjection = (1 - factorN*2) * bbary + factorN*2 * pointProjection;
//        //bbary /= (it.second.size() + fac);
//        //std::cout << "- final: " << pointProjection << " -> " << pos[it.first] * (1 - carveFactor) + (pointProjection)* carveFactor << std::endl;
//
//        //std::cout << "- bef: idV: " << it.first << " = " << pos[it.first] << " | bary: " << pointProjection  << std::endl;
//        //pos[it.first] = pos[it.first] * (1 - carveFactor) + (bbary) * carveFactor /*- triInfo[i]->normal * carveFactor*/;
//        //std::cout << "- after idV: " << it.first << " = " << pos[it.first] << std::endl;
//
//        m_contactPoints.push_back(pos[it.first]);
//        m_contactPoints.push_back(bbary);
//    }
//
//
//
//
//
//    // check tetra size to remove them
//    std::set<unsigned int> layer_tetra_toRemove;
//    SReal minVolume = d_carvingCriteria.getValue();
//    for (auto tetraId : surrounding_tetra)
//    {
//        SReal volume = topoGeo->computeTetrahedronVolume(tetraId);
//
//        if (volume < minVolume) {
//            layer_tetra_toRemove.insert(tetraId);
//        }
//
//        bool goodT = topoGeo->checkTetrahedronValidity(tetraId);
//        if (!goodT)
//            layer_tetra_toRemove.insert(tetraId);
//    }
//
//    if (!layer_tetra_toRemove.empty())
//    {
//        //std::cout << "--- START Tetra removed: " << layer_tetra_toRemove.size() << std::endl;
//        _tetraAlgo->removeTetrahedra(layer_tetra_toRemove);
//        //std::cout << "--- Tetra removed: " << layer_tetra_toRemove.size() << std::endl;
//    }
//
//    if (m_forceFeedback)
//        m_forceFeedback->setLock(false);
//
//    return true;
//}





//bool AdvancedCarvingManager::doMoveCarve()
//{
//    sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topoGeo;
//    topoGeo = topoCon->getContext()->get<sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
//
//
//    // check tetra size to remove them
//    std::set<unsigned int> layer_tetra_toRemove;
//    SReal minVolume = d_carvingCriteria.getValue();
//    for (auto tetraId : layer2_tetra)
//    {
//        SReal volume = topoGeo->computeTetrahedronVolume(tetraId);
//
//        if (volume < minVolume) {
//            layer_tetra_toRemove.insert(tetraId);
//        }
//
//        bool goodT = topoGeo->checkTetrahedronValidity(tetraId);
//        if (!goodT)
//            layer_tetra_toRemove.insert(tetraId);
//    }
//
//    if (!layer_tetra_toRemove.empty())
//    {
//        _tetraAlgo->removeTetrahedra(layer_tetra_toRemove);
//    }
//
//    return true;
//}


//void SurfaceCarvingManager::doShave_Burr(const ContactVector* contacts)
//{
//    //Get Moving Direction of Tool to calculate drilling direction
//    //Get Shaving Point index in Surface
//
//    type::vector<int> ShavingPointIdx = getShavingPointIdx(contacts);
//
//    // Deformation
//    // 1. Find Center point
//    // 2. Compute Point indices within the range range of center point
//    // 3. Smoothing - move indices around of unique_SPI 
//    type::vector<unsigned int> unique_SPI = MoveContactVertex(ShavingPointIdx);
//
//    using sofa::core::topology::BaseMeshTopology;
//    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* mo_coll = NULL;
//    modelSurface->getContext()->get(mo_coll);
//    helper::WriteAccessor< Data<VecCoord> > x_wA = mo_coll->write(core::VecCoordId::position());
//    type::vector<unsigned int>::iterator it;
//
//    for (unsigned int n = 0; n < nb_iterations.getValue(); n++)
//    {
//        VecCoord t;
//        t.resize(x_wA.size());
//        for (unsigned int i = 0; i < x_wA.size(); i++)
//        {
//            t[i] = x_wA[i];
//        }
//        for (it = unique_SPI.begin(); it != unique_SPI.end(); ++it)
//        {
//            BaseMeshTopology::VerticesAroundVertex v1 = modelSurface->getContext()->getMeshTopology()->getVerticesAroundVertex(*it);
//            if (v1.size() > 0)
//            {
//                for (unsigned int k = 0; k < v1.size(); k++)
//                {
//                    BaseMeshTopology::VerticesAroundVertex v2 = modelSurface->getContext()->getMeshTopology()->getVerticesAroundVertex(v1[k]);
//                    unsigned int* tmp_v2 = new unsigned int[v2.size()];
//
//                    for (unsigned int j = 0; j < v2.size(); j++)
//                    {
//                        tmp_v2[j] = v2[j];
//                        for (unsigned int l = 0; l < v1.size(); l++)
//                            if (v2[j] == v1[l])
//                                tmp_v2[j] = 0;
//                    }
//
//                    std::vector<unsigned int> new_v2(v2.size());
//                    std::vector<unsigned int>::iterator its;
//                    its = std::remove_copy_if(tmp_v2, tmp_v2 + v2.size(), new_v2.begin(), equal0);
//                    new_v2.resize(std::distance(new_v2.begin(), its));
//
//                    if (new_v2.size())
//                    {
//                        Coord p = Coord();
//                        for (unsigned int j = 0; j < new_v2.size(); j++)
//                            p += x_wA[new_v2[j]];
//                        t[v1[k]] = p / new_v2.size();
//                    }
//                    delete[] tmp_v2;
//                }
//
//            }
//        }
//
//        for (unsigned int i = 0; i < x_wA.size(); i++)
//        {
//
//            x_wA[i] = t[i];
//        }
//    }
//}



} // namespace sofa::component::controller
