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

bool SurfaceCarvingPerformer::runPerformer()
{
    m_tetraId2remove.clear();
    m_tetraId2refine.clear();

    // Filter tetra
    for each (contactInfo * cInfo in m_pointContacts)
    {
        if (cInfo->dist > m_refineDistance)
            continue;

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = m_topologyCon->getTetrahedraAroundVertex(cInfo->elemId);
        bool toFilter = false;
        if (cInfo->dist < m_carvingDistance)
        {
            for (auto tetraID : tetraAV)
                m_tetraId2remove.insert(tetraID);
        }
        else
        {
            for (auto tetraID : tetraAV)
            {
                if (m_tetraId2remove.find(tetraID) != m_tetraId2remove.end())
                    continue;
                    
                m_tetraId2refine.insert(tetraID);
            }
        }
    }

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

//bool AdvancedCarvingManager::doMoveCarvePoint()
//{
//    const Real& carvingDistance = d_carvingDistance.getValue();
//    std::map<BaseMeshTopology::PointID, BaseMeshTopology::TetrahedraAroundVertex> layer1_tetraAV;
//    std::set <BaseMeshTopology::TetrahedronID> layer1_tetra;
//    std::map<BaseMeshTopology::PointID, contactInfo*> contactInfoMap;
//
//    sofa::type::vector<unsigned int> layer2_tetra;
//    sofa::type::vector<contactInfo*> triInfo;
//
//    std::set <BaseMeshTopology::TetrahedronID> surrounding_tetra;
//
//    TetrahedronRefinementAlgorithms* _tetraAlgo = nullptr;
//    sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon;
//    for each (contactInfo * cInfo in m_pointContacts)
//    {
//        if (cInfo->dist > carvingDistance)
//            continue;
//
//        if (_tetraAlgo == nullptr) {
//            _tetraAlgo = cInfo->tetraAlgo;
//            topoCon = _tetraAlgo->getTopologyContainer();
//        }
//        else if (_tetraAlgo != cInfo->tetraAlgo)
//        {
//            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
//            continue;
//        }
//
//        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = topoCon->getTetrahedraAroundVertex(cInfo->elemId);
//
//        layer1_tetraAV[cInfo->elemId] = tetraAV;
//        for (auto tetraID : tetraAV) {
//            layer1_tetra.insert(tetraID);
//            surrounding_tetra.insert(tetraID);
//        }
//
//        contactInfoMap[cInfo->elemId] = cInfo;
//    }
//
//
//
//    for each (contactInfo * cInfo in m_triangleContacts)
//    {
//        if (cInfo->dist > carvingDistance)
//            continue;
//
//        if (_tetraAlgo == nullptr) {
//            _tetraAlgo = cInfo->tetraAlgo;
//            topoCon = _tetraAlgo->getTopologyContainer();
//        }
//        else if (_tetraAlgo != cInfo->tetraAlgo)
//        {
//            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
//            continue;
//        }
//
//
//        const core::topology::BaseMeshTopology::TetrahedraAroundTriangle& tetraAT = topoCon->getTetrahedraAroundTriangle(cInfo->elemId);
//
//        if (tetraAT.size() != 1)
//        {
//            msg_error() << "More than one tetra around tri: " << cInfo->elemId;
//            continue;
//        }
//
//        layer2_tetra.push_back(tetraAT[0]);
//        triInfo.push_back(cInfo);
//
//        const sofa::core::topology::Triangle& tri = cInfo->tetraAlgo->getTopologyContainer()->getTriangle(cInfo->elemId);
//        for (unsigned int i = 0; i < 3; ++i)
//        {
//            const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = topoCon->getTetrahedraAroundVertex(tri[i]);
//            for (auto tetraID : tetraAV) {
//                surrounding_tetra.insert(tetraID);
//            }
//        }
//    }
//
//
//    // nothing todo
//    if (surrounding_tetra.empty())
//        return false;
//
//    if (m_forceFeedback)
//        m_forceFeedback->setLock(true);
//
//
//    // get write accessor to the position
//    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
//    topoCon->getContext()->get(meca);
//    helper::WriteAccessor< Data<DataTypes::VecCoord> > pos = meca->write(core::VecCoordId::position());
//
//    // compute tetra barycenters
//    sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topoGeo = _tetraAlgo->getTopologyGeometry();
//    std::map<unsigned int, DataTypes::Coord> barycenter;
//    for (auto tetraID : layer1_tetra)
//    {
//        if (barycenter.find(tetraID) != barycenter.end()) // already in
//            continue;
//
//        const Tetrahedron& t = topoCon->getTetrahedron(tetraID);
//
//        DataTypes::Coord bary = (pos[t[0]] + pos[t[1]] + pos[t[2]] + pos[t[3]]) * (Real)0.25;
//        barycenter[tetraID] = bary;
//    }
//
//
//    // move point to the center of their barycenters
//    //std::cout << "--------------" << std::endl;
//    m_topoCon = topoCon;
//    m_tetra2remove.clear();
//    SReal carveFactor = d_carvingSpeed.getValue();
//
//    // reduce touched tetra
//    for (unsigned int i = 0; i < layer2_tetra.size(); ++i)
//    {
//        sofa::core::topology::Topology::Tetrahedron tetra = topoCon->getTetrahedron(layer2_tetra[i]);
//        sofa::core::topology::Topology::Triangle tri = topoCon->getTriangle(triInfo[i]->elemId);
//        unsigned int pId = sofa::core::topology::Topology::InvalidID;
//        for (unsigned int i = 0; i < 4; i++)
//        {
//            bool found = false;
//            for (unsigned int j = 0; j < 3; j++)
//                if (tetra[i] == tri[j])
//                {
//                    found = true;
//                    break;
//                }
//
//            if (!found) {
//                pId = tetra[i];
//                break;
//            }
//        }
//
//        if (pId == sofa::core::topology::Topology::InvalidID)
//        {
//            msg_error() << "Point id not found in tetra: " << tetra << "and triangle: " << tri;
//            continue;
//        }
//
//        DataTypes::Coord posOppo = pos[pId];
//        for (auto vId : tri)
//        {
//            //pos[vId] = pos[vId] * (1 - carveFactor) + posOppo * carveFactor;
//            pos[vId] = pos[vId] * (1 - carveFactor) + posOppo * carveFactor - triInfo[i]->normal * carveFactor * 0.5;
//        }
//    }
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
//    // compute the list of tetra touched by the touched triangles
//    const Real& carvingDistance = d_carvingDistance.getValue();
//    sofa::type::vector<unsigned int> layer1_tetra;
//    std::set <BaseMeshTopology::TetrahedronID> layer2_tetra;
//    sofa::type::vector<contactInfo*> triInfo;
//    sofa::type::vector<contactInfo*> pointInfo;
//    std::list<unsigned int> idsP;
//
//    TetrahedronRefinementAlgorithms* _tetraAlgo = nullptr;
//    sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon;
//
//    for each (contactInfo * cInfo in m_triangleContacts)
//    {
//        if (cInfo->dist > carvingDistance)
//            continue;
//
//        if (_tetraAlgo == nullptr) {
//            _tetraAlgo = cInfo->tetraAlgo;
//            topoCon = _tetraAlgo->getTopologyContainer();
//        }
//        else if (_tetraAlgo != cInfo->tetraAlgo)
//        {
//            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
//            continue;
//        }
//
//
//        const core::topology::BaseMeshTopology::TetrahedraAroundTriangle& tetraAT = topoCon->getTetrahedraAroundTriangle(cInfo->elemId);
//
//        if (tetraAT.size() != 1)
//        {
//            msg_error() << "More than one tetra around tri: " << cInfo->elemId;
//            continue;
//        }
//
//        layer1_tetra.push_back(tetraAT[0]);
//        layer2_tetra.insert(tetraAT[0]);
//        triInfo.push_back(cInfo);
//
//        const sofa::core::topology::Triangle& tri = cInfo->tetraAlgo->getTopologyContainer()->getTriangle(cInfo->elemId);
//        for (unsigned int i = 0; i < 3; ++i)
//            idsP.push_back(tri[i]);
//    }
//
//    for each (contactInfo * cInfo in m_pointContacts)
//    {
//        if (cInfo->dist > carvingDistance)
//            continue;
//
//        bool found = false;
//        for (auto idP : idsP)
//        {
//            if (idP == cInfo->elemId)
//            {
//                found = true;
//                break;
//            }
//        }
//
//        if (found)
//            continue;
//
//        if (_tetraAlgo == nullptr) {
//            _tetraAlgo = cInfo->tetraAlgo;
//            topoCon = _tetraAlgo->getTopologyContainer();
//        }
//        else if (_tetraAlgo != cInfo->tetraAlgo)
//        {
//            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
//            continue;
//        }
//
//        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = topoCon->getTetrahedraAroundVertex(cInfo->elemId);
//
//        for (auto tetraID : tetraAV)
//            layer2_tetra.insert(tetraID);
//
//        pointInfo.push_back(cInfo);
//    }
//
//
//    if (pointInfo.empty() && triInfo.empty())
//        return false;
//
//    // get write accessor to the position
//    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
//    topoCon->getContext()->get(meca);
//    helper::WriteAccessor< Data<DataTypes::VecCoord> > pos = meca->write(core::VecCoordId::position());
//
//    SReal carveFactor = d_carvingSpeed.getValue();
//    // reduce touched tetra
//    for (unsigned int i = 0; i < layer1_tetra.size(); ++i)
//    {
//        sofa::core::topology::Topology::Tetrahedron tetra = topoCon->getTetrahedron(layer1_tetra[i]);
//        sofa::core::topology::Topology::Triangle tri = topoCon->getTriangle(triInfo[i]->elemId);
//        unsigned int pId = sofa::core::topology::Topology::InvalidID;
//        for (unsigned int i = 0; i < 4; i++)
//        {
//            bool found = false;
//            for (unsigned int j = 0; j < 3; j++)
//                if (tetra[i] == tri[j])
//                {
//                    found = true;
//                    break;
//                }
//
//            if (!found) {
//                pId = tetra[i];
//                break;
//            }
//        }
//
//        if (pId == sofa::core::topology::Topology::InvalidID)
//        {
//            msg_error() << "Point id not found in tetra: " << tetra << "and triangle: " << tri;
//            continue;
//        }
//
//        DataTypes::Coord posOppo = pos[pId];
//        for (auto vId : tri)
//        {
//            //pos[vId] = pos[vId] * (1 - carveFactor) + posOppo * factor1;
//            pos[vId] = pos[vId] * (1 - carveFactor) + posOppo * carveFactor - triInfo[i]->normal * carveFactor;
//        }
//    }
//
//
//
//    for (unsigned int i = 0; i < pointInfo.size(); ++i)
//    {
//
//        unsigned int idP = pointInfo[i]->elemId;
//        pos[idP] = pos[idP] + pointInfo[i]->normal * carveFactor;
//    }
//
//
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




} // namespace sofa::component::controller
