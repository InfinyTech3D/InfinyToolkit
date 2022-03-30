/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/CarvingTools/RefineCarvingPerformer.h>
#include <InteractionTools/CarvingTools/AdvancedCarvingManager.h>
#include <SofaBaseMechanics/MechanicalObject.h>

namespace sofa::component::controller
{
    using namespace sofa::core::topology;

RefineCarvingPerformer::RefineCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr)
    : BaseCarvingPerformer(topo, _carvingMgr)
    , m_tetraAlgo(nullptr)
{

}


RefineCarvingPerformer::~RefineCarvingPerformer()
{
    if (m_tetraAlgo != nullptr)
    {
        delete m_tetraAlgo;
        m_tetraAlgo = nullptr;
    }
}


bool RefineCarvingPerformer::initPerformer()
{
    m_tetraAlgo = new TetrahedronRefinementAlgorithms();
    bool resInit = m_tetraAlgo->init(m_topologyCon->getContext());

    return resInit;
}


void RefineCarvingPerformer::filterContacts()
{
    if (m_tetraAlgo == nullptr)
        return;

    const SReal& refineDistance = m_carvingMgr->d_refineDistance.getValue();
    const SReal& carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
    m_tetra2Filter.clear();
    //m_tetra2Filter2.clear();
    m_triIdsToFilter.clear();

    for each (contactInfo * cInfo in m_triangleContacts)
    {
        if (cInfo->dist > refineDistance)
            continue;

        const core::topology::BaseMeshTopology::TetrahedraAroundTriangle& tetraAT = m_topologyCon->getTetrahedraAroundTriangle(cInfo->elemId);

        if (tetraAT.size() != 1)
        {
            msg_error("RefineCarvingPerformer") << "More than one tetra around tri: " << cInfo->elemId;
            continue;
        }

        m_tetra2Filter.insert(tetraAT[0]);

        //if (cInfo->dist <= carvingDistance)
        //    m_tetra2Filter2.push_back(tetraAT[0]);

       
        carvingPosition = cInfo->pointA;
    }

    for each (contactInfo * cInfo in m_pointContacts)
    {
        if (cInfo->dist > refineDistance)
            continue;

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = m_topologyCon->getTetrahedraAroundVertex(cInfo->elemId);

        for (auto tetraId : tetraAV)
            m_tetra2Filter.insert(tetraId);

        if (cInfo->dist <= carvingDistance) {
            const core::topology::BaseMeshTopology::TrianglesAroundVertex& triAV = m_topologyCon->getTrianglesAroundVertex(cInfo->elemId);

            for (auto triId : triAV)
                m_triIdsToFilter.insert(triId);
        }

        carvingPosition = cInfo->pointA;
    }
}


bool RefineCarvingPerformer::runPerformer()
{
    if (!m_tetra2Filter.empty())
    {
        sofa::Size nbrTetra = m_topologyCon->getNbTetrahedra();
        bool res = m_tetraAlgo->refineTetrahedra(m_tetra2Filter, m_carvingMgr->d_refineCriteria.getValue());

        if (res)
        {
            std::cout << "RefineCarvingPerformer::refineTetrahedra()" << std::endl;
#if 0
            return res;
#else
            sofa::Size nbrTetraNew = m_topologyCon->getNbTetrahedra();
            for (sofa::Index tetraId = nbrTetra; tetraId < nbrTetraNew; ++tetraId)
                m_tetra2Filter.insert(tetraId);
#endif
        }
#if 0
        else
        {
            surfaceCarving2();
        }
#else
        simpleCarving();
#endif
        
        
        return res;
    }

    return false;
}


void RefineCarvingPerformer::simpleCarving()
{
    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    m_topologyCon->getContext()->get(meca);
    helper::ReadAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > vertices = meca->read(core::VecCoordId::position());
    const SReal& carvingDistance = m_carvingMgr->d_carvingDistance.getValue();

    // check all tetra dist from carving element
    std::set<unsigned int> tetraToremove;
    for (auto tetraId : m_tetra2Filter)
    {
        const sofa::core::topology::Topology::Tetrahedron& tetra = m_topologyCon->getTetrahedron(tetraId);
        Vec3 bary = { 0, 0, 0 };
        for (unsigned int j = 0; j < 4; j++) {
            bary += vertices[tetra[j]];
        }
        bary *= 0.25;
        SReal dist = (carvingPosition - bary).norm();

        if (dist < carvingDistance)
            tetraToremove.insert(tetraId);
    }

    if (!tetraToremove.empty())
    {
        m_tetraAlgo->removeTetrahedra(tetraToremove);
    }
}

void RefineCarvingPerformer::surfaceCarving()
{
    std::cout << "RefineCarvingPerformer::surfaceCarving()" << std::endl;
    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    m_topologyCon->getContext()->get(meca);
    helper::WriteAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > vertices = meca->write(core::VecCoordId::position());
    const SReal& carvingDistance = m_carvingMgr->d_carvingDistance.getValue();

    // check all point to be considered
    std::set<unsigned int> pointsToCheck;
    for (auto tetraId : m_tetra2Filter)
    {
        const sofa::core::topology::Topology::Tetrahedron& tetra = m_topologyCon->getTetrahedron(tetraId);
        for (unsigned int j = 0; j < 4; j++) {
            pointsToCheck.insert(tetra[j]);
        }
    }

    // check all point dist from carving element
    const SReal invCarv = 1 / carvingDistance * 0.5;
    for (auto pointId : pointsToCheck)
    {
        Vec3& vertex = vertices[pointId];
        Vec3 dir = vertex - carvingPosition;
        const SReal dist = dir.norm();

        if (dist > carvingDistance)
            continue;

        SReal factor = (carvingDistance - dist)* invCarv; // ]0, carvingDistance]
        std::cout << "pointId: " << pointId << " | factor: " << factor << " | carvingDistance: " << carvingDistance << " | dist: " << dist << std::endl;
        vertex = vertex + dir * factor;
    }
}


void RefineCarvingPerformer::surfaceCarving2()
{
    const SReal& carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
    std::cout << "m_triIdsToFilter: " << m_triIdsToFilter.size() << std::endl;

    if (m_triIdsToFilter.empty())
        return;

    std::map<Topology::TriangleID, Topology::TetrahedronID> triIds_surf;
    m_triIds.clear();

    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    m_topologyCon->getContext()->get(meca);
    helper::WriteAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > vertices = meca->write(core::VecCoordId::position());

    sofa::component::topology::container::dynamic::TetrahedronSetGeometryAlgorithms< sofa::defaulttype::Vec3Types>* tetraGeo = nullptr;
    m_topologyCon->getContext()->get(tetraGeo);
    
    // filter triangles on border
    for (auto triId : m_triIdsToFilter)
    {
        const TetrahedronSetTopologyContainer::TetrahedraAroundTriangle& tetraATri = m_topologyCon->getTetrahedraAroundTriangle(triId);
        if (tetraATri.size() == 1)
        {
            triIds_surf[triId] = tetraATri[0];
            m_triIds.insert(triId);
        }
    }

    struct newPoint
    {
        sofa::defaulttype::Vec3Types::Coord AccPosition = { 0, 0, 0 };
        int nbP = 0;
    };
   
    const TetrahedronSetTopologyContainer::SeqTetrahedra& tetrahedra = m_topologyCon->getTetrahedra();
    const TetrahedronSetTopologyContainer::SeqTriangles& triangles = m_topologyCon->getTriangles();
    std::map<Topology::PointID, newPoint> newPoints;
    const SReal invCarv = 1 / carvingDistance * 0.5;

    for (const auto& [key, value] : triIds_surf)
    {
        const Triangle& tri = triangles[key];
        const Tetrahedron& tetra = tetrahedra[value];
        
        Topology::PointID oppoPoint;
        bool found = false;
        for (int i = 0; i < 4; i++)
        {
            oppoPoint = tetra[i];
            for (int j = 0; j < 3; j++)
                if (tetra[i] == tri[j])
                {
                    found = true;
                    continue;
                }

            if (!found)
                break;
        }

        const Vec3& oppoPCoord = vertices[oppoPoint];
        // move the 3 points from surface triangle
        for (auto pId : tri)
        {
            Vec3 vertex = vertices[pId];
            Vec3 dir = oppoPCoord - vertex; // normalise??
            const SReal dist = (vertex - carvingPosition).norm();

            if (dist > carvingDistance)
                continue;

            SReal factor = (carvingDistance - dist) * invCarv; // ]0, carvingDistance]
            vertex = vertex + dir * factor;
            std::cout << "factor: " << factor << std::endl;

            auto it = newPoints.find(pId);
            if (it == newPoints.end()) // new point
            {
                newPoint nP;
                nP.AccPosition = vertex;
                nP.nbP = 1;
                newPoints[pId] = nP;
            }
            else
            {
                newPoint& nP = it->second;
                nP.AccPosition += vertex;
                nP.nbP++;
            }
        }
    }

    // replace new positions
    for (const auto& [key, value] : newPoints)
    {
        vertices[key] = value.AccPosition / value.nbP;
    }

    std::set<unsigned int> tetra2Remove;
    for (const auto& [key, value] : triIds_surf)
    {
        bool valid = tetraGeo->checkTetrahedronValidity(value);
        if (!valid) {
            std::cout << "Not valid: " << value << std::endl;
            tetra2Remove.insert(value);
        }
    }

    if (!tetra2Remove.empty())
    {
        std::cout << "remove tetra: " << tetra2Remove << std::endl;
        m_tetraAlgo->removeTetrahedra(tetra2Remove);
    }

    std::cout << "RefineCarvingPerformer::surfaceCarving2() : " << m_triangleContacts.size() << std::endl;
    
}


void RefineCarvingPerformer::draw(const core::visual::VisualParams* vparams)
{
    BaseCarvingPerformer::draw(vparams);

    //if (m_topologyCon && !m_tetra2Filter.empty())
    //{
    //    sofa::core::behavior::BaseMechanicalState* mstate = m_topologyCon->getContext()->getMechanicalState();
    //    std::vector<Vector3> pos;
    //    
    //    for (auto tetraId : m_tetra2Filter)
    //    {
    //        const sofa::core::topology::Topology::Tetrahedron& tri = m_topologyCon->getTetrahedron(tetraId);
    //        for (unsigned int j = 0; j < 4; j++) {
    //            pos.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
    //        }
    //    }

    //    vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(0.0f, 0.5f, 1.0f, 1.0f), 0.5f);


    //    std::vector<Vector3> pos2;
    //    for (auto triId : m_triIds)
    //    {
    //        const sofa::core::topology::Topology::Triangle& tri = m_topologyCon->getTriangle(triId);
    //        for (unsigned int j = 0; j < 3; j++) {
    //            pos2.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
    //        }
    //    }
    //    vparams->drawTool()->drawTriangles(pos2, sofa::type::RGBAColor(1.0f, 0.5f, 1.0f, 1.0f));


    //    std::vector<Vector3> pos3;

    //    //for (unsigned int i = 0; i < m_tetra2Filter2.size(); ++i)
    //    //{
    //    //    const sofa::core::topology::Topology::Tetrahedron& tri = m_topologyCon->getTetrahedron(m_tetra2Filter2[i]);
    //    //    for (unsigned int j = 0; j < 4; j++) {
    //    //        pos3.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
    //    //    }
    //    //}

    //    //vparams->drawTool()->drawScaledTetrahedra(pos3, sofa::type::RGBAColor(1.0f, 0.5f, 1.0f, 1.0f), 0.8f);

    //}
}

} // namespace sofa::component::controller
