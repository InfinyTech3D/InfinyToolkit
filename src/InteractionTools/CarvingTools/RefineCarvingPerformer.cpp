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
    m_tetra2remove.clear();

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

        m_tetra2remove.push_back(tetraAT[0]);
        carvingPosition = cInfo->pointA;
    }

    for each (contactInfo * cInfo in m_pointContacts)
    {
        if (cInfo->dist > refineDistance)
            continue;

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = m_topologyCon->getTetrahedraAroundVertex(cInfo->elemId);

        for (auto tetraId : tetraAV)
            m_tetra2remove.push_back(tetraId);

        carvingPosition = cInfo->pointA;
    }
}


bool RefineCarvingPerformer::runPerformer()
{
    if (!m_tetra2remove.empty())
    {
        sofa::Size nbrTetra = m_topologyCon->getNbTetrahedra();
        bool res = m_tetraAlgo->refineTetrahedra(m_tetra2remove, m_carvingMgr->d_refineCriteria.getValue());

        if (!res)
            return res;


        sofa::Size nbrTetraNew = m_topologyCon->getNbTetrahedra();
        for (sofa::Index tetraId = nbrTetra; tetraId < nbrTetraNew; ++tetraId)
            m_tetra2remove.push_back(tetraId);

        sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
        m_topologyCon->getContext()->get(meca);
        helper::ReadAccessor< Data<sofa::defaulttype::Vec3Types::VecCoord> > vertices = meca->read(core::VecCoordId::position());
        const SReal& carvingDistance = m_carvingMgr->d_carvingDistance.getValue();

        // check all tetra dist from carving element
        std::set<unsigned int> tetraToremove;
        for (auto tetraId : m_tetra2remove)
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
        
        return res;
    }

    return false;
}


void RefineCarvingPerformer::draw(const core::visual::VisualParams* vparams)
{
    if (m_topologyCon && !m_tetra2remove.empty())
    {
        sofa::core::behavior::BaseMechanicalState* mstate = m_topologyCon->getContext()->getMechanicalState();
        std::vector<Vector3> pos;
        
        for (unsigned int i=0; i<m_tetra2remove.size(); ++i)
        {
            const sofa::core::topology::Topology::Tetrahedron& tri = m_topologyCon->getTetrahedron(m_tetra2remove[i]);
            for (unsigned int j = 0; j < 4; j++) {
                pos.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
            }
        }

        vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(0.0f, 0.5f, 1.0f, 1.0f), 0.5f);
    }
}

} // namespace sofa::component::controller
