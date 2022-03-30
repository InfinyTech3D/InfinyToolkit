/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/CarvingTools/RefineCarvingPerformer.h>

namespace sofa::component::controller
{

RefineCarvingPerformer::RefineCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, const SReal& carvingDistance, const SReal& refineDistance)
    : BaseCarvingPerformer(topo, carvingDistance, refineDistance)
    , m_tetraAlgo(nullptr)
{

}


RefineCarvingPerformer::~RefineCarvingPerformer()
{
    for (auto itM = m_tetraAlgos.begin(); itM != m_tetraAlgos.end(); ++itM)
    {
        delete itM->second;
    }

    m_tetraAlgos.clear();
    //m_tetra2remove.clear();
}


bool RefineCarvingPerformer::initPerformer()
{


    //for (unsigned int i = 0; i < m_surfaceCollisionModels.size(); ++i)
    //{
    //    const sofa::core::objectmodel::BaseContext* _node = m_surfaceCollisionModels[i]->getContext();
    //    sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = _node->get<sofa::component::topology::TetrahedronSetTopologyContainer>();

    //    if (topoCon == nullptr)
    //    {
    //        msg_error() << "no TetrahedronSetTopologyContainer found for collision model: " << m_surfaceCollisionModels[i]->getName();
    //    }

    //    //sofa::core::topology::TopologicalMapping* topoMapping;
    //    if (m_tetraAlgos.find(topoCon) != m_tetraAlgos.end()) // already in
    //        continue;

    //    TetrahedronRefinementAlgorithms* tetraAlgo = new TetrahedronRefinementAlgorithms();
    //    bool resInit = tetraAlgo->init(_node);
    //    m_tetraAlgos[topoCon] = tetraAlgo;
    //    if (!resInit)
    //    {
    //        m_carvingReady = false;
    //        return;
    //    }
    //}

    //// check if forcefeedback
    //m_forceFeedback = getContext()->get<sofa::component::controller::ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
    //if (m_forceFeedback)
    //    msg_info() << "Forcefeedback found: " << m_forceFeedback->getName();
    //else
    //    msg_info() << "NO Forcefeedback found: ";

    return true;
}


bool RefineCarvingPerformer::runPerformer()
{
    // if not inside the map of tetrahedronRefinementAlgo, or not container, skip
//sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = targetModel->getContext()->get<sofa::component::topology::TetrahedronSetTopologyContainer>();
//auto itM = m_tetraAlgos.find(topoCon);
//if (itM == m_tetraAlgos.end()) {
//    msg_error() << "No TetrahedronRefinementAlgorithms found for collision model: " << targetModel->name;
//    continue;
//}


    return true;
}

//bool AdvancedCarvingManager::doRefinement()
//{
//    const Real& refineDistance = d_refineDistance.getValue();
//    bool result = false;
//    sofa::type::vector<unsigned int> layer1_tetra;
//
//    TetrahedronRefinementAlgorithms* _tetraAlgo = nullptr;
//    for each (contactInfo * cInfo in m_triangleContacts)
//    {
//        if (cInfo->dist > refineDistance)
//            continue;
//
//        if (_tetraAlgo == nullptr)
//            _tetraAlgo = cInfo->tetraAlgo;
//        else if (_tetraAlgo != cInfo->tetraAlgo)
//        {
//            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
//            continue;
//        }
//
//        sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = _tetraAlgo->getTopologyContainer();
//        const core::topology::BaseMeshTopology::TetrahedraAroundTriangle& tetraAT = topoCon->getTetrahedraAroundTriangle(cInfo->elemId);
//
//        if (tetraAT.size() != 1)
//        {
//            msg_error() << "More than one tetra around tri: " << cInfo->elemId;
//            continue;
//        }
//
//        layer1_tetra.push_back(tetraAT[0]);
//    }
//
//    for each (contactInfo * cInfo in m_pointContacts)
//    {
//        if (cInfo->dist > refineDistance)
//            continue;
//
//        if (_tetraAlgo == nullptr)
//            _tetraAlgo = cInfo->tetraAlgo;
//        else if (_tetraAlgo != cInfo->tetraAlgo)
//        {
//            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
//            continue;
//        }
//
//        sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = _tetraAlgo->getTopologyContainer();
//        const core::topology::BaseMeshTopology::TetrahedraAroundVertex& tetraAV = topoCon->getTetrahedraAroundVertex(cInfo->elemId);
//
//        for (auto tetraId : tetraAV)
//            layer1_tetra.push_back(tetraId);
//    }
//
//    if (!layer1_tetra.empty() && _tetraAlgo != nullptr)
//        return _tetraAlgo->refineTetrahedra(layer1_tetra, d_refineCriteria.getValue());
//    else
//        return false;
//}





    //if (m_topoCon && !m_tetra2remove.empty())
    //{
    //    sofa::core::behavior::BaseMechanicalState* mstate = m_topoCon->getContext()->getMechanicalState();
    //    std::vector<Vector3> pos;
    //    
    //    
    //    if (!mstate)
    //    {
    //        std::cout << "mstate is null" << std::endl;
    //        return;
    //    }

    //    for (unsigned int i=0; i<m_tetra2remove.size(); ++i)
    //    {
    //        const sofa::core::topology::Topology::Tetrahedron& tri = m_topoCon->getTetrahedron(m_tetra2remove[i]);
    //        for (unsigned int j = 0; j < 4; j++) {
    //            pos.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
    //        }
    //    }

    //    vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(0.0f, 0.5f, 1.0f, 1.0f), 0.7f);
    //    //vparams->drawTool()->drawTriangles(pos, sofa::type::RGBAColor(0.0f, 0.5f, 1.0f, 1.0f));
    //}

    // draw refine distance
    //vparams->drawTool()->drawSpheres(spheres, d_refineDistance.getValue() + d_carvingRadius.getValue(), sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 0.5));

/*


    if (d_drawTetra.getValue() && m_topoCon != nullptr)
    {
        const sofa::helper::fixed_array<sofa::type::vector<TetraToSplit*>, 2>& neighTable = m_tetraAlgo->getNeighboorhoodTable();
        const sofa::type::vector<sofa::core::topology::Topology::Tetrahedron>& tetraArray = m_topoCon->getTetrahedra();

        sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
        const float& scale = d_drawScaleTetrahedra.getValue();

        vparams->drawTool()->setPolygonMode(0,false);

        for (unsigned int i=0; i<neighTable.size(); ++i)
        {
            for (unsigned int j=0; j<neighTable[i].size(); ++j)
            {
                std::vector<Vector3> pos;
                const TetraToSplit* tetraStruc = neighTable[i][j];
                const sofa::core::topology::Topology::Tetrahedron& tetra = tetraArray[tetraStruc->m_tetraId];
                for (unsigned int k = 0; k < 4; ++k)
                    pos.push_back(Vector3(mstate->getPX(tetra[k]), mstate->getPY(tetra[k]), mstate->getPZ(tetra[k]) ));

                unsigned int nbrP = tetraStruc->m_points.size();
                if ( nbrP == 6)
                    color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
                else if (nbrP == 5)
                    color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.2f, 1.0);
                else if (nbrP == 4)
                    color4 = sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 1.0);
                else if (nbrP == 3)
                    color4 = sofa::type::RGBAColor(0.0f, 0.8, 0.5f, 1.0);
                else if (nbrP == 2)
                    color4 = sofa::type::RGBAColor(0.0f, 0.5, 0.8f, 1.0);
                else if (nbrP == 1)
                    color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);

                if (scale >= 1.0 || scale < 0.001)
                    vparams->drawTool()->drawTetrahedra(pos, color4);
                else
                    vparams->drawTool()->drawScaledTetrahedra(pos, color4, scale);
            }
        }

        if (!m_tetra2remove.empty())
        {
            std::vector<Vector3> pos;
            for (unsigned int i=0; i<m_tetra2remove.size(); ++i)
            {
                const sofa::core::topology::Topology::Tetrahedron& tetra = tetraArray[m_tetra2remove[i]];
                for (unsigned int k = 0; k < 4; ++k)
                    pos.push_back(Vector3(mstate->getPX(tetra[k]), mstate->getPY(tetra[k]), mstate->getPZ(tetra[k]) ));
            }

            if (scale >= 1.0 || scale < 0.001)
                vparams->drawTool()->drawTetrahedra(pos, sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 1.0));
            else
                vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 1.0), scale);
        }


        vparams->drawTool()->setPolygonMode(0,vparams->displayFlags().getShowWireFrame());
    }

    */


} // namespace sofa::component::controller
