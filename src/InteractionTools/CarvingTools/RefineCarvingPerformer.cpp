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

RefineCarvingPerformer::RefineCarvingPerformer()
    : BaseCarvingPerformer()
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


} // namespace sofa::component::controller
