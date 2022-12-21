/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <InfinyToolkit/Triangle2RefinedTriangleTopologicalMapping.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/component/topology/container/dynamic/TriangleSetTopologyContainer.h>
#include <sofa/component/topology/container/dynamic/TriangleSetTopologyModifier.h>
#include <sofa/component/topology/container/dynamic/TriangleSetGeometryAlgorithms.h>

#include <sofa/core/topology/TopologyChange.h>
#include <sofa/type/Vec.h>
#include <map>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/AdvancedTimer.h>

namespace sofa::component::topology::mapping
{

using namespace sofa::defaulttype;

using namespace sofa::component::topology::mapping;
using namespace sofa::component::topology::container::dynamic;
using namespace sofa::core::topology;

// Register in the Factory
const int Triangle2RefinedTriangleTopologicalMappingClass = core::RegisterObject("Special case of mapping where TriangleSetTopology is converted into a refined TriangleSetTopology")
        .add< Triangle2RefinedTriangleTopologicalMapping >();

Triangle2RefinedTriangleTopologicalMapping::Triangle2RefinedTriangleTopologicalMapping()
    : sofa::core::topology::TopologicalMapping()
    , m_outTopoModifier(nullptr)
{
    m_inputType = TopologyElementType::TRIANGLE;
    m_outputType = TopologyElementType::TRIANGLE;
}


Triangle2RefinedTriangleTopologicalMapping::~Triangle2RefinedTriangleTopologicalMapping()
{
    auto Loc2GlobVec = sofa::helper::getWriteOnlyAccessor(Loc2GlobDataVec);
    Loc2GlobVec.clear();
    Glob2LocMap.clear();
}


void Triangle2RefinedTriangleTopologicalMapping::init()
{
    if (!this->checkTopologyInputTypes()) // method will display error message if false
    {
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return; 
    }

    container::dynamic::TriangleSetTopologyModifier* to_tstm;
    toModel->getContext()->get(to_tstm);
    if (!to_tstm)
    {
        msg_error() << "No TriangleSetTopologyModifier found in the output topology node '"
            << toModel->getContext()->getName() << "'.";
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }
    else {
        m_outTopoModifier = to_tstm;
    }

    typedef typename TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types> TriGeo3;
    using Coord = TriGeo3::Coord;

    TriGeo3* coarseGeo = nullptr;
    fromModel->getContext()->get(coarseGeo);
    TriGeo3* refinedGeo = nullptr;
    toModel->getContext()->get(refinedGeo);

    sofa::Size nbTriCoarse = fromModel->getNbTriangles();
    sofa::Size nbTriRefined = toModel->getNbTriangles();
    
    // TODO optimise this double loop
    for (unsigned int idRTri = 0; idRTri < nbTriRefined; ++idRTri)
    {
        const Coord bary = refinedGeo->computeTriangleCenter(idRTri);
        for (unsigned int idCTri = 0; idCTri < nbTriCoarse; ++idCTri)
        {
            Topology::TriangleID idnull;
            bool inside = coarseGeo->isPointInsideTriangle(idCTri, false, bary, idnull);
            if (inside)
            {
                In2OutMap[idCTri].push_back(idRTri);
                Glob2LocMap[idRTri] = idCTri;
                break;
            }
        }
    }
    
    this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}


Index Triangle2RefinedTriangleTopologicalMapping::getFromIndex(Index ind)
{
    return 0;
}

void Triangle2RefinedTriangleTopologicalMapping::updateTopologicalMappingTopDown()
{
    if (this->d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    sofa::helper::AdvancedTimer::stepBegin("Update Triangle2RefinedTriangleTopologicalMapping");

    std::list<const TopologyChange *>::const_iterator itBegin=fromModel->beginChange();
    std::list<const TopologyChange *>::const_iterator itEnd=fromModel->endChange();

    auto Loc2GlobVec = sofa::helper::getWriteAccessor(Loc2GlobDataVec);

    while( itBegin != itEnd )
    {
        TopologyChangeType changeType = (*itBegin)->getChangeType();
        std::string topoChangeType = "Triangle2RefinedTriangleTopologicalMapping - " + parseTopologyChangeTypeToString(changeType);
        sofa::helper::AdvancedTimer::stepBegin(topoChangeType);

        switch( changeType )
        {
        case core::topology::TRIANGLESREMOVED:
        {
            const sofa::type::vector<Topology::TriangleID>& tri2Remove = (static_cast<const TrianglesRemoved*>(*itBegin))->getArray();

            sofa::type::vector< Index > triIDs2Remove;
            for (auto triCID : tri2Remove)
            {
                const sofa::type::vector<Index>& tris = In2OutMap[triCID];
                for (auto id : tris)
                    triIDs2Remove.push_back(id);
            }

            m_outTopoModifier->removeTriangles(triIDs2Remove, true, false);
            break;
        }
        default:
            // Ignore events that are not Triangle related.
            break;
        };

        sofa::helper::AdvancedTimer::stepEnd(topoChangeType);
        ++itBegin;
    }

    sofa::helper::AdvancedTimer::stepEnd("Update Triangle2RefinedTriangleTopologicalMapping");
}

} //namespace sofa::component::topology::mapping
