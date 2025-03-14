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

namespace sofa::infinytoolkit
{

using namespace sofa::defaulttype;

using namespace sofa::component::topology::container::dynamic;
using namespace sofa::core::topology;

// Register in the Factory
void registerTriangle2RefinedTriangleTopologicalMapping(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("Special case of mapping where TriangleSetTopology is converted into a refined TriangleSetTopology.")
        .add< Triangle2RefinedTriangleTopologicalMapping >());
}


Triangle2RefinedTriangleTopologicalMapping::Triangle2RefinedTriangleTopologicalMapping()
    : sofa::core::topology::TopologicalMapping()
    , p_drawMapping(initData(&p_drawMapping, false, "drawMapping", "if true, will draw line between mapped triangles"))
{
    m_inputType = sofa::geometry::ElementType::TRIANGLE;
    m_outputType = sofa::geometry::ElementType::TRIANGLE;
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

    TriangleSetTopologyModifier* to_tstm;
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

    typedef typename sofa::component::topology::container::dynamic::TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types> TriGeo3;
    using Coord = TriGeo3::Coord;

    TriGeo3* coarseGeo = nullptr;
    fromModel->getContext()->get(coarseGeo);
    TriGeo3* refinedGeo = nullptr;
    toModel->getContext()->get(refinedGeo);

    sofa::Size nbTriCoarse = fromModel->getNbTriangles();
    sofa::Size nbTriRefined = toModel->getNbTriangles();
    
    bool drawMap = p_drawMapping.getValue();
    if (drawMap)
    {
        m_barycenters.resize(nbTriCoarse);
        for (unsigned int idCTri = 0; idCTri < nbTriCoarse; ++idCTri)
        {
            m_barycenters[idCTri].cIndex = idCTri;
            m_barycenters[idCTri].cBaryCenter = coarseGeo->computeTriangleCenter(idCTri);
            m_barycenters[idCTri].rBaryCenters.clear();
        }
    }

    // TODO optimise this double loop
    for (unsigned int idRTri = 0; idRTri < nbTriRefined; ++idRTri)
    {
        const Coord baryR = refinedGeo->computeTriangleCenter(idRTri);

        Topology::TriangleID idC = InvalidID;
        SReal distance = std::numeric_limits<SReal>::max();
        
        for (unsigned int idCTri = 0; idCTri < nbTriCoarse; ++idCTri)
        {
            Topology::TriangleID idnull;
            bool inside = coarseGeo->isPointInsideTriangle(idCTri, false, baryR, idnull);
            
            // If the triangulation is curve, several candidates are possible (ray vs triangle intersection)
            // Will take the closest one
            if (inside) 
            {
                const Coord baryC = coarseGeo->computeTriangleCenter(idCTri);
                SReal dist = (baryR - baryC).norm();

                if (dist < distance)
                {
                    distance = dist;
                    idC = idCTri;
                }
            }
        }

        if (idC != InvalidID)
        {
            In2OutMap[idC].push_back(idRTri);
            Glob2LocMap[idRTri] = idC;
            if (drawMap)
                m_barycenters[idC].rBaryCenters.push_back(baryR);
        }
        else
        {
            msg_error() << "No coarse triangle found for triangle: " << idRTri;
        }
        
    }

    logMaps();
    this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}


Index Triangle2RefinedTriangleTopologicalMapping::getFromIndex(Index ind)
{
    SOFA_UNUSED(ind);
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
            sofa::type::vector<Topology::TriangleID> in_tri2Remove = (static_cast<const TrianglesRemoved*>(*itBegin))->getArray();
            std::sort(in_tri2Remove.begin(), in_tri2Remove.end(), std::greater<unsigned int>());

            msg_info() << "Input triangles to remove: " << in_tri2Remove;

            // Get last triangle indices from input and output topology
            Topology::TriangleID out_lastTriID = toModel->getNbTriangles() - 1;
            Topology::TriangleID in_lastTriID = fromModel->getNbTriangles() - 1;            

            // Compute the list of output triangles to remove
            sofa::type::vector< Index > out_tri2Remove;
            for (Topology::TriangleID triCID : in_tri2Remove)
            {
                const sofa::type::vector<Index>& tris = In2OutMap[triCID];
                for (auto id : tris) {
                    out_tri2Remove.push_back(id);
                }
            }
            std::sort(out_tri2Remove.begin(), out_tri2Remove.end(), std::greater<unsigned int>());
            msg_info() << "Output triangles to remove: " << out_tri2Remove;
            
            // Update map of last ouptput triangles ID due to incoming swap and pop back
            for (Topology::TriangleID out_removeId : out_tri2Remove)
            {               
                // Need to replace last ouput Id by the current removed one.               
                // 1. Get the input id corresponding to the last tri Id of output topology
                auto itOutLast = Glob2LocMap.find(out_lastTriID);
                if (itOutLast == Glob2LocMap.end())
                {
                    msg_error() << "Output triangle id not found: " << out_lastTriID << " in map of size: " << Glob2LocMap.size();
                    continue;
                }

                // 2. Get the In2Output map corresponding to the input Id of the last output Id
                Topology::TriangleID in_idOutLast = itOutLast->second;
                sofa::type::vector<Index> outIds = In2OutMap[in_idOutLast];
                
                // 3. Replace the last output id by the current removed one in the In2Ouput map
                for (unsigned int i = 0; i < outIds.size(); i++)
                {
                    if (outIds[i] == out_lastTriID) {
                        outIds[i] = out_removeId;
                        break;
                    }
                }
                In2OutMap[in_idOutLast] = outIds;
                Glob2LocMap[out_removeId] = in_idOutLast; // need to update this map as well in case of successive last element removal

                // 4. pop back the map
                Glob2LocMap.erase(out_lastTriID);

                // 5. iterate on next output last element id
                out_lastTriID--;
            }


            // Update coarse to refined map and propagate change in In2Output map
            for (Topology::TriangleID in_removeId : in_tri2Remove)
            {
                if (In2OutMap.find(in_lastTriID) == In2OutMap.end())
                {
                    msg_error() << "Last Input triangle id not found: " << in_lastTriID << " in map of size: " << In2OutMap.size();
                }

                // Get last vector of In2Ouput map which is already fully updated with output indices
                sofa::type::vector<Index> outIds = In2OutMap[in_lastTriID];

                // replace input removed element by last vector ids
                In2OutMap[in_removeId] = outIds;
                
                // restore backward map links
                for (unsigned int i = 0; i < outIds.size(); i++)
                {
                    Glob2LocMap[outIds[i]] = in_removeId;
                }

                In2OutMap.erase(in_lastTriID);
                in_lastTriID--;
            }            

            m_outTopoModifier->removeTriangles(out_tri2Remove, true, false);
            logMaps();
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


void Triangle2RefinedTriangleTopologicalMapping::logMaps() const
{
    if (!f_printLog.getValue())
        return;
    
    msg_info() << "## Coarse -> Refine ##";

    for (const auto& in2out : In2OutMap)
    {
        msg_info() << in2out.first << " | " << in2out.second;
    }

    msg_info() << "## Refine -> Coarse ##";
    
    for (const auto& in2out : Glob2LocMap)
    {
        msg_info() << in2out.first << " | " << in2out.second;
    }
}


void Triangle2RefinedTriangleTopologicalMapping::draw(const core::visual::VisualParams* vparams)
{
    if (!p_drawMapping.getValue())
        return;

    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();

    sofa::type::vector<sofa::type::Vec3> vertices;
    for (unsigned int id = 0; id < m_barycenters.size(); ++id)
    {
        const debugData& data = m_barycenters[id];
        type::vector <type::Vec3> rBary = data.rBaryCenters;
        for (unsigned int j = 0; j < rBary.size(); ++j)
        {
            vertices.push_back(data.cBaryCenter);
            vertices.push_back(rBary[j]);
        }
    }

    vparams->drawTool()->drawLines(vertices, 1.0f, sofa::type::RGBAColor::green());
}

} //namespace sofa::infinytoolkit
