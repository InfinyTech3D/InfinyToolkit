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
#pragma once
#include <InfinyToolkit/config.h>
#include <sofa/core/topology/TopologicalMapping.h>

namespace sofa::component::topology::container::dynamic
{
    class TriangleSetTopologyModifier;
} // namespace sofa::component::topology::container::dynamic


namespace sofa::component::topology::mapping
{
/**
* This class, called Triangle2RefinedTriangleTopologicalMapping, is a specific implementation of the barycentric TopologicalMapping where :
*
* INPUT TOPOLOGY = TriangleSetTopology
* OUTPUT TOPOLOGY = TriangleSetTopology, as the refined version of the INPUT TOPOLOGY
*
* Triangle2RefinedTriangleTopologicalMapping class is templated by the pair (INPUT TOPOLOGY, OUTPUT TOPOLOGY)
*
*/
class SOFA_INFINYTOOLKIT_API Triangle2RefinedTriangleTopologicalMapping : public sofa::core::topology::TopologicalMapping
{
public:
    SOFA_CLASS(Triangle2RefinedTriangleTopologicalMapping,sofa::core::topology::TopologicalMapping);
protected:
    /** \brief Constructor.
    *
    */
    Triangle2RefinedTriangleTopologicalMapping();

    /** \brief Destructor.
    *
    * Does nothing.
    */
    ~Triangle2RefinedTriangleTopologicalMapping() override;
public:
    /** \brief Initializes the target BaseTopology from the source BaseTopology.
    */
    void init() override;


    /** \brief Translates the TopologyChange objects from the source to the target.
    *
    * Translates each of the TopologyChange objects waiting in the source list so that they have a meaning and
    * reflect the effects of the first topology changes on the second topology.
    *
    */
    void updateTopologicalMappingTopDown() override;

    Index getFromIndex(Index ind) override;

private:
    /// Pointer to the output topology modifier
    container::dynamic::TriangleSetTopologyModifier* m_outTopoModifier;
};

} //namespace sofa::component::topology::mapping
