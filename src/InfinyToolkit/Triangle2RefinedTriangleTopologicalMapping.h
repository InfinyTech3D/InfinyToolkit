/*****************************************************************************
 *                - Copyright (C) 2020-Present InfinyTech3D -                *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework.     *
 *                                                                           *
 * This file is dual-licensed:                                               *
 *                                                                           *
 * 1) Commercial License:                                                    *
 *      This file may be used under the terms of a valid commercial license  *
 *      agreement provided wih the software by InfinyTech3D.                 *
 *                                                                           *
 * 2) GNU General Public License (GPLv3) Usage                               *
 *      Alternatively, this file may be used under the terms of the          *
 *      GNU General Public License version 3 as published by the             *
 *      Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  *
 *                                                                           *
 * Contact: contact@infinytech3d.com                                         *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
#pragma once
#include <InfinyToolkit/config.h>
#include <sofa/core/topology/TopologicalMapping.h>

namespace sofa::component::topology::container::dynamic
{
    class TriangleSetTopologyModifier;
} // namespace sofa::component::topology::container::dynamic


namespace sofa::infinytoolkit
{
/**
* This class, called Triangle2RefinedTriangleTopologicalMapping, is a specific implementation of the barycentric TopologicalMapping where :
*
* INPUT TOPOLOGY = TriangleSetTopology
* OUTPUT TOPOLOGY = TriangleSetTopology, as the refined version of the INPUT TOPOLOGY
*
* Triangle2RefinedTriangleTopologicalMapping class is templated by the pair (INPUT TOPOLOGY, OUTPUT TOPOLOGY)
* In2OutMap[cId] -> [rId0, rId5, ... , rId8]   input triangle id of coarse mesh -> vector of corresponding refiened triangle id
* Glob2LocMap[rId] -> [cId]  input is id of refined triangle, output is the belonging coarse triangle
*/
class SOFA_INFINYTOOLKIT_API Triangle2RefinedTriangleTopologicalMapping : public sofa::core::topology::TopologicalMapping
{
public:
    SOFA_CLASS(Triangle2RefinedTriangleTopologicalMapping, sofa::core::topology::TopologicalMapping);
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

    void draw(const core::visual::VisualParams* vparams) override;

    /** \brief Translates the TopologyChange objects from the source to the target.
    *
    * Translates each of the TopologyChange objects waiting in the source list so that they have a meaning and
    * reflect the effects of the first topology changes on the second topology.
    *
    */
    void updateTopologicalMappingTopDown() override;

    Index getFromIndex(Index ind) override;

    struct debugData
    {
        int cIndex;
        type::Vec3 cBaryCenter;
        type::vector <type::Vec3> rBaryCenters;
    };

    Data<bool> p_drawMapping;

protected:
    void logMaps() const;

private:
    /// Pointer to the output topology modifier
    sofa::component::topology::container::dynamic::TriangleSetTopologyModifier* m_outTopoModifier = nullptr;

    type::vector <debugData> m_barycenters;
};

} //namespace sofa::infinytoolkit
