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
#include <sofa/core/DataEngine.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa::infinytoolkit
{

using sofa::type::Vec3;

/** Attach given pair of particles, projecting the positions of the second particles to the first ones.
*/
class SOFA_INFINYTOOLKIT_API GridBaryCentersPositions : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(GridBaryCentersPositions, core::DataEngine);

    typedef typename core::topology::BaseMeshTopology::SeqTetrahedra SeqTetrahedra;
    typedef typename core::topology::BaseMeshTopology::SeqHexahedra SeqHexahedra;

    GridBaryCentersPositions();
    ~GridBaryCentersPositions() override;

    void init() override;
    void doUpdate() override;

    void draw(const core::visual::VisualParams* vparams) override;

protected:
    /** 
    * 
    **/
    void computeSurfaceMeshGrid();
    
    /** 
    *
    */
    void computeOutputPositions();

public:
    /// Inputs Data
    Data< type::vector< Vec3 > > d_inputPositions; ///< Full mesh position
    Data< SeqTetrahedra > d_tetrahedra; ///< Tetrahedra of mesh subset
    Data< sofa::type::Vec< 3, int > > d_n;
    /// Outputs Data
    Data< type::vector< Vec3 > > d_outputPositions; ///< Full mesh texcoords
    Data< SeqHexahedra > d_outputGrid; ///< Hexahedra of mesh subset

    /// Parameters Data
    Data<bool> d_drawGrid; ///< Boop optio to draw the mapping computed between inputPosition and mapPosition    
    
private:
    sofa::type::vector<Vec3> m_minBBoxes;
    sofa::type::vector<Vec3> m_maxBBoxes;
    sofa::type::vector<sofa::type::vector<int> > m_indicesPerCell;

    //sofa::type::fixed_array<QuadID, 6>
    Vec3 m_fullMin, m_fullMax;
};

} //namespace sofa::component::engine
