/*****************************************************************************
 *                 - Copyright (C) - 2020 - InfinyTech3D -                   *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework      *
 *                                                                           *
 * Commercial License Usage:                                                 *
 * Licensees holding valid commercial license from InfinyTech3D may use this *
 * file in accordance with the commercial license agreement provided with    *
 * the Software or, alternatively, in accordance with the terms contained in *
 * a written agreement between you and InfinyTech3D. For further information *
 * on the licensing terms and conditions, contact: contact@infinytech3d.com  *
 *                                                                           *
 * GNU General Public License Usage:                                         *
 * Alternatively, this file may be used under the terms of the GNU General   *
 * Public License version 3. The licenses are as published by the Free       *
 * Software Foundation and appearing in the file LICENSE.GPL3 included in    *
 * the packaging of this file. Please review the following information to    *
 * ensure the GNU General Public License requirements will be met:           *
 * https://www.gnu.org/licenses/gpl-3.0.html.                                *
 *                                                                           *
 * Authors: see Authors.txt                                                  *
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
