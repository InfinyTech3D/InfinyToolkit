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
class SOFA_INFINYTOOLKIT_API NearestTexcoordsMap : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(NearestTexcoordsMap, core::DataEngine);

    NearestTexcoordsMap();
    ~NearestTexcoordsMap() override;

    void init() override;
    void doUpdate() override;

    void draw(const core::visual::VisualParams* vparams) override;

protected:
    /** Method to find, for each point of full mesh, nearest point under @sa d_radius distance from mapped position
    * The texcoords of the point found will be applied to full mesh point.
    * */
    void computeNearestPointMapping();
    
    /** Method to find the 3 closest point from the mapped position for each point from the full mesh. The barycentric coordinates of the points are then 
    * computed in the 3 closest point frame and the texcoord are interpolated using those barycentric coordinates.
    * If interpolation is not possible (out from the computed triangle), the nearest point texcoord is used like in method @sa computeNearestPointMapping
    */
    void computeTriangulationMapping();

public:
    /// Inputs Data
    Data< type::vector< Vec3 > > d_inputPositions; ///< Full mesh position
    Data< type::vector< Vec3 > > d_mapPositions; ///< Surface mesh position
    Data< type::vector<sofa::type::Vec2> > d_mapTexCoords; ///< Surface mesh texcoords
    Data<SReal> d_radius; ///< Radius to search corresponding fixed point if no indices are given
    
    /// Outputs Data
    Data< type::vector<sofa::type::Vec2> > d_outputTexCoords; ///< Full mesh texcoords

    /// Parameters Data
    Data<bool> d_useInterpolation; ///< Bool option to choose between nearest point or interpolation method
    Data<bool> d_drawInterpolation; ///< Boop optio to draw the mapping computed between inputPosition and mapPosition    
    
private:
    std::vector<Index> m_mapPositionIds; ///< vector to store map position Id per position from full mesh. (same size as @d_inputPositions)
    std::vector<sofa::type::RGBAColor> m_mapColors; ///< vector to store color to draw the mapping per position from full mesh. (same size as @d_inputPositions)    
};

} //namespace sofa::component::engine
