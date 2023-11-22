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
 ****************************************************************************/#pragma once
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

    typedef sofa::core::topology::BaseMeshTopology::Triangle                     Triangle;

    NearestTexcoordsMap();
    ~NearestTexcoordsMap() override;

    void init() override;
    void doUpdate() override;

    void draw(const core::visual::VisualParams* vparams) override;

protected:


public:
    Data< type::vector< Vec3 > > d_inputPositions; ///< Full mesh position
    Data< type::vector< Vec3 > > d_mapPositions; ///< Surface mesh position
    Data< type::vector<sofa::type::Vec2> > d_mapTexCoords; ///< Surface mesh texcoords
    Data<SReal> d_radius; ///< Radius to search corresponding fixed point if no indices are given
    
    Data<bool> d_useInterpolation;
    Data<bool> d_drawInterpolation;
    
    /// Full mesh texcoords
    Data< type::vector<sofa::type::Vec2> > d_outputTexCoords;

    std::vector<unsigned int> m_mapPos;
    std::vector<sofa::type::RGBAColor> m_colors;
    

protected:
    //void computeNearestPointMaps(const VecCoord& x1, const VecCoord& x2);
    void computeMethod1();
    void computeMethod2();
};

} //namespace sofa::component::engine
