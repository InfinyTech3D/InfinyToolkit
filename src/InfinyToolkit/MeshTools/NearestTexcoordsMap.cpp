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

#define SOFA_COMPONENT_NEARESTTEXCOORDS_CPP
#include <InfinyToolkit/MeshTools/NearestTexcoordsMap.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/Node.h>

namespace sofa::infinytoolkit
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

SOFA_DECL_CLASS(NearestTexcoordsMap)

const int NearestTexcoordsMapClass = core::RegisterObject("NearestTexcoordsMap test.")
    .add< NearestTexcoordsMap >()
;

NearestTexcoordsMap::NearestTexcoordsMap()
    : d_inputPositions(initData(&d_inputPositions, "inputPositions", "Indices of the points on the first model"))
    , d_mapPositions(initData(&d_mapPositions, "mapPositions", "Indices of the points on the second model"))
    , d_mapTexCoords(initData(&d_mapTexCoords, "mapTexCoords", "Indices of the points on the second model"))
    , d_radius(initData(&d_radius, (SReal)1, "radius", "Radius to search corresponding fixed point"))
    , d_outputTexCoords(initData(&d_outputTexCoords, "outputTexCoords", "Radius to search corresponding fixed point"))
{

}

NearestTexcoordsMap::~NearestTexcoordsMap()
{
}


void NearestTexcoordsMap::init()
{
    addInput(&d_inputPositions);
    addInput(&d_mapPositions);
    addInput(&d_mapTexCoords);

    addOutput(&d_outputTexCoords);
}


void NearestTexcoordsMap::doUpdate()
{
    sofa::helper::WriteOnlyAccessor<Data< type::vector<sofa::type::Vector2> > > outTexcoords = d_outputTexCoords;
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > fullPositions = d_inputPositions;
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > mapPositions = d_mapPositions;
    sofa::helper::ReadAccessor< Data< type::vector<sofa::type::Vector2> > > mapTexcoords = d_mapTexCoords;

    outTexcoords.resize(fullPositions.size());

    Vec3 pt2;
    auto dist = [](const Vec3& a, const Vec3& b) { return (b - a).norm(); };
    auto cmp = [&pt2, &dist](const Vec3& a, const Vec3& b) {
        return dist(a, pt2) < dist(b, pt2);
    };

    const SReal& maxR = d_radius.getValue();

    for (unsigned int i2 = 0; i2 < fullPositions.size(); ++i2)
    {
        pt2 = fullPositions[i2];
        auto el = std::min_element(std::begin(mapPositions), std::end(mapPositions), cmp);
        if (dist(*el, pt2) < maxR)
        {
            auto i1 = std::distance(std::begin(mapPositions), el);
            outTexcoords[i2] = mapTexcoords[i1];
        }
        else
        {
            outTexcoords[i2] = sofa::type::Vector2(0.0, 0.0);
        }
    }

}

} //namespace sofa::infinytoolkit
