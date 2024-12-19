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
#include <sofa/core/visual/VisualParams.h>

#include <sofa/geometry/Triangle.h>
#include <sofa/simulation/Node.h>

namespace sofa::infinytoolkit
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

const int NearestTexcoordsMapClass = core::RegisterObject("NearestTexcoordsMap test.")
    .add< NearestTexcoordsMap >()
;

NearestTexcoordsMap::NearestTexcoordsMap()
    : d_inputPositions(initData(&d_inputPositions, "inputPositions", "Indices of the points on the first model"))
    , d_mapPositions(initData(&d_mapPositions, "mapPositions", "Indices of the points on the second model"))
    , d_mapTexCoords(initData(&d_mapTexCoords, "mapTexCoords", "Indices of the points on the second model"))
    , d_radius(initData(&d_radius, (SReal)1, "radius", "Radius to search corresponding fixed point"))
    , d_useInterpolation(initData(&d_useInterpolation, false, "useInterpolation", "Radius to search corresponding fixed point"))    
    , d_drawInterpolation(initData(&d_drawInterpolation, false, "drawInterpolation", "Radius to search corresponding fixed point"))
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
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > fullPositions = d_inputPositions;

    if (d_useInterpolation.getValue())
        computeTriangulationMapping();
    else
        computeNearestPointMapping();

    m_mapColors.resize(fullPositions.size());
    for (unsigned int i = 0; i < m_mapColors.size(); ++i)
    {
        m_mapColors[i] = sofa::type::RGBAColor(SReal(rand()) / RAND_MAX, SReal(rand()) / RAND_MAX, SReal(rand()) / RAND_MAX, 1._sreal);
    }
}


void NearestTexcoordsMap::computeNearestPointMapping()
{
    sofa::helper::WriteOnlyAccessor<Data< type::vector<sofa::type::Vec2> > > outTexcoords = d_outputTexCoords;
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > fullPositions = d_inputPositions;
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > mapPositions = d_mapPositions;
    sofa::helper::ReadAccessor< Data< type::vector<sofa::type::Vec2> > > mapTexcoords = d_mapTexCoords;

    outTexcoords.resize(fullPositions.size());
    m_mapPositionIds.resize(fullPositions.size());

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
            m_mapPositionIds[i2] = i1;
        }
        else
        {
            outTexcoords[i2] = sofa::type::Vec2(0.0, 0.0);
            m_mapPositionIds[i2] = 0;
        }
    }
}


void NearestTexcoordsMap::computeTriangulationMapping()
{
    sofa::helper::WriteOnlyAccessor<Data< type::vector<sofa::type::Vec2> > > outTexcoords = d_outputTexCoords;
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > fullPositions = d_inputPositions;
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > mapPositions = d_mapPositions;
    sofa::helper::ReadAccessor< Data< type::vector<sofa::type::Vec2> > > mapTexcoords = d_mapTexCoords;

    outTexcoords.resize(fullPositions.size());
    m_mapPositionIds.resize(fullPositions.size() * 3);

    const SReal& maxR = d_radius.getValue();
    for (unsigned int idIn = 0; idIn < fullPositions.size(); ++idIn)
    {
        Vec3 ptIn = fullPositions[idIn];
        
        // Compute all distance and sort them in ascendant order using a map
        std::map <SReal, unsigned int> mapDistances;        
        for (unsigned int i3 = 0; i3 < mapPositions.size(); ++i3)
        {
            Vec3 ptmap = mapPositions[i3];
            mapDistances.insert(std::pair< SReal, unsigned int>((ptIn - ptmap).norm2(), i3));
        }

        // compute all triangles barycoord
        auto itMap = mapDistances.begin();
        sofa::type::fixed_array<Index, 3> idMap;
        sofa::type::fixed_array<Vec3, 3> ptMap;
        for (unsigned int j = 0; j < 3; ++j)
        {
            idMap[j] = (*itMap).second;            
            ptMap[j] = mapPositions[idMap[j]];
            m_mapPositionIds[idIn * 3 + j] = idMap[j];
            itMap++;
        }

        auto baryCoords3 = sofa::geometry::Triangle::getBarycentricCoordinates(ptIn, ptMap[0], ptMap[1], ptMap[2]);
        
        if (baryCoords3[2] < 0) // no interpolation possible between the 3 points
        {
            outTexcoords[idIn] = mapTexcoords[idMap[0]];
        }
        else
        {
            outTexcoords[idIn] = mapTexcoords[idMap[0]] * baryCoords3[0] + mapTexcoords[idMap[1]] * baryCoords3[1] + mapTexcoords[idMap[2]] * baryCoords3[2];
        }
    } 
} 


void NearestTexcoordsMap::draw(const core::visual::VisualParams* vparams)
{
    if (m_mapPositionIds.empty() || d_drawInterpolation.getValue() == false)
        return;

    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();
    vparams->drawTool()->setLightingEnabled(false);
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > fullPositions = d_inputPositions;
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > mapPositions = d_mapPositions;

    std::vector<sofa::type::Vec3> vertices;
    std::vector<sofa::type::RGBAColor> colors;

    if (m_mapPositionIds.size() == fullPositions.size())
    {
        for (unsigned int i = 0; i < fullPositions.size(); ++i)
        {
            vertices.emplace_back(fullPositions[i]);
            vertices.emplace_back(mapPositions[m_mapPositionIds[i]]);
            colors.emplace_back(m_mapColors[i]);
        }
    }
    else
    {
        for (unsigned int i = 0; i < fullPositions.size(); ++i)
        {
            for (unsigned int j = 0; j < 3; ++j)
            {
                vertices.emplace_back(fullPositions[i]);
                vertices.emplace_back(mapPositions[m_mapPositionIds[i * 3 + j]]);
                colors.emplace_back(m_mapColors[i]);
            }
        }
    }

    vparams->drawTool()->drawLines(vertices, 1, colors);
}


} //namespace sofa::infinytoolkit
