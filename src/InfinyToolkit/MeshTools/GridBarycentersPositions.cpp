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

#define SOFA_COMPONENT_GRIDBARYCENTERPOSITIONS_CPP
#include <InfinyToolkit/MeshTools/GridBarycentersPositions.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/geometry/Triangle.h>
#include <sofa/simulation/Node.h>

namespace sofa::infinytoolkit
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

void registerGridBaryCentersPositions(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("GridBaryCentersPositions engine test.")
        .add< GridBaryCentersPositions >());
}


GridBaryCentersPositions::GridBaryCentersPositions()
    : d_inputPositions(initData(&d_inputPositions, "positions", "Indices of the points on the first model"))
    , d_tetrahedra(initData(&d_tetrahedra, "tetrahedra", "Indices of input tetrahedra"))
    , d_n(initData(&d_n, type::Vec3i(2, 2, 2), "n", "grid resolution"))
    , d_drawGrid(initData(&d_drawGrid, false, "drawGrid", "Debug draw of the grid and barycenters points"))
    , d_outputPositions(initData(&d_outputPositions, "outputPositions", "Radius to search corresponding fixed point"))
    , d_outputGrid(initData(&d_outputGrid, "outputGrid", "Radius to search corresponding fixed point"))
{

}

GridBaryCentersPositions::~GridBaryCentersPositions()
{
}


void GridBaryCentersPositions::init()
{
    addInput(&d_inputPositions);
    addInput(&d_tetrahedra);
    addInput(&d_n);

    addOutput(&d_outputPositions);
    addOutput(&d_outputGrid);

    type::Vec3i grid = d_n.getValue();

    if (grid[0] < 2) grid[0] = 2;
    if (grid[1] < 2) grid[1] = 2;
    if (grid[2] < 2) grid[2] = 2;

    d_n.setValue(grid);
}


void GridBaryCentersPositions::doUpdate()
{
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > fullPositions = d_inputPositions;
    computeSurfaceMeshGrid();
}


void GridBaryCentersPositions::computeSurfaceMeshGrid()
{
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > fullPositions = d_inputPositions;
    m_fullMin = { std::numeric_limits<float>::max() , std::numeric_limits<float>::max() , std::numeric_limits<float>::max() };
    m_fullMax = { -std::numeric_limits<float>::max() , -std::numeric_limits<float>::max() , -std::numeric_limits<float>::max() };

    for (auto pos : fullPositions)
    {
        for (int i = 0; i < 3; i++)
        {
            if (pos[i] < m_fullMin[i])
                m_fullMin[i] = pos[i];

            if (pos[i] > m_fullMax[i])
                m_fullMax[i] = pos[i];
        }
    }
    
    const sofa::type::Vec3i& grid = d_n.getValue();

    sofa::type::Vec3 steps;
    for (int i = 0; i < 3; i++)
    {
        steps[i] = (m_fullMax[i] - m_fullMin[i]) / grid[i];
        //std::cout << "length: " << (m_fullMax[i] - m_fullMin[i]) << " | " << steps[i] << std::endl;
    }
    

    for (int i = 0; i < grid[0]; i++)
    {
        SReal xmin = m_fullMin[0] + steps[0] * i;
        SReal xmax = m_fullMin[0] + steps[0] * (i + 1);
        for (int j = 0; j < grid[1]; j++)
        {
            SReal ymin = m_fullMin[1] + steps[1] * j;
            SReal ymax = m_fullMin[1] + steps[1] * (j + 1);
            for (int k = 0; k < grid[2]; k++)
            {
                SReal zmin = m_fullMin[2] + steps[2] * k;
                SReal zmax = m_fullMin[2] + steps[2] * (k + 1);
                m_minBBoxes.push_back(Vec3(xmin, ymin, zmin));
                m_maxBBoxes.push_back(Vec3(xmax, ymax, zmax));
            }
        }
    }

    m_indicesPerCell.resize(m_minBBoxes.size());

    for (int i = 0; i < fullPositions.size(); i++)
    {
        sofa::type::Vec< 3, int > gridPos;
        for (int j = 0; j < 3; ++j)
        {
            SReal localCoord = fullPositions[i][j] - m_fullMin[j];
            gridPos[j] = int(localCoord / steps[j]);
        }
        
        int vecPosition = gridPos[2] + gridPos[1] * (grid[2]-1) + gridPos[0] * (grid[2]-1) * (grid[1]-1);
        std::cout << "pos: " << fullPositions[i] << " | " << gridPos << " | " << vecPosition << std::endl;
        m_indicesPerCell[vecPosition].push_back(i);
    }

    sofa::helper::WriteAccessor< Data< type::vector< Vec3 > > > outputPositions = d_outputPositions;
    for (int i = 0; i < m_indicesPerCell.size(); i++)
    {
        sofa::type::vector<int>& indices = m_indicesPerCell[i];

        if (indices.empty())
            continue;

        Vec3 bary = Vec3(0.0, 0.0, 0.0);
        for (int id : indices)
        {
            bary += fullPositions[id];
        }
        bary /= indices.size();

        outputPositions.push_back(bary);
    }
}


void GridBaryCentersPositions::computeOutputPositions()
{
    
} 


void GridBaryCentersPositions::draw(const core::visual::VisualParams* vparams)
{
    if (d_drawGrid.getValue() == false)
        return;

    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();
    vparams->drawTool()->setLightingEnabled(false);

    vparams->drawTool()->drawBoundingBox(m_fullMin, m_fullMax);

    type::RGBAColor red = type::RGBAColor::red();
    vparams->drawTool()->setMaterial(red);
    for (int i = 0; i < m_minBBoxes.size(); i++)
    {
        vparams->drawTool()->drawBoundingBox(m_minBBoxes[i], m_maxBBoxes[i], 0.1);
    }

    //std::vector<sofa::type::Vec3> vertices;
    //std::vector<sofa::type::RGBAColor> colors;
    type::RGBAColor green = type::RGBAColor::green();
    sofa::helper::ReadAccessor< Data< type::vector< Vec3 > > > outPositions = d_outputPositions;
    vparams->drawTool()->drawSpheres(outPositions, 0.1, green);
   
}


} //namespace sofa::infinytoolkit
