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

#include <InfinyToolkit/InteractionTools/ScissorJawModel.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyModifier.h>


namespace sofa::infinytoolkit
{

using namespace defaulttype;
using namespace sofa::core::topology;
using namespace sofa::component::topology::container::dynamic;


int ScissorJawModel::cutFromTetra(float minX, float maxX, bool cut)
{
    if (m_idBroadPhase.empty())
        return 10000;

    bool lastCut = true;

    // Classify right/left points of the plier
    sofa::type::vector<int> idsLeft;
    sofa::type::vector<int> idsRight;
    for (int i = 0; i < m_idBroadPhase.size(); i++)
    {
        Vec3 vert = Vec3(m_jaw->getPX(m_idBroadPhase[i]), m_jaw->getPY(m_idBroadPhase[i]), m_jaw->getPZ(m_idBroadPhase[i]));
        vert = matP * (vert - zero);

        if (vert[2] < -20.0 || vert[2] > 20.0) // outside on the borders
            continue;

        // backward test
        if (vert[0] < minX)
        {
            //if (vert[2] > -5.0 || vert[2] < 5.0)
            //	return -10000;
            //else
            continue;
        }

        // frontward test
        if (vert[0] > maxX)
        {
            if (vert[2] > -5.0 || vert[2] < 5.0)
                lastCut = false;

            continue;
        }

        if (vert[2] >= -20.0 && vert[2] < 0.0)
            idsLeft.push_back(m_idBroadPhase[i]);
        else if (vert[2] >= 0.0 && vert[2] < 20.0)
            idsRight.push_back(m_idBroadPhase[i]);
    }

    if (idsLeft.size() == 0 || idsRight.size() == 0)
        return 20000;

    msg_info() << "idsLeft: " << idsLeft.size();
    msg_info() << "idsRight: " << idsRight.size();

    // Detect all tetra on the cut path
    TetrahedronSetTopologyContainer* tetraCon;
    m_model->getContext()->get(tetraCon);
    if (tetraCon == nullptr) {
        msg_info() << "Error: NO tetraCon";
        return -40;
    }

    // First get all tetra that are on the first side
    sofa::type::vector<unsigned int> tetraIds;
    for (int i = 0; i < idsLeft.size(); ++i)
    {
        const BaseMeshTopology::TetrahedraAroundVertex& tetraAV = tetraCon->getTetrahedraAroundVertex(idsLeft[i]);
        for (int j = 0; j < tetraAV.size(); ++j)
        {
            int tetraId = tetraAV[j];
            bool found = false;
            for (int k = 0; k < tetraIds.size(); ++k)
                if (tetraIds[k] == tetraId)
                {
                    found = true;
                    break;
                }

            if (!found)
                tetraIds.push_back(tetraId);
        }
    }

    msg_info() << "tetraIds: " << tetraIds.size();


    // Then test for each tetra if one of the vertex is on the other side. If yes put on but path
    tetraIdsOnCut.clear();
    std::set< unsigned int > items;
    for (int i = 0; i < tetraIds.size(); ++i)
    {
        const BaseMeshTopology::Tetra& tetra = tetraCon->getTetra(tetraIds[i]);
        for (unsigned int j = 0; j < 4; ++j)
        {
            int idV = tetra[j];
            bool found = false;
            for (unsigned int k = 0; k < idsRight.size(); ++k)
            {
                if (idsRight[k] == idV)
                {
                    found = true;
                    break;
                }
            }

            if (found) {
                tetraIdsOnCut.push_back(tetraIds[i]);
                items.insert(tetraIds[i]);
                continue;
            }
        }
    }

    if (cut)
    {
        msg_info() << "tetraIdsOnCut: " << tetraIdsOnCut.size();
        TetrahedronSetTopologyModifier* tetraModif;
        m_model->getContext()->get(tetraModif);

        if (tetraModif == nullptr) {
            msg_info() << "Error: NO tetraModif";
            return -45;
        }


        sofa::type::vector<unsigned int> vitems;
        vitems.reserve(items.size());
        vitems.insert(vitems.end(), items.rbegin(), items.rend());

        for (int i = 0; i < vitems.size(); i++)
        {
            sofa::type::vector<sofa::core::topology::Topology::TetrahedronID> its;
            its.push_back(vitems[i]);
            tetraModif->removeTetrahedra(its);
        }

        //vitems.resize(30);

    }
    else
    {
        m_idBroadPhase.clear();
        m_idBroadPhase.insert(m_idBroadPhase.end(), items.rbegin(), items.rend());
    }

    if (lastCut)
        return 40000;

    return int(items.size());
}

int ScissorJawModel::pathCutFromTetra(float minX, float maxX)
{
    int res = cutFromTetra(minX, maxX, false);
    if (res > 1000)
        return 0;

    sofa::type::vector<int> tetraIds = m_idBroadPhase;
    m_idgrabed.clear();
    TetrahedronSetTopologyContainer* tetraCon;
    m_model->getContext()->get(tetraCon);
    if (tetraCon == nullptr) {
        msg_info() << "Error: NO tetraCon";
        return -40;
    }

    for (int i = 0; i < tetraIds.size(); i++)
    {
        const BaseMeshTopology::Tetra& tetra = tetraCon->getTetra(tetraIds[i]);
        for (int j = 0; j < 4; j++)
        {
            bool found = false;
            int idV = tetra[j];
            for (unsigned int k = 0; k < m_idgrabed.size(); ++k)
            {
                if (m_idgrabed[k] == idV)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
                m_idgrabed.push_back(idV);

        }
    }

    return int(m_idgrabed.size());
}


void ScissorJawModel::cutFromTriangles()
{
    // Classify right/left points of the plier
    sofa::type::vector<int> idsLeft;
    sofa::type::vector<int> idsRight;
    for (int i = 0; i < m_idgrabed.size(); i++)
    {
        Vec3 vert = Vec3(m_model->getPX(m_idgrabed[i]), m_model->getPY(m_idgrabed[i]), m_model->getPZ(m_idgrabed[i]));
        vert = matP * (vert - zero);

        if (vert[0] < 0.0 || vert[0] > 8.0)
            continue;

        if (vert[2] >= -2.0 && vert[2] < 1.0)
            idsLeft.push_back(m_idgrabed[i]);
        else if (vert[2] >= 1.0 && vert[2] < 3.0)
            idsRight.push_back(m_idgrabed[i]);
    }

    msg_info() << "idsLeft: " << idsLeft.size();
    msg_info() << "idsRight: " << idsRight.size();

    // Detect all tetra on the cut path
    std::vector<TriangleSetTopologyContainer*> triCons;
    m_model->getContext()->get<TriangleSetTopologyContainer>(&triCons, sofa::core::objectmodel::BaseContext::SearchDown);

    if (triCons.size() < 2) {
        msg_info() << "Error: NO triCons";
        return;
    }

    const sofa::type::vector<BaseMeshTopology::Triangle>& allTri = triCons[1]->getTriangleArray();
    for (int i = 0; i < allTri.size(); ++i)
    {
        const BaseMeshTopology::Triangle& tri = allTri[i];
        bool foundLeft = false;
        bool foundRight = false;
        for (int j = 0; j < 3; ++j)
        {
            int idV = tri[j];
            for (int k = 0; k < idsLeft.size(); ++k)
            {
                if (idsLeft[k] == idV)
                {
                    foundLeft = true;
                    break;
                }
            }

            if (foundLeft)
                break;
        }

        if (!foundLeft)
            continue;

        msg_info() << "found: " << i;
        for (int j = 0; j < 3; ++j)
        {
            int idV = tri[j];
            for (int k = 0; k < idsRight.size(); ++k)
            {
                if (idsRight[k] == idV)
                {
                    foundRight = true;
                    break;
                }
            }

            if (foundRight) {
                triIdsOnCut.push_back(i);
                break;
            }
        }
    }

    msg_info() << "triIdsOnCut: " << triIdsOnCut.size();
    std::vector<TriangleSetTopologyModifier*> triModifs;
    m_model->getContext()->get<TriangleSetTopologyModifier>(&triModifs, sofa::core::objectmodel::BaseContext::SearchDown);

    if (triModifs.size() < 2) {
        msg_info() << "Error: NO triModif";
        return;
    }
    msg_info() << "FOUND: " << triModifs.size();
    //tetraIdsOnCut.resize(30);
    triModifs[1]->removeItems(triIdsOnCut);
}


} // namespace sofa::infinytoolkit
