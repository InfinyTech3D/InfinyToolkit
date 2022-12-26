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

#include <InfinyToolkit/CarvingTools/CuttingPerformer.h>
#include <InfinyToolkit/CarvingTools/AdvancedCarvingManager.h>

#include <sofa/component/statecontainer/MechanicalObject.h>

namespace sofa::infinytoolkit
{

CuttingPerformer::CuttingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr)
    : BaseCarvingPerformer(topo, _carvingMgr)
{

}


bool CuttingPerformer::initPerformer()
{
    m_topoModif = m_topologyCon->getContext()->get<TetrahedronSetTopologyModifier>();

    if (m_topoModif == nullptr) {
        msg_error("CuttingPerformer") << "InitPerformer failed, no TetrahedronSetTopologyModifier found in Node: " << m_topologyCon->getContext()->getName();
        return false;
    }
    
    if (m_tetraCuttingMgr == nullptr)
    {
        m_tetraCuttingMgr = std::make_unique<sofa::meshrefinement::TetrahedronCuttingManager<sofa::defaulttype::Vec3Types> >();
        m_tetraCuttingMgr->init(m_topologyCon->getContext());
        m_tetraCuttingMgr->activateLogs(m_topologyCon->f_printLog.getValue());

        //if (d_surfaceCut.getValue())
        //{
        //    const std::string& textName = d_textureName.getValue();
        //    if (!textName.empty())
        //        m_tetraCuttingMgr->setCutTextureName(textName);
        //}
    }

    return true;
}


void CuttingPerformer::filterContacts()
{
    // Create Cut quad
    fixed_array<Vec3, 4> m_planPositions;

    // we use the cutting tool points to detect the direction and length of cut
    const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();

    // 1- get barycenter and cutting position
    std::vector<Vec3> cutPositions;
    Vec3 bary = Vec3(0, 0, 0);
    Vec3 cutDir = Vec3(0, 0, 0);
    for (const contactInfo* cInfo : m_triangleContacts)
    {
        cutPositions.push_back(cInfo->pointA);
        bary += cInfo->pointA;
        cutDir += cInfo->pointB - cInfo->pointA;
    }

    if (cutPositions.empty())
        return;

    bary /= cutPositions.size();
    cutDir /= cutPositions.size();
    cutDir.normalize();

    // 2- get cut extremity
    SReal maxLength = 0.0;
    for (const contactInfo* cInfo : m_triangleContacts)
    {
        Vec3 toolDir = cInfo->pointA - bary;
        SReal length2 = toolDir.norm2();
        if (maxLength < length2)
        {
            maxLength = length2;
            m_planPositions[0] = bary + toolDir;
            m_planPositions[1] = bary - toolDir;
        }
    }

    m_planPositions[2] = m_planPositions[1] + cutDir * _carvingDistance;
    m_planPositions[3] = m_planPositions[0] + cutDir * _carvingDistance;
    Vec3 m_planNormal = (m_planPositions[1] - m_planPositions[0]).cross(cutDir);

    // Test all tetra
    m_tetraCuttingMgr->createCutPlanPath(m_planPositions, m_planNormal, _carvingDistance * 10);
}

bool CuttingPerformer::runPerformer()
{
    m_tetraCuttingMgr->processCut(1);

    m_triangleContacts.clear();
    m_pointContacts.clear();
    return true;
}


void CuttingPerformer::draw(const core::visual::VisualParams* vparams)
{
    if (!m_triangleContacts.empty())
    {
        const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
        const SReal& _refineDistance = m_carvingMgr->d_refineDistance.getValue();

        for (const contactInfo* cInfo : m_triangleContacts)
        {
            std::vector<Vec3> pos;
            sofa::core::behavior::BaseMechanicalState* mstate = m_topologyCon->getContext()->getMechanicalState();
            sofa::core::topology::Topology::Triangle tri = m_topologyCon->getTriangle(cInfo->elemId);

            for (unsigned int j = 0; j < 3; j++) {
                pos.push_back(Vec3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
            }

            sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
            if (cInfo->dist < _carvingDistance)
                color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
            else if (cInfo->dist < _refineDistance)
                color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);

            vparams->drawTool()->drawTriangle(pos[0], pos[1], pos[2], cInfo->normal, color4);

            vparams->drawTool()->drawSphere(cInfo->pointB, float(_carvingDistance), sofa::type::RGBAColor(1.0f, 0.0f, 0.0f, 0.8f));
            vparams->drawTool()->drawSphere(cInfo->pointA, float(_carvingDistance), sofa::type::RGBAColor(0.0f, 1.0f, 0.0f, 0.8f));
        }

        m_tetraCuttingMgr->drawCutPlan(vparams);
        m_tetraCuttingMgr->drawDebugCut(vparams);

    }



}

} // namespace sofa::infinytoolkit
