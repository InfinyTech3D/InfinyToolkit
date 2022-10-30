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

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/visual/VisualParams.h>

#include <InfinyToolkit/CarvingTools/BaseCarvingPerformer.h>
#include <InfinyToolkit/CarvingTools/AdvancedCarvingManager.h>

namespace sofa::infinytoolkit
{

BaseCarvingPerformer::BaseCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr)
    : m_topologyCon(topo)
    , m_carvingMgr(_carvingMgr)
{

}


BaseCarvingPerformer::~BaseCarvingPerformer()
{
    clearContacts();
}


void BaseCarvingPerformer::clearContacts()
{
    for (unsigned int i = 0; i < m_triangleContacts.size(); i++)
    {
        delete m_triangleContacts[i];
        m_triangleContacts[i] = nullptr;
    }
    m_triangleContacts.clear();

    for (unsigned int i = 0; i < m_pointContacts.size(); i++)
    {
        delete m_pointContacts[i];
        m_pointContacts[i] = nullptr;
    }
    m_pointContacts.clear();
}


void BaseCarvingPerformer::draw(const core::visual::VisualParams* vparams)
{
    if (!m_triangleContacts.empty())
    {
        const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
        const SReal& _refineDistance = m_carvingMgr->d_refineDistance.getValue();

        for (contactInfo * cInfo : m_triangleContacts)
        {
            std::vector<Vector3> pos;
            sofa::core::behavior::BaseMechanicalState* mstate = m_topologyCon->getContext()->getMechanicalState();
            sofa::core::topology::Topology::Triangle tri = m_topologyCon->getTriangle(cInfo->elemId);

            for (unsigned int j = 0; j < 3; j++) {
                pos.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
            }

            sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
            if (cInfo->dist < _carvingDistance)
                color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
            else if (cInfo->dist < _refineDistance)
                color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);

            vparams->drawTool()->drawTriangle(pos[0], pos[1], pos[2], cInfo->normal, color4);
        }
    }

    if (!m_pointContacts.empty())
    {
        const SReal& _carvingDistance = m_carvingMgr->d_carvingDistance.getValue();
        const SReal& _refineDistance = m_carvingMgr->d_refineDistance.getValue();

        for (contactInfo * cInfo : m_pointContacts)
        {
            std::vector<Vector3> pos;
            sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
            if (cInfo->dist < _carvingDistance)
                color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
            else if (cInfo->dist < _refineDistance)
                color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);

            vparams->drawTool()->drawSphere(cInfo->pointB, 0.05f, color4);
            vparams->drawTool()->drawLine(cInfo->pointB, cInfo->pointB + cInfo->normal, sofa::type::RGBAColor(1.0, 0.0, 1.0f, 1.0));
        }

        contactInfo* cInfo = m_pointContacts[0];
        vparams->drawTool()->drawSphere(cInfo->pointA, _refineDistance, sofa::type::RGBAColor(0.0f, 0.0f, 1.0f, 0.8f));
        vparams->drawTool()->drawSphere(cInfo->pointA, _carvingDistance, sofa::type::RGBAColor(0.0f, 1.0f, 0.0f, 0.8f));
    }
}

} // namespace sofa::infinytoolkit
