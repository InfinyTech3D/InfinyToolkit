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

#include <InfinyToolkit/InteractionTools/BaseJawModel.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa::infinytoolkit
{

BaseJawModel::BaseJawModel()
    : l_jawController(initLink("jawController", "link to the first jaw model component, if not set will search through graph and take first one encountered."))
    , l_jawDofs(initLink("jawDofs", "link to the first jaw model component, if not set will search through graph and take first one encountered."))
    , l_jawCollision(initLink("jawCollision", "link to the first jaw model component, if not set will search through graph and take first one encountered."))
{

}

void BaseJawModel::init()
{
    m_jaw = l_jawDofs.get();

    if (m_jaw == nullptr)
    {
        msg_error() << "Error mechanical state not given";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
    }

    if (initImpl())
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
    else
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
}


bool BaseJawModel::computeBoundingBox()
{
    if (m_jaw == nullptr)
        return false;

    for (int i = 0; i < 3; ++i)
    {
        m_min[i] = 10000;
        m_max[i] = -10000;
    }


    for (Index i = 0; i < m_jaw->getSize(); i++)
    {
        SReal x = m_jaw->getPX(i);
        SReal y = m_jaw->getPY(i);
        SReal z = m_jaw->getPZ(i);

        if (x < m_min[0])
            m_min[0] = x;
        if (y < m_min[1])
            m_min[1] = y;
        if (z < m_min[2])
            m_min[2] = z;

        if (x > m_max[0])
            m_max[0] = x;
        if (y > m_max[1])
            m_max[1] = y;
        if (z > m_max[2])
            m_max[2] = z;
    }

    return true;
}


void BaseJawModel::addContact(GrabContactInfo* grabInfo)
{
    m_contactInfos.push_back(grabInfo);
}


void BaseJawModel::clearContacts()
{
    for (unsigned int i = 0; i < m_contactInfos.size(); i++)
    {
        delete m_contactInfos[i];
        m_contactInfos[i] = nullptr;
    }
    m_contactInfos.clear();
}


void BaseJawModel::activeTool(bool value)
{
    m_isActivated = value;
    if (m_isActivated)
        activateImpl();
    else
        deActivateImpl();
}


void BaseJawModel::computeAxis()
{
    m_origin = Vec3(0, 0, 0);
    m_xAxis = Vec3(1, 0, 0);
    m_yAxis = Vec3(0, 1, 0);
    m_zAxis = Vec3(0, 0, 1);

    if (m_jaw == nullptr)
        return;

    m_origin = Vec3(m_jaw->getPX(0), m_jaw->getPY(0), m_jaw->getPZ(0));
    m_xAxis = Vec3(m_jaw->getPX(1), m_jaw->getPY(1), m_jaw->getPZ(1));
    m_yAxis = Vec3(m_jaw->getPX(20), m_jaw->getPY(20), m_jaw->getPZ(20));
    m_zAxis = Vec3(m_jaw->getPX(100), m_jaw->getPY(100), m_jaw->getPZ(100));

    Vec3 xDir = (m_xAxis - m_origin); xDir.normalize();
    Vec3 yDir = (m_yAxis - m_origin); yDir.normalize();
    Vec3 zDir = (m_zAxis - m_origin); zDir.normalize();

    m_matP = sofa::type::Mat3x3(xDir, yDir, zDir);
}


void BaseJawModel::drawImpl(const core::visual::VisualParams* vparams)
{
    // draw contacts
    for (GrabContactInfo* cInfo : m_contactInfos)
    {
        std::vector<Vec3> vertices;

        for (int i = 0; i < 3; ++i)
        {
            vertices.push_back(Vec3(m_jaw->getPX(cInfo->idTool), m_jaw->getPY(cInfo->idTool), m_jaw->getPZ(cInfo->idTool)));
            vertices.push_back(Vec3(m_target->getPX(cInfo->idsModel[i]), m_target->getPY(cInfo->idsModel[i]), m_target->getPZ(cInfo->idsModel[i])));
        }

        sofa::type::RGBAColor color4(1.0f, 1.0, 0.0f, 1.0);
        vparams->drawTool()->drawLines(vertices, 10, color4);
    }
}

} // namespace sofa::infinytoolkit
