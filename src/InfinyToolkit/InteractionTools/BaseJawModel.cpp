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

namespace sofa::infinytoolkit
{

BaseJawModel::BaseJawModel()
    : l_jawModel(initLink("jawModel", "link to the first jaw model component, if not set will search through graph and take first one encountered."))
{

}


bool BaseJawModel::computeBoundingBox()
{
    if (m_jaw == nullptr)
    {
        msg_info() << "error mechanical state not found";
        const std::string& pathMord1 = l_jawModel.getPath();
        this->getContext()->get(m_jaw, pathMord1);

        if (m_jaw == nullptr)
            return false;
    }

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


void BaseJawModel::activeTool(bool value)
{
    m_isActivated = value;
    if (m_isActivated)
        activateImpl();
    else
        deActivateImpl();
}


void BaseJawModel::performAction()
{

}

void BaseJawModel::computeAxis()
{
    zero = Vec3(0, 0, 0);
    xAxis = Vec3(1, 0, 0);
    yAxis = Vec3(0, 1, 0);
    zAxis = Vec3(0, 0, 1);

    if (m_jaw == nullptr)
        return;

    zero = Vec3(m_jaw->getPX(0), m_jaw->getPY(0), m_jaw->getPZ(0));
    xAxis = Vec3(m_jaw->getPX(1), m_jaw->getPY(1), m_jaw->getPZ(1));
    yAxis = Vec3(m_jaw->getPX(20), m_jaw->getPY(20), m_jaw->getPZ(20));
    zAxis = Vec3(m_jaw->getPX(100), m_jaw->getPY(100), m_jaw->getPZ(100));

    Vec3 xDir = (xAxis - zero); xDir.normalize();
    Vec3 yDir = (yAxis - zero); yDir.normalize();
    Vec3 zDir = (zAxis - zero); zDir.normalize();

    matP = sofa::type::Mat3x3(xDir, yDir, zDir);
}

} // namespace sofa::infinytoolkit
