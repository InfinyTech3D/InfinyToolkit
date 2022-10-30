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

#include <InfinyToolkit/config.h>

namespace sofa::infinytoolkit
{

//Here are just several convenient functions to help user to know what contains the plugin

extern "C" {
    SOFA_INFINYTOOLKIT_API void initExternalModule();
    SOFA_INFINYTOOLKIT_API const char* getModuleName();
    SOFA_INFINYTOOLKIT_API const char* getModuleVersion();
    SOFA_INFINYTOOLKIT_API const char* getModuleLicense();
    SOFA_INFINYTOOLKIT_API const char* getModuleDescription();
    SOFA_INFINYTOOLKIT_API const char* getModuleComponentList();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

const char* getModuleName()
{
    return "InfinyToolkit";
}

const char* getModuleVersion()
{
    return "0.1";
}

const char* getModuleLicense()
{
    return "";
}


const char* getModuleDescription()
{
    return "plugin to interact with Sleeve tool.";
}

const char* getModuleComponentList()
{
    return "W3CDriver";
}

} // namespace sofa::infinytoolkit

SOFA_LINK_CLASS(PliersToolManager)
SOFA_LINK_CLASS(PliersPositionsMapper)
