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

#include <InfinyToolkit/initInfinyToolkit.h>

#include <sofa/core/ObjectFactory.h>
using sofa::core::ObjectFactory;

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

void initInfinyToolkit()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

void initExternalModule()
{
    initInfinyToolkit();
}

const char* getModuleName()
{
    return sofa_tostring(SOFA_TARGET);
}

const char* getModuleVersion()
{
    return sofa_tostring(PLUGIN_VERSION);
}

const char* getModuleLicense()
{
    return "GPL";
}


const char* getModuleDescription()
{
    return "plugin to add several component tools.";
}

const char* getModuleComponentList()
{
    static std::string classes = ObjectFactory::getInstance()->listClassesFromTarget(sofa_tostring(SOFA_TARGET));
    return classes.c_str();
}

} // namespace sofa::infinytoolkit
