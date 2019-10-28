/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <InteractionTools/config.h>

namespace sofa
{

namespace component
{

//Here are just several convenient functions to help user to know what contains the plugin

extern "C" {
    SOFA_INTERACTIONTOOLS_API void initExternalModule();
    SOFA_INTERACTIONTOOLS_API const char* getModuleName();
    SOFA_INTERACTIONTOOLS_API const char* getModuleVersion();
    SOFA_INTERACTIONTOOLS_API const char* getModuleLicense();
    SOFA_INTERACTIONTOOLS_API const char* getModuleDescription();
    SOFA_INTERACTIONTOOLS_API const char* getModuleComponentList();
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
    return "InteractionTools";
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

}

}

SOFA_LINK_CLASS(PliersToolManager)
SOFA_LINK_CLASS(PliersPositionsMapper)
