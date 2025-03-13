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
#include <sofa/helper/system/PluginManager.h>

using sofa::core::ObjectFactory;

namespace sofa::infinytoolkit
{
// Haptic and Carving
extern void registerAdvancedCarvingManager(sofa::core::ObjectFactory* factory);
extern void registerHapticCarvingManager(sofa::core::ObjectFactory* factory);
extern void registerHapticEmulator(sofa::core::ObjectFactory* factory);
extern void registerBruteForceFeedback(sofa::core::ObjectFactory* factory);

// Grasping tools
extern void registerArticulatedToolManager(sofa::core::ObjectFactory* factory);
extern void registerGrasperJawModel(sofa::core::ObjectFactory* factory);

// Engines
extern void registerCollisionDetectionDisplay(sofa::core::ObjectFactory* factory);
extern void registerNearestTexcoordsMap(sofa::core::ObjectFactory* factory);
extern void registerGridBaryCentersPositions(sofa::core::ObjectFactory* factory);

// FF
extern void registerMiddleForceField(sofa::core::ObjectFactory* factory);

// Topology
extern void registerTriangle2RefinedTriangleTopologicalMapping(sofa::core::ObjectFactory* factory);

// Old components
extern void registerNeedleTracker(sofa::core::ObjectFactory* factory);
extern void registerPliersPositionsMapper(sofa::core::ObjectFactory* factory);
extern void registerRotationEngine(sofa::core::ObjectFactory* factory);




} // namespace sofa::infinytoolkit


namespace sofa::component
{

using namespace sofa::infinytoolkit;

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
        // make sure that this plugin is registered into the PluginManager
        sofa::helper::system::PluginManager::getInstance().registerPlugin(sofa_tostring(SOFA_TARGET));

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

void registerObjects(sofa::core::ObjectFactory* factory)
{
    // Haptic and Carving
    registerAdvancedCarvingManager(factory);
    registerHapticCarvingManager(factory);
    registerHapticEmulator(factory);
    registerBruteForceFeedback(factory);

    // Grasping tools
    registerArticulatedToolManager(factory);
    registerGrasperJawModel(factory);

    // Engines
    registerCollisionDetectionDisplay(factory);
    registerNearestTexcoordsMap(factory);
    registerGridBaryCentersPositions(factory);

    // FF
    registerMiddleForceField(factory);

    // Topology
    registerTriangle2RefinedTriangleTopologicalMapping(factory);

    // Old components
    registerNeedleTracker(factory);
    registerPliersPositionsMapper(factory);
    registerRotationEngine(factory);
}

} // namespace sofa::component
