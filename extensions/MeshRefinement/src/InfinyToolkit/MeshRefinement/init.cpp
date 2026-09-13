/*******************************************************************************
 *                 - Copyright (C) 2026-Present InfinyTech3D -                 *
 *                             All Rights Reserved                             *
 *                                                                             *
 * The contents of this file are confidential and proprietary to InfinyTech3D. *
 * Unauthorized copying, modification, distribution or disclosure of this      *
 * file, via any medium, is strictly prohibited.                               *
 *                                                                             *
 * This file may only be used under the terms and conditions of a valid        *
 * commercial license agreement with InfinyTech3D.                             *
 *                                                                             *
 * For licensing inquiries: contact@infinytech3d.com                           *
 * Further information: https://infinytech3d.com                               *
 ******************************************************************************/
#include <InfinyToolkit/MeshRefinement/init.h>
#include <MeshRefinement/initMeshRefinement.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>

namespace sofa::infinytoolkit::meshrefinement
{

extern void registerTetrahedronSubdivisionController(sofa::core::ObjectFactory* factory);
#ifdef INFINYTOOLKIT_MESHREFINEMENT_USES_SOFACUDA
extern void registerCudaTetrahedronSubdivisionController(sofa::core::ObjectFactory* factory);
#endif

extern "C" {
    SOFA_EXPORT_DYNAMIC_LIBRARY void initExternalModule();
    SOFA_EXPORT_DYNAMIC_LIBRARY const char* getModuleName();
    SOFA_EXPORT_DYNAMIC_LIBRARY const char* getModuleVersion();
    SOFA_INFINYTOOLKIT_MESHREFINEMENT_API void registerObjects(sofa::core::ObjectFactory* factory);
}

void initExternalModule()
{
    init();
}

const char* getModuleName()
{
    return sofa_tostring(SOFA_TARGET);
}

const char* getModuleVersion()
{
    return sofa_tostring(PLUGIN_VERSION);
}

void init()
{
    static bool first = true;
    if (first)
    {
        sofa::helper::system::PluginManager::getInstance().registerPlugin(sofa_tostring(SOFA_TARGET));

        // the engine this controller drives lives in MeshRefinement
        sofa::meshrefinement::initMeshRefinement();
        first = false;
    }
}

void registerObjects(sofa::core::ObjectFactory* factory)
{
    registerTetrahedronSubdivisionController(factory);
#ifdef INFINYTOOLKIT_MESHREFINEMENT_USES_SOFACUDA
    registerCudaTetrahedronSubdivisionController(factory);
#endif
}

} // namespace sofa::infinytoolkit::meshrefinement
