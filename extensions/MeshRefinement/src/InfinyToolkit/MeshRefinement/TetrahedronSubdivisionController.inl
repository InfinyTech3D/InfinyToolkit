/*******************************************************************************
 *                 - Copyright (C) 2019-Present InfinyTech3D -                 *
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
#pragma once

#include <InfinyToolkit/MeshRefinement/TetrahedronSubdivisionController.h>

#include <sofa/type/RGBAColor.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/helper/AdvancedTimer.h>

namespace sofa::infinytoolkit
{

using namespace sofa::core::topology;
using namespace sofa::component::topology;


template <class DataTypes>
TetrahedronSubdivisionController<DataTypes>::TetrahedronSubdivisionController()
    : d_performCut(initData(&d_performCut, false, "performCut", "to activate cut at the current timestep"))
    , d_cutPointA(initData(&d_cutPointA, Vec3(0.0, 0.0, 0.0), "cutPointA", "(default=[0, 0, 0])"))
    , d_cutPointB(initData(&d_cutPointB, Vec3(0.0, 0.0, 0.0), "cutPointB", "(default=[0, 0, 0])"))
    , d_cutDirection(initData(&d_cutDirection, Vec3(0.0, -1.0, 0.0), "cutDir", "(default=[0, -1, 0])"))
    , d_cutDepth(initData(&d_cutDepth, SReal(0.0), "cutDepth", "depth value"))
    , d_surfaceCut(initData(&d_surfaceCut, false, "surfaceCut", "to activate new surface cut"))
    , d_textureName(initData(&d_textureName, std::string(""), "textureName", "texture to apply on the surface created by the cut"))
    , d_testID(initData(&d_testID, std::set<unsigned int>(), "testID", "Ids of the tetrahedra subdivided by the '3' key"))
    , d_refineCriteria(initData(&d_refineCriteria, SReal(0.0), "refineCriteria", "Edge length under which an edge is not subdivided. 0 subdivides regardless of length"))
    , d_delayMode(initData(&d_delayMode, false, "delayMode", "Split the '3' key in two presses: compute the neighbourhood table, then subdivide from it"))
    , d_drawTetra(initData(&d_drawTetra, false, "drawTetra", "Draw the tetrahedra held by the subdividers"))
    , d_drawScaleTetrahedra(initData(&d_drawScaleTetrahedra, (float) 1.0, "drawScaleTetrahedra", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , d_drawDebugCut(initData(&d_drawDebugCut, false, "drawDebugCut", "draw Debug Cut infos"))
{
    this->f_listening.setValue(true);
}


template <class DataTypes>
TetrahedronSubdivisionController<DataTypes>::~TetrahedronSubdivisionController()
{
    m_refineStatus = 0;
}


template <class DataTypes>
void TetrahedronSubdivisionController<DataTypes>::init()
{
    m_topoCon = this->getContext()->template get<TetrahedronSetTopologyContainer>();
    if (m_topoCon == nullptr)
    {
        msg_error() << "No topology found";
        m_controllerReady = false;
        return;
    }

    if (m_mgr == nullptr)
    {
        m_mgr = std::make_unique<sofa::meshrefinement::MeshRefinementAPI<DataTypes> >();
        m_controllerReady = m_mgr->init(this->getContext());
        m_mgr->activateLogs(this->f_printLog.getValue());

        if (d_surfaceCut.getValue())
        {
            const std::string& textName = d_textureName.getValue();
            if (!textName.empty())
                m_mgr->setCutTextureName(textName);
        }
    }
}


template <class DataTypes>
bool TetrahedronSubdivisionController<DataTypes>::prepareCutFromPlane(const Vec3& pointA, const Vec3& pointB,
    const Vec3& direction, SReal depth)
{
    if (!m_controllerReady)
    {
        msg_error() << "Controller is not initialised, cannot cut.";
        return false;
    }

    // Create Cut quad
    sofa::type::fixed_array<Vec3, 4> planPositions;
    planPositions[0] = pointA;
    planPositions[1] = pointB;

    Vec3 cutDir = direction;
    cutDir.normalize();

    planPositions[2] = planPositions[1] + cutDir * depth;
    planPositions[3] = planPositions[0] + cutDir * depth;
    const Vec3 planNormal = (planPositions[1] - planPositions[0]).cross(cutDir);

    // Test all tetra. Thickness is a quarter of the depth, as the cutting controller
    // has always used here; it is the tolerance for snapping points onto the plane.
    m_mgr->createCutPlanPath(planPositions, planNormal, depth * 0.25);
    return true;
}


template <class DataTypes>
bool TetrahedronSubdivisionController<DataTypes>::prepareCutFromTriangles(TriangleID triangleA,
    TriangleID triangleB, SReal depth)
{
    if (!m_controllerReady)
    {
        msg_error() << "Controller is not initialised, cannot cut.";
        return false;
    }

    return m_mgr->createCutPlanPathFromTriangles(triangleA, triangleB, depth);
}


template <class DataTypes>
bool TetrahedronSubdivisionController<DataTypes>::applyCut(bool createSurface)
{
    if (!m_controllerReady)
    {
        msg_error() << "Controller is not initialised, cannot cut.";
        return false;
    }

    m_mgr->processCut(createSurface);
    return true;
}


template <class DataTypes>
bool TetrahedronSubdivisionController<DataTypes>::cutFromPlane(const Vec3& pointA, const Vec3& pointB,
    const Vec3& direction, SReal depth, bool createSurface)
{
    return prepareCutFromPlane(pointA, pointB, direction, depth) && applyCut(createSurface);
}


template <class DataTypes>
bool TetrahedronSubdivisionController<DataTypes>::cutFromTriangles(TriangleID triangleA,
    TriangleID triangleB, SReal depth, bool createSurface)
{
    return prepareCutFromTriangles(triangleA, triangleB, depth) && applyCut(createSurface);
}


template <class DataTypes>
bool TetrahedronSubdivisionController<DataTypes>::refineFullMesh()
{
    if (!m_controllerReady)
    {
        msg_error() << "Controller is not initialised, cannot refine.";
        return false;
    }

    return m_mgr->refineFullMesh();
}


template <class DataTypes>
bool TetrahedronSubdivisionController<DataTypes>::refineTetrahedra(const std::set<unsigned int>& ids, SReal criteria)
{
    if (!m_controllerReady)
    {
        msg_error() << "Controller is not initialised, cannot refine.";
        return false;
    }

    // Neither this controller nor the manager used to check the ids, and an id past
    // the end is read straight out of the tetrahedron array: an access violation, not
    // an error. SimpleCubeRefinement.scn shipped testID="60" against a 44 tetrahedron
    // mesh, so pressing its refine key crashed. Guard at the API boundary.
    const auto nbTetrahedra = m_topoCon->getNbTetrahedra();
    for (const unsigned int id : ids)
    {
        if (id >= nbTetrahedra)
        {
            msg_error() << "Tetrahedron id out of range: " << id << " while the topology has "
                        << nbTetrahedra << " tetrahedra. Nothing refined.";
            return false;
        }
    }

    return m_mgr->refineTetrahedra(ids, criteria);
}


template <class DataTypes>
bool TetrahedronSubdivisionController<DataTypes>::prepareCutFromData()
{
    return prepareCutFromPlane(d_cutPointA.getValue(), d_cutPointB.getValue(),
                               d_cutDirection.getValue(), d_cutDepth.getValue());
}


template <class DataTypes>
void TetrahedronSubdivisionController<DataTypes>::refineFromData()
{
    if (!m_controllerReady)
        return;

    if (d_delayMode.getValue() && m_refineStatus == 0)
    {
        m_mgr->computeNeighboorhoodTable(d_testID.getValue(), d_refineCriteria.getValue());
        m_refineStatus++;
    }
    else
    {
        refineTetrahedra(d_testID.getValue(), d_refineCriteria.getValue());
        m_refineStatus = 0;
    }
}


template <class DataTypes>
void TetrahedronSubdivisionController<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (!m_controllerReady)
        return;

    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        dmsg_info() << "GET KEY " << ev->getKey();

        switch (ev->getKey())
        {
        case '1':
            prepareCutFromData();
            break;
        case '2':
            applyCut(d_surfaceCut.getValue());
            break;
        case '3':
            refineFromData();
            break;
        case '4':
            refineFullMesh();
            break;
        default:
            break;
        }
    }

    if (sofa::simulation::AnimateEndEvent::checkEventType(event))
    {
        if (d_performCut.getValue())
        {
            cutFromPlane(d_cutPointA.getValue(), d_cutPointB.getValue(),
                         d_cutDirection.getValue(), d_cutDepth.getValue(),
                         d_surfaceCut.getValue());

            d_performCut.setValue(false);
        }
    }
}


template <class DataTypes>
void TetrahedronSubdivisionController<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!m_controllerReady)
        return;

    [[maybe_unused]] auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();

    if (d_drawTetra.getValue())
    {
        // The engine draws its own buffers; the controller no longer reaches into them.
        m_mgr->drawSubdividedTetrahedra(vparams);
    }

    if (d_drawDebugCut.getValue())
    {
        m_mgr->drawCutPlan(vparams);
        m_mgr->drawDebugCut(vparams);
    }
}

} // namespace sofa::infinytoolkit
