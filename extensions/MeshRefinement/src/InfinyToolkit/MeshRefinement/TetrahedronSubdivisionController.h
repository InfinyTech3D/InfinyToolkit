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

#include <InfinyToolkit/MeshRefinement/config.h>
#include <MeshRefinement/MeshRefinementAPI.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>

#include <set>
#include <string>


namespace sofa::infinytoolkit
{

/**
* Single entry point for the subdivision operations this plugin performs on a
* tetrahedral mesh: cutting along a plane or between two surface triangles, and
* refining either the whole mesh or a chosen set of tetrahedra.
*
* Replaces TetrahedronCuttingController and TetrahedronRefinementController, which
* split one engine in two along a line a caller could not see - both operations
* subdivide tetrahedra - and owned a manager each, so a scene using both ran two
* engines over one topology.
*
* The engine itself stays private to the MeshRefinement plugin. This controller
* reaches it only through MeshRefinementAPI, the one header that plugin publishes,
* which is why nothing below names a subdivider, a topology container or any other
* implementation type.
*
* Cutting is two-phase by nature - the path is built first and committed second,
* which is what lets a scene draw the plane and check it before anything changes.
* Both shapes are offered: prepareCutFrom*() then applyCut() for that, and
* cutFrom*() for callers that only want the operation done.
*
* Interactive keys, used by the example scenes:
*   '1'  build the cut plane from the cutPointA / cutPointB / cutDir / cutDepth Data
*   '2'  apply the cut built by '1'
*   '3'  refine the tetrahedra listed in testID
*   '4'  refine the whole mesh
*
* '4' is new. refineFullMesh() used to answer '2' on the refinement controller,
* which collides with applying a cut now that both live in one component.
*/
template <class DataTypes>
class TetrahedronSubdivisionController : public core::behavior::BaseController
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TetrahedronSubdivisionController, DataTypes), sofa::core::behavior::BaseController);

    using Real = typename DataTypes::Real;
    using Coord = typename DataTypes::Coord;
    using VecCoord = typename DataTypes::VecCoord;

    /// Aliased in the class rather than pulled in by a namespace-scope using: the
    /// engine header used to supply these names to everyone who included it, and the
    /// facade deliberately does not.
    using Vec3 = sofa::type::Vec3;
    using TetrahedronSetTopologyContainer = sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer;

    using TriangleID = sofa::core::topology::Topology::TriangleID;

    /// Sofa API init method of the component
    void init() override;

    /// Method to handle various event like keyboard or omni.
    void handleEvent(sofa::core::objectmodel::Event* event) override;

    void draw(const core::visual::VisualParams* vparams) override;


    /// @name Cutting
    /// @{

    /// Cuts along the quad spanned by @p pointA, @p pointB and @p direction over
    /// @p depth, building and applying the cut in one call. @p createSurface also
    /// creates the new surface mesh and its components on the cut faces.
    /// @return false if the controller is unusable or the cut could not be built.
    bool cutFromPlane(const Vec3& pointA, const Vec3& pointB, const Vec3& direction,
                      SReal depth, bool createSurface = false);

    /// Cuts between the barycentres of triangles @p triangleA and @p triangleB, over
    /// @p depth along the normal of @p triangleA, in one call.
    /// @return false if the controller is unusable or either id is out of range.
    bool cutFromTriangles(TriangleID triangleA, TriangleID triangleB,
                          SReal depth, bool createSurface = false);

    /// Builds a cut path without applying it, so that it can be drawn and inspected.
    /// @sa applyCut
    bool prepareCutFromPlane(const Vec3& pointA, const Vec3& pointB, const Vec3& direction,
                             SReal depth);

    /// Builds a cut path between two surface triangles without applying it.
    /// @sa applyCut
    bool prepareCutFromTriangles(TriangleID triangleA, TriangleID triangleB, SReal depth);

    /// Applies the path built by either prepareCutFrom* method.
    /// @return false if the controller is unusable.
    bool applyCut(bool createSurface);
    /// @}


    /// @name Refinement
    /// @{

    /// Subdivides every tetrahedron of the mesh.
    /// @return false if the controller is unusable.
    bool refineFullMesh();

    /// Subdivides the tetrahedra given by @p ids. @p criteria is the edge length
    /// under which an edge is left whole; 0 subdivides regardless of length.
    /// @return false if the controller is unusable.
    bool refineTetrahedra(const std::set<unsigned int>& ids, SReal criteria = 0.0);
    /// @}


protected:
    /// Default constructor
    TetrahedronSubdivisionController();

    /// Default destructor
    ~TetrahedronSubdivisionController() override;

    /// Builds the cut quad from the Data, for the interactive '1' key and
    /// @sa d_performCut. Same geometry as @sa prepareCutFromPlane.
    bool prepareCutFromData();

    /// Drives the '3' key, honouring @sa d_delayMode
    void refineFromData();


public:
    /// Bool to perform a cut at the current timestep
    Data <bool> d_performCut;

    // To define cut from a plan defined by 2 points, a direction and a depth
    Data <Vec3> d_cutPointA; ///< First plan point position
    Data <Vec3> d_cutPointB; ///< Second plan point position
    Data <Vec3> d_cutDirection; ///< Plan 3d direction in space
    Data <SReal> d_cutDepth; ///< Depth of the cut in the plan direction

    /// Booleen to define if new surface mesh and component will be created on cut.
    Data <bool> d_surfaceCut;
    /// Texture filename to be used on the new surface mesh created by the cut. Only used if @sa d_surfaceCut is set to true.
    Data <std::string> d_textureName;

    /// Tetrahedra subdivided by the '3' key, @sa refineTetrahedra
    Data <std::set<unsigned int> > d_testID;
    /// Edge length under which an edge is not subdivided by the '3' key. Defaults to
    /// 0, i.e. subdivide regardless, which is what the refinement controller did.
    Data <SReal> d_refineCriteria;
    /// Splits the '3' key in two presses: the first computes the neighbourhood table,
    /// the second subdivides from it.
    Data <bool> d_delayMode;

    Data <bool> d_drawTetra; ///< Draw the tetrahedra held by the subdividers
    Data <float> d_drawScaleTetrahedra; ///< Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)
    Data <bool> d_drawDebugCut; ///< Bool to draw cut plan and intersection


private:
    /// The engine, behind its public handle: one manager for both operations. Owned
    /// rather than shared, because a cut is built by one call and applied by another
    /// and both halves have to reach the same engine and the same buffers.
    std::unique_ptr<sofa::meshrefinement::MeshRefinementAPI<DataTypes> > m_mgr = nullptr;

    /// Kept for the init-time check that a tetrahedral topology is present, and for
    /// the id range checks; the operations themselves go through the manager, which
    /// holds its own handles.
    TetrahedronSetTopologyContainer::SPtr m_topoCon = nullptr;

    // Bool to store the information if component has well be init and can be used.
    bool m_controllerReady = false;

    /// Two-phase '3' key handling, @sa d_delayMode
    int m_refineStatus = 0;
};

#if  !defined(SOFA_INFINYTOOLKIT_TETRAHEDRON_SUBDIVISION_CONTROLLER_CPP)
extern template class SOFA_INFINYTOOLKIT_MESHREFINEMENT_API TetrahedronSubdivisionController<sofa::defaulttype::Vec3Types>;
#endif

} // namespace sofa::infinytoolkit
