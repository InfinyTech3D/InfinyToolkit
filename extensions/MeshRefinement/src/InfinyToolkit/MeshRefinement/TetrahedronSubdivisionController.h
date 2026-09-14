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
* Cuts and refines a tetrahedral mesh, driving the MeshRefinement plugin's engine.
*
* Four operations: cut along a plane, cut between two surface triangles, refine a
* given set of tetrahedra, and refine the whole mesh.
*
* Cutting is two-phase: the path is built first and applied second, so a scene can
* draw the plane and inspect it before the topology changes. prepareCutFrom*()
* followed by applyCut() does that; cutFrom*() does both in one call.
*
* Interactive keys, used by the example scenes:
*   '1'  build the cut plane from the cutPointA / cutPointB / cutDir / cutDepth Data
*   '2'  apply the cut built by '1'
*   '3'  refine the tetrahedra listed in testID
*   '4'  refine the whole mesh
*/
template <class DataTypes>
class TetrahedronSubdivisionController : public core::behavior::BaseController
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TetrahedronSubdivisionController, DataTypes), sofa::core::behavior::BaseController);

    using Real = typename DataTypes::Real;
    using Coord = typename DataTypes::Coord;
    using VecCoord = typename DataTypes::VecCoord;

    using Vec3 = sofa::type::Vec3;
    using TetrahedronSetTopologyContainer = sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer;

    using TriangleID = sofa::core::topology::Topology::TriangleID;

    /// Sofa API init method of the component
    void init() override;

    /// Handles the key presses listed in the class description.
    void handleEvent(sofa::core::objectmodel::Event* event) override;

    /// Draws the subdivided tetrahedra and the cut plane, per @sa d_drawTetra and
    /// @sa d_drawDebugCut.
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

    /// Subdivides the tetrahedra given by @p ids. @p criteria is a fraction of the
    /// mean tetrahedron volume of the mesh: a tetrahedron whose volume is not above
    /// criteria * that mean is left alone. 0 subdivides regardless of volume.
    /// @return false if the controller is unusable.
    bool refineTetrahedra(const std::set<unsigned int>& ids, SReal criteria = 0.0);
    /// @}


protected:
    /// Default constructor
    TetrahedronSubdivisionController();

    /// Default destructor
    ~TetrahedronSubdivisionController() override;

    /// Builds the cut path from @sa d_cutPointA, @sa d_cutPointB, @sa d_cutDirection
    /// and @sa d_cutDepth. Used by the '1' key and by @sa d_performCut.
    bool prepareCutFromData();

    /// Refines the tetrahedra in @sa d_testID, honouring @sa d_delayMode.
    void refineFromData();


public:
    /// Performs a cut at the next time step, then resets itself to false.
    Data <bool> d_performCut;

    /// The cut plane: the quad spanned by the two points below, extruded along
    /// @sa d_cutDirection over @sa d_cutDepth.
    Data <Vec3> d_cutPointA; ///< First point of the cut plane
    Data <Vec3> d_cutPointB; ///< Second point of the cut plane
    Data <Vec3> d_cutDirection; ///< Direction the plane is extruded along
    Data <SReal> d_cutDepth; ///< Depth of the cut along @sa d_cutDirection

    /// Creates a new surface mesh, and the components to render it, on the cut faces.
    Data <bool> d_surfaceCut;
    /// Texture applied to the surface created by the cut. Only used when
    /// @sa d_surfaceCut is true.
    Data <std::string> d_textureName;

    /// Tetrahedra subdivided by the '3' key. @sa refineTetrahedra
    Data <std::set<unsigned int> > d_testID;
    /// Fraction of the mesh's mean tetrahedron volume under which a tetrahedron is
    /// left alone; 0 subdivides regardless of volume.
    Data <SReal> d_refineCriteria;
    /// Splits the '3' key in two presses: the first computes the neighbourhood table,
    /// the second subdivides from it.
    Data <bool> d_delayMode;

    Data <bool> d_drawTetra; ///< Draw the tetrahedra held by the subdividers
    Data <float> d_drawScaleTetrahedra; ///< Scale of the drawn tetrahedra; below 1.0 it leaves gaps between them
    Data <bool> d_drawDebugCut; ///< Draw the cut plane and the intersections it computed


private:
    /// The subdivision engine. Owned, because a cut is built by one call and applied
    /// by another and both have to reach the same engine.
    std::unique_ptr<sofa::meshrefinement::MeshRefinementAPI<DataTypes> > m_mgr = nullptr;

    /// The topology this controller operates on, used to range-check the ids given to it.
    TetrahedronSetTopologyContainer::SPtr m_topoCon = nullptr;

    /// True once init() has found a topology and the engine has initialised.
    bool m_controllerReady = false;

    /// Press counter for the two-phase '3' key. @sa d_delayMode
    int m_refineStatus = 0;
};

#if  !defined(SOFA_INFINYTOOLKIT_TETRAHEDRON_SUBDIVISION_CONTROLLER_CPP)
extern template class SOFA_INFINYTOOLKIT_MESHREFINEMENT_API TetrahedronSubdivisionController<sofa::defaulttype::Vec3Types>;
#endif

} // namespace sofa::infinytoolkit
