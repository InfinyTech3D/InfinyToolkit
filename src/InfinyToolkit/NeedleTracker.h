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
#pragma once

#include <InfinyToolkit/config.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/type/Vec.h>
#include <sofa/component/statecontainer/MechanicalObject.h>

namespace sofa::infinytoolkit
{
    
using namespace sofa::defaulttype;

/** Will search for MechanicalObject with Tag "slice" as target meshes.
* Use the position of the MechanicalObject Tag as "needle".
* At each step: 
*  - BB of each target mesh is updated (TODO: this can be optimised)
*  - Check if position of the needle is inside a BB: BroadPhase
*    - if position is inside the BB mesh (could have several), cast ray from needle position to first triangle.
*      if an odd number of triangle is intersected by the ray, needle is inside mesh otherwise outside.
* If needle is inside a mesh, @sa d_sliceName contain the name of this mesh. If none, d_sliceName return "None".
*/
class SOFA_INFINYTOOLKIT_API NeedleTracker : public sofa::core::behavior::BaseController
{
public:
    typedef defaulttype::Vec3Types DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Vec3Types> MechanicalObject3;

    NeedleTracker();
    virtual ~NeedleTracker();

    Data<bool> d_drawDebug; ///< if true, draw the Mesh BoundingBox, ray and triangle intersected
    Data<int> d_sliceID; /// id of the mesh (in @sa m_slices) where the needle is. -1 if none
    Data<std::string> d_sliceName; /// Name of the mesh where the needle is. "None" if needle is outside of all meshes.
public:
	/// Sofa API methods
    virtual void init() override;

    virtual void bwdInit() override;

    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;

    //display debug draw info
    void draw(const core::visual::VisualParams* vparams);

    /// Return the number of targeted meshes
    size_t numberOfSlice() { return m_slices.size(); }
    /// Return the name of the Slice where the needle is. "None"if needle is outside from all meshes.
    std::string getPositionInSlices(const Coord& tipPosition);    

protected:
    /// BroadPhase method, test needle position with slice BB
    bool testSliceBBIntersection(int sliceID, const Coord& tipPosition);
    /// NarrowPhase method, test needle position with slice mesh triangles
    bool testSliceDiscretIntersection(int sliceID, const Coord& tipPosition);


    /// Internal method to compute each slice BB
    void computeSlicesBB();
    /// Internal method to test the intersection between a ray and a triangle
    bool RayIntersectsTriangle(const Coord& origin, const type::Vec3& rayDirection, const Coord& P0, const Coord& P1, const Coord& P2, Coord& outIntersection);


protected:
    /// Vector storing pointer to each mechanicalObject of the target mesh
    sofa::type::vector<MechanicalObject3*> m_slices;
    /// Pointer to the MechanicalObject corresponding to the needle position
    MechanicalObject3* m_needle;

    /// Vector of min and max of the slice boudingboxes
    sofa::type::vector<Coord> m_min;
    sofa::type::vector<Coord> m_max;
    
    /// Origin and direction of the ray (for draw debug only)
    Coord m_rayDirection;
    Coord m_rayOrigin;

    /// vector of triangles positions intersected by narrow phase ray.
    sofa::type::vector <Coord> m_triPointInter;

    bool isInit;
};

} // namespace sofa::infinytoolkit
