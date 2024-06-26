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
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/type/Vec.h>

namespace sofa::infinytoolkit
{
    
using namespace sofa::defaulttype;
using namespace sofa::type;

class contactInfo
{
public:
    unsigned int idA; // in global mesh
    unsigned int idB; // in global mesh
    Vec3 pointA;
    Vec3 pointB;
    Vec3 normal; // equal to ||pB - pA||
    double dist; // equalt to (pB - pA).norm - contactDistance
};

/** 
* 
*/
class SOFA_INFINYTOOLKIT_API CollisionDetectionDisplay : public sofa::core::behavior::BaseController
{
public:
    typedef defaulttype::Vec3Types DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;

    using ContactVector = type::vector<core::collision::DetectionOutput>;

    CollisionDetectionDisplay();
    virtual ~CollisionDetectionDisplay();

    // link to the scene detection Method component (Narrow phase only)
    SingleLink<CollisionDetectionDisplay, core::collision::NarrowPhaseDetection, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_detectionNP;

    Data<bool> d_drawContacts; ///< if true, draw the collision outputs
    Data<type::vector<Vec3> > d_collisionPoints; ///< pair of collision points
public:
	/// Sofa API methods
    virtual void init() override;

    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;

    //display debug draw info
    void draw(const core::visual::VisualParams* vparams);

    /// Return the number of contact meshes
    size_t numberOfContacts() { return m_contactInfos.size(); }

protected:
    void clearContacts();

    void filterCollision();

protected:
    /// List of contacts filter during collision 
    sofa::type::vector<contactInfo*> m_contactInfos;
};

} // namespace sofa::infinytoolkit
