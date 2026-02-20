/*****************************************************************************
 *                - Copyright (C) 2020-Present InfinyTech3D -                *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework.     *
 *                                                                           *
 * This file is dual-licensed:                                               *
 *                                                                           *
 * 1) Commercial License:                                                    *
 *      This file may be used under the terms of a valid commercial license  *
 *      agreement provided wih the software by InfinyTech3D.                 *
 *                                                                           *
 * 2) GNU General Public License (GPLv3) Usage                               *
 *      Alternatively, this file may be used under the terms of the          *
 *      GNU General Public License version 3 as published by the             *
 *      Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  *
 *                                                                           *
 * Contact: contact@infinytech3d.com                                         *
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
