/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_INTERACTIONTOOLS_HAPTICECARVINGMANAGER_H
#define SOFA_INTERACTIONTOOLS_HAPTICECARVINGMANAGER_H

#include <InteractionTools/config.h>
#include <SofaCarving/CarvingManager.h>
#include <SofaHaptics/ForceFeedback.h>

namespace sofa
{

namespace component
{

namespace collision
{

/**
* The CarvingManager class will perform topological resection on a triangle surface (could be on top of tetrahedron topology)
* The tool performing the carving need to be represented by a collision model @sa toolCollisionModel
* The surface to be carved are also mapped on collision models @sa surfaceCollisionModels
* Detecting the collision is done using the scene Intersection and NarrowPhaseDetection pipeline.
*/
class SOFA_INTERACTIONTOOLS_API HapticCarvingManager : public sofa::component::collision::CarvingManager
{
public:
	SOFA_CLASS(HapticCarvingManager, sofa::component::collision::CarvingManager);

	typedef defaulttype::Vec3Types DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    
    typedef helper::vector<core::collision::DetectionOutput> ContactVector;
    
    /// Sofa API init method of the component
    void init() override;
 
    /// Impl method that will compute the intersection and check if some element have to be removed.
    virtual void doCarve();


protected:
    /// Default constructor
    HapticCarvingManager();

    /// Default destructor
    ~HapticCarvingManager() override;

    
protected:
    sofa::component::controller::ForceFeedback::SPtr m_forceFeedback;
    
};

} // namespace collision

} // namespace component

} // namespace sofa

#endif // SOFA_INTERACTIONTOOLS_HAPTICECARVINGMANAGER_H
