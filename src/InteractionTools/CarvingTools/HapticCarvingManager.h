/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InteractionTools/config.h>
#include <SofaCarving/CarvingManager.h>
#include <SofaHaptics/ForceFeedback.h>

namespace sofa::component::collision
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
    
    typedef type::vector<core::collision::DetectionOutput> ContactVector;
    
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

} // namespace sofa::component::collision
