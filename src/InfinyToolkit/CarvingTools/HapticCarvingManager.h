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
#include <InfinyToolkit/CarvingTools/AdvancedCarvingManager.h>
#include <sofa/component/haptics/ForceFeedback.h>

namespace sofa::infinytoolkit
{

/**
* The CarvingManager class will perform topological resection on a triangle surface (could be on top of tetrahedron topology)
* The tool performing the carving need to be represented by a collision model @sa toolCollisionModel
* The surface to be carved are also mapped on collision models @sa surfaceCollisionModels
* Detecting the collision is done using the scene Intersection and NarrowPhaseDetection pipeline.
*/
class SOFA_INFINYTOOLKIT_API HapticCarvingManager : public AdvancedCarvingManager
{
public:
	SOFA_CLASS(HapticCarvingManager, AdvancedCarvingManager);

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
    sofa::component::haptics::ForceFeedback::SPtr m_forceFeedback;
    
};

} // namespace sofa::infinytoolkit
