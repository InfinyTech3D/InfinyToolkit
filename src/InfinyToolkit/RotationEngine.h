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
#include <sofa/core/DataEngine.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/simulation/Node.h>

#include <fstream>

namespace sofa::infinytoolkit
{
    using core::behavior::BaseMechanicalState;
    using sofa::defaulttype::Vec3Types;
    using sofa::type::Vec3;
    using sofa::core::DataEngine;

class SOFA_INFINYTOOLKIT_API RotationEngine : public DataEngine
{
public:
	SOFA_CLASS(RotationEngine, DataEngine);
	
	typedef defaulttype::Vec3Types DataTypes;
	typedef DataTypes::Coord Coord;
	typedef DataTypes::Real Real;
	typedef DataTypes::VecCoord VecCoord;
    typedef sofa::component::statecontainer::MechanicalObject<Vec3Types> MechanicalObject3;
			
	
public:
    RotationEngine();
	~RotationEngine();

    void init() override;
    void reinit() override;
    void doUpdate() override;

    bool findNode(sofa::simulation::Node::SPtr node);

	void bwdInit();

	void handleEvent(sofa::core::objectmodel::Event* event);

public:
    Data< Vec3 > m_rotation;

protected:
    void process();

    std::vector<MechanicalObject3*> m_mstates;
    sofa::simulation::Node::SPtr m_toolNode;
};
					
} // namespace sofa::infinytoolkit
