/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
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
