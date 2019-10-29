/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2015 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_INTERACTIONTOOLS_ROTATIONENGINE_H
#define SOFA_INTERACTIONTOOLS_ROTATIONENGINE_H

#include <InteractionTools/config.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/behavior/BaseController.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/simulation/Node.h>

#include <fstream>

namespace sofa
{

//using namespace component::collision;

namespace component
{
	
namespace engine
{
    using core::behavior::BaseMechanicalState;
    using sofa::defaulttype::Vec3Types;
    using sofa::defaulttype::Vector3;
    using sofa::core::DataEngine;

class SOFA_INTERACTIONTOOLS_API RotationEngine : public DataEngine
{
public:
	SOFA_CLASS(RotationEngine, DataEngine);
	
	typedef defaulttype::Vec3Types DataTypes;
	typedef DataTypes::Coord Coord;
	typedef DataTypes::Real Real;
	typedef DataTypes::VecCoord VecCoord;
    typedef sofa::component::container::MechanicalObject<Vec3Types> MechanicalObject3;
			
	
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
    Data< Vector3 > m_rotation;

protected:
    void process();

    std::vector<MechanicalObject3*> m_mstates;
    sofa::simulation::Node::SPtr m_toolNode;
};
					
} // namespace engine
	
} // namespace component

} // namespace sofa

#endif // SOFA_INTERACTIONTOOLS_ROTATIONENGINE_H
