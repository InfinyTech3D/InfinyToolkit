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
#ifndef SOFA_INTERACTIONTOOLS_SURFACECARVINGMANAGER_H
#define SOFA_INTERACTIONTOOLS_SURFACECARVINGMANAGER_H

#include <InteractionTools/config.h>
#include <SofaEngine/BoxROI.h> 
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>

#include <fstream>

namespace sofa
{

//using namespace component::collision;

namespace component
{
	
namespace controller
{

class SOFA_INTERACTIONTOOLS_API SurfaceCarvingManager : public core::behavior::BaseController
{
public:
	SOFA_CLASS(SurfaceCarvingManager, sofa::core::behavior::BaseController);
	
	typedef defaulttype::Vec3Types DataTypes;
	typedef DataTypes::Coord Coord;
	typedef DataTypes::Real Real;
	typedef DataTypes::VecCoord VecCoord;
	typedef DataTypes::VecDeriv VecDeriv;
	typedef defaulttype::Vec<6, Real> Vec6;

	typedef core::CollisionModel ToolModel;
	typedef core::CollisionModel SurfaceModel;		

	typedef helper::vector<core::collision::DetectionOutput> ContactVector;
	
	Data < std::string > f_modelTool;
	Data < std::string > f_modelSurface;

	Data < bool > d_active;						// Button 1 activation
	Data < double > f_factor;
	Data <unsigned int> nb_iterations;
	
	std::set<ToolModel*> t_models;				// Burr		
	std::vector<SurfaceModel*> models;			// Target object
	
	Coord mv_direction;
	Data< sofa::defaulttype::Vector3 > tip_idx;

protected:
	core::collision::Intersection* intersectionMethod;
	core::collision::NarrowPhaseDetection* detectionNP;
	ToolModel* modelTool;
	core::CollisionModel* modelSurface;

	double radius;
	
public:
	SurfaceCarvingManager();
	~SurfaceCarvingManager();

	void bwdInit();

	void handleEvent(sofa::core::objectmodel::Event* event);
	
	void CheckCollisionDetection();
	void doShave_Burr(const ContactVector* contacts);

	helper::vector<int> getShavingPointIdx(const ContactVector* contacts);
	helper::vector<unsigned int> MoveContactVertex(helper::vector<int> ShavingPointIdx);
	
	// For interface 
	Data<sofa::helper::vector<Vec6> > d_alignedBoxes;

	// Interface output
	bool Coll_TF;
	Coord col_coord;
	helper::vector<Vec6> Boxes;
	helper::vector<helper::vector<unsigned int>> Indices_inROI;
	sofa::component::engine::BoxROI<DataTypes>* targetBox;
	
	// Interface - virtual function
	void getTargetBoxIndices();
	bool checkContact();
	void printBoxRoi();
	int isPointInbox();

private:
	static SurfaceCarvingManager s_Instance;
	unsigned int p_graspableIdx;
};
					
} // namespace controller
	
} // namespace component

} // namespace sofa

#endif // SOFA_INTERACTIONTOOLS_SURFACECARVINGMANAGER_H
