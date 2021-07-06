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
#include <SofaEngine/BoxROI.h> 
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/defaulttype/VecTypes.h>
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
	typedef type::Vec<6, Real> Vec6;

	typedef core::CollisionModel ToolModel;
	typedef core::CollisionModel SurfaceModel;		

	typedef type::vector<core::collision::DetectionOutput> ContactVector;
	
	Data < std::string > f_modelTool;
	Data < std::string > f_modelSurface;

	Data < bool > d_active;						// Button 1 activation
	Data < double > f_factor;
	Data <unsigned int> nb_iterations;
	
	std::set<ToolModel*> t_models;				// Burr		
	std::vector<SurfaceModel*> models;			// Target object
	
	Coord mv_direction;
	Data< sofa::type::Vector3 > tip_idx;

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

	type::vector<int> getShavingPointIdx(const ContactVector* contacts);
	type::vector<unsigned int> MoveContactVertex(type::vector<int> ShavingPointIdx);
	
	// For interface 
	Data<sofa::type::vector<Vec6> > d_alignedBoxes;

	// Interface output
	bool Coll_TF;
	Coord col_coord;
	type::vector<Vec6> Boxes;
	type::vector<type::vector<unsigned int>> Indices_inROI;
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
