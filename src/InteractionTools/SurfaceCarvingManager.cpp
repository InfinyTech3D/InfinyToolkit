/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/SurfaceCarvingManager.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <SofaMeshCollision/TriangleModel.h>
#include <SofaMeshCollision/PointModel.h>

#include <sofa/core/topology/TopologicalMapping.h>
#include <sofa/helper/gl/template.h>
#include <SofaUserInteraction/TopologicalChangeManager.h>
#include <sofa/helper/AdvancedTimer.h>
#include <SofaLoader/MeshObjLoader.h>


namespace sofa
{

using namespace component::collision;

namespace component
{

namespace controller
{

SOFA_DECL_CLASS(SurfaceCarvingManager)

int SurfaceCarvingManagerClass = core::RegisterObject("Manager handling Shaving Burr operations between a tool and an object.")
.add< SurfaceCarvingManager >()
;

SurfaceCarvingManager::SurfaceCarvingManager()
: f_modelTool( initData(&f_modelTool, "modelTool", "Tool model path"))
, f_modelSurface( initData(&f_modelSurface, "modelSurface", "TriangleSetModel or SphereModel path"))
, modelTool(NULL)
, modelSurface(NULL)
, intersectionMethod(NULL)
, detectionNP(NULL)
, d_active(initData(&d_active, false, "active", "Activate this object."))
, f_factor(initData(&f_factor, 0.015, "f_factor", "ShavigSurface deformation ratio"))
, nb_iterations(initData(&nb_iterations, (unsigned int)1, "nb_iterations", "Number of iterations of laplacian smoothing"))
, tip_idx(initData(&tip_idx, sofa::defaulttype::Vector3(4, 0, 12), "ToolTipIndex", "Tool index to calculate moving direction"))
{
    this->f_listening.setValue(true);
}

SurfaceCarvingManager::~SurfaceCarvingManager()
{
}

SurfaceCarvingManager SurfaceCarvingManager::s_Instance;

void SurfaceCarvingManager::bwdInit()
{
    if (f_modelTool.getValue().empty())
    {
        modelTool = getContext()->get<ToolModel>(core::objectmodel::Tag("Burr"), core::objectmodel::BaseContext::SearchDown);
		if (!modelTool)
			modelTool = getContext()->get<ToolModel>(core::objectmodel::BaseContext::SearchDown);     
    }
    else
        modelTool = getContext()->get<ToolModel>(f_modelTool.getValue());

	if (f_modelSurface.getValue().empty())
	{
		std::vector<SurfaceModel*> models_tmp;
		getContext()->get<SurfaceModel>(&models_tmp, core::objectmodel::Tag("ShavingBurrSurface"), core::objectmodel::BaseContext::SearchRoot);
		for (int i = 0; i < models_tmp.size(); i++)
		{
			if (!(models_tmp[i]->getCollisionTopology()->getNbQuads()) && !(models_tmp[i]->getCollisionTopology()->getNbTetrahedra()) && !(models_tmp[i]->getCollisionTopology()->getNbHexahedra()) && (models_tmp[i]->getCollisionTopology()->getNbTriangles()))
				models.push_back(models_tmp[i]);
		}
		if (models.size())
			modelSurface = models[0];
    }
    else
    {
        modelSurface = getContext()->get<core::CollisionModel>(f_modelSurface.getValue());
    }
    intersectionMethod = getContext()->get<core::collision::Intersection>();
    detectionNP = getContext()->get<core::collision::NarrowPhaseDetection>();
    bool error = false;
    if (modelTool == NULL) { msg_error() << "SurfaceCarvingManager: modelTool not found"; error = true; }
    if (modelSurface == NULL) { msg_error() << "SurfaceCarvingManager: modelSurface not found"; error = true; }
    if (intersectionMethod == NULL) { msg_error() << "SurfaceCarvingManager: intersectionMethod not found"; error = true; }
    if (detectionNP == NULL) { msg_error() << "SurfaceCarvingManager: NarrowPhaseDetection not found"; error = true; }
	if (!error)
		msg_info() << "SurfaceCarvingManager: init OK.";
	
	// TargetBox & Index 
	getTargetBoxIndices();
    d_active.setValue(false);
}

void SurfaceCarvingManager::getTargetBoxIndices()
{
	// [start] for BoxROI - Get Target Position 
	modelSurface->getContext()->get(targetBox);
	if (targetBox)
	{
		helper::vector<Vec6> Boxes_tmp;
		sofa::component::engine::BoxROI<DataTypes>* targetBox_tmp;

		std::vector< core::objectmodel::BaseObject*> listObject;
		targetBox->getContext()->get<core::objectmodel::BaseObject>(&listObject, core::objectmodel::BaseContext::Local);

		for (unsigned int i = 0, j = 0; i < listObject.size(); ++i)
		{
			targetBox_tmp = dynamic_cast<sofa::component::engine::BoxROI<DataTypes>*>(listObject[i]);

			if (targetBox_tmp)
			{
				if (targetBox_tmp->getName() == "DrillTarget")	// Find target name of BoxROI
				{
					Boxes_tmp = targetBox_tmp->d_alignedBoxes.getValue();
					Boxes.push_back(Boxes_tmp[0]);				// Save Boxes position
					j += 1;
				}
			}
		}
		if (!Boxes.empty())
		{
			/*msg_info() << "BOXROI (size: " << Boxes.size() << ")";	// for Multiple BoxROI
			for (int i = 0; i < Boxes.size(); ++i)
			{
				msg_info() << "BoxROI [" << i << "] :" << "[ " << Boxes[i] << " ]";
			}*/
		}
		else
			msg_info() << "no DrillTarget BOXROI";
	}
	else
		msg_info() << "no BOXROI in scn";

	Coll_TF = false;
	// [end] for BoxROI - Get Target Position  collision contact
}

void SurfaceCarvingManager::CheckCollisionDetection()
{
	if (modelTool == NULL || modelSurface == NULL || intersectionMethod == NULL || detectionNP == NULL) return;

	const bool continuous = intersectionMethod->useContinuous();
	const double dt = getContext()->getDt();
	const int depth = 6;

	if (continuous)
		modelTool->computeContinuousBoundingTree(dt, depth);
	else
		modelTool->computeBoundingTree(depth);

	for (unsigned int i = 0; i < models.size(); i++)
	{
		modelSurface = models[i];
		if (continuous)
			modelSurface->computeContinuousBoundingTree(dt, depth);
		else
			modelSurface->computeBoundingTree(depth);

		sofa::helper::vector<std::pair<core::CollisionModel*, core::CollisionModel*> > vectCMPair;
		vectCMPair.push_back(std::make_pair(modelSurface->getFirst(), modelTool->getFirst()));

		detectionNP->setInstance(this);
		detectionNP->setIntersectionMethod(intersectionMethod);
		detectionNP->beginNarrowPhase();
		detectionNP->addCollisionPairs(vectCMPair);
		detectionNP->endNarrowPhase();

		const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = detectionNP->getDetectionOutputs();

		const ContactVector* contacts = NULL;
		core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); //find(std::make_pair(modelSurface,modelTool));
		if (it != detectionOutputs.end())
		{
			contacts = dynamic_cast<const ContactVector*>(it->second);
		}
		size_t ncontacts = 0;
		if (contacts != NULL)
		{
			Coll_TF = true;
			ncontacts = contacts->size();
			
			// Get Tip position - for interface 
			sofa::core::topology::BaseMeshTopology* topo_curr;
			topo_curr = modelTool->getContext()->getMeshTopology();
			core::behavior::MechanicalState<sofa::defaulttype::Vec3Types>* mstate = topo_curr->getContext()->get<core::behavior::MechanicalState<sofa::defaulttype::Vec3Types> >();
			helper::WriteAccessor< Data<VecCoord> > x_wA = *mstate->write(core::VecCoordId::position());

			unsigned int c_point = tip_idx.getValue()[0];// 0
			col_coord = x_wA[c_point];

			if (dynamic_cast<sofa::component::collision::TriangleModel*>(modelSurface))
				doShave_Burr(contacts);
		}

		detectionNP->setInstance(NULL);
	}
}

// for doShave_Burr
bool equal0(unsigned int value) {
	return (value == 0);
}

void SurfaceCarvingManager::doShave_Burr(const ContactVector* contacts)
{
	//Get Moving Direction of Tool to calculate drilling direction
	//Get Shaving Point index in Surface
	
	helper::vector<int> ShavingPointIdx = getShavingPointIdx(contacts);
	
	// Deformation
	// 1. Find Center point
	// 2. Compute Point indices within the range range of center point
	// 3. Smoothing - move indices around of unique_SPI 
	helper::vector<unsigned int> unique_SPI = MoveContactVertex(ShavingPointIdx);

	using sofa::core::topology::BaseMeshTopology;
	sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* mo_coll = NULL;
	modelSurface->getContext()->get(mo_coll);
	helper::WriteAccessor< Data<VecCoord> > x_wA = mo_coll->write(core::VecCoordId::position());
	helper::vector<unsigned int>::iterator it;

	for (unsigned int n = 0; n < nb_iterations.getValue(); n++)
	{
		VecCoord t;
		t.resize(x_wA.size());
		for (unsigned int i = 0; i < x_wA.size(); i++)
		{
			t[i] = x_wA[i];
		}
		for (it = unique_SPI.begin(); it != unique_SPI.end(); ++it)
		{
			BaseMeshTopology::VerticesAroundVertex v1 = modelSurface->getContext()->getMeshTopology()->getVerticesAroundVertex(*it);
			if (v1.size()>0)
			{
				for (unsigned int k = 0; k < v1.size(); k++)
				{
					BaseMeshTopology::VerticesAroundVertex v2 = modelSurface->getContext()->getMeshTopology()->getVerticesAroundVertex(v1[k]);
					unsigned int* tmp_v2 = new unsigned int[v2.size()];

					for (unsigned int j = 0; j < v2.size(); j++)
					{
						tmp_v2[j] = v2[j];
						for (unsigned int l = 0; l < v1.size(); l++)
							if (v2[j] == v1[l])
								tmp_v2[j] = 0;
					}

					std::vector<unsigned int> new_v2(v2.size());
					std::vector<unsigned int>::iterator its;
					its = std::remove_copy_if(tmp_v2, tmp_v2 + v2.size(), new_v2.begin(), equal0);
					new_v2.resize(std::distance(new_v2.begin(), its));

					if (new_v2.size())
					{
						Coord p = Coord();
						for (unsigned int j = 0; j < new_v2.size(); j++)
							p += x_wA[new_v2[j]];
						t[v1[k]] = p / new_v2.size();
					}
					delete[] tmp_v2;
				}

			}
		}

		for (unsigned int i = 0; i < x_wA.size(); i++)
		{

			x_wA[i] = t[i];
		}
	}
}

helper::vector<int> SurfaceCarvingManager::getShavingPointIdx(const ContactVector* contacts)
{

	sofa::component::collision::TTriangleModel< sofa::defaulttype::Vec3Types >* surf_TTM = NULL;
	modelSurface->getContext()->get(surf_TTM);

	const VecCoord& x_surf = surf_TTM->getX();

	helper::vector<int> ShavingPointIdx;

	for (unsigned int j = 0; j < contacts->size(); ++j)
	{
		const ContactVector::value_type& c = (*contacts)[j];
		int triangleIdx = (c.elem.first.getCollisionModel() == modelSurface ? c.elem.first.getIndex() : c.elem.second.getIndex());

		for (unsigned int k = 0; k < 3; k++)
		{
			ShavingPointIdx.push_back(modelSurface->getCollisionTopology()->getTriangle(triangleIdx)[k]);
		}
	}

	return ShavingPointIdx;
}

helper::vector<unsigned int> SurfaceCarvingManager::MoveContactVertex(helper::vector<int> ShavingPointIdx)
{
	sofa::component::collision::PointCollisionModel< DataTypes >* toolTPM = nullptr;
	modelTool->getContext()->get(toolTPM);
	const VecCoord& x_tool = toolTPM->getMechanicalState()->read(core::ConstVecCoordId::position())->getValue();

	sofa::component::collision::TriangleCollisionModel< sofa::defaulttype::Vec3Types >* surf_TTM = nullptr;
	modelSurface->getContext()->get(surf_TTM);
	const VecCoord& x_surf = surf_TTM->getX();

	Coord center_of_contact = Coord();

	for (int ii = 0; ii < ShavingPointIdx.size(); ii++)
		center_of_contact += x_surf[ShavingPointIdx[ii]];
	center_of_contact = center_of_contact / (ShavingPointIdx.size());

	double radius = ((x_tool[tip_idx.getValue()[0]]) - (x_tool[tip_idx.getValue()[2]])).norm();
	
	sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* mo_coll = NULL;
	modelSurface->getContext()->get(mo_coll);
	helper::WriteAccessor< Data<VecCoord> > x_wA = mo_coll->write(core::VecCoordId::position());

	helper::vector<int> ShavingPointIdx1;
	for (int ii = 0; ii < x_wA.size(); ii++)
	{
		double tmp = (x_surf[ii] - center_of_contact).norm();
		if (tmp<radius)
		{
			DataTypes::add(x_wA[ii], mv_direction[0], mv_direction[1], mv_direction[2]);
			ShavingPointIdx1.push_back(ii);
		}
	}

	helper::vector<unsigned int> unique_SPI(ShavingPointIdx1.size());
	helper::vector<unsigned int>::iterator it;

	std::sort(ShavingPointIdx1.begin(), ShavingPointIdx1.end());
	it = std::unique_copy(ShavingPointIdx1.begin(), ShavingPointIdx1.end(), unique_SPI.begin());

	unique_SPI.resize(std::distance(unique_SPI.begin(), it));

	return unique_SPI;
}

bool SurfaceCarvingManager::checkContact()
{
	return Coll_TF;
}

void SurfaceCarvingManager::printBoxRoi()
{
	if (!Boxes.empty())
	{
		msg_info() << "BOXROI (size: " << Boxes.size() << ")";// For multiple BoxROI
		for (int i = 0; i < Boxes.size(); ++i)
		{
			msg_info() << "BoxROI [" << i << "] :" << "[ " << Boxes[i] << " ]";
		}
	}
	else
		msg_info() << "no DrillTarget BOXROI";

}

int SurfaceCarvingManager::isPointInbox()
{

	//Boxes : xmin, ymin, zmin, xmax, ymax, zmax
	Coord &p = col_coord;

	if (!Boxes.empty())
	{
		for (int i = 0; i < Boxes.size(); ++i)
		{
			if (p[0] >= Boxes[i][0] && p[0] <= Boxes[i][3] && p[1] >= Boxes[i][1] && p[1] <= Boxes[i][4] && p[2] >= Boxes[i][2] && p[2] <= Boxes[i][5])
			{
				return i;  // Position is in Box, return Box number
				break;
			}
		}
		return -1; // Position is not in Box, return -1
	}
	else
	{
		msg_info() << "no Box";
		return -1;
	}
}

void SurfaceCarvingManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::core::objectmodel::HapticDeviceEvent * ev = dynamic_cast<sofa::core::objectmodel::HapticDeviceEvent *>(event))
    {
        if (ev->getButtonState()==1) d_active.setValue(true);
        else if (ev->getButtonState()==0) d_active.setValue(false);

    }

    else if (/* simulation::AnimateEndEvent* ev = */ dynamic_cast<simulation::AnimateEndEvent*>(event))
    {
        if (d_active.getValue()) {
            CheckCollisionDetection();
        }
    }
}

} // namespace controller

} // namespace component

} // namespace sofa
