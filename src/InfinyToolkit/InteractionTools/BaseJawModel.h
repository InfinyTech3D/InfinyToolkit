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
#include <sofa/type/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/CollisionModel.h>


namespace sofa::infinytoolkit
{

class GrabContactInfo
{
public:
	sofa::Index idTool = sofa::InvalidID; // in global mesh
	sofa::Index idvModel = sofa::InvalidID;
	sofa::core::topology::BaseMeshTopology::Triangle idsModel; // in global mesh
	//Vec3 pointA;
	//Vec3 pointB;
	sofa::type::Vec3 normal; // equal to ||pB - pA||
	double dist; // equalt to (pB - pA).norm - contactDistance
};

class SOFA_INFINYTOOLKIT_API BaseJawModel : public core::objectmodel::BaseObject
{
public:
	SOFA_CLASS(BaseJawModel, core::objectmodel::BaseObject);

	using Vec3 = sofa::type::Vec3;

	BaseJawModel();
	virtual void init() override;
	virtual ~BaseJawModel() = default;
	
	int getModelId() { return m_modelId; }

	/// Main API public method to activate/deactivate tool. Will call @sa activateImpl or @sa deActivateImpl
	void activateTool(bool value);
	bool isToolActivated() { return m_isActivated; }

	/// Main API public method to launch the action of the Jaw
	virtual void performAction() {}
	/// Main API public method to stop the action of the Jaw
	virtual void stopAction() {}

	/// Main API public method to launch the action of the Jaw
	virtual void performSecondaryAction() {}
	/// Main API public method to stop the action of the Jaw
	virtual void stopSecondaryAction() {}


	/// Method to compute tool axis. Will fill @sa m_matP, @sa m_origin, @sa m_xAxis, @sa m_yAxis, @sa m_zAxis
	void computeAxis();
	void setAxis(sofa::type::Mat3x3 _matP) { m_matP = _matP; }
	void setOrigin(Vec3 _origin) { m_origin = _origin; }

	/// Method to compute BoundingBox, will fill @sa m_min, @sa m_max
	bool computeBoundingBox();

	virtual void addContact(GrabContactInfo* grabInfo);
	virtual void clearContacts();
	const sofa::type::vector<GrabContactInfo*>& getContacts() const { return m_contactInfos; }
	const sofa::type::vector<int>& getRawContactModelIds() const { return m_rawIds; }

	void setTargetModel(sofa::core::behavior::BaseMechanicalState* model) { m_target = model; }
	virtual void drawImpl(const core::visual::VisualParams* vparams);

protected:
	/// Internal API to init the component, will be called by @sa init()
	virtual bool initImpl() { return true; }

	/// Internal API to activate/deActivate the jaw
	virtual void activateImpl() {}
	virtual void deActivateImpl() {}

public:
	/// Link to the Jaw controller (mechanicalOjbect linked to the restShapeSpringFF)
	SingleLink<BaseJawModel, sofa::core::behavior::BaseMechanicalState, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_jawController;
	/// Link to the Jaw current Dofs (mechanicalOjbect under the restShapeSpringFF)
	SingleLink<BaseJawModel, sofa::core::behavior::BaseMechanicalState, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_jawDofs;
	/// Link to the Jaw collision model (CollisionModel in the same node as the mechanicalObject(linked to the restShapeSpringFF)
	SingleLink<BaseJawModel, sofa::core::CollisionModel, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_jawCollision;


protected:
	int m_modelId = sofa::InvalidID;
	bool m_isActivated = false;

	// Projection matrix to move into plier coordinate. X = along the plier, Y -> up, Z -> ortho to plier. 
	// Will be computed by computeAxis
	sofa::type::Mat3x3 m_matP;
	Vec3 m_origin;
	Vec3 m_xAxis;
	Vec3 m_yAxis;
	Vec3 m_zAxis;

	// Min Max value of the boundingBox
	sofa::type::Vec3 m_min, m_max;


	sofa::core::behavior::BaseMechanicalState* m_jaw = nullptr;
	sofa::core::behavior::BaseMechanicalState* m_target = nullptr;

	/// List of contacts filter during collision 
	sofa::type::vector<GrabContactInfo*> m_contactInfos;

	type::vector<int> m_rawIds;
};

} // namespace sofa::infinytoolkit
	