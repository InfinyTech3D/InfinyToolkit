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

#include <sofa/type/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <InfinyToolkit/config.h>

namespace sofa::infinytoolkit
{

class SOFA_INFINYTOOLKIT_API BaseJawModel : public core::objectmodel::BaseObject
{
public:
	SOFA_CLASS(BaseJawModel, core::objectmodel::BaseObject);

	using Vec3 = sofa::type::Vec3;

	BaseJawModel();

	virtual ~BaseJawModel() = default;
	
	int getModelId() { return m_modelId; }

	bool computeBoundingBox();
	
	void activeTool(bool value);
	bool isToolActivated() { return m_isActivated; }

	virtual void performAction();

	void computeAxis();
	void setAxis(sofa::type::Mat3x3 _matP) { matP = _matP; }
	void setOrigin(Vec3 _zero) { zero = _zero; }

protected:
	virtual void activateImpl() {}
	virtual void deActivateImpl() {}

public:
	SingleLink<BaseJawModel, sofa::core::behavior::BaseMechanicalState, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> m_jawModel;


protected:
	// Buffer of points ids 
	sofa::type::vector <int> m_idgrabed;
	sofa::type::vector <int> m_idBroadPhase;

	int m_modelId = sofa::InvalidID;
	bool m_isActivated = false;

	// Projection matrix to move into plier coordinate. X = along the plier, Y -> up, Z -> ortho to plier
	sofa::type::Mat3x3 matP;
	Vec3 zero;
	Vec3 xAxis;
	Vec3 yAxis;
	Vec3 zAxis;

	sofa::type::Vec3 m_min, m_max;

	sofa::core::behavior::BaseMechanicalState* m_jaw;
};

} // namespace sofa::infinytoolkit
	