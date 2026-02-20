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
#include <InfinyToolkit/InteractionTools/BaseJawModel.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa::infinytoolkit
{

class SOFA_INFINYTOOLKIT_API ScissorJawModel : public BaseJawModel
{
public:
	SOFA_CLASS(ScissorJawModel, sofa::infinytoolkit::BaseJawModel);

	using TriangleID = sofa::core::topology::Topology::TriangleID;
	using TetrahedronID = sofa::core::topology::Topology::TetrahedronID;
	ScissorJawModel() : BaseJawModel()
	{}

	virtual ~ScissorJawModel() = default;
	
	const sofa::type::vector<TriangleID>& getTriangleIdsOnCut() const { return triIdsOnCut; }
	const sofa::type::vector<TetrahedronID>& getTetraIdsOnCut() const { return tetraIdsOnCut; }


	// API for cutting
	int cutFromTetra(float minX, float maxX, bool cut = true);
	int pathCutFromTetra(float minX, float maxX);
	void cutFromTriangles();

private:
	// Buffer of points ids 
	sofa::type::vector <int> m_idgrabed;
	sofa::type::vector <int> m_idBroadPhase;


	// Keep it for debug drawing
	sofa::type::vector<TetrahedronID> tetraIdsOnCut;
	sofa::type::vector<TriangleID> triIdsOnCut;

	sofa::core::behavior::BaseMechanicalState* m_model = nullptr;
};

} // namespace sofa::infinytoolkit
	