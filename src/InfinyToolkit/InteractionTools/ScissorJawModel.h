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
#include <InfinyToolkit/InteractionTools/BaseJawModel.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa::infinytoolkit
{

class SOFA_INFINYTOOLKIT_API ScissorJawModel : public BaseJawModel
{
public:
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
	// Keep it for debug drawing
	sofa::type::vector<TetrahedronID> tetraIdsOnCut;
	sofa::type::vector<TriangleID> triIdsOnCut;

	sofa::core::behavior::BaseMechanicalState* m_model = nullptr;
};

} // namespace sofa::infinytoolkit
	