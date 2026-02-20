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

#include <InfinyToolkit/CarvingTools/BaseCarvingPerformer.h>

#include <MeshRefinement/TetrahedronRefinementAlgorithms.h>

namespace sofa::infinytoolkit
{

using namespace sofa::meshrefinement;

class SOFA_INFINYTOOLKIT_API RefineCarvingPerformer : public BaseCarvingPerformer
{
public:
	RefineCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr);

	virtual ~RefineCarvingPerformer();

	bool initPerformer() override;

	void filterContacts();

	bool runPerformer() override;

	void draw(const core::visual::VisualParams* vparams) override;

protected:
	void simpleCarving();

	void surfaceCarving();

	void surfaceCarving2();

protected:
	TetrahedronRefinementAlgorithms* m_tetraAlgo;

	std::set<unsigned int> m_tetra2Filter;

	sofa::type::vector<unsigned int> m_tetra2Filter2;
	std::set<unsigned int> m_triIdsToFilter;
	std::set<unsigned int> m_triIds;

	Vec3 carvingPosition;
};
					
} // namespace sofa::infinytoolkit
	