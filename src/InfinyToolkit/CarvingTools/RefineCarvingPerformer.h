/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InfinyToolkit/CarvingTools/BaseCarvingPerformer.h>

#include <MeshRefinement/TetrahedronRefinementAlgorithms.h>

namespace sofa::component::controller
{

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
					
} // namespace sofa::component::controller
	