/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InteractionTools/CarvingTools/BaseCarvingPerformer.h>

#include <MeshRefinement/TetrahedronRefinementAlgorithms.h>

namespace sofa::component::controller
{

class SOFA_INTERACTIONTOOLS_API RefineCarvingPerformer : public BaseCarvingPerformer
{
public:
	RefineCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr);

	virtual ~RefineCarvingPerformer();

	bool initPerformer() override;

	void filterContacts();

	bool runPerformer() override;

	void draw(const core::visual::VisualParams* vparams) override;

protected:
	TetrahedronRefinementAlgorithms* m_tetraAlgo;

	sofa::type::vector<unsigned int> m_tetra2remove;

	Vec3 carvingPosition;
};
					
} // namespace sofa::component::controller
	