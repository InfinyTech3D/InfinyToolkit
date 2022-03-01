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

namespace sofa::component::controller
{
using namespace sofa::core::topology;

class SOFA_INTERACTIONTOOLS_API SurfaceCarvingPerformer : public BaseCarvingPerformer
{
public:
	SurfaceCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, const SReal& carvingDistance, const SReal& refineDistance);

	virtual ~SurfaceCarvingPerformer() = default;

	bool initPerformer() override;

	bool runPerformer() override;

	void draw(const core::visual::VisualParams* vparams) override;

private:
	std::set<BaseMeshTopology::TetrahedronID> m_tetraId2refine;
	std::set<BaseMeshTopology::TetrahedronID> m_tetraId2remove;
};
					
} // namespace sofa::component::controller
	