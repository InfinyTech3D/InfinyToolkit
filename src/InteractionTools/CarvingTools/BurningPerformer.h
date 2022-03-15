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

class SOFA_INTERACTIONTOOLS_API BurningPerformer : public BaseCarvingPerformer
{
public:
	using TexCoord = sofa::type::Vec<2, float>;
	using VecTexCoord = type::vector<TexCoord>;

	BurningPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr);

	virtual ~BurningPerformer() = default;

	bool initPerformer() override;

	bool runPerformer() override;

	void draw(const core::visual::VisualParams* vparams) override;

};
					
} // namespace sofa::component::controller
	