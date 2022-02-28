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

class SOFA_INTERACTIONTOOLS_API RefineCarvingPerformer : public BaseCarvingPerformer
{
public:
	RefineCarvingPerformer();

	virtual ~RefineCarvingPerformer() = default;

	bool initPerformer() override;

	bool runPerformer() override;
};
					
} // namespace sofa::component::controller
	