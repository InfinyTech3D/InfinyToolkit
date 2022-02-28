/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InteractionTools/config.h>

namespace sofa::component::controller
{

class SOFA_INTERACTIONTOOLS_API BaseCarvingPerformer
{
public:
	BaseCarvingPerformer();

	virtual bool initPerformer() = 0;

	virtual bool runPerformer() = 0;
};
					
} // namespace sofa::component::controller
	