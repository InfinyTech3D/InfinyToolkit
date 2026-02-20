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

namespace sofa::infinytoolkit
{
using namespace sofa::core::topology;

class SOFA_INFINYTOOLKIT_API BurningPerformer : public BaseCarvingPerformer
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
					
} // namespace sofa::infinytoolkit
	