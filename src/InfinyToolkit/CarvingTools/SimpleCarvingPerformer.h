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
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyModifier.h>

namespace sofa::infinytoolkit
{
using namespace sofa::core::topology;
using namespace sofa::component::topology::container::dynamic;

class SOFA_INFINYTOOLKIT_API SimpleCarvingPerformer : public BaseCarvingPerformer
{
public:
	SimpleCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr);

	virtual ~SimpleCarvingPerformer() = default;

	bool initPerformer() override;

	bool runPerformer() override;

private:
	TetrahedronSetTopologyModifier::SPtr m_topoModif = nullptr;

	std::vector<Index> m_tetra2Remove;
};
					
} // namespace sofa::infinytoolkit
	