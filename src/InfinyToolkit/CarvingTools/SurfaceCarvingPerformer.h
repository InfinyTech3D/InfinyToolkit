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

namespace sofa::infinytoolkit
{
using namespace sofa::core::topology;

class SOFA_INFINYTOOLKIT_API SurfaceCarvingPerformer : public BaseCarvingPerformer
{
public:
	SurfaceCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr);

	virtual ~SurfaceCarvingPerformer() = default;

	bool initPerformer() override;

	void filterContacts() override;

	bool runPerformer() override;

	void draw(const core::visual::VisualParams* vparams) override;

protected:
	void doMoveCarve1();

	void doMoveCarve2();

private:
	std::set<BaseMeshTopology::TetrahedronID> m_tetraId2refine;
	std::set<BaseMeshTopology::TetrahedronID> m_tetraId2remove;
};
					
} // namespace sofa::infinytoolkit
	