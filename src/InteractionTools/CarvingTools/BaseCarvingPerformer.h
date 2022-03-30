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
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>

using namespace sofa::type;

class contactInfo
{
public:
    unsigned int elemId; // in global mesh
    Vector3 pointA;
    Vector3 pointB;
    Vector3 normal;
    double dist;
};

namespace sofa::component::collision
{
class AdvancedCarvingManager;
}

namespace sofa::component::controller
{
    using namespace sofa::component::topology;
    using namespace sofa::core::topology;
    using namespace sofa::component::collision;


class SOFA_INTERACTIONTOOLS_API BaseCarvingPerformer
{
public:
	BaseCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr);

    ~BaseCarvingPerformer();

    void clearContacts();
    
    virtual bool initPerformer() = 0;

    virtual void filterContacts() {}

	virtual bool runPerformer() = 0;

    virtual void draw(const core::visual::VisualParams* vparams);

    TetrahedronSetTopologyContainer::SPtr getTopology() const {return m_topologyCon;}

    /// List of triangle contacts filter during collision 
    sofa::type::vector<contactInfo*> m_triangleContacts;
    /// List of point contacts filter during collision 
    sofa::type::vector<contactInfo*> m_pointContacts;

    std::map<BaseMeshTopology::TetrahedronID, Vec3> m_baryMap;

protected:
    TetrahedronSetTopologyContainer::SPtr m_topologyCon = nullptr;

    AdvancedCarvingManager* m_carvingMgr = nullptr;
};
					
} // namespace sofa::component::controller
	