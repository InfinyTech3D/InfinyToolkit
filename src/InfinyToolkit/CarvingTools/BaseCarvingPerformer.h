/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InfinyToolkit/config.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>

using namespace sofa::type;
using sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer;

class contactInfo
{
public:
    unsigned int elemId; // in global mesh
    Vector3 pointA;
    Vector3 pointB;
    Vector3 normal;
    double dist;
};

namespace sofa::infinytoolkit
{
class AdvancedCarvingManager;
}

namespace sofa::infinytoolkit
{
    using namespace sofa::component::topology;
    using namespace sofa::core::topology;


class SOFA_INFINYTOOLKIT_API BaseCarvingPerformer
{
public:
	BaseCarvingPerformer(TetrahedronSetTopologyContainer::SPtr topo, AdvancedCarvingManager* _carvingMgr);

    virtual ~BaseCarvingPerformer();

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
					
} // namespace sofa::infinytoolkit
	