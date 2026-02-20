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
    Vec3 pointA;
    Vec3 pointB;
    Vec3 normal;
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

    const std::string& getType() { return m_performerType; }
    
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

    std::string m_performerType = "BaseCarvingPerformer";
};
					
} // namespace sofa::infinytoolkit
	