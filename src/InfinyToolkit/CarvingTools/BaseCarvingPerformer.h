/*****************************************************************************
 *                 - Copyright (C) - 2020 - InfinyTech3D -                   *
 *                                                                           *
 * This file is part of the InfinyToolkit plugin for the SOFA framework      *
 *                                                                           *
 * Commercial License Usage:                                                 *
 * Licensees holding valid commercial license from InfinyTech3D may use this *
 * file in accordance with the commercial license agreement provided with    *
 * the Software or, alternatively, in accordance with the terms contained in *
 * a written agreement between you and InfinyTech3D. For further information *
 * on the licensing terms and conditions, contact: contact@infinytech3d.com  *
 *                                                                           *
 * GNU General Public License Usage:                                         *
 * Alternatively, this file may be used under the terms of the GNU General   *
 * Public License version 3. The licenses are as published by the Free       *
 * Software Foundation and appearing in the file LICENSE.GPL3 included in    *
 * the packaging of this file. Please review the following information to    *
 * ensure the GNU General Public License requirements will be met:           *
 * https://www.gnu.org/licenses/gpl-3.0.html.                                *
 *                                                                           *
 * Authors: see Authors.txt                                                  *
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
	