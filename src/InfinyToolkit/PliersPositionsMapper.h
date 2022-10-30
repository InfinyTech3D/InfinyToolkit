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
#include <sofa/core/DataEngine.h>


namespace sofa::component::topology::container::dynamic
{
    class TetrahedronSetTopologyContainer;
}

namespace sofa::infinytoolkit
{

using sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer;
/** 
*
*/
class SOFA_INFINYTOOLKIT_API PliersPositionsMapper: public sofa::core::DataEngine
{
public:
    SOFA_CLASS(PliersPositionsMapper, sofa::core::DataEngine);
    

protected:
    PliersPositionsMapper();

    virtual ~PliersPositionsMapper(); 

public:
    void init() override;
    void reinit() override;
	void doUpdate() override;


    void draw(const core::visual::VisualParams* vparams) override;

	void handleTopologyChange();

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MeshTopology.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        return BaseObject::canCreate(obj, context, arg);
    }

protected:
	TetrahedronSetTopologyContainer* m_topo;

	Data< type::vector<sofa::type::Vec<3, SReal> > > d_positions;
	Data<sofa::type::vector<int> > m_tetraTube;
	Data<sofa::type::vector<int> > m_tetraFat;
	Data< type::vector<sofa::type::Vec<3, SReal> > > m_tubePositions;
	Data< type::vector<sofa::type::Vec<3, SReal> > > m_grasPositions;
};


} // namespace sofa::infinytoolkit
