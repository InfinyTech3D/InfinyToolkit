/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_INTERACTIONTOOLS_PLIERSPOSITIONSMAPPER_H
#define SOFA_INTERACTIONTOOLS_PLIERSPOSITIONSMAPPER_H
#include <InteractionTools/config.h>

#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/Event.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>


#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/simulation/Visitor.h>

#include <sofa/defaulttype/Vec.h>

namespace sofa
{

namespace component
{

namespace engine
{
	using core::DataEngine;

/** 
*
*/
class SOFA_INTERACTIONTOOLS_API PliersPositionsMapper: public DataEngine
{
public:
    SOFA_CLASS(PliersPositionsMapper,core::objectmodel::BaseObject);

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
 
    sofa::core::behavior::BaseMechanicalState* m_model;
	sofa::component::topology::TetrahedronSetTopologyContainer* m_topo;

	Data< helper::vector<sofa::defaulttype::Vec<3, SReal> > > d_positions;
	Data<sofa::helper::vector<int> > m_tetraTube;
	Data<sofa::helper::vector<int> > m_tetraFat;
	Data< helper::vector<sofa::defaulttype::Vec<3, SReal> > > m_tubePositions;
	Data< helper::vector<sofa::defaulttype::Vec<3, SReal> > > m_grasPositions;
};


} // namespace misc

} // namespace component

} // namespace sofa

#endif // SOFA_INTERACTIONTOOLS_PLIERSPOSITIONSMAPPER_H
