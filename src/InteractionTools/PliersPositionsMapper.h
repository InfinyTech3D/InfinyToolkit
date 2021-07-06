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
#include <sofa/core/DataEngine.h>

namespace sofa
{

namespace component
{

namespace topology
{
    class TetrahedronSetTopologyContainer;
}

namespace engine
{

/** 
*
*/
class SOFA_INTERACTIONTOOLS_API PliersPositionsMapper: public sofa::core::DataEngine
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
	sofa::component::topology::TetrahedronSetTopologyContainer* m_topo;

	Data< type::vector<sofa::type::Vec<3, SReal> > > d_positions;
	Data<sofa::type::vector<int> > m_tetraTube;
	Data<sofa::type::vector<int> > m_tetraFat;
	Data< type::vector<sofa::type::Vec<3, SReal> > > m_tubePositions;
	Data< type::vector<sofa::type::Vec<3, SReal> > > m_grasPositions;
};


} // namespace misc

} // namespace component

} // namespace sofa
