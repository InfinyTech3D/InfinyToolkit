/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/PliersPositionsMapper.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/type/Vec.h>


namespace sofa
{

namespace component
{

namespace engine
{

SOFA_DECL_CLASS(PliersPositionsMapper)

using namespace defaulttype;
using namespace sofa::core::topology;

int PliersPositionsMapperClass = core::RegisterObject("Handle sleeve key positions.")
        .add< PliersPositionsMapper >();


PliersPositionsMapper::PliersPositionsMapper()
    : m_topo(nullptr)
	, d_positions(initData(&d_positions, "position", "Rest position coordinates of the degrees of freedom."))
	, m_tetraTube(initData(&m_tetraTube, "tetraTube", "list of tetra id representing the tube"))
	, m_tetraFat(initData(&m_tetraFat, "tetraFat", "list of tetra id representing the fat"))
	, m_tubePositions(initData(&m_tubePositions, "tubePositions", "list of tetra id representing the fat"))
	, m_grasPositions(initData(&m_grasPositions, "grasPositions", "list of tetra id representing the fat"))
{
    this->f_listening.setValue(true);

}


PliersPositionsMapper::~PliersPositionsMapper()
{

}


void PliersPositionsMapper::init()
{
	this->getContext()->get(m_topo);
    if (m_topo == nullptr) {
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        msg_error() << "PliersPositionsMapper need a topology pointer";
        return;
    }

	addInput(&d_positions);
	addOutput(&m_tubePositions);
	addOutput(&m_grasPositions);	
	setDirtyValue();
}

void PliersPositionsMapper::reinit()
{

}

void PliersPositionsMapper::doUpdate()
{
	msg_info() << "PliersPositionsMapper::update()";
	cleanDirty();

	const sofa::type::vector<int>& _tetraTube = m_tetraTube.getValue();
	const sofa::type::vector<type::Vec3 >& _positions = d_positions.getValue();
	type::vector<type::Vec3 >& tubePositions = *m_tubePositions.beginEdit();
	tubePositions.resize(_tetraTube.size());
	for (int i = 0; i < _tetraTube.size(); i++)
	{
		int idTetra = _tetraTube[i];
		if (idTetra == -1) {
			tubePositions[i] = type::Vec3(666.666, 666.666, 666.666);
			continue;
		}

		const BaseMeshTopology::Tetra& tetra = m_topo->getTetra(idTetra);

		sofa::type::Vec3f p0 = _positions[tetra[0]];
		sofa::type::Vec3f p1 = _positions[tetra[1]];
		sofa::type::Vec3f p2 = _positions[tetra[2]];
		sofa::type::Vec3f p3 = _positions[tetra[3]];

		tubePositions[i] = (p0 + p1 + p2 + p3) / 4;
	}
	m_tubePositions.endEdit();

	const sofa::type::vector<int>& _tetraFat = m_tetraFat.getValue();
	type::vector<type::Vec3 >& grasPositions = *m_grasPositions.beginEdit();
	grasPositions.resize(_tetraFat.size());
	for (int i = 0; i < _tetraFat.size()/3; i++)
	{
		type::Vec3i idTetras;

		idTetras[0] = _tetraFat[i * 3];
		idTetras[1] = _tetraFat[i * 3 + 1];
		idTetras[2] = _tetraFat[i * 3 + 2];

		if (idTetras[0] == -1 || idTetras[1] == -1 || idTetras[2] == -1) {
			grasPositions[i * 3] = type::Vec3(666.666, 666.666, 666.666);
			grasPositions[i * 3 + 1] = type::Vec3(666.666, 666.666, 666.666);
			grasPositions[i * 3 + 2] = type::Vec3(666.666, 666.666, 666.666);
			continue;
		}

		for (int j = 0; j < 3; j++)
		{
			const BaseMeshTopology::Tetra& tetra = m_topo->getTetra(idTetras[j]);
			type::Vec3 p0 = _positions[tetra[0]];
			type::Vec3 p1 = _positions[tetra[1]];
			type::Vec3 p2 = _positions[tetra[2]];
			type::Vec3 p3 = _positions[tetra[3]];

			grasPositions[i * 3 + j] = (p0 + p1 + p2 + p3) / 4;
		}
	}
	m_grasPositions.endEdit();

}

void PliersPositionsMapper::handleTopologyChange()
{
	std::list<const TopologyChange *>::const_iterator itBegin = m_topo->beginChange();
	std::list<const TopologyChange *>::const_iterator itEnd = m_topo->endChange();

	while (itBegin != itEnd)
	{
		core::topology::TopologyChangeType changeType = (*itBegin)->getChangeType();
		switch (changeType)
		{
		case core::topology::TETRAHEDRAREMOVED:
		{
			sofa::type::vector<int>& _tetraTube = *m_tetraTube.beginEdit();
			sofa::type::vector<int>& _tetraFat = *m_tetraFat.beginEdit();

			const sofa::type::vector<sofa::core::topology::Topology::TetrahedronID> &tab = (static_cast< const sofa::core::topology::TetrahedraRemoved *>(*itBegin))->getArray();
			int idLastTetra = int(m_topo->getNumberOfTetrahedra()-1);
			bool updateNeeded = false;
			
			for (unsigned int i = 0; i < tab.size(); ++i)
			{
				int idTetraRemoved = tab[i];
				msg_info() << "idTetraRemoved: " << idTetraRemoved;
				// check tetra tube
				for (unsigned int j = 0; j < _tetraTube.size(); ++j)
				{
					if (idLastTetra == _tetraTube[j])
					{
						msg_info() << "tetra tube switch: " << _tetraTube[j] << " by " << idTetraRemoved;
						_tetraTube[j] = idTetraRemoved;
						updateNeeded = true;
					}

					if (idTetraRemoved == _tetraTube[j])
						_tetraTube[j] = -1;
				}

				// check tetra fat
				for (unsigned int j = 0; j < _tetraFat.size(); ++j)
				{
					if (idLastTetra == _tetraFat[j])
					{
						msg_info() << "tetra fat switch: " << _tetraFat[j] << " by " << idTetraRemoved;
						_tetraFat[j] = idTetraRemoved;
						updateNeeded = true;
					}

					if (idTetraRemoved == _tetraFat[j])
						_tetraFat[j] = -1;
				}
				idLastTetra--;
			}
				
			m_tetraTube.endEdit();
			m_tetraFat.endEdit();

			if (updateNeeded)
				update();

			break;
		}
		default:
			break;
		}
		
		++itBegin;
	}
}

void PliersPositionsMapper::draw(const core::visual::VisualParams* vparams)
{
   if (!vparams->displayFlags().getShowBehaviorModels())
        return;
   
    if (m_topo == nullptr)
        return;

	sofa::type::RGBAColor color(0.2f, 1.0f, 1.0f, 1.0f);
	const sofa::type::vector<type::Vec<3, SReal> >& _positions = d_positions.getValue();
	const sofa::type::vector<int>& _tetraTube = m_tetraTube.getValue();
    for (int i = 0; i < _tetraTube.size(); i++)
    {
        const BaseMeshTopology::Tetra& tetra = m_topo->getTetra(_tetraTube[i]);
        
		sofa::type::Vec3f p0 = _positions[tetra[0]];
        sofa::type::Vec3f p1 = _positions[tetra[1]];
        sofa::type::Vec3f p2 = _positions[tetra[2]];
        sofa::type::Vec3f p3 = _positions[tetra[3]];
        vparams->drawTool()->drawTetrahedron(p0, p1, p2, p3, color);
    }

	
	const type::vector<type::Vec<3, SReal> >& tubePositions = m_tubePositions.getValue();

	for (int i = 0; i < tubePositions.size()-1; i++)
	{
		vparams->drawTool()->drawLine(tubePositions[i], tubePositions[i+1], sofa::type::RGBAColor(1.0, 0.0, 1.0, 1.0));
	}

	const type::vector<type::Vec<3, SReal> >& grasPositions = m_grasPositions.getValue();

	for (int i = 0; i < grasPositions.size(); i++)
	{
		vparams->drawTool()->drawPoint(grasPositions[i], sofa::type::RGBAColor(1.0, 0.0, 1.0, 1.0));
	}

   // msg_info() << "drawLine: " << m_min[0] << " " << m_min[1] << " " << m_min[2];
}



} // namespace misc

} // namespace component

} // namespace sofa
