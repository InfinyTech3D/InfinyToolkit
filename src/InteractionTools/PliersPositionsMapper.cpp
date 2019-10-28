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
#include <PliersTools/PliersPositionsMapper.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/Node.h>


#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>

#include <sofa/simulation/Simulation.h>

#include <time.h>

namespace sofa
{

namespace component
{

namespace engine
{

SOFA_DECL_CLASS(SleevePositionsMapper)

using namespace defaulttype;
using namespace sofa::core::topology;

typedef sofa::core::behavior::MechanicalState< sofa::defaulttype::Vec3Types > mechaState;

int SleevePositionsMapperClass = core::RegisterObject("Handle sleeve key positions.")
        .add< SleevePositionsMapper >();


SleevePositionsMapper::SleevePositionsMapper()
    : m_model(NULL)
	, m_topo(NULL)
	, d_positions(initData(&d_positions, "position", "Rest position coordinates of the degrees of freedom."))
	, m_tetraTube(initData(&m_tetraTube, "tetraTube", "list of tetra id representing the tube"))
	, m_tetraFat(initData(&m_tetraFat, "tetraFat", "list of tetra id representing the fat"))
	, m_tubePositions(initData(&m_tubePositions, "tubePositions", "list of tetra id representing the fat"))
	, m_grasPositions(initData(&m_grasPositions, "grasPositions", "list of tetra id representing the fat"))
{
    this->f_listening.setValue(true);

}


SleevePositionsMapper::~SleevePositionsMapper()
{

}


void SleevePositionsMapper::init()
{
	this->getContext()->get(m_topo);
	if (m_topo == NULL)
		std::cout << "Error: NO tetraCon" << std::endl;

	this->getContext()->get(m_model);
	if (m_model == NULL)
		std::cout << "Error: NO mechaObj" << std::endl;

	addInput(&d_positions);
	addOutput(&m_tubePositions);
	addOutput(&m_grasPositions);	
	setDirtyValue();
}

void SleevePositionsMapper::reinit()
{
/*    if (!m_useDataInputs.getValue())
        this->readDataFile();
        */
}

void SleevePositionsMapper::update()
{
	std::cout << "SleevePositionsMapper::update()" << std::endl;
	cleanDirty();

	const sofa::helper::vector<int>& _tetraTube = m_tetraTube.getValue();
	const sofa::helper::vector<sofa::defaulttype::Vec<3, SReal> >& _positions = d_positions.getValue();
	helper::vector<sofa::defaulttype::Vec<3, SReal> >& tubePositions = *m_tubePositions.beginEdit();
	tubePositions.resize(_tetraTube.size());
	for (int i = 0; i < _tetraTube.size(); i++)
	{
		int idTetra = _tetraTube[i];
		if (idTetra == -1) {
			tubePositions[i] = sofa::defaulttype::Vec3f(666.666, 666.666, 666.666);
			continue;
		}

		const BaseMeshTopology::Tetra& tetra = m_topo->getTetra(idTetra);

		sofa::defaulttype::Vec3f p0 = _positions[tetra[0]];
		sofa::defaulttype::Vec3f p1 = _positions[tetra[1]];
		sofa::defaulttype::Vec3f p2 = _positions[tetra[2]];
		sofa::defaulttype::Vec3f p3 = _positions[tetra[3]];

		tubePositions[i] = (p0 + p1 + p2 + p3) / 4;
	}
	m_tubePositions.endEdit();

	const sofa::helper::vector<int>& _tetraFat = m_tetraFat.getValue();
	helper::vector<sofa::defaulttype::Vec<3, SReal> >& grasPositions = *m_grasPositions.beginEdit();
	grasPositions.resize(_tetraFat.size());
	for (int i = 0; i < _tetraFat.size()/3; i++)
	{
		sofa::defaulttype::Vec3i idTetras;

		idTetras[0] = _tetraFat[i * 3];
		idTetras[1] = _tetraFat[i * 3 + 1];
		idTetras[2] = _tetraFat[i * 3 + 2];

		if (idTetras[0] == -1 || idTetras[1] == -1 || idTetras[2] == -1) {
			grasPositions[i * 3] = sofa::defaulttype::Vec3f(666.666, 666.666, 666.666);
			grasPositions[i * 3 + 1] = sofa::defaulttype::Vec3f(666.666, 666.666, 666.666);
			grasPositions[i * 3 + 2] = sofa::defaulttype::Vec3f(666.666, 666.666, 666.666);
			continue;
		}

		for (int j = 0; j < 3; j++)
		{
			const BaseMeshTopology::Tetra& tetra = m_topo->getTetra(idTetras[j]);
			sofa::defaulttype::Vec3f p0 = _positions[tetra[0]];
			sofa::defaulttype::Vec3f p1 = _positions[tetra[1]];
			sofa::defaulttype::Vec3f p2 = _positions[tetra[2]];
			sofa::defaulttype::Vec3f p3 = _positions[tetra[3]];

			grasPositions[i * 3 + j] = (p0 + p1 + p2 + p3) / 4;
		}
	}
	m_grasPositions.endEdit();

}

void SleevePositionsMapper::handleTopologyChange()
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
			sofa::helper::vector<int>& _tetraTube = *m_tetraTube.beginEdit();
			sofa::helper::vector<int>& _tetraFat = *m_tetraFat.beginEdit();

			const sofa::helper::vector<unsigned int> &tab = (static_cast< const sofa::core::topology::TetrahedraRemoved *>(*itBegin))->getArray();
			int idLastTetra = m_topo->getNumberOfTetrahedra()-1;
			bool updateNeeded = false;
			
			for (unsigned int i = 0; i < tab.size(); ++i)
			{
				int idTetraRemoved = tab[i];
				std::cout << "idTetraRemoved: " << idTetraRemoved << std::endl;
				// check tetra tube
				for (unsigned int j = 0; j < _tetraTube.size(); ++j)
				{
					if (idLastTetra == _tetraTube[j])
					{
						std::cout << "tetra tube switch: " << _tetraTube[j] << " by " << idTetraRemoved << std::endl;
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
						std::cout << "tetra fat switch: " << _tetraFat[j] << " by " << idTetraRemoved << std::endl;
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

void SleevePositionsMapper::draw(const core::visual::VisualParams* vparams)
{
   if (!vparams->displayFlags().getShowBehaviorModels())
        return;
   
    if (m_model == NULL || m_topo == NULL)
        return;

	sofa::defaulttype::Vec4f color = sofa::defaulttype::Vec4f(0.2f, 1.0f, 1.0f, 1.0f);
	const sofa::helper::vector<sofa::defaulttype::Vec<3, SReal> >& _positions = d_positions.getValue();
	const sofa::helper::vector<int>& _tetraTube = m_tetraTube.getValue();
    for (int i = 0; i < _tetraTube.size(); i++)
    {
        const BaseMeshTopology::Tetra& tetra = m_topo->getTetra(_tetraTube[i]);
        
		sofa::defaulttype::Vec3f p0 = _positions[tetra[0]];
        sofa::defaulttype::Vec3f p1 = _positions[tetra[1]];
        sofa::defaulttype::Vec3f p2 = _positions[tetra[2]];
        sofa::defaulttype::Vec3f p3 = _positions[tetra[3]];
        vparams->drawTool()->drawTetrahedron(p0, p1, p2, p3, color);
    }

	/*color = sofa::defaulttype::Vec4f(0.2f, 0.0f, 1.0f, 1.0f);
	const sofa::helper::vector<unsigned int>& _tetraFat = m_tetraFat.getValue();
	for (int i = 0; i < _tetraFat.size(); i++)
	{
		const BaseMeshTopology::Tetra& tetra = m_topo->getTetra(_tetraFat[i]);

		sofa::defaulttype::Vec3f p0 = sofa::defaulttype::Vec3f(m_model->getPX(tetra[0]), m_model->getPY(tetra[0]), m_model->getPZ(tetra[0]));
		sofa::defaulttype::Vec3f p1 = sofa::defaulttype::Vec3f(m_model->getPX(tetra[1]), m_model->getPY(tetra[1]), m_model->getPZ(tetra[1]));
		sofa::defaulttype::Vec3f p2 = sofa::defaulttype::Vec3f(m_model->getPX(tetra[2]), m_model->getPY(tetra[2]), m_model->getPZ(tetra[2]));
		sofa::defaulttype::Vec3f p3 = sofa::defaulttype::Vec3f(m_model->getPX(tetra[3]), m_model->getPY(tetra[3]), m_model->getPZ(tetra[3]));

		vparams->drawTool()->drawTriangle(p0, p1, p2, p3, color);
	}
      */

	const helper::vector<sofa::defaulttype::Vec<3, SReal> >& tubePositions = m_tubePositions.getValue();

	for (int i = 0; i < tubePositions.size()-1; i++)
	{
		vparams->drawTool()->drawLine(tubePositions[i], tubePositions[i+1], Vec<4, float>(1.0, 0.0, 1.0, 1.0));
	}

	const helper::vector<sofa::defaulttype::Vec<3, SReal> >& grasPositions = m_grasPositions.getValue();

	for (int i = 0; i < grasPositions.size(); i++)
	{
		vparams->drawTool()->drawPoint(grasPositions[i], Vec<4, float>(1.0, 0.0, 1.0, 1.0));
	}

   // std::cout << "drawLine: " << m_min[0] << " " << m_min[1] << " " << m_min[2] << std::endl;
}



} // namespace misc

} // namespace component

} // namespace sofa
