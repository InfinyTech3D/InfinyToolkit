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

#include <InfinyToolkit/InteractionTools/GrasperJawModel.h>

#include <sofa/core/ObjectFactory.h>

namespace sofa::infinytoolkit
{

void registerGrasperJawModel(sofa::core::ObjectFactory* factory)
{
	factory->registerObjects(sofa::core::ObjectRegistrationData("Grasper jaw models to be used with ArticulatedToolManager")
		.add< GrasperJawModel >());
}


//using namespace sofa::core::objectmodel;
typedef sofa::core::behavior::MechanicalState< sofa::defaulttype::Vec3Types > mechaState;

GrasperJawModel::GrasperJawModel()
	: BaseJawModel()
	, d_stiffness(initData(&d_stiffness, SReal(100.0), "stiffness", "jaw speed factor."))
{
}


bool GrasperJawModel::initImpl()
{
	return (createStiffSpringFF() == 1);
}


void GrasperJawModel::activateImpl()
{
	sofa::core::CollisionModel* toolCol = l_jawCollision.get();
	if (toolCol == nullptr)
	{
		msg_error() << "No collision model given";
		return;
	}

	toolCol->setActive(true);
}


void GrasperJawModel::deActivateImpl()
{
	sofa::core::CollisionModel* toolCol = l_jawCollision.get();
	if (toolCol == nullptr)
	{
		msg_error() << "No collision model given";
		return;
	}

	toolCol->setActive(false);
}


void GrasperJawModel::performAction()
{
	//deActivateImpl();	
	addJawSprings();
}

void GrasperJawModel::stopAction()
{
	// clean springs
	m_forcefield->clear();
	m_rawIds.clear();
	//activateImpl();
}


void GrasperJawModel::performSecondaryAction()
{
	/*TetrahedronSetTopologyContainer* tetraCon;
	m_target->getContext()->get(tetraCon);

	if (tetraCon == nullptr) {
		msg_warning("GrasperJawModel") << "Error: NO tetraCon";
		return;
	}*/


}


void GrasperJawModel::stopSecondaryAction()
{

}


int GrasperJawModel::createStiffSpringFF()
{
	std::cout << this->getName() << " + createStiffSpringFF()" << std::endl;
	SReal stiffness = d_stiffness.getValue();

	m_forcefield = sofa::core::objectmodel::New<SpringFF>(dynamic_cast<mechaState*>(m_jaw), dynamic_cast<mechaState*>(m_target));
	SpringFF* stiffspringforcefield = static_cast<SpringFF*>(m_forcefield.get());
	stiffspringforcefield->setName(this->getName() + "_SpringFF");
	m_target->getContext()->addObject(stiffspringforcefield);

	return 1;
}


void GrasperJawModel::addJawSprings()
{
	SpringFF* stiffspringforcefield = static_cast<SpringFF*>(m_forcefield.get());
	SReal stiffness = d_stiffness.getValue();

	for (GrabContactInfo* cInfo : m_contactInfos)
	{
		Vec3 posTool = Vec3(m_jaw->getPX(cInfo->idTool), m_jaw->getPY(cInfo->idTool), m_jaw->getPZ(cInfo->idTool));

		if (cInfo->idvModel != sofa::InvalidID) // pointModel
		{
			Vec3 posModel = Vec3(m_target->getPX(cInfo->idvModel), m_target->getPY(cInfo->idvModel), m_target->getPZ(cInfo->idvModel));
			SReal dist = (posModel - posTool).norm();
			stiffspringforcefield->addSpring(cInfo->idTool, cInfo->idvModel, stiffness, 0.0, dist);
			
			bool found = false;
			for (int id : m_rawIds)
			{
				if (id == cInfo->idvModel)
				{
					found = true;
					break;
				}
			}
			
			if (!found)
				m_rawIds.push_back(cInfo->idvModel);

			std::cout << "Add spring: " << cInfo->idTool << " model: " << cInfo->idvModel << std::endl;
		}
		else
		{

		}

		//for (int i = 0; i < 3; ++i)
		//{
		//	Vec3 posModel = Vec3(m_target->getPX(cInfo->idsModel[i]), m_target->getPY(cInfo->idsModel[i]), m_target->getPZ(cInfo->idsModel[i]));
		//	SReal dist = (posModel - posTool).norm();
		//	stiffspringforcefield->addSpring(cInfo->idTool, cInfo->idsModel[i], stiffness, 0.0, dist);
		//}
	}

	std::cout << "m_rawIds: " << m_rawIds << std::endl;
}


} // namespace sofa::infinytoolkit
