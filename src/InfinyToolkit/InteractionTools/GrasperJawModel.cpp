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
int GrasperJawModelClass = core::RegisterObject("Handle grasper.")
	.add< GrasperJawModel >();

//using namespace sofa::core::objectmodel;
typedef sofa::core::behavior::MechanicalState< sofa::defaulttype::Vec3Types > mechaState;

GrasperJawModel::GrasperJawModel()
	: BaseJawModel()
	, d_stiffness(initData(&d_stiffness, SReal(100.0), "stiffness", "jaw speed factor."))
{
}


bool GrasperJawModel::initImpl()
{
	//return (createStiffSpringFF() == 1);
	return true;
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
	std::cout << "GrasperJawModel::performAction()" << std::endl;
	deActivateImpl();	
	addModelSprings();	
}

void GrasperJawModel::stopAction()
{
	std::cout << "GrasperJawModel::stopAction()" << std::endl;
	// clean springs
	m_forcefield->clear();
	activateImpl();
}


void GrasperJawModel::releaseGrab()
{
	//if (!m_forcefieldUP || !m_forcefieldDOWN)
	//	return;
	//m_idgrabed.clear();
	//m_idBroadPhase.clear();

	//// Clear springs created during the grab
	//StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
	//stiffspringforcefield_UP->clear();

	//StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
	//stiffspringforcefield_DOWN->clear();

	//// Restaure the default collision behavior
	//std::vector<SphereModel*> col_models;
	//m_mord1->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	//if (!col_models.empty())
	//{
	//	SphereModel* col_model = col_models[0];
	//	col_model->setContactStiffness(m_oldCollisionStiffness); // TODO: check why this doesn't work
	//}

	//col_models.clear();
	//m_mord2->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	//if (!col_models.empty())
	//{
	//	SphereModel* col_model = col_models[0];
	//	col_model->setContactStiffness(m_oldCollisionStiffness);
	//}
}


int GrasperJawModel::createStiffSpringFF()
{
	std::cout << this->getName() << " + createStiffSpringFF()" << std::endl;
	SReal stiffness = d_stiffness.getValue();

	m_forcefield = sofa::core::objectmodel::New<StiffSpringFF>(dynamic_cast<mechaState*>(m_jaw), dynamic_cast<mechaState*>(m_target));
	StiffSpringFF* stiffspringforcefield = static_cast<StiffSpringFF*>(m_forcefield.get());
	stiffspringforcefield->setName(this->getName() + "_StiffSpringFF");
	this->getContext()->addObject(stiffspringforcefield);
	stiffspringforcefield->setStiffness(stiffness);
	stiffspringforcefield->setDamping(0);
	stiffspringforcefield->init();
	return 1;
}


void GrasperJawModel::addModelSprings()
{
	StiffSpringFF* stiffspringforcefield = static_cast<StiffSpringFF*>(m_forcefield.get());
	SReal stiffness = d_stiffness.getValue();

	for (GrabContactInfo* cInfo : m_contactInfos)
	{
		Vec3 posTool = Vec3(m_jaw->getPX(cInfo->idTool), m_jaw->getPY(cInfo->idTool), m_jaw->getPZ(cInfo->idTool));

		for (int i = 0; i < 3; ++i)
		{
			Vec3 posModel = Vec3(m_target->getPX(cInfo->idsModel[i]), m_target->getPY(cInfo->idsModel[i]), m_target->getPZ(cInfo->idsModel[i]));
			SReal dist = (posModel - posTool).norm();
			stiffspringforcefield->addSpring(cInfo->idTool, cInfo->idsModel[i], stiffness, 0.0, dist);
		}
	}
}


} // namespace sofa::infinytoolkit
