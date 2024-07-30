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

GrasperJawModel::GrasperJawModel()
	: BaseJawModel()
	, d_stiffness(initData(&d_stiffness, SReal(100.0), "stiffness", "jaw speed factor."))
{
}


void GrasperJawModel::activateImpl()
{
	//// Restaure the default collision behavior
	//std::vector<SphereModel*> col_models;
	//m_mord1->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	//if (!col_models.empty())
	//{
	//	SphereModel* col_model = col_models[0];
	//	col_model->setContactStiffness(m_oldCollisionStiffness);
	//}

	//col_models.clear();
	//m_mord2->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	//if (!col_models.empty())
	//{
	//	SphereModel* col_model = col_models[0];
	//	col_model->setContactStiffness(m_oldCollisionStiffness);
	//}

}


void GrasperJawModel::deActivateImpl()
{
	//if (m_model == nullptr)
	//	return false;

	//std::vector<SphereModel*> col_models;
	//m_mord1->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	//if (!col_models.empty())
	//{
	//	SphereModel* col_model = col_models[0];
	//	SReal contactS = col_model->getContactStiffness(0);
	//	if (m_oldCollisionStiffness < contactS)
	//		m_oldCollisionStiffness = contactS;
	//	col_model->setContactStiffness(0.0);
	//}

	//col_models.clear();
	//m_mord2->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	//if (!col_models.empty())
	//{
	//	SphereModel* col_model = col_models[0];
	//	col_model->setContactStiffness(0.0);
	//}

	//if (!m_forcefieldUP || !m_forcefieldDOWN)
	//	return false;
	//m_idgrabed.clear();
	//m_idBroadPhase.clear();

	//// Clear springs created during the grab
	//StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
	//stiffspringforcefield_UP->clear();

	//StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
	//stiffspringforcefield_DOWN->clear();
}


void GrasperJawModel::performAction()
{
	std::cout << "GrasperJawModel::performAction()" << std::endl;
}

void GrasperJawModel::stopAction()
{
	std::cout << "GrasperJawModel::stopAction()" << std::endl;
}


const sofa::type::vector< int >& GrasperJawModel::grabModel()
{
	//// If no point in the broadphase, nothing to do
	//if (m_idBroadPhase.size() == 0)
	//	return m_idBroadPhase;

	//if (m_forcefieldUP == nullptr || m_forcefieldDOWN == nullptr)
	//	createFF(500);


	//StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
	//StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());

	//// For each point in the broadphase search the neariest point of the plier
	//// If none under minDist = 2; point is rejected
	//size_t nbrVM1 = m_mord1->getSize();
	//size_t nbrVM2 = m_mord2->getSize();
	//for (int i = 0; i < m_idBroadPhase.size(); i++)
	//{
	//	SReal Mx = m_model->getPX(m_idBroadPhase[i]);
	//	SReal My = m_model->getPY(m_idBroadPhase[i]);
	//	SReal Mz = m_model->getPZ(m_idBroadPhase[i]);

	//	bool attached = false;
	//	int idModel1 = -1;
	//	int idModel2 = -1;
	//	SReal minDist1 = 2;
	//	SReal minDist2 = 2;
	//	// compute bary on mordUP
	//	for (int j = 0; j < nbrVM1; j++)
	//	{
	//		SReal x = m_mord1->getPX(j);
	//		SReal y = m_mord1->getPY(j);
	//		SReal z = m_mord1->getPZ(j);
	//		SReal dist = (Mx - x) * (Mx - x) + (My - y) * (My - y) + (Mz - z) * (Mz - z);
	//		dist = sqrt(dist);

	//		if (dist < minDist1) {
	//			minDist1 = dist;
	//			idModel1 = j;
	//		}
	//	}

	//	if (idModel1 != -1)
	//	{

	//		//stiffspringforcefield_UP->addSpring(m_idBroadPhase[i], idModel, m_stiffness, 0.0, minDist);
	//		//attach->addConstraint(idsModel[i], idModel, 1.0);
	//		//m_idgrabed.push_back(m_idBroadPhase[i]);
	//	}


	//	attached = false;
	//	//idModel = -1;
	//	//minDist = 2;

	//	// compute bary on mordUP
	//	for (int j = 0; j < nbrVM2; j++)
	//	{
	//		SReal x = m_mord2->getPX(j);
	//		SReal y = m_mord2->getPY(j);
	//		SReal z = m_mord2->getPZ(j);
	//		SReal dist = (Mx - x) * (Mx - x) + (My - y) * (My - y) + (Mz - z) * (Mz - z);
	//		dist = sqrt(dist);

	//		if (dist < minDist2) {
	//			minDist2 = dist;
	//			idModel2 = j;
	//		}
	//	}

	//	if (idModel2 != -1)
	//	{
	//		//stiffspringforcefield_DOWN->addSpring(m_idBroadPhase[i], idModel, m_stiffness, 0.0, minDist);
	//		//attach->addConstraint(idsModel[i], idModel, 1.0);
	//		//m_idgrabed.push_back(m_idBroadPhase[i]);
	//	}

	//	int choice = 0;
	//	if (idModel1 != -1 && idModel2 != -1)
	//	{
	//		if (minDist1 < minDist2)
	//			choice = 1;
	//		else
	//			choice = 2;
	//	}
	//	else if (idModel1 != -1)
	//		choice = 1;
	//	else if (idModel2 != -1)
	//		choice = 2;

	//	if (choice == 1)
	//	{
	//		stiffspringforcefield_UP->addSpring(m_idBroadPhase[i], idModel1, m_stiffness, 0.0, minDist1);
	//		m_idgrabed.push_back(m_idBroadPhase[i]);
	//	}
	//	else if (choice == 2)
	//	{
	//		stiffspringforcefield_DOWN->addSpring(m_idBroadPhase[i], idModel2, m_stiffness, 0.0, minDist2);
	//		m_idgrabed.push_back(m_idBroadPhase[i]);
	//	}


	//}

	//// Reduce collision spheres
	//if (m_idgrabed.size() > 0)
	//{
	//	msg_info() << "Passe la ";
	//	std::vector<SphereModel*> col_models;

	//	m_mord1->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	//	if (!col_models.empty())
	//	{
	//		msg_info() << "Passe la 2";
	//		SphereModel* col_model = col_models[0];
	//		m_oldCollisionStiffness = col_model->getContactStiffness(0);
	//		col_model->setContactStiffness(1);
	//	}

	//	col_models.clear();
	//	m_mord2->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	//	if (!col_models.empty())
	//	{
	//		SphereModel* col_model = col_models[0];
	//		col_model->setContactStiffness(1);
	//	}

	//}

	//msg_info() << "Narrow Phase detection: " << m_idgrabed.size();
	return m_idgrabed;
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


int GrasperJawModel::createFF(float _stiffness)
{
	//m_stiffness = _stiffness;

	//m_forcefield = sofa::core::objectmodel::New<StiffSpringFF>(dynamic_cast<mechaState*>(m_model), dynamic_cast<mechaState*>(m_mord1));
	//StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefield.get());
	//stiffspringforcefield_UP->setName("pince_Forcefield_UP");
	//this->getContext()->addObject(stiffspringforcefield_UP);
	//stiffspringforcefield_UP->setStiffness(m_stiffness);
	//stiffspringforcefield_UP->setDamping(0);
	//stiffspringforcefield_UP->init();
	return -1001;
}

} // namespace sofa::infinytoolkit
