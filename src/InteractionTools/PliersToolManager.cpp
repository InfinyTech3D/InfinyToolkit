/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/PliersToolManager.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>

#include <SofaBaseCollision/SphereModel.h>


namespace sofa
{

namespace component
{

namespace misc
{

SOFA_DECL_CLASS(PliersToolManager)

using namespace defaulttype;
using namespace sofa::core::topology;
using namespace sofa::component::topology;

typedef sofa::core::behavior::MechanicalState< sofa::defaulttype::Vec3Types > mechaState;
using SphereModel = sofa::component::collision::SphereCollisionModel< sofa::defaulttype::Vec3Types >;

using sofa::core::objectmodel::KeypressedEvent;
using sofa::core::objectmodel::KeyreleasedEvent;

int PliersToolManagerClass = core::RegisterObject("Handle sleeve Pince.")
        .add< PliersToolManager >();


PliersToolManager::PliersToolManager()
    : m_pathMord1(initData(&m_pathMord1, "pathMord1", "Path to mord1"))
    , m_pathMord2(initData(&m_pathMord2, "pathMord2", "Path to mord2"))
    , m_pathModel(initData(&m_pathModel, "pathModel", "Path to model"))
    , m_mord1(NULL)
    , m_mord2(NULL)
    , m_model(NULL)
    , m_forcefieldUP(NULL)
    , m_forcefieldDOWN(NULL)
	, m_oldCollisionStiffness(5000)
    , m_stiffness(500)
{
    this->f_listening.setValue(true);
    m_idgrabed.clear();
}


PliersToolManager::~PliersToolManager()
{

}


void PliersToolManager::init()
{
    const std::string& pathMord1 = m_pathMord1.getValue();
    const std::string& pathMord2 = m_pathMord2.getValue();
    const std::string& pathModel = m_pathModel.getValue();

    if (pathMord1.empty() && pathMord2.empty())
    {
        msg_error() << "no input mords found !!";
        return;
    }

    this->getContext()->get(m_mord1, pathMord1);
    this->getContext()->get(m_mord2, pathMord2);
    this->getContext()->get(m_model, pathModel);

    if (m_mord1 == NULL || m_mord2 == NULL || m_model == NULL)
    {
        msg_error() << "error mechanical state not found";
        return;
    }

    msg_info() << "m_mord1: " << m_mord1->getName();
    msg_info() << "m_mord2: " << m_mord2->getName();
    msg_info() << "m_mord2: " << m_model->getName();

    computeBoundingBox();
}

int PliersToolManager::testModels()
{
    if (m_mord1 == NULL)
        return -20;

    if (m_mord2 == NULL)
        return -21;

    if (m_model == NULL)
        return -22;

    return 52;
}

bool PliersToolManager::computeBoundingBox()
{
    if (m_mord1 == NULL || m_mord2 == NULL)
    {
        msg_info() << "error mechanical state not found";
        const std::string& pathMord1 = m_pathMord1.getValue();
        const std::string& pathMord2 = m_pathMord2.getValue();
        this->getContext()->get(m_mord1, pathMord1);
        this->getContext()->get(m_mord2, pathMord2);

        if (m_mord1 == NULL || m_mord2 == NULL)
            return false;
    }

    for (int i = 0; i < 3; ++i)
    {
        m_min[i] = 10000;
        m_max[i] = -10000;
    }


    for (int i = 0; i < m_mord1->getSize(); i++)
    {
        SReal x = m_mord1->getPX(i);
        SReal y = m_mord1->getPY(i);
        SReal z = m_mord1->getPZ(i);
        //msg_info() << "drawLine: " << x << " " << y << " " << z;
        if (x < m_min[0])
            m_min[0] = x;
        if (y < m_min[1])
            m_min[1] = y;
        if (z < m_min[2])
            m_min[2] = z;

        if (x > m_max[0])
            m_max[0] = x;
        if (y > m_max[1])
            m_max[1] = y;
        if (z > m_max[2])
            m_max[2] = z;
    }

    for (int i = 0; i < m_mord2->getSize(); i++)
    {
        SReal x = m_mord2->getPX(i);
        SReal y = m_mord2->getPY(i);
        SReal z = m_mord2->getPZ(i);

        if (x < m_min[0])
            m_min[0] = x;
        if (y < m_min[1])
            m_min[1] = y;
        if (z < m_min[2])
            m_min[2] = z;

        if (x > m_max[0])
            m_max[0] = x;
        if (y > m_max[1])
            m_max[1] = y;
        if (z > m_max[2])
            m_max[2] = z;
    }

    return true;
}

void PliersToolManager::reinit()
{
/*    if (!m_useDataInputs.getValue())
        this->readDataFile();
        */
}

void PliersToolManager::computeVertexIdsInBroadPhase(float margin)
{
    // First compute boundingbox
    computeBoundingBox();    

    if (m_model == NULL)
        return;

    // Add to m_idBroadPhase all model vertices inside the BB
    m_idBroadPhase.clear();
    for (int i = 0; i < m_model->getSize(); i++)
    {
        SReal x = m_model->getPX(i);
        SReal y = m_model->getPY(i);
        SReal z = m_model->getPZ(i);
        if (x > m_min[0] - margin && x < m_max[0] + margin
            && y > m_min[1] - margin && y < m_max[1] + margin
            && z > m_min[2] - margin && z < m_max[2] + margin)
        {
            m_idBroadPhase.push_back(i);
        }
    }
}

bool PliersToolManager::unactiveTool()
{
	if (m_model == NULL)
		return false;

	std::vector<SphereModel*> col_models;
	m_mord1->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	if (!col_models.empty())
	{
		SphereModel* col_model = col_models[0];
		SReal contactS = col_model->getContactStiffness(0);
		if (m_oldCollisionStiffness < contactS)
			m_oldCollisionStiffness = contactS;
		col_model->setContactStiffness(0.0);
	}

	col_models.clear();
	m_mord2->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	if (!col_models.empty())
	{
		SphereModel* col_model = col_models[0];
		col_model->setContactStiffness(0.0);
	}

	if (!m_forcefieldUP || !m_forcefieldDOWN)
		return false;
	m_idgrabed.clear();
	m_idBroadPhase.clear();

	// Clear springs created during the grab
	StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
	stiffspringforcefield_UP->clear();

	StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
	stiffspringforcefield_DOWN->clear();

	return true;
}

bool PliersToolManager::reactiveTool()
{
	// Restaure the default collision behavior
	std::vector<SphereModel*> col_models;
	m_mord1->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	if (!col_models.empty())
	{
		SphereModel* col_model = col_models[0];
		col_model->setContactStiffness(m_oldCollisionStiffness);
	}

	col_models.clear();
	m_mord2->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	if (!col_models.empty())
	{
		SphereModel* col_model = col_models[0];
		col_model->setContactStiffness(m_oldCollisionStiffness);
	}

	return true;
}


const sofa::type::vector< int >& PliersToolManager::grabModel()
{
    // If no point in the broadphase, nothing to do
    if (m_idBroadPhase.size() == 0)
        return m_idBroadPhase;

    if (m_forcefieldUP == NULL || m_forcefieldDOWN == NULL)
        createFF(500);


    StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
    StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());

    // For each point in the broadphase search the neariest point of the plier
    // If none under minDist = 2; point is rejected
    size_t nbrVM1 = m_mord1->getSize();
    size_t nbrVM2 = m_mord2->getSize();
    for (int i = 0; i < m_idBroadPhase.size(); i++)
    {
        SReal Mx = m_model->getPX(m_idBroadPhase[i]);
        SReal My = m_model->getPY(m_idBroadPhase[i]);
        SReal Mz = m_model->getPZ(m_idBroadPhase[i]);

        bool attached = false;
        int idModel1 = -1;
		int idModel2 = -1;
        SReal minDist1 = 2;
        SReal minDist2 = 2;
        // compute bary on mordUP
        for (int j = 0; j < nbrVM1; j++)
        {
            SReal x = m_mord1->getPX(j);
            SReal y = m_mord1->getPY(j);
            SReal z = m_mord1->getPZ(j);
            SReal dist = (Mx - x)*(Mx - x) + (My - y)*(My - y) + (Mz - z)*(Mz - z);
            dist = sqrt(dist);

            if (dist < minDist1) {
				minDist1 = dist;
				idModel1 = j;
            }
        }

        if (idModel1 != -1)
        {

            //stiffspringforcefield_UP->addSpring(m_idBroadPhase[i], idModel, m_stiffness, 0.0, minDist);
            //attach->addConstraint(idsModel[i], idModel, 1.0);
            //m_idgrabed.push_back(m_idBroadPhase[i]);
        }

        
        attached = false;
		//idModel = -1;
		//minDist = 2;

        // compute bary on mordUP
        for (int j = 0; j < nbrVM2; j++)
        {
            SReal x = m_mord2->getPX(j);
            SReal y = m_mord2->getPY(j);
            SReal z = m_mord2->getPZ(j);
            SReal dist = (Mx - x)*(Mx - x) + (My - y)*(My - y) + (Mz - z)*(Mz - z);
            dist = sqrt(dist);

			if (dist < minDist2) {
				minDist2 = dist;
				idModel2 = j;
			}			
        }

		if (idModel2 != -1)
		{
			//stiffspringforcefield_DOWN->addSpring(m_idBroadPhase[i], idModel, m_stiffness, 0.0, minDist);
			//attach->addConstraint(idsModel[i], idModel, 1.0);
			//m_idgrabed.push_back(m_idBroadPhase[i]);
		}

		int choice = 0;
		if (idModel1 != -1 && idModel2 != -1)
		{
			if (minDist1 < minDist2)
				choice = 1;
			else
				choice = 2;
		}
		else if (idModel1 != -1)
			choice = 1;
		else if (idModel2 != -1)
			choice = 2;

		if (choice == 1)
		{
			stiffspringforcefield_UP->addSpring(m_idBroadPhase[i], idModel1, m_stiffness, 0.0, minDist1);
			m_idgrabed.push_back(m_idBroadPhase[i]);
		}
		else if (choice == 2)
		{
			stiffspringforcefield_DOWN->addSpring(m_idBroadPhase[i], idModel2, m_stiffness, 0.0, minDist2);
			m_idgrabed.push_back(m_idBroadPhase[i]);
		}


    }

    sout << m_idgrabed;


	// Reduce collision spheres
	if (m_idgrabed.size() > 0)
	{
		msg_info() << "Passe la ";
		std::vector<SphereModel*> col_models;

		m_mord1->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
		if (!col_models.empty())
		{
			msg_info() << "Passe la 2";
			SphereModel* col_model = col_models[0];
			m_oldCollisionStiffness = col_model->getContactStiffness(0);
			col_model->setContactStiffness(1);
		}

		col_models.clear();
		m_mord2->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
		if (!col_models.empty())
		{
			SphereModel* col_model = col_models[0];
			col_model->setContactStiffness(1);
		}

	}

    msg_info() << "Narrow Phase detection: " << m_idgrabed.size();
    return m_idgrabed;
}

void PliersToolManager::releaseGrab()
{    
    if (!m_forcefieldUP || !m_forcefieldDOWN)
        return;
    m_idgrabed.clear();
    m_idBroadPhase.clear();

    // Clear springs created during the grab
    StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
    stiffspringforcefield_UP->clear();

    StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
    stiffspringforcefield_DOWN->clear();

    // Restaure the default collision behavior
	std::vector<SphereModel*> col_models;
	m_mord1->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	if (!col_models.empty())
	{
		SphereModel* col_model = col_models[0];
		col_model->setContactStiffness(m_oldCollisionStiffness); // TODO: check why this doesn't work
	}

	col_models.clear();
	m_mord2->getContext()->get<SphereModel>(&col_models, sofa::core::objectmodel::BaseContext::Local);
	if (!col_models.empty())
	{
		SphereModel* col_model = col_models[0];
		col_model->setContactStiffness(m_oldCollisionStiffness);
	}
}

int PliersToolManager::createFF(float _stiffness)
{
    m_stiffness = _stiffness;

    m_forcefieldUP = sofa::core::objectmodel::New<StiffSpringFF>(dynamic_cast<mechaState*>(m_model), dynamic_cast<mechaState*>(m_mord1));
    StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
    stiffspringforcefield_UP->setName("pince_Forcefield_UP");
    this->getContext()->addObject(stiffspringforcefield_UP);
    stiffspringforcefield_UP->setStiffness(m_stiffness);
    stiffspringforcefield_UP->setDamping(0);
    stiffspringforcefield_UP->init();

    m_forcefieldDOWN = sofa::core::objectmodel::New<StiffSpringFF>(dynamic_cast<mechaState*>(m_model), dynamic_cast<mechaState*>(m_mord2));
    StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
    stiffspringforcefield_DOWN->setName("pince_Forcefield_DOWN");
    this->getContext()->addObject(stiffspringforcefield_DOWN);
    stiffspringforcefield_DOWN->setStiffness(m_stiffness);
    stiffspringforcefield_DOWN->setDamping(0);
    stiffspringforcefield_DOWN->init();

    if (m_forcefieldUP == NULL)
        return -1001;

    if (m_forcefieldDOWN == NULL)
        return -1002;

    return 0;
}

void PliersToolManager::computePlierAxis()
{
    zero = Vec3(0, 0, 0);
    xAxis = Vec3(1, 0, 0);
    yAxis = Vec3(0, 1, 0);
    zAxis = Vec3(0, 0, 1);

    if (m_mord1 == NULL)
        return;

    zero = Vec3(m_mord1->getPX(0), m_mord1->getPY(0), m_mord1->getPZ(0));
    xAxis = Vec3(m_mord1->getPX(1), m_mord1->getPY(1), m_mord1->getPZ(1));
    yAxis = Vec3(m_mord1->getPX(20), m_mord1->getPY(20), m_mord1->getPZ(20));
    zAxis = Vec3(m_mord1->getPX(100), m_mord1->getPY(100), m_mord1->getPZ(100));

    Vec3 xDir = (xAxis - zero); xDir.normalize();
    Vec3 yDir = (yAxis - zero); yDir.normalize();
    Vec3 zDir = (zAxis - zero); zDir.normalize();

    matP = sofa::type::Mat3x3(xDir, yDir, zDir);

    //Vec3 test1 = Vec3(m_mord1->getPX(3), m_mord1->getPY(3), m_mord1->getPZ(3));
    //Vec3 test2 = Vec3(m_mord1->getPX(40), m_mord1->getPY(40), m_mord1->getPZ(40));
    //Vec3 test3 = Vec3(m_mord1->getPX(45), m_mord1->getPY(45), m_mord1->getPZ(45));
    //msg_info() << "test1 : " << test1 << " -> " << matP*(test1 - zero);
    //msg_info() << "test2 : " << test2 << " -> " << matP*(test2 - zero);
    //msg_info() << "test3 : " << test3 << " -> " << matP*(test3 - zero);

}

int PliersToolManager::cutFromTetra(float minX, float maxX, bool cut)
{
    if (m_idBroadPhase.empty())
        return 10000;

	bool lastCut = true;

    // Classify right/left points of the plier
    sofa::type::vector<int> idsLeft;
    sofa::type::vector<int> idsRight;
    for (int i = 0; i < m_idBroadPhase.size(); i++)
    {
        Vec3 vert = Vec3(m_model->getPX(m_idBroadPhase[i]), m_model->getPY(m_idBroadPhase[i]), m_model->getPZ(m_idBroadPhase[i]));
        vert = matP*(vert - zero);

		if (vert[2] < -20.0 || vert[2] > 20.0) // outside on the borders
			continue;
		
		// backward test
		if (vert[0] < minX)
		{
			//if (vert[2] > -5.0 || vert[2] < 5.0)
			//	return -10000;
			//else
				continue;
		}

		// frontward test
		if (vert[0] > maxX)
		{
			if (vert[2] > -5.0 || vert[2] < 5.0)
				lastCut = false;

			continue;
		}

        if (vert[2] >= -20.0 && vert[2] < 0.0)
            idsLeft.push_back(m_idBroadPhase[i]);
        else if (vert[2] >= 0.0 && vert[2] < 20.0)
            idsRight.push_back(m_idBroadPhase[i]);
    }

    if (idsLeft.size() == 0 || idsRight.size() == 0)
        return 20000;

    msg_info() << "idsLeft: " << idsLeft.size();
    msg_info() << "idsRight: " << idsRight.size();

    // Detect all tetra on the cut path
    TetrahedronSetTopologyContainer* tetraCon;
    m_model->getContext()->get(tetraCon);
    if (tetraCon == NULL) {
        msg_info() << "Error: NO tetraCon";
        return -40;
    }

    // First get all tetra that are on the first side
    sofa::type::vector<unsigned int> tetraIds;
    for (int i = 0; i < idsLeft.size(); ++i)
    {
        const BaseMeshTopology::TetrahedraAroundVertex& tetraAV = tetraCon->getTetrahedraAroundVertex(idsLeft[i]);
        for (int j = 0; j < tetraAV.size(); ++j)
        {
            int tetraId = tetraAV[j];
            bool found = false;
            for (int k = 0; k<tetraIds.size(); ++k)
                if (tetraIds[k] == tetraId)
                {
                    found = true;
                    break;
                }

            if (!found)
                tetraIds.push_back(tetraId);
        }
    }

    msg_info() << "tetraIds: " << tetraIds.size();


    // Then test for each tetra if one of the vertex is on the other side. If yes put on but path
    tetraIdsOnCut.clear();
    std::set< unsigned int > items;
    for (int i = 0; i < tetraIds.size(); ++i)
    {
        const BaseMeshTopology::Tetra& tetra = tetraCon->getTetra(tetraIds[i]);
        for (unsigned int j = 0; j < 4; ++j)
        {
            int idV = tetra[j];
            bool found = false;
            for (unsigned int k = 0; k < idsRight.size(); ++k)
            {
                if (idsRight[k] == idV)
                {
                    found = true;
                    break;
                }
            }

            if (found) {
                tetraIdsOnCut.push_back(tetraIds[i]);
                items.insert(tetraIds[i]);
                continue;
            }
        }
    }

    if (cut)
    {
        msg_info() << "tetraIdsOnCut: " << tetraIdsOnCut.size();
        TetrahedronSetTopologyModifier* tetraModif;
        m_model->getContext()->get(tetraModif);

        if (tetraModif == NULL) {
            msg_info() << "Error: NO tetraModif";
            return -45;
        }


        sofa::type::vector<unsigned int> vitems;
        vitems.reserve(items.size());
        vitems.insert(vitems.end(), items.rbegin(), items.rend());

        for (int i = 0; i < vitems.size(); i++)
        {
            sofa::type::vector<sofa::core::topology::Topology::TetrahedronID> its;
            its.push_back(vitems[i]);
            tetraModif->removeTetrahedra(its);
        }

        //vitems.resize(30);
        
    }
    else
    {
        m_idBroadPhase.clear();
        m_idBroadPhase.insert(m_idBroadPhase.end(), items.rbegin(), items.rend());
    }

	if (lastCut)
		return 40000;

    return int(items.size());
}

int PliersToolManager::pathCutFromTetra(float minX, float maxX)
{    
    int res = cutFromTetra(minX, maxX, false);
    if (res > 1000)
        return 0;

    sofa::type::vector<int> tetraIds = m_idBroadPhase;
    m_idgrabed.clear();
    TetrahedronSetTopologyContainer* tetraCon;
    m_model->getContext()->get(tetraCon);
    if (tetraCon == NULL) {
        msg_info() << "Error: NO tetraCon";
        return -40;
    }

    for (int i = 0; i < tetraIds.size(); i++)
    {
        const BaseMeshTopology::Tetra& tetra = tetraCon->getTetra(tetraIds[i]);
        for (int j = 0; j < 4; j++) 
        {
            bool found = false;
            int idV = tetra[j];
            for (unsigned int k = 0; k < m_idgrabed.size(); ++k)
            {
                if (m_idgrabed[k] == idV)
                {
                    found = true;
                    break;
                }
            }

        if (!found) 
            m_idgrabed.push_back(idV);
        
        }
    }

    return int(m_idgrabed.size());
}


void PliersToolManager::cutFromTriangles()
{
    // Classify right/left points of the plier
    sofa::type::vector<int> idsLeft;
    sofa::type::vector<int> idsRight;
    for (int i = 0; i < m_idgrabed.size(); i++)
    {
        Vec3 vert = Vec3(m_model->getPX(m_idgrabed[i]), m_model->getPY(m_idgrabed[i]), m_model->getPZ(m_idgrabed[i]));
        vert = matP*(vert - zero);

        if (vert[0] < 0.0 || vert[0] > 8.0)
            continue;

        if (vert[2] >= -2.0 && vert[2] < 1.0)
            idsLeft.push_back(m_idgrabed[i]);
        else if (vert[2] >= 1.0 && vert[2] < 3.0)
            idsRight.push_back(m_idgrabed[i]);
    }

    msg_info() << "idsLeft: " << idsLeft.size();
    msg_info() << "idsRight: " << idsRight.size();

    // Detect all tetra on the cut path
    std::vector<TriangleSetTopologyContainer*> triCons;
    m_model->getContext()->get<TriangleSetTopologyContainer>(&triCons, sofa::core::objectmodel::BaseContext::SearchDown);

    if (triCons.size() < 2) {
        msg_info() << "Error: NO triCons";
        return;
    }

    const sofa::type::vector<BaseMeshTopology::Triangle> & allTri = triCons[1]->getTriangleArray();
    for (int i = 0; i < allTri.size(); ++i)
    {
        const BaseMeshTopology::Triangle& tri = allTri[i];
        bool foundLeft = false;
        bool foundRight = false;
        for (int j = 0; j < 3; ++j)
        {
            int idV = tri[j];
            for (int k = 0; k < idsLeft.size(); ++k)
            {
                if (idsLeft[k] == idV)
                {
                    foundLeft = true;
                    break;
                }
            }

            if (foundLeft)
                break;
        }

        if (!foundLeft)
            continue;

        msg_info() << "found: " << i;
        for (int j = 0; j < 3; ++j)
        {
            int idV = tri[j];
            for (int k = 0; k < idsRight.size(); ++k)
            {
                if (idsRight[k] == idV)
                {
                    foundRight = true;
                    break;
                }
            }

            if (foundRight) {
                triIdsOnCut.push_back(i);
                break;
            }
        }
    }

    msg_info() << "triIdsOnCut: " << triIdsOnCut.size();
    std::vector<TriangleSetTopologyModifier*> triModifs;
    m_model->getContext()->get<TriangleSetTopologyModifier>(&triModifs, sofa::core::objectmodel::BaseContext::SearchDown);

    if (triModifs.size() < 2 ) {
        msg_info() << "Error: NO triModif";
        return;
    }
    msg_info() << "FOUND: " << triModifs.size();
    //tetraIdsOnCut.resize(30);
    triModifs[1]->removeItems(triIdsOnCut);
}


void PliersToolManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (KeypressedEvent::checkEventType(event))
    {
        KeypressedEvent *ev = static_cast<KeypressedEvent *>(event);

        switch (ev->getKey())
        {

        case 'T':
        case 't':
        {
            releaseGrab();

            computeVertexIdsInBroadPhase();
            grabModel();
           
            break;
        }
        case 'G':
        case 'g':
        {
            releaseGrab();
            break;
        }
        case 'Y':
        case 'y':
        {
            m_mord1->applyTranslation(0, -0.1, 0);
            m_mord2->applyTranslation(0, 0.1, 0);
            break;
        }
        case 'H':
        case 'h':
        {
            m_mord1->applyTranslation(0, 0.1, 0);
            m_mord2->applyTranslation(0, -0.1, 0);
            break;
        }
        case 'J':
        case 'j':
        {
            m_mord1->applyTranslation(0, 0.1, 0);
            m_mord2->applyTranslation(0, 0.1, 0);
            break;
        }
        case 'F':
        case 'f':
        {            
            releaseGrab();

            computeVertexIdsInBroadPhase();
            computePlierAxis();
            //cutFromTriangles();
            //for (int i=0; i<7; i++)
            //    cutFromTetra(i*2, i*2+2);

            cutFromTetra(0, 14, false);

            
            break;
        }
        case 'D':
        case 'd':
        {
            releaseGrab();

            computeVertexIdsInBroadPhase();
            computePlierAxis();
            //cutFromTriangles();
            /*for (int i=0; i<7; i++)
                cutFromTetra(i*2, i*2+2);*/

            cutFromTetra(0, 14, true);


            break;
        }
        }
    }
}

void PliersToolManager::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    sofa::type::RGBAColor color(0.2f, 1.0f, 1.0f, 1.0f);
    vparams->drawTool()->drawLine(m_min, m_max, sofa::type::RGBAColor(1.0, 0.0, 1.0, 1.0));
    
    vparams->drawTool()->drawLine(zero, xAxis, sofa::type::RGBAColor(1.0, 0.0, 0.0, 0.0));
    vparams->drawTool()->drawLine(zero, yAxis, sofa::type::RGBAColor(0.0, 1.0, 0.0, 0.0));
    vparams->drawTool()->drawLine(zero, zAxis, sofa::type::RGBAColor(0.0, 0.0, 1.0, 0.0));

    if (m_model == NULL)
        return;

    for (int i = 0; i < m_idgrabed.size(); i++)
    {
        SReal x = m_model->getPX(m_idgrabed[i]);
        SReal y = m_model->getPY(m_idgrabed[i]);
        SReal z = m_model->getPZ(m_idgrabed[i]);
        vparams->drawTool()->drawPoint(Vec3(x, y, z), sofa::type::RGBAColor(255.0, 0.0, 0.0, 1.0));
    }


    TetrahedronSetTopologyContainer* tetraCon;
    m_model->getContext()->get(tetraCon);
    if (tetraCon == NULL) {
        msg_info() << "Error: NO tetraCon";
        return;
    }

    for (int i = 0; i < tetraIdsOnCut.size(); i++)
    {
        const BaseMeshTopology::Tetra& tetra = tetraCon->getTetra(tetraIdsOnCut[i]);
        
        Vec3 p0 = Vec3(m_model->getPX(tetra[0]), m_model->getPY(tetra[0]), m_model->getPZ(tetra[0]));
        Vec3 p1 = Vec3(m_model->getPX(tetra[1]), m_model->getPY(tetra[1]), m_model->getPZ(tetra[1]));
        Vec3 p2 = Vec3(m_model->getPX(tetra[2]), m_model->getPY(tetra[2]), m_model->getPZ(tetra[2]));
        Vec3 p3 = Vec3(m_model->getPX(tetra[3]), m_model->getPY(tetra[3]), m_model->getPZ(tetra[3]));

        vparams->drawTool()->drawTetrahedron(p0, p1, p2, p3, color);
    }

        
   // msg_info() << "drawLine: " << m_min[0] << " " << m_min[1] << " " << m_min[2];
}



} // namespace misc

} // namespace component

} // namespace sofa
