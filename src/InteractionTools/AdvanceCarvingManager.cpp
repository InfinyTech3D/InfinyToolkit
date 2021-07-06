/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/AdvanceCarvingManager.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/objectmodel/ScriptEvent.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>

#include <sofa/core/topology/TopologicalMapping.h>
#include <SofaUserInteraction/TopologicalChangeManager.h>
#include <sofa/helper/AdvancedTimer.h>

#include <MeshRefinement/TetrahedronRefinementAlgorithms.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/BaseMechanicalState.h>

#include <SofaBaseTopology/TopologyData.inl>


namespace sofa
{

namespace component
{

namespace collision
{

    using namespace sofa::core::topology;

int AdvanceCarvingManagerClass = core::RegisterObject("Manager handling carving operations between a tool and an object.")
.add< AdvanceCarvingManager >()
;


AdvanceCarvingManager::AdvanceCarvingManager()
    : d_toolModelPath( initData(&d_toolModelPath, "toolModelPath", "Tool model path"))
    , d_surfaceModelPath( initData(&d_surfaceModelPath, "surfaceModelPath", "TriangleSetModel or SphereModel path"))
    , d_active( initData(&d_active, false, "active", "Activate this object.\nNote that this can be dynamically controlled by using a key") )
    , d_carvingDistance( initData(&d_carvingDistance, 0.0, "carvingDistance", "Collision distance at which cavring will start. Equal to contactDistance by default."))    
    , d_carvingRadius(initData(&d_carvingRadius, (Real) 0.0, "carvingRadius", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , d_carvingCriteria( initData(&d_carvingCriteria, 0.3, "carvingCriteria", "Collision distance at which cavring will start. Equal to contactDistance by default."))
    , d_refineDistance(initData(&d_refineDistance, 0.0, "refineDistance", "Collision distance at which cavring will start. Equal to contactDistance by default."))    
    , d_refineCriteria( initData(&d_refineCriteria, 0.5, "refineCriteria", "Collision distance at which cavring will start. Equal to contactDistance by default."))
    , d_carvingSpeed(initData(&d_carvingSpeed, 0.001, "carvingSpeed", "Collision distance at which cavring will start. Equal to contactDistance by default."))
    , d_refineThreshold(initData(&d_refineThreshold, 1.0, "refineThreshold", "Collision distance at which cavring will start. Equal to contactDistance by default."))
    , d_delayMode(initData(&d_delayMode, false, "delayMode", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , m_testID(initData(&m_testID, sofa::type::vector<unsigned int>(), "testID", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , d_drawTetra( initData(&d_drawTetra, false, "drawTetra", "Activate this object.\nNote that this can be dynamically controlled by using a key") )
    , d_drawContacts( initData(&d_drawContacts, false, "drawContacts", "Activate this object.\nNote that this can be dynamically controlled by using a key") )
    , d_drawScaleTetrahedra(initData(&d_drawScaleTetrahedra, (float) 1.0, "drawScaleTetrahedra", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , d_sphereRadius(initData(&d_sphereRadius, (Real) 0.1, "sphereRadius", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , d_activatorName(initData(&d_activatorName, "button1", "activatorName", "Name to active the script event parsing. Will look for 'pressed' or 'release' keyword. For example: 'button1_pressed'"))
    , m_toolCollisionModel(nullptr)
    , m_detectionNP(nullptr)
    , m_topoCon(nullptr)
    , m_topoSurface(nullptr)
    , m_carvingReady(false)    
    , m_canCarve(true)
    , m_mgrStatus(0)
    , m_tetraAlgo(nullptr)
    , m_forceFeedback(nullptr)
{
    this->f_listening.setValue(true);
}


AdvanceCarvingManager::~AdvanceCarvingManager()
{
    for (auto itM = m_tetraAlgos.begin(); itM != m_tetraAlgos.end(); ++itM )
    {
        delete itM->second;
    }

    m_tetraAlgos.clear();
    m_mgrStatus = 0;
}


void AdvanceCarvingManager::bwdInit()
{
    // Search for collision model corresponding to the tool.
    if (d_toolModelPath.getValue().empty())
        m_toolCollisionModel = getContext()->get<core::CollisionModel>(core::objectmodel::Tag("CarvingTool"), core::objectmodel::BaseContext::SearchRoot);
    else
        m_toolCollisionModel = getContext()->get<core::CollisionModel>(d_toolModelPath.getValue());

    // Search for the surface collision model.
    if (d_surfaceModelPath.getValue().empty())
    {
        // we look for a CollisionModel relying on a TetrahedronSetTopology.
        std::vector<core::CollisionModel*> models;
        getContext()->get<core::CollisionModel>(&models, core::objectmodel::Tag("CarvingSurface"), core::objectmodel::BaseContext::SearchRoot);
        
        // If topological mapping, iterate into child Node to find mapped topology
        for (size_t i=0;i<models.size();++i)
        {
            //sofa::core::topology::TopologicalMapping* topoMapping;
            core::CollisionModel* m = models[i];
            //m->getContext()->get(topoMapping);
            //if (topoMapping == nullptr) continue;

            m_surfaceCollisionModels.push_back(m);
        }
    }
    else
    {
        m_surfaceCollisionModels.push_back(getContext()->get<core::CollisionModel>(d_surfaceModelPath.getValue()));
    }

    m_detectionNP = getContext()->get<core::collision::NarrowPhaseDetection>();
    m_carvingReady = true;

    if (m_toolCollisionModel == NULL) { msg_error() << "m_toolCollisionModel not found"; m_carvingReady = false; }
    if (m_surfaceCollisionModels.empty()) { msg_error() << "CarvingManager: m_surfaceCollisionModels not found"; m_carvingReady = false; }
    if (m_detectionNP == NULL) { msg_error() << "CarvingManager: NarrowPhaseDetection not found"; m_carvingReady = false; }    
    
    if (!m_carvingReady) {
        msg_error() << "AdvanceCarvingManager: initialisation failed.";
        return;
    }
 
    
    for (unsigned int i = 0; i < m_surfaceCollisionModels.size(); ++i)
    {
        const sofa::core::objectmodel::BaseContext* _node = m_surfaceCollisionModels[i]->getContext();
        sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = _node->get<sofa::component::topology::TetrahedronSetTopologyContainer>();

        if (topoCon == nullptr)
        {
            msg_error() << "no TetrahedronSetTopologyContainer found for collision model: " << m_surfaceCollisionModels[i]->getName();
        }

        //sofa::core::topology::TopologicalMapping* topoMapping;
        if (m_tetraAlgos.find(topoCon) != m_tetraAlgos.end()) // already in
            continue;

        TetrahedronRefinementAlgorithms* tetraAlgo = new TetrahedronRefinementAlgorithms();
        bool resInit = tetraAlgo->init(_node);
        m_tetraAlgos[topoCon] = tetraAlgo;
        if (!resInit)
        {
            m_carvingReady = false;
            return;
        }        
    }

    // check if forcefeedback
    m_forceFeedback = getContext()->get<sofa::component::controller::ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
    if (m_forceFeedback)
        msg_info() << "Forcefeedback found: " << m_forceFeedback->getName();
    else
        msg_info() << "NO Forcefeedback found: ";
}

void AdvanceCarvingManager::clearContacts()
{
    for (unsigned int i = 0; i < m_triangleContacts.size(); i++)
    {
        delete m_triangleContacts[i];
        m_triangleContacts[i] = nullptr;
    }

    m_triangleContacts.clear();
    for (unsigned int i = 0; i < m_pointContacts.size(); i++)
    {
        delete m_pointContacts[i];
        m_pointContacts[i] = nullptr;
    }
    m_pointContacts.clear();

    m_tetra2remove.clear();
    m_realContacts.clear();
    m_contactPoints.clear();
    //m_toolForceFeedBack = Vector3(0, 0, 0);
}


void AdvanceCarvingManager::filterCollision()
{
    if (!m_carvingReady)
        return;

    lockContraints.lock();
    clearContacts();

    // get the collision output
    const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = m_detectionNP->getDetectionOutputs();
    if (detectionOutputs.size() == 0)
    {
        lockContraints.unlock();
        return;
    }


    // vector of indices and distance found by min proximity pipeline.
    sofa::type::vector<unsigned int> vIdCarve;
    sofa::type::vector<unsigned int> vIdRefine;

    // Get distance values
    const Real& carvingDistance = d_carvingDistance.getValue();
    const Real& refineDistance = d_refineDistance.getValue();

    // loop on the contact to get the one between the CarvingSurface and the CarvingTool collision model
    const ContactVector* contacts = nullptr;
    for (core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); it != detectionOutputs.end(); ++it)
    {
        sofa::core::CollisionModel* collMod1 = it->first.first;
        sofa::core::CollisionModel* collMod2 = it->first.second;
        sofa::core::CollisionModel* targetModel = nullptr;

        // get the good collision contact
        if (collMod1 == m_toolCollisionModel && collMod2->hasTag(sofa::core::objectmodel::Tag("CarvingSurface")))
            targetModel = collMod2;
        else if (collMod2 == m_toolCollisionModel && collMod1->hasTag(sofa::core::objectmodel::Tag("CarvingSurface")))
            targetModel = collMod1;
        else
            continue;


        // Get the number of contacts        
        contacts = dynamic_cast<const ContactVector*>(it->second);
        size_t ncontacts = contacts->size();
        if (contacts == nullptr || contacts->size() == 0)
            continue;

        // if not inside the map of tetrahedronRefinementAlgo, or not container, skip
        sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = targetModel->getContext()->get<sofa::component::topology::TetrahedronSetTopologyContainer>();
        auto itM = m_tetraAlgos.find(topoCon);
        if (itM == m_tetraAlgos.end()) {
            msg_error() << "No TetrahedronRefinementAlgorithms found for collision model: " << targetModel->name;
            continue;
        }

        int mode = -1; // 0 = triangleModel, 1 = pointModel
        if (targetModel->getTypeName().find("TriangleCollisionModel") != std::string::npos)
        {
            mode = 0;
        }
        else if (targetModel->getTypeName().find("PointCollisionModel") != std::string::npos)
        {
            mode = 1;
        }
        else
        {
            msg_error() << targetModel->getTypeName() << " collision model not handled";
            continue;
        }
        
        
        // check if mapping is present
        sofa::core::topology::TopologicalMapping* topoMapping;
        targetModel->getContext()->get(topoMapping);
        
        for (size_t j = 0; j < ncontacts; ++j)
        {
            const ContactVector::value_type& c = (*contacts)[j];
            int elemIdx = (c.elem.first.getCollisionModel() == m_toolCollisionModel ? c.elem.second.getIndex() : c.elem.first.getIndex());

            // update the triangle id if a mapping is present
            if (mode == 0 && topoMapping != nullptr)
                elemIdx = topoMapping->getGlobIndex(elemIdx);

            contactInfo* info = new contactInfo();
            info->elemId = elemIdx;
            info->normal = c.normal;
            info->pointA = c.point[0];
            info->pointB = c.point[1];
            info->dist = c.value;
            info->tetraAlgo = itM->second;

            if (mode == 0)
                m_triangleContacts.push_back(info);
            else if (mode == 1)
                m_pointContacts.push_back(info);
        }
        
        // process the collision               
    }
    lockContraints.unlock();
    if (d_active.getValue())
        processCollision();
}


Vector3 AdvanceCarvingManager::computeForceFeedBack(const Vector3& position)
{
    lockContraints.lock();
    // todo change that
    const Real& refineDistance = d_refineDistance.getValue();
    m_toolForceFeedBack = Vector3(0, 0, 0);
    m_toolPosition = position;
    int cpt = 0;
    SReal facMax = 33;
    
    //SReal responseDist = 0.3;
    SReal responseDist = refineDistance;
    SReal contactDist = 0.1;
    const SReal& sphereRadius = d_sphereRadius.getValue();


    std::list<unsigned int> idsP;

    if (!m_triangleContacts.empty())
    {
        msg_info() << "-------------- TRIANGLE START ----------------";
        cpt = 0;
        Vector3 ffTri = Vector3(0, 0, 0);
        for each (contactInfo* cInfo in m_triangleContacts)
        {
            Vector3 vec = position - cInfo->pointA;
            SReal realDistance = vec.norm();
            SReal collDistance = realDistance - contactDist - sphereRadius;
            msg_info() << "id: " << cInfo->elemId << " realDistance: " << realDistance << " | collDistance: " << collDistance << " | cInfo->dist: " << cInfo->dist;

            if (collDistance > responseDist)
                continue;

            if (collDistance < 0.0)
            {
                std::cout << "collDistance: " << collDistance << std::endl;
                facMax *= (1 + fabs(collDistance));
                collDistance = 0.0;
            }

            SReal factorN = (collDistance - responseDist) * (collDistance - responseDist);
            msg_info() << "(collDistance - responseDist):  " << (collDistance - responseDist) << " | facMax: " << facMax;
            factorN *= factorN;
            factorN *= facMax;
            msg_info() << "factorN:  " << factorN;
            vec.normalize();

            //cInfo->normal = vec * factorN;
            m_toolForceFeedBack += cInfo->normal * factorN;

            const sofa::core::topology::Triangle& tri = cInfo->tetraAlgo->getTopologyContainer()->getTriangle(cInfo->elemId);
            for (unsigned int i = 0; i < 3; ++i)
                idsP.push_back(tri[i]);
            
            cpt++;
            msg_info() << "collDistance:  " << collDistance << " - " << responseDist << " = " << factorN;
            msg_info() << "vec: " << cInfo->normal.norm() << " ++ " << m_toolForceFeedBack.norm();
        }

        msg_info() << " ### force: " << m_toolForceFeedBack.norm();
        msg_info() << "-------------- TRIANGLE END ----------------";
    }

    facMax = 33;
    if (!m_pointContacts.empty())
    {
        msg_info() << "-------------- POINT START ----------------";
        for each (contactInfo* cInfo in m_pointContacts)
        {
            Vector3 vec = position - cInfo->pointB;
            SReal realDistance = vec.norm();
            SReal collDistance = realDistance - contactDist - sphereRadius;
            
            if (collDistance > responseDist)
                continue;

            bool found = false;
            for (auto idP : idsP)
            {
                if (idP == cInfo->elemId)
                {
                    found = true;
                    break;
                }
            }

            if (found)
                continue;
            
            msg_info() << "id: " << cInfo->elemId << " realDistance: " << realDistance << " | collDistance: " << collDistance << " | cInfo->dist: " << cInfo->dist;

            if (collDistance < 0.0)
            {
                std::cout << "collDistance: " << collDistance << std::endl;
                facMax *= fabs(collDistance);
                collDistance = 0.0;
            }
            
            SReal factorN = (collDistance - responseDist) * (collDistance - responseDist) * facMax;
            factorN *= factorN;
            vec.normalize();
            
            //cInfo->normal = vec * factorN;
            m_toolForceFeedBack += vec * factorN;

            cpt++;
            msg_info() << "collDistance:  " << collDistance << " - "  << responseDist << " = " << factorN;
            msg_info() << "vec: " << cInfo->normal.norm() << " ++ " << m_toolForceFeedBack.norm();
        }

        msg_info() << " ### force: " << m_toolForceFeedBack.norm();
        msg_info() << "-------------- POINT END ----------------";
    }
    

    lockContraints.unlock();
    return m_toolForceFeedBack;
}


void AdvanceCarvingManager::processCollision()
{
    if (m_triangleContacts.empty() && m_pointContacts.empty())
        return;

    m_topoCon = nullptr;
   // bool resRef = doRefinement();    
    //if (resRef == false) // no refinement at this step, carve
        //doMoveCarve();
        doMoveCarvePoint();
}


bool AdvanceCarvingManager::doRefinement()
{
    const Real& refineDistance = d_refineDistance.getValue();
    bool result = false;
    sofa::type::vector<unsigned int> layer1_tetra;
    
    TetrahedronRefinementAlgorithms* _tetraAlgo = nullptr;
    for each (contactInfo* cInfo in m_triangleContacts)
    {
        if (cInfo->dist > refineDistance)
            continue;

        if (_tetraAlgo == nullptr)
            _tetraAlgo = cInfo->tetraAlgo;
        else if (_tetraAlgo != cInfo->tetraAlgo)
        {
            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
            continue;
        }

        sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = _tetraAlgo->getTopologyContainer();
        const core::topology::BaseMeshTopology::TetrahedraAroundTriangle & tetraAT = topoCon->getTetrahedraAroundTriangle(cInfo->elemId);

        if (tetraAT.size() != 1)
        {
            msg_error() << "More than one tetra around tri: " << cInfo->elemId;
            continue;
        }

        layer1_tetra.push_back(tetraAT[0]);
    }

    for each (contactInfo* cInfo in m_pointContacts)
    {
        if (cInfo->dist > refineDistance)
            continue;

        if (_tetraAlgo == nullptr)
            _tetraAlgo = cInfo->tetraAlgo;
        else if (_tetraAlgo != cInfo->tetraAlgo)
        {
            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
            continue;
        }

        sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = _tetraAlgo->getTopologyContainer();
        const core::topology::BaseMeshTopology::TetrahedraAroundVertex & tetraAV = topoCon->getTetrahedraAroundVertex(cInfo->elemId);

        for (auto tetraId: tetraAV)
            layer1_tetra.push_back(tetraId);
    }

    if (!layer1_tetra.empty() && _tetraAlgo != nullptr)
        return _tetraAlgo->refineTetrahedra(layer1_tetra, d_refineCriteria.getValue()); 
    else
        return false;
}


bool AdvanceCarvingManager::doMoveCarvePoint()
{
    const Real& carvingDistance = d_carvingDistance.getValue();
    std::map<BaseMeshTopology::PointID, BaseMeshTopology::TetrahedraAroundVertex> layer1_tetraAV;
    std::set <BaseMeshTopology::TetrahedronID> layer1_tetra;        
    std::map<BaseMeshTopology::PointID, contactInfo*> contactInfoMap;

    sofa::type::vector<unsigned int> layer2_tetra;
    sofa::type::vector<contactInfo*> triInfo;

    std::set <BaseMeshTopology::TetrahedronID> surrounding_tetra;

    TetrahedronRefinementAlgorithms* _tetraAlgo = nullptr;
    sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon;
    for each (contactInfo* cInfo in m_pointContacts)
    {
        if (cInfo->dist > carvingDistance)
            continue;

        if (_tetraAlgo == nullptr) {
            _tetraAlgo = cInfo->tetraAlgo;
            topoCon = _tetraAlgo->getTopologyContainer();
        }
        else if (_tetraAlgo != cInfo->tetraAlgo)
        {
            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
            continue;
        }

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex & tetraAV = topoCon->getTetrahedraAroundVertex(cInfo->elemId);

        layer1_tetraAV[cInfo->elemId] = tetraAV;
        for (auto tetraID : tetraAV) {
            layer1_tetra.insert(tetraID);
            surrounding_tetra.insert(tetraID);
        }

        contactInfoMap[cInfo->elemId] = cInfo;
    }



    for each (contactInfo* cInfo in m_triangleContacts)
    {
        if (cInfo->dist > carvingDistance)
            continue;

        if (_tetraAlgo == nullptr) {
            _tetraAlgo = cInfo->tetraAlgo;
            topoCon = _tetraAlgo->getTopologyContainer();
        }
        else if (_tetraAlgo != cInfo->tetraAlgo)
        {
            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
            continue;
        }


        const core::topology::BaseMeshTopology::TetrahedraAroundTriangle & tetraAT = topoCon->getTetrahedraAroundTriangle(cInfo->elemId);

        if (tetraAT.size() != 1)
        {
            msg_error() << "More than one tetra around tri: " << cInfo->elemId;
            continue;
        }

        layer2_tetra.push_back(tetraAT[0]);
        triInfo.push_back(cInfo);

        const sofa::core::topology::Triangle& tri = cInfo->tetraAlgo->getTopologyContainer()->getTriangle(cInfo->elemId);
        for (unsigned int i = 0; i < 3; ++i)
        {
            const core::topology::BaseMeshTopology::TetrahedraAroundVertex & tetraAV = topoCon->getTetrahedraAroundVertex(tri[i]);
            for (auto tetraID : tetraAV) {
                surrounding_tetra.insert(tetraID);
            }
        }
    }


    // nothing todo
    if (surrounding_tetra.empty())
        return false;

    if (m_forceFeedback)
        m_forceFeedback->setLock(true);
    

    // get write accessor to the position
    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    topoCon->getContext()->get(meca);
    helper::WriteAccessor< Data<DataTypes::VecCoord> > pos = meca->write(core::VecCoordId::position());

    // compute tetra barycenters
    sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topoGeo = _tetraAlgo->getTopologyGeometry();
    std::map<unsigned int, DataTypes::Coord> barycenter;
    for (auto tetraID : layer1_tetra)
    {
        if (barycenter.find(tetraID) != barycenter.end()) // already in
            continue;

        const Tetrahedron& t = topoCon->getTetrahedron(tetraID);

        DataTypes::Coord bary = (pos[t[0]] + pos[t[1]] + pos[t[2]] + pos[t[3]]) * (Real) 0.25;
        barycenter[tetraID] = bary;
    }
    

    // move point to the center of their barycenters
    //std::cout << "--------------" << std::endl;
    m_topoCon = topoCon;
    m_tetra2remove.clear();
    SReal carveFactor = d_carvingSpeed.getValue();

    // reduce touched tetra
    for (unsigned int i = 0; i<layer2_tetra.size(); ++i)
    {
        sofa::core::topology::Topology::Tetrahedron tetra = topoCon->getTetrahedron(layer2_tetra[i]);
        sofa::core::topology::Topology::Triangle tri = topoCon->getTriangle(triInfo[i]->elemId);
        unsigned int pId = sofa::core::topology::Topology::InvalidID;
        for (unsigned int i = 0; i < 4; i++)
        {
            bool found = false;
            for (unsigned int j = 0; j < 3; j++)
                if (tetra[i] == tri[j])
                {
                    found = true;
                    break;
                }

            if (!found) {
                pId = tetra[i];
                break;
            }
        }

        if (pId == sofa::core::topology::Topology::InvalidID)
        {
            msg_error() << "Point id not found in tetra: " << tetra << "and triangle: " << tri;
            continue;
        }

        DataTypes::Coord posOppo = pos[pId];
        for (auto vId : tri)
        {
            //pos[vId] = pos[vId] * (1 - carveFactor) + posOppo * carveFactor;
            pos[vId] = pos[vId] * (1 - carveFactor) + posOppo * carveFactor - triInfo[i]->normal * carveFactor *0.5;
        }
    }



    for (auto it : layer1_tetraAV)
    {
        DataTypes::Coord bbary;
        for (auto tetraID : it.second)
        {
            auto itB = barycenter.find(tetraID);
            if (itB == barycenter.end())
            {
                msg_error() << "barycenter not found for tetra: " << tetraID;
                continue;
            }
            bbary += itB->second;
        }

        bbary /= it.second.size();
        double dist = (bbary - pos[it.first]).norm2();
        auto itM = contactInfoMap.find(it.first);
        if (itM == contactInfoMap.end())
        {
            msg_error() << "point not found in contact map: " << it.first;
            continue;
        }

        if (dist < 0.01)
        {
            std::cout << "dist: " << dist << std::endl;
            
            // compute direction from tool to point to move

            DataTypes::Coord pointProjection = DataTypes::Coord(0.0, 0.0, 0.0);
            pointProjection = pos[it.first] + itM->second->normal/* * itM->second->dist*/;
            pos[it.first] = pos[it.first] * (1 - carveFactor) + (pointProjection)* carveFactor*0.5;
        }
        else
        {
            pos[it.first] = pos[it.first] * (1 - carveFactor) + (bbary)* carveFactor + (itM->second->normal)* carveFactor*0.5;
        }
        
       
        //int fac = int(std::round(it.second.size() / 3));

        //bbary += pointProjection * fac;
        //bbary /= it.second.size();

        //pointProjection = (1 - factorN*2) * bbary + factorN*2 * pointProjection;
        //bbary /= (it.second.size() + fac);
        //std::cout << "- final: " << pointProjection << " -> " << pos[it.first] * (1 - carveFactor) + (pointProjection)* carveFactor << std::endl;
        
        //std::cout << "- bef: idV: " << it.first << " = " << pos[it.first] << " | bary: " << pointProjection  << std::endl;
        //pos[it.first] = pos[it.first] * (1 - carveFactor) + (bbary) * carveFactor /*- triInfo[i]->normal * carveFactor*/;
        //std::cout << "- after idV: " << it.first << " = " << pos[it.first] << std::endl;

        m_contactPoints.push_back(pos[it.first]);
        m_contactPoints.push_back(bbary);
    }


   


    // check tetra size to remove them
    std::set<unsigned int> layer_tetra_toRemove;
    SReal minVolume = d_carvingCriteria.getValue();
    for (auto tetraId : surrounding_tetra)
    {        
        SReal volume = topoGeo->computeTetrahedronVolume(tetraId);

        if (volume < minVolume) {
            layer_tetra_toRemove.insert(tetraId);
        }

        bool goodT = topoGeo->checkTetrahedronValidity(tetraId);
        if (!goodT)
            layer_tetra_toRemove.insert(tetraId);
    }

    if (!layer_tetra_toRemove.empty())
    {
        //std::cout << "--- START Tetra removed: " << layer_tetra_toRemove.size() << std::endl;
        _tetraAlgo->removeTetrahedra(layer_tetra_toRemove);
        //std::cout << "--- Tetra removed: " << layer_tetra_toRemove.size() << std::endl;
    }

    if (m_forceFeedback)
        m_forceFeedback->setLock(false);

    return true;
}


bool AdvanceCarvingManager::doMoveCarve()
{
    // compute the list of tetra touched by the touched triangles
    const Real& carvingDistance = d_carvingDistance.getValue();
    sofa::type::vector<unsigned int> layer1_tetra;
    std::set <BaseMeshTopology::TetrahedronID> layer2_tetra;
    sofa::type::vector<contactInfo*> triInfo;
    sofa::type::vector<contactInfo*> pointInfo;
    std::list<unsigned int> idsP;

    TetrahedronRefinementAlgorithms* _tetraAlgo = nullptr;
    sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon;

    for each (contactInfo* cInfo in m_triangleContacts)
    {
        if (cInfo->dist > carvingDistance)
            continue;

        if (_tetraAlgo == nullptr) {
            _tetraAlgo = cInfo->tetraAlgo;
            topoCon = _tetraAlgo->getTopologyContainer();
        }
        else if (_tetraAlgo != cInfo->tetraAlgo)
        {
            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
            continue;
        }

         
        const core::topology::BaseMeshTopology::TetrahedraAroundTriangle & tetraAT = topoCon->getTetrahedraAroundTriangle(cInfo->elemId);

        if (tetraAT.size() != 1)
        {
            msg_error() << "More than one tetra around tri: " << cInfo->elemId;
            continue;
        }

        layer1_tetra.push_back(tetraAT[0]);
        layer2_tetra.insert(tetraAT[0]);
        triInfo.push_back(cInfo);

        const sofa::core::topology::Triangle& tri = cInfo->tetraAlgo->getTopologyContainer()->getTriangle(cInfo->elemId);
        for (unsigned int i = 0; i < 3; ++i)
            idsP.push_back(tri[i]);
    }

    for each (contactInfo* cInfo in m_pointContacts)
    {
        if (cInfo->dist > carvingDistance)
            continue;

        bool found = false;
        for (auto idP : idsP)
        {
            if (idP == cInfo->elemId)
            {
                found = true;
                break;
            }
        }

        if (found)
            continue;

        if (_tetraAlgo == nullptr) {
            _tetraAlgo = cInfo->tetraAlgo;
            topoCon = _tetraAlgo->getTopologyContainer();
        }
        else if (_tetraAlgo != cInfo->tetraAlgo)
        {
            msg_error() << "More than one TetrahedronRefinementAlgorithms in the list of contact. Case not handled yet: " << cInfo->elemId;
            continue;
        }

        const core::topology::BaseMeshTopology::TetrahedraAroundVertex & tetraAV = topoCon->getTetrahedraAroundVertex(cInfo->elemId);

        for (auto tetraID : tetraAV)
            layer2_tetra.insert(tetraID);

        pointInfo.push_back(cInfo);
    }


    if (pointInfo.empty() && triInfo.empty())
        return false;

    // get write accessor to the position
    sofa::component::container::MechanicalObject< sofa::defaulttype::Vec3Types>* meca = nullptr;
    topoCon->getContext()->get(meca);
    helper::WriteAccessor< Data<DataTypes::VecCoord> > pos = meca->write(core::VecCoordId::position());

    SReal carveFactor = d_carvingSpeed.getValue();
    // reduce touched tetra
    for (unsigned int i = 0; i<layer1_tetra.size(); ++i)
    {
        sofa::core::topology::Topology::Tetrahedron tetra = topoCon->getTetrahedron(layer1_tetra[i]);
        sofa::core::topology::Topology::Triangle tri = topoCon->getTriangle(triInfo[i]->elemId);
        unsigned int pId = sofa::core::topology::Topology::InvalidID;
        for (unsigned int i = 0; i < 4; i++)
        {
            bool found = false;
            for (unsigned int j = 0; j < 3; j++)
                if (tetra[i] == tri[j])
                {
                    found = true;
                    break;
                }

            if (!found) {
                pId = tetra[i];
                break;
            }
        }

        if (pId == sofa::core::topology::Topology::InvalidID)
        {
            msg_error() << "Point id not found in tetra: " << tetra << "and triangle: " << tri;
            continue;
        }

        DataTypes::Coord posOppo = pos[pId];
        for (auto vId : tri)
        {
            //pos[vId] = pos[vId] * (1 - carveFactor) + posOppo * factor1;
            pos[vId] = pos[vId] * (1 - carveFactor) + posOppo * carveFactor -triInfo[i]->normal * carveFactor;
        }
    }


    
    for (unsigned int i = 0; i < pointInfo.size(); ++i)
    {
        
        unsigned int idP = pointInfo[i]->elemId;
        pos[idP] = pos[idP] + pointInfo[i]->normal * carveFactor;
    }


    sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topoGeo;
    topoGeo = topoCon->getContext()->get<sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();


    // check tetra size to remove them
    std::set<unsigned int> layer_tetra_toRemove;
    SReal minVolume = d_carvingCriteria.getValue();
    for (auto tetraId : layer2_tetra)
    {
        SReal volume = topoGeo->computeTetrahedronVolume(tetraId);
        
        if (volume < minVolume) {
            layer_tetra_toRemove.insert(tetraId);
        }

        bool goodT = topoGeo->checkTetrahedronValidity(tetraId);
        if (!goodT)
            layer_tetra_toRemove.insert(tetraId);
    }

    if (!layer_tetra_toRemove.empty())
    {
       std::cout << "--- START Tetra removed: " << layer_tetra_toRemove.size() << std::endl;
       _tetraAlgo->removeTetrahedra(layer_tetra_toRemove);
       std::cout << "--- Tetra removed: " << layer_tetra_toRemove.size() << std::endl;
    }
    
    return true;
}




void AdvanceCarvingManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (!m_carvingReady)
        return;

    if (simulation::AnimateEndEvent::checkEventType(event))
    {
        //clearContacts();
        //if (d_active.getValue())
        filterCollision();
        /*
        if (m_toolCollisionModel)
        {
            core::behavior::BaseMechanicalState* state = m_toolCollisionModel->getContext()->getMechanicalState();
            if(state->getSize() > 0)
                m_toolPosition = Vector3(state->getPX(1), state->getPY(1), state->getPZ(1));
            //std::cout << "handleEvent: " << state->getSize() << std::endl;
            //std::cout << "handleEvent: " << state->getPX(0) << " - " << state->getPY(0) << " - " << state->getPZ(0) << std::endl;
            //if (dynamic_cast<core::behavior::MechanicalState<TDataTypes>*>(context->getMechanicalState())
        }
        */
    }
    else if (sofa::core::objectmodel::HapticDeviceEvent * ev = dynamic_cast<sofa::core::objectmodel::HapticDeviceEvent *>(event))
    {
        if (ev->getButtonState() == 1) d_active.setValue(true);
        else if (ev->getButtonState() == 0) d_active.setValue(false);
    }
    else if (sofa::core::objectmodel::ScriptEvent *ev = dynamic_cast<sofa::core::objectmodel::ScriptEvent *>(event))
    {
        const std::string& eventS = ev->getEventName();
        if (eventS.find(d_activatorName.getValue()) != std::string::npos && eventS.find("pressed") != std::string::npos)
            d_active.setValue(true);
        if (eventS.find(d_activatorName.getValue()) != std::string::npos && eventS.find("released") != std::string::npos)
            d_active.setValue(false);
    }

    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        if (ev->getKey() == 'C')
        {
            for (auto itm : m_tetraAlgos)
            {
                const sofa::type::vector <sofa::core::topology::Topology::TetraID>& tetraIds = itm.second->getTopologyGeometry()->computeBadTetrahedron();
                std::cout << "Bad tetra Nb: " << tetraIds.size() << std::endl;
            }
        }
    }
}

void AdvanceCarvingManager::draw(const core::visual::VisualParams* vparams)
{
    if (!m_carvingReady)
        return;

    if (!d_drawContacts.getValue())
        return;

    if (!m_triangleContacts.empty())
    {
        //std::vector<Vector3> normals;
        //std::vector<sofa::type::RGBAColor> colors;
        const Real& carvingDistance = d_carvingDistance.getValue();
        const Real& refineDistance = d_refineDistance.getValue();
        
        for each (contactInfo* cInfo in m_triangleContacts)
        {
            std::vector<Vector3> pos;

            sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = cInfo->tetraAlgo->getTopologyContainer();
            sofa::core::behavior::BaseMechanicalState* mstate = topoCon->getContext()->getMechanicalState();
            sofa::core::topology::Topology::Triangle tri = topoCon->getTriangle(cInfo->elemId);

            for (unsigned int j = 0; j < 3; j++) {
                pos.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
            }
            //normals.push_back(cInfo->normal);

            sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
            if (cInfo->dist < carvingDistance)
                color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
            else if (cInfo->dist < refineDistance)
                color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);
            
            //colors.push_back(color4);
            vparams->drawTool()->drawTriangle(pos[0], pos[1], pos[2], cInfo->normal, color4);
            //vparams->drawTool()->drawSphere(cInfo->pointA, 0.1f, sofa::type::RGBAColor(0.0, 1.0, 0.0f, 1.0));
        }        
    }
    
    if (!m_pointContacts.empty())
    {
        const Real& carvingDistance = d_carvingDistance.getValue();
        const Real& refineDistance = d_refineDistance.getValue();

        for each (contactInfo* cInfo in m_pointContacts)
        {
            std::vector<Vector3> pos;
            sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
            if (cInfo->dist < carvingDistance)
                color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
            else if (cInfo->dist < refineDistance)
                color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);

            //vparams->drawTool()->drawSphere(cInfo->pointA, 0.1f, sofa::type::RGBAColor(1.0, 0.0, 1.0f, 1.0));
            vparams->drawTool()->drawSphere(cInfo->pointB, 0.05f, color4);

            vparams->drawTool()->drawLine(cInfo->pointB, cInfo->pointB + cInfo->normal, sofa::type::RGBAColor(1.0, 0.0, 1.0f, 1.0));
        }
    }
    

    

    // draw tool position
    vparams->drawTool()->drawSphere(m_toolPosition, 0.1f, sofa::type::RGBAColor(1.0, 1.0, 1.0, 1.0));
    vparams->drawTool()->drawLine(m_toolPosition, m_toolPosition + m_toolForceFeedBack, sofa::type::RGBAColor(1.0, 0.0, 0.0f, 1.0));
    //        vparams->drawTool()->drawSphere(triInfo->pointB, 0.01f, green);

   
    if (m_topoCon && !m_tetra2remove.empty())
    {
        sofa::core::behavior::BaseMechanicalState* mstate = m_topoCon->getContext()->getMechanicalState();
        std::vector<Vector3> pos;
        
        
        if (!mstate)
        {
            std::cout << "mstate is null" << std::endl;
            return;
        }

        for (unsigned int i=0; i<m_tetra2remove.size(); ++i)
        {
            const sofa::core::topology::Topology::Tetrahedron& tri = m_topoCon->getTetrahedron(m_tetra2remove[i]);
            for (unsigned int j = 0; j < 4; j++) {
                pos.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
            }
        }

        vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(0.0f, 0.5f, 1.0f, 1.0f), 0.7f);
        //vparams->drawTool()->drawTriangles(pos, sofa::type::RGBAColor(0.0f, 0.5f, 1.0f, 1.0f));
    }
    
    // draw refine distance
    //std::vector<Vector3> spheres; spheres.push_back(m_toolPosition);
    //vparams->drawTool()->drawSpheres(spheres, d_refineDistance.getValue() + d_carvingRadius.getValue(), sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 0.5));


    if (!m_contactPoints.empty())
    {
        for (unsigned int i = 0; i<m_contactPoints.size()*0.5; ++i)
            vparams->drawTool()->drawLine(m_contactPoints[i*2], m_contactPoints[i*2+1], sofa::type::RGBAColor(0.0, 0.0, 1.0f, 1.0));
    }
    
/*
    

    if (d_drawTetra.getValue() && m_topoCon != nullptr)
    {
        const sofa::helper::fixed_array<sofa::type::vector<TetraToSplit*>, 2>& neighTable = m_tetraAlgo->getNeighboorhoodTable();
        const sofa::type::vector<sofa::core::topology::Topology::Tetrahedron>& tetraArray = m_topoCon->getTetrahedra();

        sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
        const float& scale = d_drawScaleTetrahedra.getValue();

        vparams->drawTool()->setPolygonMode(0,false);

        for (unsigned int i=0; i<neighTable.size(); ++i)
        {
            for (unsigned int j=0; j<neighTable[i].size(); ++j)
            {
                std::vector<Vector3> pos;
                const TetraToSplit* tetraStruc = neighTable[i][j];
                const sofa::core::topology::Topology::Tetrahedron& tetra = tetraArray[tetraStruc->m_tetraId];
                for (unsigned int k = 0; k < 4; ++k)
                    pos.push_back(Vector3(mstate->getPX(tetra[k]), mstate->getPY(tetra[k]), mstate->getPZ(tetra[k]) ));

                unsigned int nbrP = tetraStruc->m_points.size();
                if ( nbrP == 6)
                    color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
                else if (nbrP == 5)
                    color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.2f, 1.0);
                else if (nbrP == 4)
                    color4 = sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 1.0);
                else if (nbrP == 3)
                    color4 = sofa::type::RGBAColor(0.0f, 0.8, 0.5f, 1.0);
                else if (nbrP == 2)
                    color4 = sofa::type::RGBAColor(0.0f, 0.5, 0.8f, 1.0);
                else if (nbrP == 1)
                    color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);

                if (scale >= 1.0 || scale < 0.001)
                    vparams->drawTool()->drawTetrahedra(pos, color4);
                else
                    vparams->drawTool()->drawScaledTetrahedra(pos, color4, scale);
            }
        }

        if (!m_tetra2remove.empty())
        {
            std::vector<Vector3> pos;
            for (unsigned int i=0; i<m_tetra2remove.size(); ++i)
            {
                const sofa::core::topology::Topology::Tetrahedron& tetra = tetraArray[m_tetra2remove[i]];
                for (unsigned int k = 0; k < 4; ++k)
                    pos.push_back(Vector3(mstate->getPX(tetra[k]), mstate->getPY(tetra[k]), mstate->getPZ(tetra[k]) ));
            }

            if (scale >= 1.0 || scale < 0.001)
                vparams->drawTool()->drawTetrahedra(pos, sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 1.0));
            else
                vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(1.0f, 0.0, 0.0f, 1.0), scale);
        }


        vparams->drawTool()->setPolygonMode(0,vparams->displayFlags().getShowWireFrame());
    }

    */
}


} // namespace collision

} // namespace component

} // namespace sofa
