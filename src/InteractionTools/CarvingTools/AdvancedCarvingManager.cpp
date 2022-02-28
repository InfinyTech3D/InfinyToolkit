/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/CarvingTools/AdvancedCarvingManager.h>
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


namespace sofa::component::collision
{

using namespace sofa::core::topology;

int AdvancedCarvingManagerClass = core::RegisterObject("Manager handling carving operations between a tool and an object.")
.add< AdvancedCarvingManager >()
;


AdvancedCarvingManager::AdvancedCarvingManager()
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
    , m_testID(initData(&m_testID, sofa::type::vector<unsigned int>(), "testID", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , d_drawTetra( initData(&d_drawTetra, false, "drawTetra", "Activate this object.\nNote that this can be dynamically controlled by using a key") )
    , d_drawContacts( initData(&d_drawContacts, false, "drawContacts", "Activate this object.\nNote that this can be dynamically controlled by using a key") )
    , d_drawScaleTetrahedra(initData(&d_drawScaleTetrahedra, (float) 1.0, "drawScaleTetrahedra", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , d_sphereRadius(initData(&d_sphereRadius, (Real) 0.1, "sphereRadius", "Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)"))
    , d_activatorName(initData(&d_activatorName, "button1", "activatorName", "Name to active the script event parsing. Will look for 'pressed' or 'release' keyword. For example: 'button1_pressed'"))
    , m_toolCollisionModel(nullptr)
    , m_detectionNP(nullptr)
    //, m_topoCon(nullptr)
    , m_carvingReady(false)    
    , m_canCarve(true)
    , m_mgrStatus(0)
    
    //, m_forceFeedback(nullptr)
{
    this->f_listening.setValue(true);
}


AdvancedCarvingManager::~AdvancedCarvingManager()
{
    m_mgrStatus = 0;
}


void AdvancedCarvingManager::bwdInit()
{
    // Search for collision model corresponding to the tool.
    if (d_toolModelPath.getValue().empty())
        m_toolCollisionModel = getContext()->get<ToolCollisionModel>(core::objectmodel::Tag("CarvingTool"), core::objectmodel::BaseContext::SearchRoot);
    else
        m_toolCollisionModel = getContext()->get<ToolCollisionModel>(d_toolModelPath.getValue());

    // Search for the surface collision model.
    if (d_surfaceModelPath.getValue().empty())
    {
        // we look for a CollisionModel relying on a TetrahedronSetTopology.
        std::vector<SurfaceCollisionModel*> models;
        getContext()->get<SurfaceCollisionModel>(&models, core::objectmodel::Tag("CarvingSurface"), core::objectmodel::BaseContext::SearchRoot);
        
        // If topological mapping, iterate into child Node to find mapped topology
        for (size_t i=0; i<models.size(); ++i)
        {
            core::CollisionModel* m = models[i];
            m_surfaceCollisionModels.push_back(m);
        }
    }
    else
    {
        m_surfaceCollisionModels.push_back(getContext()->get<core::CollisionModel>(d_surfaceModelPath.getValue()));
    }


    m_detectionNP = getContext()->get<core::collision::NarrowPhaseDetection>();
    m_carvingReady = true;

    if (m_toolCollisionModel == nullptr) { msg_error() << "m_toolCollisionModel not found"; m_carvingReady = false; }
    if (m_surfaceCollisionModels.empty()) { msg_error() << "m_surfaceCollisionModels not found"; m_carvingReady = false; }
    if (m_detectionNP == nullptr) { msg_error() << "NarrowPhaseDetection not found"; m_carvingReady = false; }
    
    if (!m_carvingReady) {
        msg_error() << "AdvancedCarvingManager: initialisation failed.";
        return;
    }
}


void AdvancedCarvingManager::clearContacts()
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
}


void AdvancedCarvingManager::filterCollision()
{
    if (!m_carvingReady)
        return;

    lockContraints.lock();
    clearContacts();

    // get the collision output
    const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = m_detectionNP->getDetectionOutputs();
    if (detectionOutputs.size() == 0) // exit if no collision
    {
        lockContraints.unlock();
        return;
    }


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
        //sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = targetModel->getContext()->get<sofa::component::topology::TetrahedronSetTopologyContainer>();
        //auto itM = m_tetraAlgos.find(topoCon);
        //if (itM == m_tetraAlgos.end()) {
        //    msg_error() << "No TetrahedronRefinementAlgorithms found for collision model: " << targetModel->name;
        //    continue;
        //}

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
            info->topo = targetModel->getCollisionTopology();
            //info->tetraAlgo = itM->second;

            if (mode == 0)
                m_triangleContacts.push_back(info);
            else if (mode == 1)
                m_pointContacts.push_back(info);
        }
        
        // process the collision               
    }
    lockContraints.unlock();
    //if (d_active.getValue())
      //  processCollision();
}


void AdvancedCarvingManager::processCollision()
{
    //if (m_triangleContacts.empty() && m_pointContacts.empty())
    //    return;

    //m_topoCon = nullptr;
    //bool resRef = doRefinement();    
    //if (resRef == false) // no refinement at this step, carve
    //    doMoveCarve();
    //    //doMoveCarvePoint();
}



void AdvancedCarvingManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (!m_carvingReady)
        return;

    if (simulation::AnimateEndEvent::checkEventType(event))
    {
        filterCollision();
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

    //if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    //{
    //    if (ev->getKey() == 'C')
    //    {
    //        for (auto itm : m_tetraAlgos)
    //        {
    //            const sofa::type::vector <sofa::core::topology::Topology::TetraID>& tetraIds = itm.second->getTopologyGeometry()->computeBadTetrahedron();
    //            std::cout << "Bad tetra Nb: " << tetraIds.size() << std::endl;
    //        }
    //    }
    //}
}

void AdvancedCarvingManager::draw(const core::visual::VisualParams* vparams)
{
    return;
    if (!m_carvingReady)
        return;

    if (!d_drawContacts.getValue())
        return;

    //if (!m_triangleContacts.empty())
    //{
    //    const Real& carvingDistance = d_carvingDistance.getValue();
    //    const Real& refineDistance = d_refineDistance.getValue();
    //    
    //    for each (contactInfo* cInfo in m_triangleContacts)
    //    {
    //        std::vector<Vector3> pos;

    //        sofa::component::topology::TetrahedronSetTopologyContainer::SPtr topoCon = cInfo->topo;
    //        sofa::core::behavior::BaseMechanicalState* mstate = topoCon->getContext()->getMechanicalState();
    //        sofa::core::topology::Topology::Triangle tri = topoCon->getTriangle(cInfo->elemId);

    //        for (unsigned int j = 0; j < 3; j++) {
    //            pos.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
    //        }
    //        //normals.push_back(cInfo->normal);

    //        sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
    //        if (cInfo->dist < carvingDistance)
    //            color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
    //        else if (cInfo->dist < refineDistance)
    //            color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);
    //        
    //        //colors.push_back(color4);
    //        vparams->drawTool()->drawTriangle(pos[0], pos[1], pos[2], cInfo->normal, color4);
    //        //vparams->drawTool()->drawSphere(cInfo->pointA, 0.1f, sofa::type::RGBAColor(0.0, 1.0, 0.0f, 1.0));
    //    }        
    //}
    

    //if (!m_pointContacts.empty())
    //{
    //    const Real& carvingDistance = d_carvingDistance.getValue();
    //    const Real& refineDistance = d_refineDistance.getValue();

    //    for each (contactInfo* cInfo in m_pointContacts)
    //    {
    //        std::vector<Vector3> pos;
    //        sofa::type::RGBAColor color4(1.0f, 0.0, 0.0f, 1.0);
    //        if (cInfo->dist < carvingDistance)
    //            color4 = sofa::type::RGBAColor(0.0f, 1.0, 0.0f, 1.0);
    //        else if (cInfo->dist < refineDistance)
    //            color4 = sofa::type::RGBAColor(0.0f, 0.0, 1.0f, 1.0);

    //        //vparams->drawTool()->drawSphere(cInfo->pointA, 0.1f, sofa::type::RGBAColor(1.0, 0.0, 1.0f, 1.0));
    //        vparams->drawTool()->drawSphere(cInfo->pointB, 0.05f, color4);

    //        vparams->drawTool()->drawLine(cInfo->pointB, cInfo->pointB + cInfo->normal, sofa::type::RGBAColor(1.0, 0.0, 1.0f, 1.0));
    //    }
    //}
    

    

    // draw tool position
    vparams->drawTool()->drawSphere(m_toolPosition, 0.1f, sofa::type::RGBAColor(1.0, 1.0, 1.0, 1.0));
    vparams->drawTool()->drawLine(m_toolPosition, m_toolPosition + m_toolForceFeedBack, sofa::type::RGBAColor(1.0, 0.0, 0.0f, 1.0));
    //        vparams->drawTool()->drawSphere(triInfo->pointB, 0.01f, green);

   
    //if (m_topoCon && !m_tetra2remove.empty())
    //{
    //    sofa::core::behavior::BaseMechanicalState* mstate = m_topoCon->getContext()->getMechanicalState();
    //    std::vector<Vector3> pos;
    //    
    //    
    //    if (!mstate)
    //    {
    //        std::cout << "mstate is null" << std::endl;
    //        return;
    //    }

    //    for (unsigned int i=0; i<m_tetra2remove.size(); ++i)
    //    {
    //        const sofa::core::topology::Topology::Tetrahedron& tri = m_topoCon->getTetrahedron(m_tetra2remove[i]);
    //        for (unsigned int j = 0; j < 4; j++) {
    //            pos.push_back(Vector3(mstate->getPX(tri[j]), mstate->getPY(tri[j]), mstate->getPZ(tri[j])));
    //        }
    //    }

    //    vparams->drawTool()->drawScaledTetrahedra(pos, sofa::type::RGBAColor(0.0f, 0.5f, 1.0f, 1.0f), 0.7f);
    //    //vparams->drawTool()->drawTriangles(pos, sofa::type::RGBAColor(0.0f, 0.5f, 1.0f, 1.0f));
    //}
    
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


} // namespace sofa::component::collision
