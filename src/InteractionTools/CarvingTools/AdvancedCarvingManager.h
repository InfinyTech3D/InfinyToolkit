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
#include <InteractionTools/CarvingTools/BaseCarvingPerformer.h>
#include <sofa/type/Vec.h>

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>

#include <SofaBaseTopology/TopologyData.h>
#include <SofaHaptics/ForceFeedback.h>

#include <fstream>
#include <thread>
#include <mutex>

namespace sofa::component::collision
{

using namespace sofa::type;

class contactInfo
{
public:
    unsigned int elemId; // in global mesh
    Vector3 pointA;
    Vector3 pointB;
    Vector3 normal;
    double dist;
    //TetrahedronRefinementAlgorithms* tetraAlgo;
    sofa::core::topology::BaseMeshTopology* topo;
};


/**
* The AdvancedCarvingManager class will perform topological resection on a triangle surface (could be on top of tetrahedron topology)
* The tool performing the carving need to be represented by a collision model @sa toolCollisionModel
* The surface to be carved are also mapped on collision models @sa surfaceCollisionModels
* Detecting the collision is done using the scene Intersection and NarrowPhaseDetection pipeline.
*/
class SOFA_INTERACTIONTOOLS_API AdvancedCarvingManager : public core::behavior::BaseController
{
public:
    SOFA_CLASS(AdvancedCarvingManager,sofa::core::behavior::BaseController);

    typedef defaulttype::Vec3Types DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;

    using ToolCollisionModel = sofa::core::CollisionModel;
    using SurfaceCollisionModel = sofa::core::CollisionModel;
    using ContactVector = type::vector<core::collision::DetectionOutput>;
    using BaseCarvingPerformer = sofa::component::controller::BaseCarvingPerformer;

    /// Sofa API init method of the component
    void bwdInit() override;

    /// Method to handle various event like keyboard or omni.
    void handleEvent(sofa::core::objectmodel::Event* event) override;

    /// Impl method that will compute the intersection and check if some element have to be removed.
    virtual void filterCollision();

    void draw(const core::visual::VisualParams* vparams) override;
    void clearContacts();

protected:
    /// Default constructor
    AdvancedCarvingManager();

    /// Default destructor
    ~AdvancedCarvingManager() override;

    void processCollision();

public:
    /// Tool model path
    Data < std::string > d_toolModelPath;
    /// TriangleSetModel or SphereModel path
    Data < std::string > d_surfaceModelPath;

    Data < bool > d_active;
    /// Collision distance at which cavring will start. Equal to contactDistance by default.
    Data < Real > d_carvingDistance;
    Data < Real > d_carvingRadius;
    Data < Real > d_refineDistance;
    Data < Real > d_carvingCriteria;
    Data < Real > d_refineCriteria;
    Data < Real > d_carvingSpeed;
    Data < Real > d_refineThreshold;

    Data < Real > d_sphereRadius;

    ///< Activate carving with string Event, the activator name has to be inside the script event. Will look for 'pressed' or 'release' keyword. For example: 'button1_pressed'
    Data < std::string > d_activatorName;

    Data<sofa::type::vector<unsigned int> > m_testID;

    Data < bool > d_drawTetra;
    Data < bool > d_drawContacts;
    Data <float> d_drawScaleTetrahedra; ///< Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)

protected:
    std::mutex lockContraints;

    /// Pointer to the tool collision model
    ToolCollisionModel* m_toolCollisionModel;

    // Pointer to the target object collision model
    std::vector<SurfaceCollisionModel*> m_surfaceCollisionModels;
    
    

    //sofa::component::controller::ForceFeedback::SPtr m_forceFeedback;

    // Pointer to the scene detection Method component (Narrow phase only)
    core::collision::NarrowPhaseDetection* m_detectionNP;
    
    //sofa::component::topology::TetrahedronSetTopologyContainer::SPtr m_topoCon;

    Vector3 m_toolPosition;

    Vector3 m_toolForceFeedBack;

    // Bool to store the information if component has well be init and can be used.
    bool m_carvingReady;

    sofa::type::vector< BaseCarvingPerformer*> m_carvingPerformer;
    sofa::type::vector<contactInfo*> m_triangleContacts;
    sofa::type::vector<contactInfo*> m_pointContacts;

    sofa::type::vector<contactInfo> m_realContacts;

    sofa::type::vector<int> m_tetra2remove;

    sofa::type::vector<int> _tetra2remove;
    std::map <unsigned int, SReal> m_tetraVolumes;

    sofa::type::vector<Vector3> m_contactPoints;

    bool m_canCarve;

    int m_mgrStatus;
};

} // namespace sofa::component::collision
