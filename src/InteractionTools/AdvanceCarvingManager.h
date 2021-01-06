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
#include <MeshRefinement/TetrahedronRefinementAlgorithms.h>

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

namespace sofa
{

namespace component
{

namespace collision
{

    class contactInfo
    {
    public:
        unsigned int elemId; // in global mesh
        defaulttype::Vector3 pointA;
        defaulttype::Vector3 pointB;
        defaulttype::Vector3 normal;
        double dist;
        TetrahedronRefinementAlgorithms* tetraAlgo;
    };

/**
* The AdvanceCarvingManager class will perform topological resection on a triangle surface (could be on top of tetrahedron topology)
* The tool performing the carving need to be represented by a collision model @sa toolCollisionModel
* The surface to be carved are also mapped on collision models @sa surfaceCollisionModels
* Detecting the collision is done using the scene Intersection and NarrowPhaseDetection pipeline.
*/
class SOFA_INTERACTIONTOOLS_API AdvanceCarvingManager : public core::behavior::BaseController
{
public:
    SOFA_CLASS(AdvanceCarvingManager,sofa::core::behavior::BaseController);

    typedef defaulttype::Vec3Types DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;

    typedef helper::vector<core::collision::DetectionOutput> ContactVector;

    /// Sofa API init method of the component
    void bwdInit() override;

    /// Method to handle various event like keyboard or omni.
    void handleEvent(sofa::core::objectmodel::Event* event) override;

    /// Impl method that will compute the intersection and check if some element have to be removed.
    virtual void filterCollision();

    void draw(const core::visual::VisualParams* vparams) override;
    void clearContacts();

    defaulttype::Vector3 computeForceFeedBack(const defaulttype::Vector3& position);
protected:
    /// Default constructor
    AdvanceCarvingManager();

    /// Default destructor
    ~AdvanceCarvingManager() override;

    bool doRefinement();

    bool doMoveCarve();

    bool doMoveCarvePoint();

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

    Data < bool > d_delayMode;
    Data<sofa::helper::vector<unsigned int> > m_testID;

    Data < bool > d_drawTetra;
    Data < bool > d_drawContacts;
    Data <float> d_drawScaleTetrahedra; ///< Scale of the terahedra (between 0 and 1; if <1.0, it produces gaps between the tetrahedra)

protected:
    std::mutex lockContraints;
    /// Pointer to the tool collision model
    core::CollisionModel* m_toolCollisionModel;

    // Pointer to the target object collision model
    std::vector<core::CollisionModel*> m_surfaceCollisionModels;
    std::map<sofa::component::topology::TetrahedronSetTopologyContainer::SPtr, TetrahedronRefinementAlgorithms*> m_tetraAlgos;
    TetrahedronRefinementAlgorithms* m_tetraAlgo;


    sofa::component::controller::ForceFeedback::SPtr m_forceFeedback;

    // Pointer to the scene detection Method component (Narrow phase only)
    core::collision::NarrowPhaseDetection* m_detectionNP;
    
    sofa::component::topology::TetrahedronSetTopologyContainer::SPtr m_topoCon;
    sofa::component::topology::TriangleSetTopologyContainer::SPtr m_topoSurface;

    defaulttype::Vector3 m_toolPosition;

    defaulttype::Vector3 m_toolForceFeedBack;

    // Bool to store the information if component has well be init and can be used.
    bool m_carvingReady;

    sofa::helper::vector<contactInfo*> m_triangleContacts;
    sofa::helper::vector<contactInfo*> m_pointContacts;

    sofa::helper::vector<contactInfo> m_realContacts;

    sofa::helper::vector<int> m_tetra2remove;

    sofa::helper::vector<int> _tetra2remove;
    std::map <unsigned int, SReal> m_tetraVolumes;

    sofa::helper::vector<defaulttype::Vector3> m_contactPoints;

    bool m_canCarve;

    int m_mgrStatus;
    
};

} // namespace collision

} // namespace component

} // namespace sofa
