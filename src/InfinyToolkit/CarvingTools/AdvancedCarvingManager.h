/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InfinyToolkit/config.h>
#include <InfinyToolkit/CarvingTools/BaseCarvingPerformer.h>
#include <sofa/type/Vec.h>

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/core/CollisionModel.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/objectmodel/HapticDeviceEvent.h>

#include <sofa/core/topology/TopologyData.h>

#include <fstream>
#include <thread>
#include <mutex>

namespace sofa::component::collision
{

using namespace sofa::type;
using namespace sofa::component::controller;
using namespace sofa::core::topology;

/**
* The AdvancedCarvingManager class will perform topological resection on a triangle surface (could be on top of tetrahedron topology)
* The tool performing the carving need to be represented by a collision model @sa toolCollisionModel
* The surface to be carved are also mapped on collision models @sa surfaceCollisionModels
* Detecting the collision is done using the scene Intersection and NarrowPhaseDetection pipeline.
*/
class SOFA_INFINYTOOLKIT_API AdvancedCarvingManager : public core::behavior::BaseController
{
public:
    SOFA_CLASS(AdvancedCarvingManager,sofa::core::behavior::BaseController);

    using ToolCollisionModel = sofa::core::CollisionModel;
    using SurfaceCollisionModel = sofa::core::CollisionModel;
    using ContactVector = type::vector<core::collision::DetectionOutput>;
    using BaseCarvingPerformer = sofa::component::controller::BaseCarvingPerformer;
    using TetrahedronSetTopologyContainer = sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer;
    
    using TexCoord = sofa::type::Vec<2, float>;
    using VecTexCoord = type::vector<TexCoord>;

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

public:
    /// Tool model path
    Data < std::string > d_toolModelPath;
    /// TriangleSetModel or SphereModel path
    Data < std::string > d_surfaceModelPath;

    Data < bool > d_active;
    Data < bool > d_carvingWithBurning;
    Data < bool > d_carvingWithRefinement;
    
    
    /// Collision distance at which cavring will start. Equal to contactDistance by default.
    Data < SReal > d_carvingDistance;
    Data < SReal > d_refineDistance;
    Data < SReal > d_refineCriteria;
    Data < SReal > d_carvingSpeed;
    Data < SReal > d_refineThreshold;
    Data<sofa::type::vector<unsigned int> > m_testID;    
    
    core::topology::PointData< VecTexCoord > m_vtexcoords; ///< coordinates of the texture

    Data < bool > d_drawContacts;

private:
    /// Mutex to lock constraints for haptic use
    std::mutex lockConstraints;

    /// Pointer to the tool collision model
    ToolCollisionModel* m_toolCollisionModel = nullptr;

    // Pointer to the target object collision model
    std::vector<SurfaceCollisionModel*> m_surfaceCollisionModels;
    
    
    // Pointer to the scene detection Method component (Narrow phase only)
    core::collision::NarrowPhaseDetection* m_detectionNP = nullptr;
    
    // Bool to store the information if component has well be init and can be used.
    bool m_carvingReady = false;
    
    bool m_texCoordsHandling = false;

    sofa::type::vector< BaseCarvingPerformer*> m_carvingPerformer;
};

} // namespace sofa::component::collision