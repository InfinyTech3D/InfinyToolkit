/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_INTERACTIONTOOLS_HAPTICEMULATOR_H
#define SOFA_INTERACTIONTOOLS_HAPTICEMULATOR_H

#include <InteractionTools/config.h>

#include <sofa/helper/LCPcalc.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <SofaUserInteraction/Controller.h>
#include <SofaHaptics/ForceFeedback.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>

#include <SofaBaseMechanics/MechanicalObject.h>


namespace sofa 
{

namespace component 
{

namespace controller 
{

using namespace sofa::defaulttype;
using namespace sofa::simulation;
using core::objectmodel::Data;


class HapticEmulator;

class SOFA_INTERACTIONTOOLS_API HapticEmulatorTask : public CpuTask
{
public:
    HapticEmulatorTask(HapticEmulator* ptr, CpuTask::Status* pStatus);

    virtual ~HapticEmulatorTask() {}

    virtual MemoryAlloc run() override final;

private:
    HapticEmulator * m_driver;
};


/**
* Geomagic driver
*/
class SOFA_INTERACTIONTOOLS_API HapticEmulator : public Controller
{

public:
    SOFA_CLASS(HapticEmulator, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    typedef defaulttype::Vec4f Vec4f;
    typedef defaulttype::Vector3 Vector3;


    Data< std::string > d_deviceName; ///< Name of device Configuration
    Data<Vec3> d_positionBase; ///< Position of the interface base in the scene world coordinates
    Data<Quat> d_orientationTool; ///< Orientation of the tool
    Data<double> d_scale; ///< Default scale applied to the Phantom Coordinates
    Data<double> d_forceScale; ///< Default forceScale applied to the force feedback. 
    Data< Coord > d_posDevice; ///< position of the base of the part of the device    
    
    Data<bool> d_button_1; ///< Button state 1
    Data<bool> d_button_2; ///< Button state 2
    Data<std::string> d_toolNodeName;
    Data <SReal> d_speedFactor; /// < factor to increase/decrease the movements speed
    Data<double> d_maxInputForceFeedback; ///< Maximum value of the normed input force feedback for device security
    sofa::simulation::Node::SPtr m_toolNode;

    HapticEmulator();
	virtual ~HapticEmulator();

    virtual void init() override;
    virtual void bwdInit() override;

    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;
    void updatePosition();
    void updateButtonStates(bool emitEvent);
    void initDevice(int cptInitPass = 0);
    void clearDevice();
    void activateTool(bool value);
    ForceFeedback::SPtr m_forceFeedback;


    void applyTranslation(sofa::defaulttype::Vec3 translation);
    void worldToLocal(sofa::defaulttype::Vec3& vector);
    void moveUp();
    void moveDown();
    void moveLeft();
    void moveRight();
    void moveForward();
    void moveBackward();

    simulation::Node::SPtr m_omniVisualNode;
    component::container::MechanicalObject<sofa::defaulttype::Rigid3dTypes>::SPtr rigidDOF;

    int m_errorDevice; ///< Int detecting any error coming from device / detection
    bool m_isActivated; /// <Boolean storing hte information if Sofa has started the simulation (changed by AnimateBeginEvent)
    bool m_isInContact;

    sofa::helper::fixed_array<bool, 2> oldStates;

    /**
    * @brief Key Press event callback.
    */
    void onKeyPressedEvent(core::objectmodel::KeypressedEvent *kEvent);

    /**
    * @brief Key Release event callback.
    */
    void onKeyReleasedEvent(core::objectmodel::KeyreleasedEvent *kEvent);


private:
    void handleEvent(core::objectmodel::Event *) override;

    bool findNode(sofa::simulation::Node::SPtr node);
        
    

public:
    sofa::simulation::TaskScheduler* _taskScheduler;
    sofa::simulation::CpuTask::Status _simStepStatus;
    sofa::defaulttype::Vector3 m_toolForceFeedBack;

    std::mutex lockPosition;

    bool m_terminate;

    Vec3 m_toolPosition;
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_INTERACTIONTOOLS_HAPTICEMULATOR_H
