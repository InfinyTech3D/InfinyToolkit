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
#pragma once

#include <InfinyToolkit/config.h>

#include <sofa/helper/LCPcalc.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/component/controller/Controller.h>
#include <sofa/component/haptics/ForceFeedback.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>

#include <sofa/simulation/Node.h>
#include <sofa/component/statecontainer/MechanicalObject.h>


namespace sofa::infinytoolkit
{

using namespace sofa::defaulttype;
using namespace sofa::simulation;
using core::objectmodel::Data;


class HapticEmulator;

class SOFA_INFINYTOOLKIT_API HapticEmulatorTask : public CpuTask
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
class SOFA_INFINYTOOLKIT_API HapticEmulator : public sofa::component::controller::Controller
{

public:
    SOFA_CLASS(HapticEmulator, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    typedef type::Vec3 Vec3;
    typedef type::Quatf Quat;

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
    sofa::component::haptics::ForceFeedback::SPtr m_forceFeedback;


    void applyTranslation(Vec3 translation);
    void worldToLocal(Vec3& vector);
    void moveUp();
    void moveDown();
    void moveLeft();
    void moveRight();
    void moveForward();
    void moveBackward();

    simulation::Node::SPtr m_omniVisualNode;
    sofa::component::statecontainer::MechanicalObject<sofa::defaulttype::Rigid3dTypes>::SPtr rigidDOF;

    int m_errorDevice; ///< Int detecting any error coming from device / detection
    bool m_isActivated; /// <Boolean storing hte information if Sofa has started the simulation (changed by AnimateBeginEvent)
    bool m_isInContact;

    sofa::type::fixed_array<bool, 2> oldStates;

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
    sofa::type::Vec3 m_toolForceFeedBack;

    std::mutex lockPosition;

    bool m_terminate;

    Vec3 m_toolPosition;
};

} // namespace sofa::infinytoolkit
