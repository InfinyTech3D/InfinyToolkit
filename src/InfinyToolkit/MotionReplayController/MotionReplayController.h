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

#include <sofa/component/controller/Controller.h>

#include <sofa/type/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/DataFileName.h>

#include <vector>
#include <string>

namespace sofa::infinytoolkit
{

class MotionReplayController
    : public sofa::component::controller::Controller
{
public:
   
    SOFA_CLASS(MotionReplayController,
                sofa::component::controller::Controller);
    
    MotionReplayController();
    ~MotionReplayController() override = default;


   using Coord = sofa::type::Vec3d;
   using VecCoord = std::vector<Coord>;


   
   void init() override;
   void handleEvent(sofa::core::objectmodel::Event* event) override;

private:
    
    sofa::core::objectmodel::DataFileName d_motionFile; /// CSV file containing the frames
    sofa::core::objectmodel::Data<double> d_dt; /// Simulation time-step

    sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes>* mGridState{nullptr}; ///Controlled grid
    
    std::vector<VecCoord> frames;
    
    size_t currentIndex{0};

    void loadMotion();
};


} // namespace sofa::infinytoolkit
