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

#include <sofa/component/controller/MechanicalStateController.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/Event.h>

#include <vector>
#include <string>

namespace sofa::infinytoolkit
{


template<class DataTypes>
class MotionReplayController
    : public sofa::component::controller::MechanicalStateController<DataTypes>
{
public:
    SOFA_CLASS(
        SOFA_TEMPLATE(MotionReplayController, DataTypes),
        SOFA_TEMPLATE(sofa::component::controller::MechanicalStateController, DataTypes)
    );

    using Inherit = sofa::component::controller::MechanicalStateController<DataTypes>;
    using Coord    = typename DataTypes::Coord;
    using VecCoord = typename DataTypes::VecCoord;
    using Real     = typename DataTypes::Real;

    MotionReplayController();
    ~MotionReplayController() override = default;

   void init() override;
   void handleEvent(sofa::core::objectmodel::Event* event) override;

private:
    sofa::core::objectmodel::Data<std::string> d_motionFile;
    sofa::core::objectmodel::Data<double> d_dt;

    std::vector<VecCoord> frames;
    size_t currentIndex;

    void loadMotion();
};

#if !defined(SOFA_COMPONENT_MOTIONREPLAYCONTROLLER_CPP)
extern template class SOFA_INFINYTOOLKIT_API MotionReplayController<sofa::defaulttype::Vec3Types>;
#endif 


} // namespace sofa::infinytoolkit
