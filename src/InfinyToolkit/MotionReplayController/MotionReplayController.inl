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

#include <InfinyToolkit/MotionReplayController/MotionReplayController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/component/statecontainer/MechanicalObject.h>


#include <sofa/core/objectmodel/Context.h>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <fstream>
#include <sstream>
#include <string>

namespace sofa::infinytoolkit
{

MotionReplayController::MotionReplayController()
    : d_motionFile(initData(&d_motionFile, "motionFile",
        "Path to CSV motion file, where each row contains one frame."))
    , d_dt(initData(&d_dt, 0.02, "dt", "Time step of the SOFA scene"))
    {
    }

void MotionReplayController::init()
    {
        mGridState = this->getContext()->get<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes>>();

    if (!mGridState)
    {
        msg_error() << "[MotionReplay] MechanicalState is null!";
        return;
    }

    this->f_listening.setValue(true);

    loadMotion();
        
    }

 
void MotionReplayController::handleEvent(sofa::core::objectmodel::Event* event)
   {
           
       if (!sofa::simulation::AnimateBeginEvent::checkEventType(event))
           return;

       if (frames.empty())
           return;

       // Always loop like Python version
       if (currentIndex >= frames.size())
           currentIndex = 0;

       auto positions =  mGridState->writePositions();

       if (positions.size() != frames[currentIndex].size())
       {
           msg_error() << "[MotionReplay] Frame size mismatch: "
               << "MO points = " << positions.size()
               << ", frame points = " << frames[currentIndex].size();
           return;
       }

       for (size_t i = 0; i < positions.size(); ++i)
       {
           positions[i] = Coord(
               frames[currentIndex][i][0],
               frames[currentIndex][i][1],
               frames[currentIndex][i][2]);           
       }

       ++currentIndex;
   }

 void MotionReplayController::loadMotion()
    {
        frames.clear();
        currentIndex = 0;

        std::string filename = d_motionFile.getValue();


        if (filename.empty())
        {
            msg_error() << "[MotionReplay] motionFile not specified!";
            return;
        }

        std::string fullpath = sofa::helper::system::DataRepository.getFile(filename);
        std::ifstream file(fullpath);
        if (!file.is_open())
        {
            msg_error() << "[MotionReplay] Cannot open file: " << filename;
            return;
        }

        size_t numPoints = mGridState->getSize();

        std::string line;
        size_t lineNumber = 0;
        while (std::getline(file, line))
        {
            ++lineNumber;
            std::stringstream ss(line);
            std::string value;

            std::vector<double> values;
            while (std::getline(ss, value, ','))
            {
                values.push_back(std::stod(value));
            }

            if (values.size() != numPoints * 3)
            {
                msg_error() << "[MotionReplay] Line " << lineNumber
                    << ": expected " << numPoints * 3
                    << " values, got " << values.size();
                frames.clear();
                return;
            }

            VecCoord frame;
            frame.reserve(numPoints); // have to be checked

            for (size_t i = 0; i < numPoints; ++i)
            {
                Coord c;
               c[0] = values[3 * i + 0];
               c[1] = values[3 * i + 1];
               c[2] = values[3 * i + 2];
               frame.push_back(c);
            }

            frames.push_back(std::move(frame));
        }

        msg_info() << "[MotionReplay] Loaded " << frames.size()
            << " frames from " << filename;
    }



} // namespace sofa::infinytoolkit
