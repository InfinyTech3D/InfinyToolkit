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
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/core/objectmodel/DataFileName.cpp>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>


namespace sofa::infinytoolkit
{

    using namespace sofa::defaulttype;

    void registerMotionReplayController(sofa::core::ObjectFactory* factory)
    {
        factory->registerObjects(
            sofa::core::ObjectRegistrationData("Motion replay controller to induce the heart motion.")
            .add< MotionReplayController >()
        );
    }

    MotionReplayController::MotionReplayController()
        :l_gridState(initLink("gridState", "Link to the grid control."))
        , d_fixedIndices(initData(&d_fixedIndices, "fixedIndices", "Indices of the nodes that should be fixed."))
        , d_motionFile(initData(&d_motionFile, "motionFile",
            "Path to CSV motion file, where each row contains one frame."))
        , d_dvfTimeStep(initData(&d_dvfTimeStep, 0.02,
            "dvfTimeStep", " Time step used to record the DVF."))
        , d_displacementAmplitude(initData(&d_displacementAmplitude, 1.0, " displacementAmplitude", "Amplitude for extra motion."))
        , d_displacementAxis(initData(&d_displacementAxis, 1, "displacementAxis", " Axis along which the extra motion is applied: 0=X, 1=Y, 2=Z."))
        , d_infinyLoop(initData(&d_infinyLoop, true, "motionLoop", "Replay motion infinitely."))

    {

    }

    void MotionReplayController::init()
    {
        // Resolve MechanicalState
        if (l_gridState.get() == nullptr)
        {
            msg_error() << "Error no target grid found!";
            this->d_componentState.setValue(
                sofa::core::objectmodel::ComponentState::Invalid);
            return;

        }

        int axis = d_displacementAxis.getValue();

        if (axis < 0 || axis > 2)
        {
            msg_warning() << "Invalid motion axis: ", axis, ". Valid values are 0=X, 1=Y, 2=Z.";
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

        double currentTime = this->getContext()->getTime();
        // Compute frame index based on the current time and DVF time step
        size_t dvfIndex = static_cast<size_t>(currentTime / d_dvfTimeStep.getValue());
        if (dvfIndex >= frames.size())
        {
            if (d_infinyLoop.getValue())
                dvfIndex = 0;
            else
                return;
        }


        auto positions = l_gridState->writePositions();

        if (positions.size() != frames[dvfIndex].size())
        {
            msg_error() << "[MotionReplay] Frame size mismatch: "
                << "MO points = " << positions.size()
                << ", frame points = " << frames[dvfIndex].size();
            return;
        }

        double offset = 0.0;
        auto amplitudeOffset = d_displacementAmplitude.getValue();

        if (amplitudeOffset)
        {
            double frequency = 0.1;  // Hz
            double t = this->getContext()->getTime();
            offset = amplitudeOffset * sin(2.0 * M_PI * frequency * t);
        }

        const auto& fixedIndices = d_fixedIndices.getValue();

        if (fixedIndices.empty())
        {
            msg_warning() << "No fixed indices provided.";
            return;
        }

        std::unordered_set<unsigned int> fixedSet(
            fixedIndices.begin(),
            fixedIndices.end()
        );

        for (size_t i = 0; i < positions.size(); ++i)
        {
            positions[i][0] = frames[dvfIndex][i][0];
            positions[i][1] = frames[dvfIndex][i][1];
            positions[i][2] = frames[dvfIndex][i][2];

            if (fixedSet.find(static_cast<unsigned int>(i)) == fixedSet.end())
            {
                int axis = d_displacementAxis.getValue();  // 0=X, 1=Y, 2=Z
                positions[i][1] += offset;
            }
        }

        currentIndex = dvfIndex + 1;
    }

    void MotionReplayController::loadMotion()
    {
        frames.clear();
        currentIndex = 0;

        const std::string filename = d_motionFile.getFullPath();

        if (filename.empty())
        {
            msg_error() << "[MotionReplay] motionFile not specified!";
            return;
        }

        // Open file stream
        std::ifstream file(filename);
        if (!file.is_open())
        {
            msg_error() << "[MotionReplay] Cannot open file: " << filename;
            return;
        }

        size_t numPoints = l_gridState->getSize();

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
            frame.reserve(numPoints);

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
