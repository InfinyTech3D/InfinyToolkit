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

#include <InfinyToolkit/ProximityOscillatorConstraint.h>
#include <sofa/type/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/TopologySubsetData.inl>
#include <sofa/core/MechanicalParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa::infinytoolkit
{

template<class DataTypes>
ProximityOscillatorConstraint<DataTypes>::ProximityOscillatorConstraint()
    : d_positions(initData(&d_positions, "position", "List of coordinates points"))
    , d_outputPositions(initData(&d_outputPositions, "outputPositions", "List of output coordinates points"))
    , d_centers(initData(&d_centers, "centers", "List of center coordinates points"))
    , d_pace(initData(&d_pace, 1.0_sreal, "pace", "Time to perform a full Pace (deflate + inflate). Same scale as the simulation time."))
	, d_amplitude(initData(&d_amplitude, 0.8_sreal, "amplitude", "Amplitude of the oscillation"))
    , p_showMotion(initData(&p_showMotion, bool(false), "showMotion", "Parameter to display the force direction"))
    , m_startTime()
{ 
    addInput(&d_positions);
    addInput(&d_centers);

    addOutput(&d_outputPositions);
	this->f_listening.setValue(true);
}


template<class DataTypes>
void ProximityOscillatorConstraint<DataTypes>::init()
{
    size_t nbPoints = d_positions.getValue().size();
    if (nbPoints == 0)
    {
        msg_error() << "No position set or empty vector given into field: 'position'. Won't be able to compute pace.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    if (!d_pace.isSet())
    {
        msg_error() << "Pace is not set. Won't be able to compute pace.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    computeDistribution();

    m_startTime = std::chrono::system_clock::now();
    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
	nextStart = d_pace.getValue() / length;
}


template<class DataTypes>
void ProximityOscillatorConstraint<DataTypes>::computeDistribution()
{
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _x = d_positions;
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _centers = d_centers;
	
	m_centersOrdered = _centers;
	std::sort(m_centersOrdered.begin(), m_centersOrdered.end(),
		[](const Coord& a, const Coord& b) {
            return a[1] > b[1];
		});

    if (f_printLog.getValue())
    {
        for (size_t i = 0; i < _centers.size(); ++i)
        {
            std::cout << i << ": " << _centers[i] << " | " << m_centersOrdered[i] << std::endl;
        }
    }

    m_distribution.resize(_x.size());

    Coord pt2;
    auto dist = [](const Coord& a, const Coord& b) { return (b - a).norm(); };
    auto cmp = [&pt2, &dist](const Coord& a, const Coord& b) {
        return dist(a, pt2) < dist(b, pt2);
    };

    for (size_t i = 0; i < _x.size(); ++i)
    {
        pt2 = _x[i];
        auto it = std::min_element(m_centersOrdered.begin(), m_centersOrdered.end(), cmp);
        m_distribution[i] = std::distance(m_centersOrdered.begin(), it);
	}
}


template<class DataTypes>
void ProximityOscillatorConstraint<DataTypes>::doUpdate()
{
    if (sofa::core::objectmodel::BaseObject::d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    sofa::helper::WriteAccessor< core::objectmodel::Data< VecDeriv > > outX = d_outputPositions;
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > inX = d_positions;
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _centers = d_centers;

    outX.resize(inX.size());
    Real time = this->getContext()->getTime();

    const Real pace = d_pace.getValue();

    // we apply a force proportional to the pace rate. 0 Force at start of pace, 0 at end, F at half pace
    const Real pacePercent = std::fmod(time, pace) / pace;
 
	if (time >= nextStart) // at every pace/2 we start moving a new center and stop previous one
    {
        centerStart++;
        centerDone++;
		nextStart += pace / length;
    }

    // G ---- X -- X0

    const Real amplitude = d_amplitude.getValue();
	const Real frequency = pace;    

    for (size_t i = 0; i < inX.size(); ++i)
    {
        int centerId = m_distribution[i];
        if (centerId > centerStart || centerId <= centerDone )
        {
			outX[i] = inX[i];
			continue;
        }

		const Coord& center = m_centersOrdered[centerId];
        const Coord& p0 = inX[i];
        Coord dir = center - p0;

        //Real omega = centerId * i;//2.0 * M_PI * frequency;
        //Real omega = Real(centerId) / Real(_centers.size()) * M_PI * 12;
        Real omega = centerId * 2.0 * M_PI / length;
		Real oscillation = amplitude * std::cos(pacePercent * 2.0 * M_PI - omega);
        oscillation = amplitude - (oscillation + amplitude) / 2.0_sreal; // normalize between 0 and 0.9

        outX[i] = p0 + dir * oscillation;
    }

    if (centerDone >= int(m_centersOrdered.size()) && pacePercent >= 0.99)
    {
		if (f_printLog.getValue())
            std::cout << "centerDone " << centerDone << " | " << m_centersOrdered.size() << std::endl;
    
        centerDone = -4;
        centerStart = 0;
        nextStart = time + pace / length;
    }
}

template<class DataTypes>
void ProximityOscillatorConstraint<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    //std::cout << "event" << std::endl;
    if (simulation::AnimateBeginEvent::checkEventType(event))
    {
        d_positions.setDirtyOutputs();
    }
}


template<class DataTypes>
void ProximityOscillatorConstraint<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!p_showMotion.getValue()) {
        return;
    }
    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();
    
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _x = d_outputPositions;
    //sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _centers = d_centers;

    size_t nbPoints = _x.size();
    std::vector<sofa::type::Vec3> vertices;
    for (size_t i = 0; i < nbPoints; ++i)
    {
        vertices.emplace_back(_x[i]);
        vertices.emplace_back(m_centersOrdered[m_distribution[i]]);
    }
    vparams->drawTool()->drawLines(vertices, 1, sofa::type::RGBAColor(0, 1, 0, 1));
}

} // namespace sofa::infinytoolkit
