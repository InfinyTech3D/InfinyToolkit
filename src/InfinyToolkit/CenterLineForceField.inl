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

#include <InfinyToolkit/CenterLineForceField.h>
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
CenterLineForceField<DataTypes>::CenterLineForceField()
    : d_positions(initData(&d_positions, "position", "List of coordinates points"))
    , d_restPositions(initData(&d_restPositions, "restPositions", "List of rest coordinates points"))
	, d_outputPositions(initData(&d_outputPositions, "outputPositions", "List of output coordinates points"))
    , d_centers(initData(&d_centers, "centers", "List of center coordinates points"))
    , d_force(initData(&d_force, 1.0_sreal, "force", "Applied force to all points to simulate maximum compression."))
    , d_uniformForce(initData(&d_uniformForce, bool(false), "uniformForce", "If true, will apply the same force at each vertex otherwise will apply force proportional to the distance to the barycenter"))
    , d_pace(initData(&d_pace, 1.0_sreal, "pace", "Time to perform a full Pace (deflate + inflate). Same scale as the simulation time."))
    
    , d_stiffness(initData(&d_stiffness, 1000.0_sreal, "stiffness", "Time to perform a full Pace (deflate + inflate). Same scale as the simulation time."))

    
    , d_refreshBaryRate(initData(&d_refreshBaryRate, (unsigned int)(0), "refreshBaryRate", "To recompute barycenter every X pace. 0 by default == no refresh"))
    , p_showForce(initData(&p_showForce, bool(false), "showForce", "Parameter to display the force direction"))
    , d_syncRealTime(initData(&d_syncRealTime, false, "syncRealTime", "Synchronize with the real time instead of simulation time."))
    , d_frequency(initData(&d_frequency, 1.0_sreal, "frequency", "Frequency at which the full deflate+inflate is done, in Hz, i.e x/sec. Used if pace is not set."))
    , m_startTime()
{ 
    addInput(&d_restPositions);
    addInput(&d_positions);
    addInput(&d_centers);

    addOutput(&d_outputPositions);
	this->f_listening.setValue(true);
}


template<class DataTypes>
void CenterLineForceField<DataTypes>::init()
{
    size_t nbPoints = d_positions.getValue().size();
    if (nbPoints == 0)
    {
        msg_error() << "No position set or empty vector given into field: 'position'. Won't be able to compute pace.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    if (!d_pace.isSet() && !d_frequency.isSet())
    {
        msg_error() << "Neither Pace nor frequency are set. Won't be able to compute pace.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    if (d_pace.isSet() && d_frequency.isSet())
    {
        msg_warning() << "Pace and frequency are both set. Will take into account the frequency value.";
    }

    computeDistribution();

    m_startTime = std::chrono::system_clock::now();
    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
	nextStart = d_pace.getValue() / length;
}


template<class DataTypes>
void CenterLineForceField<DataTypes>::computeBarycenter()
{
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _x = d_positions;
    
    m_bary = Coord(0.0, 0.0, 0.0);
    size_t nbPoints = _x.size();
    for (size_t i = 0; i < nbPoints; ++i)
    {
        m_bary += _x[i];        
    }

    m_bary /= nbPoints;
}



template<class DataTypes>
void CenterLineForceField<DataTypes>::computeDistribution()
{
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _x = d_positions;
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _centers = d_centers;
	//m_centersOrdered.resize(_centers.size());
	m_centersOrdered = _centers;
	std::sort(m_centersOrdered.begin(), m_centersOrdered.end(),
		[](const Coord& a, const Coord& b) {
            return a[1] > b[1];
		});

    for (size_t i = 0; i < _centers.size(); ++i)
    {
		std::cout << i << ": " << _centers[i] << " | " << m_centersOrdered[i] << std::endl;
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
void CenterLineForceField<DataTypes>::doUpdate()
{
    if (sofa::core::objectmodel::BaseObject::d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    sofa::helper::WriteAccessor< core::objectmodel::Data< VecDeriv > > outX = d_outputPositions;
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > inX = d_positions;
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _centers = d_centers;

    outX.resize(inX.size());
    Real pacePercent = 1.0_sreal;
    Real time = 1.0_sreal;

    const Real pace = (!d_frequency.isSet()) ? d_pace.getValue() : Real(1.0 / d_frequency.getValue());

    if (d_syncRealTime.getValue())
    {
        using namespace std::chrono;

        std::chrono::duration<Real> duration = system_clock::now() - m_startTime;
        time = duration.count();
    }
    else
    {
        time = this->getContext()->getTime();
    }

    // we apply a force proportional to the pace rate. 0 Force at start of pace, 0 at end, F at half pace
    pacePercent = std::fmod(time, pace) / pace;
    //std::cout << "std::fmod(time, pace): " << std::fmod(time, pace) << std::endl;
    //std::cout << "std::fmod(time, pace)/2*pace: " << std::fmod(time, pace)/(2*pace) << std::endl;
	//std::cout << "time: " << time << " | pace: " << pace << " | pacePercent: " << pacePercent << std::endl;

	if (time >= nextStart) // at every pace/2 we start moving a new center and stop previous one
    {
        centerStart++;
        centerDone++;

        //if (centerCurrent == 4)
        {
		/*	centerDone++;
			centerCurrent = 0;*/
            std::cout << "Center " << centerDone << " done at time " << time << std::endl;
        }
        
        //centerDone++;
        std::cout << "Center " << centerStart << " started at time " << time << std::endl;
        
		nextStart += pace / length;
    }

    // G ---- X -- X0

    const Real amplitude = 0.8;
	const Real frequency = pace;    

    //std::cout << "oscillation: " << oscillation << std::endl;
	//std::cout << "centerStart: " << centerStart << " | " << centerDone << " / " << _centers.size() << " centers done." << std::endl;
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

        // cos: [-1; 1]  1  -1  1
        // [0; 2]   2   0   2
		// [0; 1]   1   0   1
		// [1; 0]   0   1   0
        
		// 0.9   -0.9   0.9
		// 1.8    0      1.8
		// 0.9    0      0.9
		// 0     0.9    0

        outX[i] = p0 + dir * oscillation;
    }
	


    if (centerDone >= int(m_centersOrdered.size()) && pacePercent >= 0.99)
    {
        std::cout << "centerDone " << centerDone << " | " << m_centersOrdered.size() << std::endl;
    
        centerDone = -4;
        centerStart = 0;
        nextStart = time + pace / length;
    }


}

template<class DataTypes>
void CenterLineForceField<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    //std::cout << "event" << std::endl;
    if (simulation::AnimateBeginEvent::checkEventType(event))
    {
        d_positions.setDirtyOutputs();
    }
}


template<class DataTypes>
void CenterLineForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (/*!vparams->displayFlags().getShowForceFields() || */!p_showForce.getValue()) {
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
