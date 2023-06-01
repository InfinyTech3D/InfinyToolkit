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

#include <InfinyToolkit/MiddleForceField.h>
#include <sofa/type/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/TopologySubsetData.inl>
#include <sofa/core/MechanicalParams.h>

namespace sofa::infinytoolkit
{

template<class DataTypes>
MiddleForceField<DataTypes>::MiddleForceField()
    : d_positions(initData(&d_positions, "position", "List of coordinates points"))
    , d_force(initData(&d_force, 1.0_sreal, "force", "Applied force to all points to simulate maximum compression."))
    , d_uniformForce(initData(&d_uniformForce, bool(false), "uniformForce", "If true, will apply the same force at each vertex otherwise will apply force proportional to the distance to the barycenter"))
    , d_pace(initData(&d_pace, 1.0_sreal, "pace", "Time to perform a full Pace (deflate + inflate). Same scale as the simulation time."))
    , d_refreshBaryRate(initData(&d_refreshBaryRate, (unsigned int)(0), "refreshBaryRate", "To recompute barycenter every X pace. 0 by default == no refresh"))
    , p_showForce(initData(&p_showForce, bool(false), "showForce", "Parameter to display the force direction"))
    , d_syncRealTime(initData(&d_syncRealTime, false, "syncRealTime", "Synchronize with the real time instead of simulation time."))
    , d_frequency(initData(&d_frequency, 1.0_sreal, "frequency", "Frequency at which the full deflate+inflate is done, in Hz, i.e x/sec. Used if pace is not set."))
    , m_startTime()
{ 

}


template<class DataTypes>
void MiddleForceField<DataTypes>::init()
{
    this->Inherit::init();

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

    computeBarycenter();

    m_startTime = std::chrono::system_clock::now();
    sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);
}


template<class DataTypes>
void MiddleForceField<DataTypes>::computeBarycenter()
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
void MiddleForceField<DataTypes>::addForce(const core::MechanicalParams* /*mparams*/, DataVecDeriv& f1, const DataVecCoord& p1, const DataVecDeriv&)
{
    if (sofa::core::objectmodel::BaseObject::d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    sofa::helper::WriteAccessor< core::objectmodel::Data< VecDeriv > > _f1 = f1;
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _p1 = p1;

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
    pacePercent = std::fmod(time, pace) / (pace * 0.5);

    Real factorForce = (pacePercent >= 1.0) ? 2 - pacePercent : pacePercent;

    msg_info() << "Time: " << time << " -> pacePercent: " << pacePercent << " -> " << factorForce;

    const Real force = d_force.getValue() * factorForce;
    const bool uniformF = d_uniformForce.getValue();
    for (size_t i = 0; i < _p1.size(); ++i)
    {
        Coord dir = m_bary - _p1[i];
        if (uniformF)
        {
            type::Vec3 dir3 = dir;
            dir3.normalize();
            _f1[i] += force * dir3;
        }
        else
        {
            _f1[i] += force * dir;
        }
    }


    // check if update barycenters
    unsigned int refreshRate = d_refreshBaryRate.getValue();
    if (refreshRate == 0)
        return;

    const int paceCpt = floor(time / pace);
    if (paceCpt - m_lastBaryRefresh == refreshRate) // need to recompute
    {
        m_lastBaryRefresh = paceCpt;
        computeBarycenter();
    }
    
}

template<class DataTypes>
void MiddleForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams, DataVecDeriv& d_df, const DataVecDeriv& d_dx )
{
    //TODO: implement this if really needed...
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(d_df);
    SOFA_UNUSED(d_dx);

    // remove warning about kFactor not being used in Debug mode
#ifndef NDEBUG
    mparams->setKFactorUsed(true);
#endif
}

template<class DataTypes>
void MiddleForceField<DataTypes>::addKToMatrix(linearalgebra::BaseMatrix* matrix, SReal kFact, unsigned int& offset)
{
    SOFA_UNUSED(matrix);
    SOFA_UNUSED(kFact);
    SOFA_UNUSED(offset);
}

template<class DataTypes>
SReal MiddleForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams* mparams, const DataVecCoord& x) const
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(x);
    SReal e = 0;
    return e;
}


template<class DataTypes>
void MiddleForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowForceFields() || !p_showForce.getValue()) {
        return;
    }
    const auto stateLifeCycle = vparams->drawTool()->makeStateLifeCycle();
    
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _x = d_positions;
    size_t nbPoints = _x.size();
    std::vector<sofa::type::Vec3> vertices;
    for (size_t i = 0; i < nbPoints; ++i)
    {
        vertices.emplace_back(_x[i]);
        vertices.emplace_back(m_bary);
    }
    vparams->drawTool()->drawLines(vertices, 1, sofa::type::RGBAColor(0, 1, 0, 1));
}

} // namespace sofa::infinytoolkit
