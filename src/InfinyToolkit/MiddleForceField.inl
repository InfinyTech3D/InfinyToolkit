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

namespace sofa::infinytoolkit
{

template<class DataTypes>
MiddleForceField<DataTypes>::MiddleForceField()
    : d_positions(initData(&d_positions, "position", "List of coordinates points"))
    , d_force(initData(&d_force, 1.0_sreal, "force", "Applied force to all points to simulate maximum compression."))
    , d_pace(initData(&d_pace, 1.0_sreal, "pace", "Time to perform a full Pace (deflate + inflate). Same scale as the simulation time."))
    , p_showForce(initData(&p_showForce, bool(false), "showForce", "Parameter to display the force direction"))
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

    computeBarycenter();

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

    Real cT = this->getContext()->getTime();
    
    const Real& pace = d_pace.getValue();
    // we apply a force proportional to the pace rate. 0 Force at start of pace, 0 at end, F at half pace
    const Real pacePercent = fmod(cT, pace) / (pace * 0.5);
    const Real factorForce = (pacePercent >= 1.0) ? 2 - pacePercent : pacePercent;
    
    dmsg_info() << "cT: " << cT << " -> pacePercent: " << pacePercent << " -> " << factorForce;

    const Real force = d_force.getValue() * factorForce;
    for (size_t i = 0; i < _p1.size(); ++i)
    {
        Coord dir = m_bary - _p1[i];
        _f1[i] += force * dir;
    }
}

template<class DataTypes>
void MiddleForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams, DataVecDeriv& d_df, const DataVecDeriv& d_dx )
{
    //TODO: implement this if really needed...
    SOFA_UNUSED(d_df);
    SOFA_UNUSED(d_dx);
}

template<class DataTypes>
void MiddleForceField<DataTypes>::addKToMatrix(linearalgebra::BaseMatrix* matrix, SReal kFact, unsigned int& offset)
{
    SOFA_UNUSED(matrix);
    SOFA_UNUSED(kFact);
    SOFA_UNUSED(offset);
}

template<class DataTypes>
SReal MiddleForceField<DataTypes>::getPotentialEnergy(const core::MechanicalParams* /*mparams*/, const DataVecCoord& x) const
{
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
    std::vector<sofa::type::Vector3> vertices;
    for (size_t i = 0; i < nbPoints; ++i)
    {
        vertices.emplace_back(_x[i]);
        vertices.emplace_back(m_bary);
    }
    vparams->drawTool()->drawLines(vertices, 1, sofa::type::RGBAColor(0, 1, 0, 1));
}

} // namespace sofa::infinytoolkit
