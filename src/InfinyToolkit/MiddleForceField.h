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
#include <sofa/core/behavior/ForceField.h>

#include <chrono>

namespace sofa::infinytoolkit
{

/** Apply forces changing to given degres of freedom. Some keyTimes are given
* and the force to be applied is linearly interpolated between keyTimes. */
template<class DataTypes>
class MiddleForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MiddleForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    using Inherit = core::behavior::ForceField<DataTypes>;
    using VecCoord = typename DataTypes::VecCoord;
    using VecDeriv = typename DataTypes::VecDeriv;
    using Coord = typename DataTypes::Coord;
    using Deriv = typename DataTypes::Deriv;
    using Real = typename Coord::value_type;
    using DataVecCoord = core::objectmodel::Data<VecCoord>;
    using DataVecDeriv = core::objectmodel::Data<VecDeriv>;
    
    MiddleForceField();

    void init() override;

    // ForceField methods
    void addForce(const core::MechanicalParams* mparams, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;
    void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx) override;

    void addKToMatrix(linearalgebra::BaseMatrix* matrix, SReal kFact, unsigned int& offset) override;

    SReal getPotentialEnergy(const core::MechanicalParams* mparams, const DataVecCoord& x) const override;

    void draw(const core::visual::VisualParams* vparams) override;

protected:
    /// Will compute the barycenter of the given set of coordinates.
    void computeBarycenter();

public:
    /// List of coordinates points
    Data<VecCoord> d_positions;

    /// Applied force to all points to simulate maximum compression.
    Data<Real> d_force;

    /// If true, will apply the same force at each vertex otherwise will apply force proportional to the distance to the barycenter
    Data<bool> d_uniformForce;

    /// Time to perform a full Pace (deflate + inflate). Same scale as the simulation time.
    Data<Real> d_pace;

    /// To recompute barycenter every X pace. 0 by default == no refresh
    Data<unsigned int> d_refreshBaryRate;


    /// Parameter to display the force direction
    Data<bool> p_showForce;

    // Synchronize with real time (instead of simulation time)
    Data<bool> d_syncRealTime;

    // Frequency at which a full deflate+inflate is done
    Data<Real> d_frequency;
    
private :
    /// Computed barycenter of the given positions @sa d_positions
    Coord m_bary;

    /// counter to the last pace the barycenter has been refreshed. To be used with @sa d_refreshBaryRate
    unsigned int m_lastBaryRefresh = 0;

    // keep trace of the latest time we measured
    std::chrono::time_point<std::chrono::system_clock> m_startTime;

}; // definition of the MiddleForceField class



#if !defined(SOFA_COMPONENT_FORCEFIELD_MiddleForceField_CPP)
extern template class SOFA_INFINYTOOLKIT_API MiddleForceField<sofa::defaulttype::Vec3Types>;
#endif //  !defined(SOFA_COMPONENT_FORCEFIELD_MiddleForceField_CPP)

} // namespace sofa::infinytoolkit
