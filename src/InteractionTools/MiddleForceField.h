/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_FORCEFIELD_MiddleForceField_H
#define SOFA_COMPONENT_FORCEFIELD_MiddleForceField_H

#include <InteractionTools/config.h>
#include <sofa/core/behavior/ForceField.h>

namespace sofa
{

namespace component
{

namespace forcefield
{


/** Apply forces changing to given degres of freedom. Some keyTimes are given
* and the force to be applied is linearly interpolated between keyTimes. */
template<class DataTypes>
class MiddleForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MiddleForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename Coord::value_type Real;
    
    /// applied force for all the points
    Data< Real > d_force;

    /// the key frames when the forces are defined by the user
    Data< Real > d_pace;

protected:
    MiddleForceField();

public:
    void doInit() override;

    // ForceField methods
    /// Add the forces
    void addForce (const core::MechanicalParams* mparams, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;

    /// Compute the force derivative
    void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& /* d_df */, const DataVecDeriv& /* d_dx */) override;

    void addKToMatrix(sofa::defaulttype::BaseMatrix * matrix, SReal kFact, unsigned int &offset) override;

    SReal getPotentialEnergy(const core::MechanicalParams* mparams, const DataVecCoord& x) const override;

private :
    Coord m_bary;

}; // definition of the MiddleForceField class



#if  !defined(SOFA_COMPONENT_FORCEFIELD_MiddleForceField_CPP)
extern template class SOFA_INTERACTIONTOOLS_API MiddleForceField<sofa::defaulttype::Vec3Types>;
#endif //  !defined(SOFA_COMPONENT_FORCEFIELD_MiddleForceField_CPP)

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_MiddleForceField_H
