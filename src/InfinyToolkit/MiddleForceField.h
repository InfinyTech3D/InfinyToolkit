/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InfinyToolkit/config.h>
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

    using Inherit = core::behavior::ForceField<DataTypes>;
    using VecCoord = typename DataTypes::VecCoord;
    using VecDeriv = typename DataTypes::VecDeriv;
    using Coord = typename DataTypes::Coord;
    using Deriv = typename DataTypes::Deriv;
    using Real = typename Coord::value_type;
    using DataVecCoord = core::objectmodel::Data<VecCoord>;
    using DataVecDeriv = core::objectmodel::Data<VecDeriv>;
    
    /// applied force for all the points
    Data< Real > d_force;

    /// the key frames when the forces are defined by the user
    Data< Real > d_pace;

protected:
    MiddleForceField();

public:
    void init() override;

    // ForceField methods
    /// Add the forces
    void addForce (const core::MechanicalParams* mparams, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;

    /// Compute the force derivative
    void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& /* d_df */, const DataVecDeriv& /* d_dx */) override;

    void addKToMatrix(linearalgebra::BaseMatrix * matrix, SReal kFact, unsigned int &offset) override;

    SReal getPotentialEnergy(const core::MechanicalParams* mparams, const DataVecCoord& x) const override;

private :
    Coord m_bary;

}; // definition of the MiddleForceField class



#if !defined(SOFA_COMPONENT_FORCEFIELD_MiddleForceField_CPP)
extern template class SOFA_INFINYTOOLKIT_API MiddleForceField<sofa::defaulttype::Vec3Types>;
#endif //  !defined(SOFA_COMPONENT_FORCEFIELD_MiddleForceField_CPP)

} // namespace forcefield

} // namespace component

} // namespace sofa
