/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/
#pragma once

#include <InteractionTools/MiddleForceField.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/BaseVector.h>

#include <SofaBaseTopology/TopologySubsetData.inl>

namespace sofa
{

namespace component
{

namespace forcefield
{

template<class DataTypes>
MiddleForceField<DataTypes>::MiddleForceField()
    : d_force(initData(&d_force, (Real)1.0, "force", "applied force to all points"))
    , d_pace(initData(&d_pace, (Real)1.0, "pace", "applied force to all points"))
{ 

}


template<class DataTypes>
void MiddleForceField<DataTypes>::init()
{
    core::behavior::BaseMechanicalState* state = this->getContext()->getMechanicalState();
    m_bary = Coord(0.0, 0.0, 0.0);

    if (state == nullptr || state->getSize() == 0)
    {
        msg_warning() << "Wrong BaseMechanicalState pointer or buffer size";
        return;
    }

    size_t nbPoints = state->getSize();
    for (size_t i = 0; i < nbPoints; ++i)
    {
        m_bary[0] += state->getPX(i);
        m_bary[1] += state->getPY(i);
        m_bary[2] += state->getPZ(i);
    }

    m_bary /= nbPoints;
}

template<class DataTypes>
void MiddleForceField<DataTypes>::addForce(const core::MechanicalParams* /*mparams*/, DataVecDeriv& f1, const DataVecCoord& p1, const DataVecDeriv&)
{
    sofa::helper::WriteAccessor< core::objectmodel::Data< VecDeriv > > _f1 = f1;
    sofa::helper::ReadAccessor< core::objectmodel::Data< VecCoord > > _p1 = p1;

    Real cT = this->getContext()->getTime();
    
    const Real& pace = d_pace.getValue();
    Real pacePercent = fmod (cT, pace) / pace;
    
    Real factor = 0;
    if (pacePercent < 0.5)
        factor = pacePercent * 2;
    else
        factor = 2 - (pacePercent * 2);

    Real factorForce = d_force.getValue()*factor;

    for (size_t i = 0; i < _p1.size(); ++i)
    {
        Coord dir = m_bary - _p1[i];
        _f1[i] += factorForce * dir;
    }
}

template<class DataTypes>
void MiddleForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams, DataVecDeriv& /* d_df */, const DataVecDeriv& /* d_dx */)
{
    //TODO: remove this line (avoid warning message) ...
    //mparams->setKFactorUsed(true);
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


} // namespace forcefield

} // namespace component

} // namespace sofa
