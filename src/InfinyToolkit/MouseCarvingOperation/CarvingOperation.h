/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, June 2021          *
 ****************************************************************************/
#pragma once

#include <MeshRefinement/config.h>
#include <MeshRefinement/TetrahedronCuttingManager.h>
#include <sofa/gui/common/MouseOperations.h>
#include <sofa/gui/component/performer/InteractionPerformer.h>

using sofa::meshrefinement::TetrahedronCuttingManager;

namespace sofa::gui::component::performer
{

class SOFA_MESHREFINEMENT_API CuttingPerformer : public InteractionPerformer
{
public:
    CuttingPerformer(BaseMouseInteractor* interactor);
    
    void start();
    void execute();
    void draw(const core::visual::VisualParams* vparams);

    void setIncisionDepth(double value) { m_depth = value; }

protected:
    sofa::meshrefinement::TetrahedronCuttingManager<sofa::defaulttype::Vec3Types>* m_tetraCuttingMgr;
    TetrahedronSetTopologyContainer* m_topoCon;
    TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr m_topoGeo;

    Vec3 m_pos0;
    Vec3 m_pos1;
    
    double m_depth;

    bool isInit;
    int clickCount;
};

} // namespace sofa::gui::component::performer


namespace sofa::gui::common
{

class SOFA_MESHREFINEMENT_API CuttingOperation : public Operation
{
public:
    CuttingOperation();
    ~CuttingOperation();

    void start() override;
    void execution() override;
    void end() override;
    void endOperation() override;

    virtual double getDepth() const { return incisionDepth; }

    static std::string getDescription() { return "3D mesh incision using the Mouse"; }

protected:
    double incisionDepth;
    int clickCount;
};



} // namespace sofa::gui::common
