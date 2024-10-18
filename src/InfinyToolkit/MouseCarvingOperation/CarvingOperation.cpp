/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, June 2021          *
 ****************************************************************************/
#include <MeshRefinement/MouseCuttingOperation/CuttingOperation.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/gui/common/MouseOperations.h>
#include <sofa/gui/component/performer/ComponentMouseInteraction.h>
#include <sofa/gui/common/PickHandler.h>

namespace sofa::gui::component::performer
{

CuttingPerformer::CuttingPerformer(BaseMouseInteractor* interactor)
    : InteractionPerformer(interactor)
    , m_tetraCuttingMgr(nullptr)
    , m_topoCon(nullptr)
    , m_topoGeo(nullptr)
    , isInit(false)
    , clickCount(0)
    , m_depth(1.0)
{
    m_tetraCuttingMgr = new sofa::meshrefinement::TetrahedronCuttingManager<sofa::defaulttype::Vec3Types>();
}


void CuttingPerformer::start()
{
    ////pickHandle->getInteraction()->mouseInteractor;
    //std::cout << "CuttingPerformer::start()" << std::endl;

    BodyPicked picked = this->m_interactor->getBodyPicked();
    if (!picked.body) return;

    if (!isInit)
    {
        sofa::core::objectmodel::BaseContext* base = picked.body->getContext();

        m_topoCon = base->get<TetrahedronSetTopologyContainer>();
        if (m_topoCon == nullptr) // possible Tetra2Triangle topology
        {
            base->get(m_topoCon, core::objectmodel::BaseContext::SearchUp);
            if (m_topoCon == nullptr)
            {
                msg_warning("CuttingPerformer") << "No TetrahedronSetTopologyContainer found in target object.";
                isInit = false;
                return;
            }
            else
            {
                m_topoGeo = m_topoCon->getContext()->get<TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
            }
        }
        else {
            m_topoGeo = m_topoCon->getContext()->get<TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
        }

        m_tetraCuttingMgr->init(m_topoCon->getContext());
        isInit = true;
    }

    
    if (clickCount == 0)
    {
        m_pos0 = picked.point;
        clickCount++;
    }
    else if (clickCount == 1)
    {
        if (!m_topoGeo)
            return;

        m_pos1 = picked.point;
        
        fixed_array<Vec3, 4> m_planPositions;
        m_planPositions[0] = m_pos0;
        m_planPositions[1] = m_pos1;

       // sofa::component::collision::RayCollisionModel* rayModel = this->interactor->getMouseRayModel();
        sofa::component::collision::geometry::Ray ray = this->m_interactor->getMouseRayModel()->getRay(0);
        Vec3 normA = -ray.direction();
        normA.normalize();
//         = m_topoGeo->computeTriangleNormal(picked.indexCollisionElement);
        //normA.normalize();

        m_planPositions[2] = m_planPositions[1] - normA * m_depth * 0.1;
        m_planPositions[3] = m_planPositions[0] - normA * m_depth * 0.1;
        
        Vec3 m_planNormal = (m_planPositions[1] - m_planPositions[0]).cross(normA);
        m_tetraCuttingMgr->createCutPlanPath(m_planPositions, m_planNormal, 1.0);

        clickCount++;
    }
    else if (clickCount == 2)
    {
        m_tetraCuttingMgr->processCut(false);
    }
}


void CuttingPerformer::execute()
{
    BodyPicked picked = this->m_interactor->getBodyPicked();
    if (!picked.body) return;

    if (clickCount == 1)
    {
        m_pos1 = picked.point;
    }

}


void CuttingPerformer::draw(const core::visual::VisualParams* vparams)
{
    if (!isInit)
        return;
    
    if (clickCount > 0) {
        vparams->drawTool()->drawSphere(m_pos0, float(1.0), RGBAColor::red());
    }
    if (clickCount > 1) {
        vparams->drawTool()->drawSphere(m_pos1, float(1.0), RGBAColor::green());
        m_tetraCuttingMgr->drawCutPlan(vparams);
        m_tetraCuttingMgr->drawDebugCut(vparams);
    }

    

    

}


sofa::helper::Creator<InteractionPerformer::InteractionPerformerFactory, CuttingPerformer >  CuttingPerformerClass("CuttingPerformer", true);

} // namespace sofa::gui::component::performer



namespace sofa::gui::common
{

CuttingOperation::CuttingOperation()
    : clickCount(0)
{
    
}

CuttingOperation::~CuttingOperation()
{

}

void CuttingOperation::start()
{
    if (performer == nullptr)
    {
        performer = component::performer::InteractionPerformer::InteractionPerformerFactory::getInstance()->createObject("CuttingPerformer", pickHandle->getInteraction()->mouseInteractor.get());
        pickHandle->getInteraction()->mouseInteractor->addInteractionPerformer(performer);

        component::performer::CuttingPerformer* performerCut = dynamic_cast<component::performer::CuttingPerformer*>(performer);
        if (performerCut)
            performerCut->setIncisionDepth(this->getDepth());
    }

    if (performer) {
        performer->start();
    }

    //if (clickCount == 0)
    //{
    //    
    //}

}

void CuttingOperation::execution()
{
    //std::cout << "CuttingOperation::execution()" << std::endl;
}

void CuttingOperation::end()
{
    //std::cout << "CuttingOperation::end()" << std::endl;
}

void CuttingOperation::endOperation()
{
    if (performer)
    {
        pickHandle->getInteraction()->mouseInteractor->removeInteractionPerformer(performer);
        delete performer; performer = nullptr;
        clickCount = 0;
    }
}


} // namespace sofa::gui::common