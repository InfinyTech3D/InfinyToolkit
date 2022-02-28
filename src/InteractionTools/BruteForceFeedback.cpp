/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InteractionTools/BruteForceFeedback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/Node.h>

using namespace std;

namespace sofa
{
namespace component
{
namespace controller
{

BruteForceFeedback::BruteForceFeedback()
    : forceCoef(initData(&forceCoef, 0.03, "forceCoef", "multiply haptic force by this coef."))
    , m_ACarving(NULL)
{

}

void BruteForceFeedback::init()
{
    this->ForceFeedback::init();
    currentForce = sofa::type::Vector3(0, 0, 0);

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_ACarving = context->get<sofa::component::collision::AdvancedCarvingManager>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_ACarving == NULL)
    {
        msg_error() << "AdvanceCarvingManager not found";
    }
};

void BruteForceFeedback::computeForce(SReal x, SReal y, SReal z, SReal /*u*/, SReal /*v*/, SReal /*w*/, SReal /*q*/, SReal& fx, SReal& fy, SReal& fz)
{
    const SReal& fCoef = forceCoef.getValue();
    Vec3 position = Vec3(x, y, z);
    currentForce = Vec3(0, 0, 0);
    if (m_ACarving)
    {
        //currentForce = m_ACarving->computeForceFeedBack(position);
    }

    fx = currentForce[0] * fCoef;
    fy = currentForce[1] * fCoef;
    fz = currentForce[2] * fCoef;
};

void BruteForceFeedback::computeWrench(const sofa::defaulttype::SolidTypes<SReal>::Transform &/*world_H_tool*/, const sofa::defaulttype::SolidTypes<SReal>::SpatialVector &/*V_tool_world*/, sofa::defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world )
{
    W_tool_world.clear();
};



//Vector3 AdvanceCarvingManager::computeForceFeedBack(const Vector3& position)
//{
//    lockContraints.lock();
//    // todo change that
//    const Real& refineDistance = d_refineDistance.getValue();
//    m_toolForceFeedBack = Vector3(0, 0, 0);
//    m_toolPosition = position;
//    int cpt = 0;
//    SReal facMax = 33;
//    
//    //SReal responseDist = 0.3;
//    SReal responseDist = refineDistance;
//    SReal contactDist = 0.1;
//    const SReal& sphereRadius = d_sphereRadius.getValue();
//
//
//    std::list<unsigned int> idsP;
//
//    if (!m_triangleContacts.empty())
//    {
//        msg_info() << "-------------- TRIANGLE START ----------------";
//        cpt = 0;
//        Vector3 ffTri = Vector3(0, 0, 0);
//        for each (contactInfo* cInfo in m_triangleContacts)
//        {
//            Vector3 vec = position - cInfo->pointA;
//            SReal realDistance = vec.norm();
//            SReal collDistance = realDistance - contactDist - sphereRadius;
//            msg_info() << "id: " << cInfo->elemId << " realDistance: " << realDistance << " | collDistance: " << collDistance << " | cInfo->dist: " << cInfo->dist;
//
//            if (collDistance > responseDist)
//                continue;
//
//            if (collDistance < 0.0)
//            {
//                std::cout << "collDistance: " << collDistance << std::endl;
//                facMax *= (1 + fabs(collDistance));
//                collDistance = 0.0;
//            }
//
//            SReal factorN = (collDistance - responseDist) * (collDistance - responseDist);
//            msg_info() << "(collDistance - responseDist):  " << (collDistance - responseDist) << " | facMax: " << facMax;
//            factorN *= factorN;
//            factorN *= facMax;
//            msg_info() << "factorN:  " << factorN;
//            vec.normalize();
//
//            //cInfo->normal = vec * factorN;
//            m_toolForceFeedBack += cInfo->normal * factorN;
//
//            const sofa::core::topology::Triangle& tri = cInfo->tetraAlgo->getTopologyContainer()->getTriangle(cInfo->elemId);
//            for (unsigned int i = 0; i < 3; ++i)
//                idsP.push_back(tri[i]);
//            
//            cpt++;
//            msg_info() << "collDistance:  " << collDistance << " - " << responseDist << " = " << factorN;
//            msg_info() << "vec: " << cInfo->normal.norm() << " ++ " << m_toolForceFeedBack.norm();
//        }
//
//        msg_info() << " ### force: " << m_toolForceFeedBack.norm();
//        msg_info() << "-------------- TRIANGLE END ----------------";
//    }
//
//    facMax = 33;
//    if (!m_pointContacts.empty())
//    {
//        msg_info() << "-------------- POINT START ----------------";
//        for each (contactInfo* cInfo in m_pointContacts)
//        {
//            Vector3 vec = position - cInfo->pointB;
//            SReal realDistance = vec.norm();
//            SReal collDistance = realDistance - contactDist - sphereRadius;
//            
//            if (collDistance > responseDist)
//                continue;
//
//            bool found = false;
//            for (auto idP : idsP)
//            {
//                if (idP == cInfo->elemId)
//                {
//                    found = true;
//                    break;
//                }
//            }
//
//            if (found)
//                continue;
//            
//            msg_info() << "id: " << cInfo->elemId << " realDistance: " << realDistance << " | collDistance: " << collDistance << " | cInfo->dist: " << cInfo->dist;
//
//            if (collDistance < 0.0)
//            {
//                std::cout << "collDistance: " << collDistance << std::endl;
//                facMax *= fabs(collDistance);
//                collDistance = 0.0;
//            }
//            
//            SReal factorN = (collDistance - responseDist) * (collDistance - responseDist) * facMax;
//            factorN *= factorN;
//            vec.normalize();
//            
//            //cInfo->normal = vec * factorN;
//            m_toolForceFeedBack += vec * factorN;
//
//            cpt++;
//            msg_info() << "collDistance:  " << collDistance << " - "  << responseDist << " = " << factorN;
//            msg_info() << "vec: " << cInfo->normal.norm() << " ++ " << m_toolForceFeedBack.norm();
//        }
//
//        msg_info() << " ### force: " << m_toolForceFeedBack.norm();
//        msg_info() << "-------------- POINT END ----------------";
//    }
//    
//
//    lockContraints.unlock();
//    return m_toolForceFeedBack;
//}



static int nullForceFeedbackClass = sofa::core::RegisterObject("Null force feedback for haptic feedback device")
        .add< BruteForceFeedback >();

} // namespace controller
} // namespace component
} // namespace sofa
