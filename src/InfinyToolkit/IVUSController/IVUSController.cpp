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

#include <InfinyToolkit/IVUSController/IVUSController.h>
#include< sofa/component/collision/geometry/TriangleCollisionModel.h >
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/geometry/Triangle.h>
#include <cmath>
#include<vector>
#include <algorithm>
#include <numbers>


namespace sofa::infinytoolkit
{

    using namespace sofa::defaulttype;
    using namespace sofa::core::behavior;
    using namespace sofa::defaulttype;
    using namespace sofa::component::collision::geometry;
    using namespace  sofa::component::topology::container::dynamic;


    void registerIVUSController(sofa::core::ObjectFactory* factory)
    {
        factory->registerObjects(
            sofa::core::ObjectRegistrationData("IVUSController")
            .add< IVUSController >()
        );
    }

    IVUSController::IVUSController()
        :l_CatheterState(initLink("catheterState", "Link to the mechanical object of the catheter."))
        , l_triangleGeo(initLink("triangleGeo", "Link to the collision model of the vessel."))
        , l_vesselNode(initLink("vesselNode", "Link to the vessel node."))
        , r_catheter(initData(&r_catheter, 1.0, "device_radius", "Device raduis where the prob located."))
        , d_N_rays(initData(&d_N_rays, (unsigned int)128, "N_rays", "Number of angular rays"))
        , d_N_depth(initData(&d_N_depth, (unsigned int)256, "N_depth", "Number of radial depth samples"))
        , d_maxDepth(initData(&d_maxDepth, 10.0, "maxDepth", "Maximum probe depth"))
        , d_alpha(initData(&d_alpha, 0.15, "alpha", "Attenuation coefficient"))
        , d_reflectionCoeff(initData(&d_reflectionCoeff, 1.5, "reflectionCoeff", "Reflection boost"))
        , d_noiseSigma(initData(&d_noiseSigma, 10.0, "noiseSigma", "Gaussian noise sigma"))
        , d_K_points(initData(&d_K_points, (unsigned int)3, "K_points", "Points near probe tip"))
        , d_maxStoredFrames(initData(&d_maxStoredFrames, (unsigned int)400, "maxStoredFrames", "Number of stored IVUS frames"))
        , d_useDebugRay(initData(&d_useDebugRay, false, "useDebugRay", "Enable debug ray intersection."))
        , d_drawROI(initData(&d_drawROI, false, "drawROI", "Draw ROI triangles."))
        , d_drawHitPoints(initData(&d_drawHitPoints, true, "drawHitPoints", "Draw debug hit points."))
    {
    }
    
    
    void IVUSController::init()
    {
        // Resolve the MechanicalState of the catheter
        if (l_CatheterState.get() == nullptr)
        {
            msg_error() << "Error no catheter is found!";
            this->d_componentState.setValue(
                sofa::core::objectmodel::ComponentState::Invalid);
            return;

        }
        
        if (l_triangleGeo.get() == nullptr)
        {
            msg_info() << "No TriangleSetGeometryAlgorithms link set. "
                << "Trying to find first one in current context...";
            d_componentState.setValue(
                sofa::core::objectmodel::ComponentState::Invalid);
            return;
        }

       
        if (l_vesselNode.get() == nullptr)
        {
            msg_error() << "Vessel node link not set!";
            return;
        }


        this->f_listening.setValue(true);
        
        //currentFrame = cv::Mat::zeros(d_N_depth.getValue(), d_N_rays.getValue(), CV_8UC1);
    }

    void IVUSController::handleEvent(sofa::core::objectmodel::Event* event)
    {

        // End of the animation step
        if (dynamic_cast<sofa::simulation::AnimateEndEvent*>(event))
        {
            auto* mech = l_CatheterState.get();
            const VecCoord& positions = mech->readPositions();

            // Extract last K points near tip
            std::vector<Vec3> probePositions;
            std::vector<Vec3> probeTangents;

            unsigned int K = std::min(
                static_cast<unsigned int>(d_K_points.getValue()),
                static_cast<unsigned int>(positions.size())
            );

            for (unsigned int k = 0; k < K; ++k)
            {
                // Index of this probe
                unsigned int idx = positions.size() - 1 - k;
                const Coord& rigid = positions[idx];
                auto center = rigid.getCenter();

                probePositions.push_back(center);

                // Compute tangent using neighbor points
                Vec3 p_prev, p_next;

                if (idx == 0)
                {
                    p_prev = positions[idx].getCenter();
                    p_next = positions[idx + 1].getCenter();
                }
                else if (idx == positions.size() - 1)
                {
                    p_prev = positions[idx - 1].getCenter();
                    p_next = positions[idx].getCenter();
                }
                else
                {
                    p_prev = positions[idx - 1].getCenter();
                    p_next = positions[idx + 1].getCenter();
                }

                // Tangent with fallback for undeployed catheter
                Vec3 diff = p_next - p_prev;
                Vec3 tangent;

                if (norm(diff) < 1e-8)
                {
                    tangent = Vec3(0, 0, 1);  // fallback axis (default deployment)
                }
                else
                {
                    tangent = diff.normalized();
                }

                probeTangents.push_back(tangent);

            }

            if (d_useDebugRay.getValue())
            {
                // DEBUG MODE
                debug_RayIntersections_hitPoints.clear();

                Vec3 tangent = probeTangents[0];  // tip tangent
                Vec3 probePos = probePositions[0];

                debugSingleRayIntersection(
                    probePos,
                    tangent,
                    M_PI / 2.0,
                    IntersectionMethod::RayTriangle,
                    "HandleEventDebug"
                );
                return;
            }


            // Accumulate frames for multi-point averaging
            currentFrames.clear();

            cv::Mat accumulated = cv::Mat::zeros(d_N_depth.getValue(), d_N_rays.getValue(), CV_64F);

           

            for (unsigned int k = 0; k < probePositions.size(); ++k)
            {
                unsigned int catheterIndex = positions.size() - 1 - k;

               /* std::cout << "Processing probe k=" << k
                    << " (catheter index = " << catheterIndex << ")"
                    << std::endl;*/


                cv::Mat tempFrame = computeSingleProbeFrame(probePositions[k], probeTangents[k]);

                //// Add Gaussian noise
                //cv::Mat noise(tempFrame.size(), CV_8UC1);
                //cv::randn(noise, 0, d_noiseSigma.getValue());
                //tempFrame += noise;

                //// Convert for accumulation
                //cv::Mat tempFrame64;
                //tempFrame.convertTo(tempFrame64, CV_64F);

                //accumulated += tempFrame64;

                //// Store displayable version
                //currentFrames.push_back(tempFrame.clone());
            }

            // Average over K points
          /*  accumulated /= static_cast<double>(K);
            cv::Mat averagedFrame;
            accumulated.convertTo(averagedFrame, CV_8UC1);*/

            
            // Store frame
           /* ivusFrames.push_back(currentFrame.clone());
            if (ivusFrames.size() > d_maxStoredFrames.getValue())
                ivusFrames.erase(ivusFrames.begin());*/

            // Build longitudinal image (side view)
            //longitudinalImage = cv::Mat::zeros(d_N_depth.getValue(), ivusFrames.size(), CV_8UC1);
            //unsigned int selectedAngle = d_N_rays.getValue() / 2;  // central radial slice
            //for (size_t t = 0; t < ivusFrames.size(); ++t)
            //{
            //    for (unsigned int d = 0; d < d_N_depth.getValue(); ++d)
            //    {
            //        longitudinalImage.at<uint8_t>(d, t) =
            //            ivusFrames[t].at<uint8_t>(d, selectedAngle);
            //    }
            //}

            // Display images
            //cv::namedWindow("currentFrame", cv::WINDOW_AUTOSIZE);
            //cv::imshow("IVUS Longitudinal", longitudinalImage);
         /*   for (size_t k = 0; k < currentFrames.size(); ++k)
            {
                std::string name = "Probe " + std::to_string(k);
                cv::imshow(name, currentFrames[k]);
            }*/

            //cv::imshow("Averaged IVUS", averagedFrame);

            //cv::waitKey(1);
        }
       
    }

    cv::Mat IVUSController::computeSingleProbeFrame(const Vec3& probepos, const Vec3& probeTangent)
    {
        //Precompute constants
        const double maxDepth = d_maxDepth.getValue();
        const unsigned int N_rays = d_N_rays.getValue();
        const unsigned int N_depth = d_N_depth.getValue();
        const double alpha = d_alpha.getValue();
        const double reflectionCoeff = d_reflectionCoeff.getValue();

        cv::Mat frame = cv::Mat::zeros(N_depth, N_rays, CV_8UC1);
    
        // build roi
        std::vector<unsigned int> roiTriangles;

        IVUSController::buildROI(probepos, roiTriangles);
        
    
        auto* triModel = l_vesselNode->getNodeObject<TriangleCollisionModel<sofa::defaulttype::Vec3Types>>();

        // Access mechanical object positions
        const auto& positions = triModel->getX();
        // Access topology triangles
        const auto& triangles = triModel->getTriangles();


        debug_hitPoints.clear();

        // Build local plane
        Vec3 tangent = probeTangent;
      
        // Compute orthonormal plane perpendicular to tangent
        Vec3 u, v;
        computePerpendicularPlane(tangent, u, v);

        
        
        //Shoot rays in the (u,v) plane
        for (unsigned int i = 0; i < N_rays; ++i)
        {
            double angle = 2.0 * std::numbers::pi * i / N_rays;

           
            Vec3 direction = cos(angle) * u + sin(angle) * v;
            Vec3 origin = probepos;

            Vec3 hitPoint = computeRayHitPoint(origin, direction, roiTriangles);

            double nearestHit = (hitPoint - origin).norm();
            
         
            if (nearestHit < maxDepth)
            {
                debug_hitPoints.push_back(hitPoint);
            }

            // Fill ultrasound frame
            for (unsigned int j = 0; j < N_depth; ++j)
            {
                double depth = maxDepth * j / N_depth;
                double attenuation = exp(-alpha * depth);
                double intensity = 40 * attenuation;

                if (std::abs(depth - nearestHit) < (maxDepth / N_depth))
                    intensity = 255 * d_reflectionCoeff.getValue() * attenuation;

                frame.at<uint8_t>(j, i) = static_cast<uint8_t>(std::min(255.0, intensity));
            }
        }

        return frame;
        
    }
    
    void IVUSController::buildROI(
        const Vec3& probepos,
        std::vector<unsigned int>& triangleIndices)
    {
        triangleIndices.clear();
        auto* triModel = l_vesselNode->getNodeObject<TriangleCollisionModel<sofa::defaulttype::Vec3Types>>();
        if (!triModel)
        {
            msg_error() << "No TriangleCollisionModel found in vesselNode!";
            return;
        }

        // Access mechanical object positions
        const auto& positions = triModel->getX();  // This is a vector of Rigid3dTypes::Coord (or Vec3)

        // Access topology triangles
        const auto& triangles = triModel->getTriangles(); // SeqTriangles from BaseMeshTopology

        debug_roiTrianglesPerProbe.clear();


        for (sofa::Index i = 0; i < triangles.size(); ++i)
        {
            const auto& tri = triangles[i];  // tri is an array of 3 vertex indices

            // Get vertex positions from mechanical state
            const auto& v0 = positions[tri[0]];
            const auto& v1 = positions[tri[1]];
            const auto& v2 = positions[tri[2]];

            // Use the first vertex or the centroid to check distance
            Vec3 center = (v0 + v1 + v2) / 3.0;


            if ((center - probepos).norm() < d_maxDepth.getValue())
            {
                std::vector<sofa::type::Vec3> roiVertices;
               
                roiVertices.push_back(v0);
                roiVertices.push_back(v1);
                roiVertices.push_back(v2);
                
                // store triangles for this probe
                debug_roiTrianglesPerProbe.push_back(roiVertices);

                triangleIndices.push_back(i);

            }
                
        }

        
    }

   
    IVUSController::Vec3 IVUSController::computeRayHitPoint(const Vec3& origin, const Vec3& rayDir, const std::vector<unsigned int>& roiTriangles)
    {
        auto* triModel = l_vesselNode->getNodeObject<TriangleCollisionModel<sofa::defaulttype::Vec3Types>>();

        const auto& positions = triModel->getX();
        const auto& triangles = triModel->getTriangles();

        double nearestT = d_maxDepth.getValue();
        Vec3 nearestHit = origin;

        for (auto triIndex : roiTriangles)
        {
            const auto& tri = triangles[triIndex];
            Vec3 n0 = positions[tri[0]];
            Vec3 n1 = positions[tri[1]];
            Vec3 n2 = positions[tri[2]];

            double t, u_coef, v_coef;
            bool hit = sofa::geometry::Triangle::rayIntersection(n0, n1, n2, origin, rayDir, t, u_coef, v_coef);

            if (hit && t < nearestT)
            {
                nearestT = t;
                nearestHit = origin + rayDir * t;
            }
        }

        return nearestHit;
    }

    void IVUSController::debugSingleRayIntersection(
        const Vec3& probePos,
        const Vec3& tangent,
        double angle,
        IntersectionMethod method,
        const std::string& debugName)
    {
        // ROI
        std::vector<unsigned int> roiTriangles;
        IVUSController::buildROI(probePos, roiTriangles);

        // Triangle model (needed for RayTriangle)
        auto* triModel = l_vesselNode->getNodeObject<TriangleCollisionModel<sofa::defaulttype::Vec3Types>>();

        const auto& positions = triModel->getX();
        const auto& triangles = triModel->getTriangles();

        // Buffers (for LineTriangle)
        sofa::type::vector<sofa::Index> indices;
        sofa::type::vector<double> vecBaryCoef;
        sofa::type::vector<double> vecCoordKmin;

        // Ray definition
        // Compute orthonormal plane perpendicular to tangent
        Vec3 u, v;
        computePerpendicularPlane(tangent, u, v);

        // Ray direction in the plane
        Vec3 dir = std::cos(angle) * u + std::sin(angle) * v;

        Vec3 origin = probePos;
        Vec3 p2 = origin + dir * d_maxDepth.getValue();

        double nearestHit = d_maxDepth.getValue();
        bool foundHit = false;

        std::cout << "\n[DEBUG: " << debugName << "]\n";
        std::cout << "Angle: " << angle << "\n";

        //--------------------------------------------------
        for (auto triIndex : roiTriangles)
        {
            bool hit = false;
            double t = 0.0;

            if (method == IntersectionMethod::LineTriangle)
            {
                hit = l_triangleGeo->computeIntersectionsLineTriangle(
                    false,
                    origin,
                    p2,
                    triIndex,
                    indices,
                    vecBaryCoef,
                    vecCoordKmin
                );

                if (hit && !vecCoordKmin.empty())
                {
                    t = vecCoordKmin[0];
                }

                indices.clear();
                vecBaryCoef.clear();
                vecCoordKmin.clear();
            }
            else // RayTriangle
            {
                const auto& tri = triangles[triIndex];

                Vec3 n0 = positions[tri[0]];
                Vec3 n1 = positions[tri[1]];
                Vec3 n2 = positions[tri[2]];

                double u, v;
                hit = sofa::geometry::Triangle::rayIntersection(
                    n0, n1, n2,
                    origin, dir,
                    t, u, v
                );
            }

            //--------------------------------------------------

            if (hit && t < nearestHit)
            {
                nearestHit = t;
                foundHit = true;
            }
        }

        //--------------------------------------------------

        if (foundHit)
        {
            Vec3 hitPoint;

            if (method == IntersectionMethod::LineTriangle)
            {
                hitPoint = origin + (p2 - origin) * (1.0 - nearestHit);
            }
            else
            {
                hitPoint = origin + dir * nearestHit;
            }

            std::cout << "Nearest t: " << nearestHit << "\n";
            std::cout << "HitPoint = ("
                << hitPoint[0] << ", "
                << hitPoint[1] << ", "
                << hitPoint[2] << ")\n";

            debug_RayIntersections_hitPoints.push_back(hitPoint);
        }
        else
        {
            std::cout << "No intersection found.\n";
        }
    }

  



    void IVUSController::computePerpendicularPlane(
        const Vec3& tangent, // catheter tangent
        Vec3& u,             // first perpendicular vector (output)
        Vec3& v              // second perpendicular vector (output)
    )
    {
        // Step 1: normalize tangent
        Vec3 t = tangent.normalized();

        // Step 2: pick a vector not parallel to tangent
        Vec3 arbitrary;
        if (std::abs(t[2]) < 0.9)        // not mostly along Z
            arbitrary = Vec3(0, 0, 1);
        else
            arbitrary = Vec3(1, 0, 0);

        // Step 3: compute first perpendicular vector
        u = cross(t, arbitrary).normalized();

        // Step 4: second perpendicular vector
        v = cross(t, u).normalized();
    }


    void IVUSController::draw(const sofa::core::visual::VisualParams* vparams)
    {
        if (!vparams || !vparams->displayFlags().getShowVisualModels())
            return;

        auto* drawTool = vparams->drawTool();
        if (!drawTool)
            return;

        //----------------------------------
        // Colors (define once)
        static const sofa::type::RGBAColor roiColor(0.f, 1.f, 0.f, 1.f);   // green
        static const sofa::type::RGBAColor hitColor(0.f, 0.f, 1.f, 1.f);   // blue
        static const sofa::type::RGBAColor rayColor(1.f, 0.f, 0.f, 1.f);   // red

        //----------------------------------
        // ROI TRIANGLES
        if (d_drawROI.getValue())
        {
            for (const auto& probeTriangles : debug_roiTrianglesPerProbe)
            {
                if (!probeTriangles.empty())
                    drawTool->drawTriangles(probeTriangles, roiColor);
            }
        }

        //----------------------------------
        // GENERAL HIT POINTS
        if (d_drawHitPoints.getValue() && !debug_hitPoints.empty())
        {
            drawTool->drawPoints(debug_hitPoints, 5.0f, hitColor);
        }

        //----------------------------------
        // RAY INTERSECTION POINTS
        if (d_useDebugRay.getValue() && !debug_RayIntersections_hitPoints.empty())
        {
            drawTool->drawPoints(debug_RayIntersections_hitPoints, 8.0f, rayColor);
        }
    }


  
    
    

}
