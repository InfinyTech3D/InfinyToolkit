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
        , d_N_rays(initData(&d_N_rays, (unsigned int)128, "N_rays", "Number of angular rays"))
        , d_N_depth(initData(&d_N_depth, (unsigned int)256, "N_depth", "Number of radial depth samples"))
        , d_maxDepth(initData(&d_maxDepth, 10.0, "maxDepth", "Maximum probe depth"))
        , d_alpha(initData(&d_alpha, 0.15, "alpha", "Attenuation coefficient"))
        , d_reflectionCoeff(initData(&d_reflectionCoeff, 1.5, "reflectionCoeff", "Reflection boost"))
        , d_noiseSigma(initData(&d_noiseSigma, 10.0, "noiseSigma", "Gaussian noise sigma"))
        , d_K_points(initData(&d_K_points, (unsigned int)3, "K_points", "Points near probe tip"))
        , d_maxStoredFrames(initData(&d_maxStoredFrames, (unsigned int)400, "maxStoredFrames", "Number of stored IVUS frames"))
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

        //vesselNode = this->getRoot()->getChild("VesselNode");

       ///* msg_info() << "Catheter DOFs: " << cath->getSize();
       // msg_info() << "Triangles: " << geo->getTopology()->getNbTriangles();*/

        if (l_vesselNode.get() == nullptr)
        {
            msg_error() << "Vessel node link not set!";
            return;
        }


        this->f_listening.setValue(true);
        
        currentFrame = cv::Mat::zeros(d_N_depth.getValue(), d_N_rays.getValue(), CV_8UC1);
    }

    void IVUSController::handleEvent(sofa::core::objectmodel::Event* event)
    {
       
       
        auto* mech = l_CatheterState.get();
        const VecCoord& positions = mech->readPositions();
        
        // Extract last K points near tip
        std::vector<Vec3> probePositions;

        unsigned int K = std::min(
            static_cast<unsigned int>(d_K_points.getValue()), 
            static_cast<unsigned int>(positions.size())
        );
        
        for (unsigned int k = 0; k < K; ++k)
        {
            const Coord& rigid = positions[positions.size() - 1 - k];
            auto center = rigid.getCenter();
            std::cout << "Probe " << k << " center: "
                << center[0] << ", "
                << center[1] << ", "
                << center[2] << std::endl;

            probePositions.push_back(rigid.getCenter());
        }


        // Accumulate frames for multi-point averaging
        cv::Mat accumulated = cv::Mat::zeros(d_N_depth.getValue(), d_N_rays.getValue(), CV_64F);

        for (auto& probePos : probePositions)
        {
            cv::Mat tempFrame = computeSingleProbeFrame(probePos);

            //cv::imshow("Probe Frame", tempFrame);
            //cv::waitKey(1);   // needed for OpenCV window refresh


            tempFrame.convertTo(tempFrame, CV_64F);
            accumulated += tempFrame;
        }

        // Average over K points
        accumulated /= static_cast<double>(K);
        accumulated.convertTo(currentFrame, CV_8UC1);

        // Add Gaussian noise
        cv::Mat noise(currentFrame.size(), CV_8UC1);
        cv::randn(noise, 0, d_noiseSigma.getValue());
        currentFrame += noise;

        // Store frame
        ivusFrames.push_back(currentFrame.clone());
        if (ivusFrames.size() > d_maxStoredFrames.getValue())
            ivusFrames.erase(ivusFrames.begin());

        // Build longitudinal image (side view)
        longitudinalImage = cv::Mat::zeros(d_N_depth.getValue(), ivusFrames.size(), CV_8UC1);
        unsigned int selectedAngle = d_N_rays.getValue() / 2;  // central radial slice
        for (size_t t = 0; t < ivusFrames.size(); ++t)
        {
            for (unsigned int d = 0; d < d_N_depth.getValue(); ++d)
            {
                longitudinalImage.at<uint8_t>(d, t) =
                    ivusFrames[t].at<uint8_t>(d, selectedAngle);
            }
        }

        // Display images
        cv::imshow("IVUS Cross Section", currentFrame);
        cv::imshow("IVUS Longitudinal", longitudinalImage);
        cv::waitKey(1);
    }

    cv::Mat IVUSController::computeSingleProbeFrame(const Vec3& probepos)
    {
        cv::Mat frame = cv::Mat::zeros(d_N_depth.getValue(), d_N_rays.getValue(), CV_8UC1);
    
        // build roi
        std::vector<unsigned int> roiTriangles;
        IVUSController::buildROI(probepos, roiTriangles);
    
        auto* trimodel = l_vesselNode->getNodeObject<TriangleCollisionModel<sofa::defaulttype::Vec3Types>>();

        // Mechanical positions (Rigid3d coordinates)
        const auto& positions = trimodel->getX(); // VecCoord
        const auto& triangles = trimodel->getTriangles(); // SeqTriangles from topology

        // Temporary storage required by computeIntersectionsLineTriangle
        sofa::type::vector<sofa::Index> indices;
        sofa::type::vector<double> vecBaryCoef;
        sofa::type::vector<double> vecCoordKmin;


        for (unsigned int i = 0; i < d_N_rays.getValue(); ++i)
        {
            double angle = 2.0 * std::numbers::pi * i / d_N_rays.getValue();
            Vec3 dir(cos(angle), sin(angle), 0.0);

            Vec3 p1 = probepos;
            Vec3 p2 = probepos + dir * d_maxDepth.getValue();

            double nearestHit = d_maxDepth.getValue();

            // Loop over triangles in ROI
            for (auto triIndex : roiTriangles)
            {
                bool hit = l_triangleGeo->computeIntersectionsLineTriangle(
                    false,                               // is_entered
                    sofa::type::Vec<3, double>(p1[0], p1[1], p1[2]),
                    sofa::type::Vec<3, double>(p2[0], p2[1], p2[2]),
                    triIndex,
                    indices,
                    vecBaryCoef,
                    vecCoordKmin
                );

                if (hit && !vecCoordKmin.empty() && vecCoordKmin[0] < nearestHit)
                {
                    nearestHit = vecCoordKmin[0];
                }

                // Clear vectors for next triangle
                indices.clear();
                vecBaryCoef.clear();
                vecCoordKmin.clear();
            }

            // Fill ultrasound frame
            for (unsigned int j = 0; j < d_N_depth.getValue(); ++j)
            {
                double depth = d_maxDepth.getValue() * j / d_N_depth.getValue();
                double attenuation = exp(-d_alpha.getValue() * depth);
                double intensity = 40 * attenuation;

                if (std::abs(depth - nearestHit) < (d_maxDepth.getValue() / d_N_depth.getValue()))
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
                triangleIndices.push_back(i);
        }

        // For Debug
       /* if (!triangleIndices.empty())
        {
            std::cout << "Triangles found: " << triangleIndices.size() << std::endl;

            for (const auto& idx : triangleIndices)
            {
                std::cout << "Triangle index: " << idx << std::endl;
            }
        }
        else
        {
            std::cout << "triangleIndices vector is empty." << std::endl;
        }*/

      
    }

  /*  void IVUSController::buildROI(
        const Vec3& probepos,
        std::vector<unsigned int>& triangleIndices)
    {
        triangleIndices.clear();

        if (!l_triangleGeo)
        {
            msg_error() << "TriangleSetGeometryAlgorithms not initialized!";
            return;
        }

        const sofa::Size nbTriangles = l_triangleGeo->getTopology()->getNbTriangles();
        const auto maxDepth = d_maxDepth.getValue();

        for (sofa::Index i = 0; i < nbTriangles; ++i)
        {
            Vec3 center = l_triangleGeo->computeTriangleCenter(i);

            if ((center - probepos).norm() < maxDepth)
            {
                triangleIndices.push_back(i);
            }
        }
    }*/
    
    

}
