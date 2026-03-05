#include <InfinyToolkit/IVUSController/IVUSController.h>
#include< sofa/component/collision/geometry/TriangleCollisionModel.h >
#include <sofa/core/ObjectFactory.h>
#include <cmath>
#include<vector>
#include <algorithm>
#include <numbers>

using sofa::defaulttype::Vec3dTypes;



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
            sofa::core::ObjectRegistrationData("")
            .add< IVUSController >()
        );
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
        
        if (l_triangleGeo.empty())
        {
            msg_info() << "No TriangleSetGeometryAlgorithms link set. "
                << "Trying to find first one in current context...";
            //l_triangleGeo.set(this->getContext()->getNodeObject<TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>()); //will back to it
        }
        
        currentFrame = cv::Mat::zeros(N_depth, N_rays, CV_8UC1);
    }

    void IVUSController::handleEvent(sofa::core::objectmodel::Event* event)
    {
       
       
        auto* mech = l_CatheterState.get();
        const VecCoord& positions = mech->readPositions();
        
        // Extract last K points near tip
        std::vector<Vec3> probePositions;

        unsigned int K = std::min(
            static_cast<unsigned int>(K_points), // should be passed by getValue
            static_cast<unsigned int>(positions.size())
        );
        
        for (unsigned int k = 0; k < K; ++k)
        {
            const Coord& rigid = positions[positions.size() - 1 - k];
            probePositions.push_back(rigid.getCenter());
        }


        // Accumulate frames for multi-point averaging
        cv::Mat accumulated = cv::Mat::zeros(N_depth, N_rays, CV_64F);

        for (auto& probePos : probePositions)
        {
            cv::Mat tempFrame = computeSingleProbeFrame(probePos);
            tempFrame.convertTo(tempFrame, CV_64F);
            accumulated += tempFrame;
        }

        // Average over K points
        accumulated /= static_cast<double>(K);
        accumulated.convertTo(currentFrame, CV_8UC1);

        // Add Gaussian noise
        cv::Mat noise(currentFrame.size(), CV_8UC1);
        cv::randn(noise, 0, noiseSigma);
        currentFrame += noise;

        // Store frame
        ivusFrames.push_back(currentFrame.clone());
        if (ivusFrames.size() > maxStoredFrames)
            ivusFrames.erase(ivusFrames.begin());

        // Build longitudinal image (side view)
        longitudinalImage = cv::Mat::zeros(N_depth, ivusFrames.size(), CV_8UC1);
        unsigned int selectedAngle = N_rays / 2;  // central radial slice
        for (size_t t = 0; t < ivusFrames.size(); ++t)
        {
            for (unsigned int d = 0; d < N_depth; ++d)
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
        cv::Mat frame = cv::Mat::zeros(N_depth, N_rays, CV_8UC1);
    
        // build roi
        std::vector<unsigned int> roiTriangles;
        IVUSController::buildROI(probepos, roiTriangles);
    
        auto* trimodel = vesselNode->getNodeObject<TriangleCollisionModel<sofa::defaulttype::Vec3Types>>();

        // Mechanical positions (Rigid3d coordinates)
        const auto& positions = trimodel->getX(); // VecCoord
        const auto& triangles = trimodel->getTriangles(); // SeqTriangles from topology

        // Temporary storage required by computeIntersectionsLineTriangle
        sofa::type::vector<sofa::Index> indices;
        sofa::type::vector<double> vecBaryCoef;
        sofa::type::vector<double> vecCoordKmin;


        for (unsigned int i = 0; i < N_rays; ++i)
        {
            double angle = 2.0 * std::numbers::pi * i / N_rays;
            Vec3 dir(cos(angle), sin(angle), 0.0);

            Vec3 p1 = probepos;
            Vec3 p2 = probepos + dir * maxDepth;

            double nearestHit = maxDepth;

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
            for (unsigned int j = 0; j < N_depth; ++j)
            {
                double depth = maxDepth * j / N_depth;
                double attenuation = exp(-alpha * depth);
                double intensity = 40 * attenuation;

                if (std::abs(depth - nearestHit) < (maxDepth / N_depth))
                    intensity = 255 * reflectionCoeff * attenuation;

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
        auto* triModel = vesselNode->getNodeObject<TriangleCollisionModel<sofa::defaulttype::Vec3Types>>();
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

            if ((center - probepos).norm() < maxDepth)
                triangleIndices.push_back(i);
        }

      
    }
    
    

}
