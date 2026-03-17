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

#include <sofa/component/controller/Controller.h>
#include <sofa/simulation/Node.h>
#include <sofa/type/Vec.h>
#include <sofa/defaulttype/VecTypes.h> // Not sure I will use it
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/component/collision/geometry/TriangleCollisionModel.h>
#include <sofa/component/topology/container/dynamic/TriangleSetGeometryAlgorithms.h>
#include <opencv2/opencv.hpp>
#include <sofa/core/visual/VisualParams.h>
#include <vector>


namespace sofa::infinytoolkit
{ 

    using namespace sofa::core::objectmodel;
    using namespace  sofa::component::topology::container::dynamic;
    using sofa::core::objectmodel::Data;

    class IVUSController : public sofa::component::controller::Controller
    {
    public:
        SOFA_CLASS(IVUSController, sofa::component::controller::Controller);

        IVUSController();
        ~IVUSController() override = default;

        using DataTypes = sofa::defaulttype::Rigid3dTypes;

        using Coord = DataTypes::Coord;
        using VecCoord = DataTypes::VecCoord;
        using Vec3 = DataTypes::Vec3;
        using Quat = DataTypes::Quat;



        // Scene references , nor sure if I use it
        sofa::simulation::Node* catheterNode = nullptr;

        Data<double> r_catheter;
        Data<unsigned int> d_N_rays;
        Data<unsigned int> d_N_depth;
        Data<double> d_maxDepth;
        Data<double> d_alpha;
        Data<double> d_reflectionCoeff;
        Data<double> d_noiseSigma;
        Data<unsigned int> d_K_points;
        Data<unsigned int> d_maxStoredFrames;


        // Images
        std::vector<cv::Mat> currentFrames;                   // cross-sectional frame for each prob
        cv::Mat longitudinalImage;              // longitudinal side view
        std::vector<cv::Mat> ivusFrames;       // history buffer

        void init() override;
        void handleEvent(sofa::core::objectmodel::Event* event) override;
        void draw(const sofa::core::visual::VisualParams* vparams) override;

        private:

            SingleLink< IVUSController,
                sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3dTypes>,
                BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_CatheterState;

            
            SingleLink<IVUSController,
                TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3dTypes>,
                BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_triangleGeo; // To be removed when the whole node is passed

            SingleLink<IVUSController,
                sofa::simulation::Node,
                BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_vesselNode;

           
        //    void computeIVUSFrame();
            cv::Mat computeSingleProbeFrame(const Vec3& probePos,const Vec3& probrTangent);
            void buildROI(
                const Vec3& probePos,
                std::vector<unsigned int>& triangleIndices);

            // For debug
            std::vector<std::vector<sofa::type::Vec3>> debug_roiTrianglesPerProbe;
            std::vector<sofa::type::Vec3> debug_hitPoints;

            std::vector<sofa::type::Vec3> debug_RayIntersections_hitPoints;

            void debugcomputeIntersectionsLineTriangle(const Vec3& probePos);
            void debugrayIntersection(const Vec3& probePos);

    };
}



