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
#include <InfinyToolkit/InteractionTools/GrasperJawModel.h>
#include <InfinyToolkit/InteractionTools/ScissorJawModel.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>

namespace sofa::infinytoolkit
{


/** 
*
*/
class SOFA_INFINYTOOLKIT_API ArticulatedToolManager : public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(ArticulatedToolManager,core::objectmodel::BaseObject);

    using Vec3 = sofa::type::Vec3;
    using RigidCoord = sofa::defaulttype::RigidTypes::Coord;
    using ContactVector = type::vector<core::collision::DetectionOutput>;

protected:
    ArticulatedToolManager();

    ~ArticulatedToolManager() override = default;

public:
    virtual void init() override;
    int testModels();
    
    const sofa::type::vector< int >& vertexIdsInBroadPhase() { return m_idBroadPhase; }
    const sofa::type::vector< int >& vertexIdsGrabed() { return m_idgrabed; }


    // global methods    
    bool computeBoundingBox();

    // Main API to activate or 
	bool deActivateTool();
	bool activateTool();

    int performAction();
    bool stopAction();

    void closeTool();
    void openTool();


    // Method from intern test
    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;
    
    void draw(const core::visual::VisualParams* vparams) override;

protected:
    void clearContacts();

    void filterCollision();

public:
    // Path to the different JawModel
    SingleLink<ArticulatedToolManager, BaseJawModel, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_jawModel1;
    SingleLink<ArticulatedToolManager, BaseJawModel, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_jawModel2;

    // link to the scene detection Method component (Narrow phase only)
    SingleLink<ArticulatedToolManager, core::collision::NarrowPhaseDetection, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_detectionNP;

    SingleLink<ArticulatedToolManager, sofa::core::behavior::BaseMechanicalState, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_targetModel;

    
    Data<SReal> d_angleJaw1; //up
    Data<SReal> d_angleJaw2; //down
    Data<SReal> d_handleFactor;

    Data <RigidCoord> d_inputPosition;

    Data <type::vector<RigidCoord> > d_outputPositions;
    Data<bool> d_drawContacts; ///< if true, draw the collision outputs

protected:
    // Buffer of points ids 
    sofa::type::vector <int> m_idgrabed;
    sofa::type::vector <int> m_idBroadPhase;

    // Pointer to the mechanicalObject
    BaseJawModel::SPtr m_jawModel1 = nullptr;
    BaseJawModel::SPtr m_jawModel2 = nullptr;

};


} // namespace sofa::infinytoolkit
