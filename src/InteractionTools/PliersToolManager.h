/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_INTERACTIONTOOLS_PLIERSTOOLMANAGER_H
#define SOFA_INTERACTIONTOOLS_PLIERSTOOLMANAGER_H
#include <InteractionTools/config.h>

#include <SofaDeformable/StiffSpringForceField.h>
#include <SofaGeneralObjectInteraction/AttachConstraint.h>

namespace sofa
{

namespace component
{

namespace misc
{

typedef sofa::component::interactionforcefield::StiffSpringForceField< sofa::defaulttype::Vec3Types > StiffSpringFF;
typedef sofa::component::interactionforcefield::StiffSpringForceField< sofa::defaulttype::Vec3Types > StiffSpringFF;
typedef sofa::component::projectiveconstraintset::AttachConstraint< sofa::defaulttype::Vec3Types > AttachConstraint;

/** 
*
*/
class SOFA_INTERACTIONTOOLS_API PliersToolManager: public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(PliersToolManager,core::objectmodel::BaseObject);

protected:
    PliersToolManager();

    virtual ~PliersToolManager();

public:
    virtual void init() override;
    virtual void reinit() override;
    int testModels();
    
    const sofa::helper::vector< int >& vertexIdsInBroadPhase() { return m_idBroadPhase; }
    const sofa::helper::vector< int >& vertexIdsGrabed() { return m_idgrabed; }


    // global methods    
    int createFF(float _stiffness);
    bool computeBoundingBox();
    void computeVertexIdsInBroadPhase(float margin = 0.0);

	bool unactiveTool();
	bool reactiveTool();

    // API from grabing
    const sofa::helper::vector< int >& grabModel();

    void releaseGrab();

    
    // API for cutting
    int cutFromTetra(float minX, float maxX, bool cut = true);
    int pathCutFromTetra(float minX, float maxX);
    void cutFromTriangles();
    

    // Method from intern test
    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;
    void computePlierAxis();

    void setPlierAxis(sofa::defaulttype::Mat3x3 _matP) { matP = _matP; }
    void setPlierOrigin(sofa::defaulttype::Vec3 _zero) { zero = _zero; }

    sofa::defaulttype::Vector3 m_min, m_max;


    void draw(const core::visual::VisualParams* vparams) override;

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MeshTopology.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        return BaseObject::canCreate(obj, context, arg);
    }

protected:
    

public:
    // Path to the different mechanicalObject
    Data<std::string> m_pathMord1;
    Data<std::string> m_pathMord2;
    Data<std::string> m_pathModel;
	    
protected:
    // Buffer of points ids 
    sofa::helper::vector <int> m_idgrabed;
    sofa::helper::vector <int> m_idBroadPhase;

    SReal m_oldCollisionStiffness;

    // Projection matrix to move into plier coordinate. X = along the plier, Y -> up, Z -> ortho to plier
    sofa::defaulttype::Mat3x3 matP;
    sofa::defaulttype::Vec3 zero;
    sofa::defaulttype::Vec3 xAxis;
    sofa::defaulttype::Vec3 yAxis;
    sofa::defaulttype::Vec3 zAxis;

    // Pointer to the mechanicalObject
    sofa::core::behavior::BaseMechanicalState* m_mord1;
    sofa::core::behavior::BaseMechanicalState* m_mord2;
    sofa::core::behavior::BaseMechanicalState* m_model;

    // Pointer to the stiffspring FF created.
    StiffSpringFF::SPtr m_forcefieldUP;
    StiffSpringFF::SPtr m_forcefieldDOWN;
    float m_stiffness;

    // Keep it for debug drawing
    sofa::helper::vector<sofa::core::topology::Topology::TetrahedronID> tetraIdsOnCut;
    sofa::helper::vector<sofa::core::topology::Topology::TriangleID> triIdsOnCut;
};


} // namespace misc

} // namespace component

} // namespace sofa

#endif // SOFA_INTERACTIONTOOLS_PLIERSTOOLMANAGER_H
